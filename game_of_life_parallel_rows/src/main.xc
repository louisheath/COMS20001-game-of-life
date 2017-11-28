// COMS20001 - Game of Life: Jack Jones and Louis Heath
// (using the XMOS i2c accelerometer)
// Version 2 : Four workers

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <assert.h>

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width
#define  NWKS 4                   //number of workers
#define  NPKT IMWD/8              //number of packets in a row

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0]: port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0]: port p_sda = XS1_PORT_1F;

on tile[0]: in port buttons = XS1_PORT_4E; //port to access xCore-200 buttons
on tile[0]: out port leds = XS1_PORT_4F;   //port to access xCore-200 LEDs
    //1st bit...separate green LED
    //2nd bit...blue LED
    //3rd bit...green LED
    //4th bit...red LED

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

void tests();

// modulo function that works with negatives
int mod(int n,int x) {
    while (x < 0) x += n;
    return (x % n);
}

uchar pack(int index, uchar byte, uchar c) {    // pack c into packet 'byte' at index 'index'
    if (!c) { // if (c == 0x00)
        return (byte & ~(1 << index));
    }
    // else if (c == 0xFF)
    return (byte | (1 << index));
}

int unpack(int index, uchar byte) {             // unpack bit from index 'index' of packet 'byte'
    return (byte >> index) & 1;
}

// function that constantly tracks processing time and checks for clock overflow
void checkTime(chanend req, chanend pause) {
    // timer variables. timer ticks at 100MHz, 100000 ticks is 1ms.
    timer t;
    uint32_t prev,
             curr;
    uint32_t timeTaken = 0;     // total time in milliseconds

    // variable for identifying whether game is paused
    int paused = 0;

    // receive signal to start timing
    req :> uchar x;
    t :> prev;
    //printf("Timer starting\n");

    while (1) {
        // Timer isn't paused so increment and wait for output request
        while (!paused){

            [[ordered]]
            select {
                case req :> uchar x:    // check to see if distributer wants the time
                    if (x == 1) {
                        req <: timeTaken;
                        paused = 1;     // pause while distributer outputs
                    }
                    break;
                case pause :> uchar x:  // check to see if board is tilted and timing should pause.
                    if (x == 1) {
                        paused = 1;
                        req <: timeTaken;   // tell distributer to pause
                        printf("Timer paused\n");
                    }
                    break;
                default:
                    t :> curr;

                    // increment change in time
                    timeTaken += curr / 100000;
                    timeTaken -=  prev / 100000;
                    // if overflow is hit, curr is 42950 too small. Compensate.
                    if (prev > curr) timeTaken += 42950;
                    prev = curr;
                    break;
            }
        }

        // Timer is paused, wait for permission to unpause
        select {
            case pause :> uchar x:  // if board is untilted
                if (x == 0) {
                    paused = 0;
                    t :> prev;
                    req <: (uchar) 0;   // tell distributer to unpause
                    printf("Timer unpaused by tilt\n");
                }
                break;
            case req :> uchar x:    // if distributer has finished outputting and wants to resume
                if (x == 0) {
                    paused = 0;
                    t :> prev;
                    printf("Timer unpaused by dist\n");
                }
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{
    int res;
    uchar line[ IMWD ];

    printf( "DataInStream: Start...\n");
    //Open PGM file
    res = _openinpgm( infname, IMWD, IMHT );
    if( res ) {
        printf( "DataInStream: Error openening %s\n.", infname );
        return;
    }

    //Read image line-by-line and send byte by byte to channel c_out
    for( int y = 0; y < IMHT; y++ ) {
        _readinline( line, IMWD );

        uchar packet = 0x00;
        for ( int p = 0; p < NPKT; p++) {         // for every packet we can fit in the row
            for ( int x = 0; x < 8; x++) {          // for each of the eight bits to be packed
                packet = pack(x, packet, line[x + p*8]);
                //printf( "-%4.1d ", line[p*8 + x] ); //show image values
            }
            c_out <: packet;
            packet = 0x00;
        }
        //printf("\n");
    }

    //Close PGM image file
    _closeinpgm();
    printf( "DataInStream: Complete\n");
    return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Distributor receives current world state from DataStreamIn,
//             divides world and distributes it to workers,
//             receives new cells from workers and builds new world,
//             and sends new world to DataStreamOut
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend c_control, in port b, out port LEDs, chanend worker[NWKS], chanend reqTime)
{
    tests();

    // game state
    uchar val[NPKT][IMHT];

    // variables for hardware control
    int button = 0;           // button input from board/button listener
    uchar flashState = 0;     // state of flashing seperate green LED

    // variables for output when paused
    uint32_t timeTaken;       // time taken to process image
    int numAlive = 0;         // number of alive cells at pause
    int i = 0;                // current round at pause

    //Starting up and wait for tilting of the xCore-200 Explorer
    printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
    printf( "Waiting for button press...\n" );

    // wait for SW1 button input
    while (button != 14) {
        b :> button;
    }

    LEDs <: 4; // indicate start with green LED

    printf( "Processing...\n" );

    // Read input from DataInStream
    for(int y = 0; y < IMHT; y++ ) {
        for( int p = 0; p < NPKT; p++ ) {
            c_in :> val[p][y];
        }
    }

    // Inform orientation to start detecting tilts
    c_control <: (uchar) 1;
    // Start timing
    reqTime <: (uchar) 0;

    // Divide work between workers
    for(int w = 0; w < NWKS; w++) {                         // for each of the workers
        for(int y = 0; y < ((IMHT / NWKS) + 2); y++ ) {     // for the portion of rows to be given to the worker
            for( int p = 0; p < NPKT; p++ ) {               // for every packet
                // send cell values to the worker, who will combine them into a new array
                worker[w] <: val[p][mod(IMHT, (y + (w*(IMHT / NWKS)) - 1))];
            }
        }
    }

    // game runs infinitely
    while (1) {
        // loop whilst SW2 hasn't been pressed
        while (button != 13) {
            select {
                // if worker 0 tell us we are at the 100th iteration
                case worker[0] :> uchar x:
                    button = 13;
                    break;
                // if timer gives us the current time, pause
                case reqTime :> timeTaken:
                    // display red pause LED
                    LEDs <: (uchar) 8;
                    // send signal to worker for pausing
                    for (int w = 0; w < NWKS; w++) worker[w] <: (uchar) 1;
                    printf("Told workers to pause/n");
                    // recieve number of alive cells from each worker
                    for(int w = 0; w < NWKS; w++) {
                        int x; // temporary storage of number of alive cells value from workers
                        worker[w] :> x;
                        numAlive += x;
                    }

                    // get round iteration from last worker in 'pipeline'
                    worker[NWKS - 1] :> i;

                    printf("Status Report:\n"
                           " Number of rounds processed: %d\n"
                           " Current number of live cells: %d\n"
                           " Processing time elapsed: %dms\n", i, numAlive, timeTaken);
                    // wait until unpause
                    reqTime :> uchar resume;
                    numAlive = 0; // reset alive cell counter
                    break;
                default:
                    // read button input, if any
                    b :> button;
                    // send signal to worker for processing
                    for(int w = 0; w < NWKS; w++) worker[w] <: (uchar) 0;
                    // flash processing LED via state change
                    flashState ^= 1;
                    LEDs <: flashState;
                    break;
            }
        }

        // send signal to worker for outputting
        // must be done before recieve values to prevent other workers
        for(int w = 0; w < NWKS; w++) {
            worker[w] <: (uchar) 2;
        }

        // Receive processed cells from workers
        for(int w = 0; w < NWKS; w++) {                   // for each of the workers
            for(int y = 0; y < ((IMHT / NWKS)); y++ ) {   // for each row that will be used in new array (edge rows not returned)
                for( int p = 0; p < NPKT; p++ ) {         // for every packet
                   // receive cell values from worker and place into new world array
                   worker[w] :> val[p][y + (w*(IMHT / NWKS))];
                }
            }
        }

        /* SW2 pressed, produce output */

        reqTime <: (uchar) 1; // request timeTaken from timer, also pausing it
        printf("SW2 pressed, asked timer for time and paused it\n");
        reqTime :> timeTaken; // store returned time
        printf("Got timeTaken\n");

        printf("Processing: Outputting after %dms\n", timeTaken);

        LEDs <: 2; // send blue LED to be lit

        // tell DataOutStream to open an output file
        //   this is necessary to prevent the previous file being corrupted
        c_out <: (uchar) 1;

        // Send processed data to DataOutStream
        for(int y = 0; y < IMHT; y++ ) {
            for( int p = 0; p < NPKT; p++ ) { // for every packet
                c_out <: val[p][y];           // send cell information to DataOutStream
            }
        }
        LEDs <: 0; // turn off blue LED

        printf("Outputted\n");

        reqTime <: (uchar) 0; // unpause the timer after output
        b :> button;          // get new button status
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Count how many neighbouring cells are alive
//
/////////////////////////////////////////////////////////////////////////////////////////
int getNeighbours(int x, int y, uchar rowVal[NPKT][IMHT / NWKS + 2]) {
    // variables used in finding neighbours
    int xRight = mod(IMWD, x+1);
    int xLeft = mod(IMWD, x-1);
        // we don't mod the y, because we never call edge cases in worker

    // coordinates for neighbours
    int nCoords[8][2] = {
            {xLeft, y - 1}, {x, y - 1}, {xRight, y - 1},
            {xLeft, y},                 {xRight, y},
            {xLeft, y + 1}, {x, y + 1}, {xRight, y + 1}
    };

    // store states of neighbours in array
    int neighbours[8];

    // e.g. 26th bit is going to be the 3rd bit of the 4th packet
    //      in this case b = 3, p = 4
    int b, p;
    for (int n = 0; n < 8; n++) {   // for each neighbour
        b = mod(8, nCoords[n][0]);      // get index in block of 8 bits
        p = nCoords[n][0] / 8;          // get in which block of 8 bits we're processing

        neighbours[n] = unpack(b, rowVal[p][nCoords[n][1]]); //unpack neighbour value from byte
    }

    // count number of alive neighbours
    int alive = 0;
    for (int i = 0; i < 8; i++) {
        if (neighbours[i] == 1) alive++;
    }
    return alive;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise workers to process given rows of cells
//
/////////////////////////////////////////////////////////////////////////////////////////
void worker(int id, chanend fromFarmer, chanend wLeft, chanend wRight)
{
    // number of rows worker is processing (excluding ghost rows)
    int load = IMHT / NWKS;
    // array of rows worker will work on
    uchar rowVal[NPKT][(IMHT / NWKS) + 2];
    // arrays for storing processed rows
    uchar prevRow[NPKT];
    uchar currRow[NPKT];

    int i = 0;                // iteration counter
    int numAlive = 0;         // number of alive workers at current iteration

    // contruct array to work on
    for( int y = 0; y < (load + 2); y++ ) {     // for every row to be input
        for( int x = 0; x < NPKT; x++ ) {         // for every column
            // read in rows cell by cell from distributer to be worked on
            fromFarmer :> rowVal[x][y];
        }
    }

    uchar s;
    // game runs infinitely
    while (1) {
        fromFarmer :> s; //receive signal from dist
        // default case: run iteration
        if (s == 0) {

            // for sake of testing, get an output on 100th iteration
            if (i == 100) {
                // tell distributor ready to output
                if (id == 0) fromFarmer <: (uchar) 1;
                // receive signal
                fromFarmer :> uchar outSignal;
                for( int y = 1; y < load + 1; y++ ) {               // for every row excluding edge rows
                    for( int p = 0; p < NPKT; p++ ) {           // for each packet
                        fromFarmer <: rowVal[p][y];             // send to farmer (distributer)
                    }
                }
            }

            // TEST: write out packed world

//            if (id == 1) {
//                printf("Before processing %d\n",i);
//                for( int y = 0; y < (IMHT / NWKS) + 2; y++ ) {
//                    for( int p = 0; p < NPKT; p++ ) {
//                        uchar packet = rowVal[p][y];
//
//                        // unpack each bit and add to output line
//                        for ( int x = 0; x < 8; x++ ) {
//                            printf( "-%4.1d ", unpack(x, packet) ); //show image values
//                        }
//                    }
//                    printf( "\n" );
//                }
//            }

            numAlive = 0; //reset number of alive cells counter

            // Look at neighbouring cells and work out the next state of each cell
            for( int y = 1; y < load + 1; y++ ) {              // for every row excluding edge rows
                for( int p = 0; p < NPKT; p++ ) {              // for every packet in the row
                    // packet for new cell values, start with all alive
                    uchar newP = 0xFF;
                    numAlive += 8;

                    for ( int x = 0; x < 8; x++) {             // for every bit in the current packet
                        // get number of alive neighbours
                        int alive = getNeighbours(x + p*8, y, rowVal);

                        // If currently alive
                        if (unpack(x, rowVal[p][y]) == 1) {
                            // If number of alive neighbours isn't two or three, die. Else stay alive by default
                            if (alive != 2 && alive != 3){
                                newP = pack(x, newP, 0x0);
                                numAlive--;
                            }
                        }
                        // Else cell is currently dead: if not exactly three alive neighbours stay dead
                        else if (alive != 3){
                            newP = pack(x, newP, 0x0);
                            numAlive--;
                        }
                    }
                    currRow[p] = newP;
                }

                for( int p = 0; p < NPKT; p++ ) {
                    // if we're at the last row, update it as the y loop is going to stop

                    // if we're on at least the second row we no longer depend on prevRow and can output
                    if (y > 1) {
                        rowVal[p][y - 1] = prevRow[p];                      // update non-overlapping rows
                        if (y == load) rowVal[p][y] = currRow[p];
                    }
                    prevRow[p] = currRow[p];                                // move current processed row to previous row storage for writing next iteration
                }
            }

//            // TEST: write out packed world
//            if (id == 1) {
//                printf("After processing\n\n");
//                for( int y = 0; y < (IMHT / NWKS) + 2; y++ ) {
//                    for( int p = 0; p < NPKT; p++ ) {
//                        uchar packet = rowVal[p][y];
//
//                        // unpack each bit and add to output line
//                        for ( int x = 0; x < 8; x++ ) {
//                            printf( "-%4.1d ", unpack(x, packet) ); //show image values
//                        }
//                    }
//                    printf( "\n" );
//                }
//            }

            /*
            Update ghost row states for next iteration by communicating with other workers
            as well as send ghost row states for other workers to use
            e.g: four workers:
               w0 <--- w1      w2 <--- w3
               w0      w1 ---> w2      w3 --->
               w0 ---> w1      w2 ---> w3
               w0      w1 <--- w2      w3 <---
            */

            if (id % 2 == 1) { // odd numbered workers
                // 1. send to left
                for ( int p = 0; p < NPKT; p++ )
                    wLeft <: rowVal[p][1];
                // 2. send to right
                for ( int p = 0; p < NPKT; p++ )
                    wRight <: rowVal[p][load];
                // 3. receive from left
                for ( int p = 0; p < NPKT; p++ )
                    wLeft :> rowVal[p][0];
                // 4. receive from right
                for ( int p = 0; p < NPKT; p++ )
                    wRight :> rowVal[p][load + 1];
            }
            else {             // even numbered workers
                // 1. receive from right
                for ( int p = 0; p < NPKT; p++ )
                    wRight :> rowVal[p][load + 1];
                // 2. receive from left
                for ( int p = 0; p < NPKT; p++ )
                    wLeft :> rowVal[p][0];
                // 3. send to right
                for ( int p = 0; p < NPKT; p++ )
                    wRight <: rowVal[p][load];
                // 4. send to left
                for ( int p = 0; p < NPKT; p++ )
                    wLeft <: rowVal[p][1];
            }

            //increment iteration counter
            i++;
        }
        // case where board is tilted and status print is required
        else if (s == 1) {
            // send number of currently alive cells
            fromFarmer <: numAlive;
            // if you're the last worker, send the loop iteration
            if (id == (NWKS - 1)) fromFarmer <: i;
            // wait for unpause signal to proceed
            fromFarmer :> uchar unpause;
        }
        // case where SW2 is pressed and game needs to be output
        else if (s == 2) {
            // Send new cell states to farmer for combining
            for( int y = 1; y < load + 1; y++ ) {               // for every row excluding edge rows
                for( int p = 0; p < NPKT; p++ ) {           // for each packet
                    fromFarmer <: rowVal[p][y];             // send to farmer (distributer)
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
    int res;
    uchar line[ IMWD ];

    //Open PGM file
    printf( "DataOutStream: Start...\n" );

    while(1) {
        c_in :> uchar x; // if distributer has a game state for outputting

        res = _openoutpgm( outfname, IMWD, IMHT );
        if( res ) {
            printf( "DataOutStream: Error opening %s\n.", outfname );
            return;
        }
        //Compile each line of the image and write the image line-by-line
        for( int y = 0; y < IMHT; y++ ) {
            for( int p = 0; p < NPKT; p++ ) {
                uchar packet;
                c_in :> packet;

                // unpack each bit and add to output line
                for ( int x = 0; x < 8; x++ ) {
                    int bit = unpack(x, packet);
                    if (bit) line[p*8 + x] = 0xFF;
                    else line[p*8 + x] = 0x00;
                    //printf( "-%4.1d", line[p*8 + x]); //show image values
              }
        }
        //printf( "\n" );
        _writeoutline( line, IMWD );
        //printf( "DataOutStream: Line written...\n" );
        }

        //Close the PGM image
        _closeoutpgm();

        printf( "DataOutStream: Complete\n");
    }

    return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend dist, chanend toTimer) {
    i2c_regop_res_t result;
    char status_data = 0;
    int tilted = 0;

    // Configure FXOS8700EQ
    result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
    if (result != I2C_REGOP_SUCCESS) {
        printf("I2C write reg failed\n");
    }

    // Enable FXOS8700EQ
    result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
    if (result != I2C_REGOP_SUCCESS) {
        printf("I2C write reg failed\n");
    }

    // wait for start signal from distributor
    dist :> uchar x;

    //Probe the orientation x-axis forever
    while (1) {

      //check until new orientation data is available
      do {
          status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
      } while (!status_data & 0x08);

      //get new x-axis tilt value
      int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);
      //pause / unpause distributer and timer when tilted / untilted
      if (!tilted) {
          if (x>30) {
              tilted = 1;
              toTimer <: (uchar) 1;
          }
      }
      else if (x<10) {
          tilted = 0;
          toTimer <: (uchar) 0;
      }
    }
}

// Unit tests
void tests() {
    assert(NPKT == IMWD/8);
    assert(mod(2,5) == 1);
    assert(mod(8, 17) == 1);
    assert(mod(8, -1) == 7);
    assert(mod(8, 0) == 0);
    assert(5 / 2 == 2);                 // checking that division rounds down
    //
    // Testing packing
    //
    assert(pack(3, 0x00, 0xFF) == 0x08);
    assert(pack(7, 0x00, 0xFF) == 0x80);
    assert(pack(0, 0x00, 0xFF) == 0x01);
    assert(pack(0, 0xFF, 0x00) == 0xFE);
    assert(pack(0, 0x00, 0x00) == 0x00);
    assert(pack(7, 0xFF, 0x00) == 0x7F);
    assert(pack(0, 0x01, 0x00) == 0x00);
    //
    // Testing unpacking with 11100110
    //
    uchar testVal = 0xE6;
    assert(unpack(0, testVal) == 0);
    assert(unpack(1, testVal) == 1);
    assert(unpack(2, testVal) == 1);
    assert(unpack(3, testVal) == 0);
    assert(unpack(4, testVal) == 0);
    assert(unpack(5, testVal) == 1);
    assert(unpack(6, testVal) == 1);
    assert(unpack(7, testVal) == 1);
    //
    // Testing getNeighbours with every packet containing 11100110
    //   note: getNeighbours is designed not to take edge rows
    uchar testState[NPKT][IMHT / NWKS + 2];
    for (int y = 0; y < IMHT / NWKS + 2; y++) {
        for (int p = 0; p < NPKT; p++) {
            testState[p][y] = testVal;
        }
    }
    assert(getNeighbours(3, 1, testState) == 3);
    assert(getNeighbours(11, 1, testState) == 3);

    printf("Tests successful\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {
    i2c_master_if i2c[1];                          // interface to orientation sensor

    // Channel definitions
    chan c_inIO,     // DataStreamIn
         c_outIO,    // DataStreamOut
         c_control,  // communication between orientation and distributor
         WtoD[NWKS], // Workers to Distributer
         WtoW[NWKS], // Workers to Workers
         reqTime,    // request time
         pauseTime;  // pause time

    par {
        on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);                      //server thread providing orientation data
        on tile[0] : orientation(i2c[0], c_control, pauseTime);                 //client thread reading orientation data
        on tile[0] : DataInStream("16x16.pgm", c_inIO);                       //thread to read in a PGM image
        on tile[0] : DataOutStream("16x16out.pgm", c_outIO);                  //thread to write out a PGM image
        on tile[0] : distributor(c_inIO, c_outIO, c_control, buttons, leds, WtoD, reqTime);    //thread to coordinate work on image
        on tile[0] : checkTime(reqTime, pauseTime);
        //initialise workers
        par (int i = 0; i < (NWKS) ; i++){
            on tile[1] : worker((i+1) % NWKS, WtoD[(i+1) % NWKS], WtoW[i], WtoW[(i+1) % NWKS]);
        }
//        par (int i = (NWKS - 1); i < NWKS ; i++){
//            on tile[0] : worker((i+1) % NWKS, WtoD[(i+1) % NWKS], WtoW[i], WtoW[(i+1) % NWKS]);
//        }
        // channels between workers: e.g. 4 workers
        //
        //    <--- w0 <-----> w1 <-----> w2 <-----> w3 --->
        //  WtoW[3]   WtoW[0]    WtoW[1]    WtoW[2]    WtoW[3]
    }

    return 0;
}
