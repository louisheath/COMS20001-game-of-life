// COMS20001 - Game of Life: Jack Jones and Louis Heath
// (using the XMOS i2c accelerometer)
// Final version: Eight workers

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <assert.h>

#define  IMHT 512                  //image height
#define  IMWD 512                  //image width
#define  NWKS 8                   //number of workers
#define  NPKT IMWD/8              //number of packets in a row

#define  printAt 100

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

// modulo function that works with negatives, returns x mod n
int mod(int n,int x) {
    while (x < 0) x += n;
    return x % n;
}

// pack c into packet 'byte' at index 'index'
uchar pack(int index, uchar byte, uchar c) {
    if (c == 0x00) {
        byte &= ~(1 << index);
    }
    else if (c == 0xFF) {
        byte |= (1 << index);
    }
    return byte;
}

// unpack bit from index 'index' of packet 'byte'
int unpack(int index, uchar byte) {
    return (byte >> index) & 1;
}

// function that constantly tracks processing time and checks for clock overflow
void checkTime(chanend dstr) {
    // timer variables. timer ticks at 100MHz, 100000 ticks is 1ms.
    timer t;
    uint32_t prev,
             curr;
    uint32_t timeTaken = 0;     // total time in milliseconds

    // variable for identifying whether game is paused
    int paused = 0;

    // receive signal to start timing
    dstr :> int x;
    t :> prev;

    while (1) {
        while (!paused){
            [[ordered]]
            select {
                // if distributor says pause
                case dstr :> int pause:
                    dstr <: timeTaken;
                    paused = 1;     // pause while distributer outputs
                    break;
                // else keep checking for overflow
                default:
                    t :> curr;

                    // if overflow is hit, curr is 42950 too small. Compensate.
                    if (prev > curr) timeTaken += 42950;
                    // increment change in time
                    timeTaken += curr / 100000;
                    timeTaken -= prev / 100000;

                    prev = curr;
                    break;
            }
        }

        // wait until distributer says to resume
        dstr :> int resume;
        paused = 0;
        t :> prev;
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
//             manages hardware,
//             receives new cells from workers and builds new world,
//             and sends new world to DataStreamOut
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend tilt, chanend worker[NWKS], chanend time, in port b, out port LEDs)
{
    // run unit tests
    tests();

    // game state
    uchar val[NPKT][IMHT];

    // variables for hardware control
    int button = 0;           // button input from board/button listener
    uchar flashState = 0;     // state of flashing seperate green LED

    // variables for pausing
    int tilted = 0;           // tilt state
    int output = 0;           // whether game should be output
    uint32_t timeTaken;       // time taken to process image
    int numAlive = 0;         // number of alive cells at pause
    int i = 0;                // current iteration

    //Starting up and wait for tilting of the xCore-200 Explorer
    printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
    printf( "Waiting for button press...\n" );

    // wait for SW1 button input
    while (button != 14) {
        b :> button;
    }

    // indicate start with green LED
    LEDs <: 4;
    printf( "Processing...\n" );

    // Read input from DataInStream
    for(int y = 0; y < IMHT; y++ ) {
        for( int p = 0; p < NPKT; p++ ) {
            c_in :> val[p][y];
        }
    }

    // Divide work between workers
    for(int w = 0; w < NWKS; w++) {
        for(int y = 0; y < ((IMHT / NWKS) + 2); y++ ) {     // for each row in a worker's portion
            int row = y + w*(IMHT / NWKS) - 1;              //      and the packets in it
            for( int p = 0; p < NPKT; p++ ) {
                // send cell values to the worker, who will combine them into a new array
                worker[w] <: val[p][mod(IMHT, row)];
            }
        }
    }

    // Start detecting tilts
    tilt <: 1;
    // Start timing
    time <: 1;

    // game runs infinitely
    while (1) {
        // while not paused, keep workers working and check for pauses
        while (!tilted && !output) {
            select {
                case tilt :> int x:
                    tilted = 1;
                    printf("tilted\n");

                    break;
                default:
                    // check for SW2
                    b :> button;
                    if (button == 13) {
                        output = 1;
                        break;
                    }

                    // start next iteration
                    for(int w = 0; w < NWKS; w++) {
                        worker[w] <: 0; // zero indicates work
                    }
                    i++;
                    if (i == printAt) {
                        output = 1;
                        break;
                    }

                    // flash green LED
                    flashState ^= 1;
                    LEDs <: flashState;

                    break;
            }
        }

        /* game paused, either produce output or print update */

        // send signal to worker for pausing, indicating the form of output
        int signal;
        if (tilted) signal = 1;     // 1 returns numAlive
        else        signal = 2;     // 2 returns game state
        for (int w = 0; w < NWKS; w++) {
            worker[w] <: signal;
        }

        // tell timer to pause
        time <: 1;
        time :> timeTaken;
        printf("Paused at %dms\n", timeTaken);

        // update LEDs
        if (tilted) LEDs <: 8;  // red
        else        LEDs <: 2;  // blue

        // run logic for printing / outputting
        if (tilted) {
            // receive number of alive cells from each worker
            int temp;
            for (int w = 0; w < NWKS; w++) {
                worker[w] :> temp;
                numAlive += temp;
            }

            printf("Status Report:\n"
                   " Number of rounds processed: %d\n"
                   " Current number of live cells: %d\n"
                   " Processing time elapsed: %dms\n", i, numAlive, timeTaken);

            // wait until we're untilted
            tilt :> int x;
            tilted = 0;

            numAlive = 0;
        }
        else {
            // tell DataOutStream to open an output file
            //   this is necessary to prevent the previous file being corrupted
            c_out <: (uchar) 1;

            // Receive processed cells from workers
            for(int w = 0; w < NWKS; w++) {                   // for each of the workers
                for(int y = 0; y < ((IMHT / NWKS)); y++ ) {   // for each row that will be used in new array (ghost rows not returned)
                    int row = y + (w*(IMHT / NWKS));
                    for( int p = 0; p < NPKT; p++ ) {         // for every packet
                       // receive cell values from worker and place into new world array
                       worker[w] :> val[p][row];
                       // send the packet to DataOut
                       c_out <: val[p][row];
                    }
                }
            }

            output = 0;
        }

        // resume timer
        time <: 1;

        // prepare for next iteration
        LEDs <: 0;
        b :> button;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Count how many neighbouring cells are alive
//
/////////////////////////////////////////////////////////////////////////////////////////
int getNeighbours(int x, int y, uchar rowVal[NPKT][IMHT / NWKS + 2]) {
    // counter for number of alive neighbours
    int alive = 0;
    int b, p; // an x coordinate is the bth bit in the pth packet

    for (int i = -1; i < 2; i++) {
        int xN = mod(IMWD, x + i);  // x coordinate of each neighbour cell

        for (int j = -1; j < 2; j++) {
            int yN = y + j;

            if (j != 0 || i != 0) { // when j == i == 0, x == xN and y == yN. avoid.
                b = mod(8, xN);      // get index in block of 8 bits
                p = xN / 8;          // get in which block of 8 bits we're processing

                if (unpack(b, rowVal[p][yN]) == 1) alive++;; //unpack neighbour value and find out if it's alive
            }
        }
    }

    return alive;
}

// function to send ghost rows between worker
void rowSender(int id, chanend wLeft, chanend wRight, uchar rowVal[NPKT][IMHT / NWKS + 2]){
    // number of rows worker is processing (excluding ghost rows)
    int load = IMHT / NWKS;

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
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise workers to process given rows of cells
//
/////////////////////////////////////////////////////////////////////////////////////////
void worker(int id, chanend dstr, chanend wLeft, chanend wRight)
{
    // number of rows worker is processing (excluding ghost rows)
    int load = IMHT / NWKS;
    // array of rows worker will work on
    uchar rowVal[NPKT][(IMHT / NWKS) + 2];
    // arrays for storing processed rows
    uchar prevRow[NPKT];
    uchar currRow[NPKT];

    int numAlive = 0;         // number of alive workers at current iteration

    // contruct array to work on
    for( int y = 0; y < (load + 2); y++ ) {     // for every row to be input
        for( int x = 0; x < NPKT; x++ ) {       // for every column
            // read in rows cell by cell from distributer to be worked on
            dstr :> rowVal[x][y];
        }
    }

    int s;
    // game runs infinitely
    while (1) {
        // receive signal from dist
        dstr :> s;
        // default case: run iteration
        if (s == 0) {
            // reset number of alive cells counter
            numAlive = 0;

            // Look at neighbouring cells and work out the next state of each cell
            for( int y = 1; y < load + 1; y++ ) {              // for every row excluding ghost rows
                for( int p = 0; p < NPKT; p++ ) {              //   and the packets in them
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
                    // if we're on at least the second row we no longer depend on prevRow and can output
                    if (y > 1) {
                        rowVal[p][y - 1] = prevRow[p];                      // update non-overlapping rows
                        // if we're at the last row, update it as the y loop is going to stop
                        if (y == load) rowVal[p][y] = currRow[p];
                    }
                    prevRow[p] = currRow[p];                                // move current processed row to previous row storage for writing next iteration
                }
            }

            // update adjacent workers' ghost rows as well as own
            rowSender(id, wLeft, wRight, rowVal);
        }
        // case where board is tilted and status print is required
        else if (s == 1) {
            // send number of currently alive cells
            dstr <: numAlive;
        }
        // case where SW2 is pressed and game info needs to be output
        else if (s == 2) {
            // Send new cell states to farmer for combining
            for( int y = 1; y < load + 1; y++ ) {               // for every row excluding ghost rows
                for( int p = 0; p < NPKT; p++ ) {           // for each packet
                    dstr <: rowVal[p][y];             // send to farmer (distributor)
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
void orientation( client interface i2c_master_if i2c, chanend dstr) {
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
    dstr :> int x;

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
              dstr <: 1;
          }
      }
      else if (x<10) {
          tilted = 0;
          dstr <: 0;
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
    //   note: getNeighbours is designed not to take ghost rows
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
    // Interfaces
    i2c_master_if i2c[1];                          // interface to orientation sensor

    // Channel definitions
    chan inIO,       // DataStreamIn
         outIO,      // DataStreamOut
         tilt,       // communication between orientation and distributor
         WtoD[NWKS], // Workers to Distributer
         WtoW[NWKS], // Workers to Workers
         time;       // request time

    par {
        on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);                                  //server thread providing orientation data
        on tile[0] : orientation(i2c[0], tilt);                             //client thread reading orientation data
        on tile[0] : DataInStream("512x512.pgm", inIO);                                     //thread to read in a PGM image
        on tile[0] : DataOutStream("512x512out.pgm", outIO);                                //thread to write out a PGM image
        on tile[0] : distributor(inIO, outIO, tilt, WtoD, time, buttons, leds); // farmer
        on tile[0] : checkTime(time);
        // initialise workers
        par (int i = 0; i < (NWKS) ; i++){
            on tile[1] : worker((i+1) % NWKS, WtoD[(i+1) % NWKS], WtoW[i], WtoW[(i+1) % NWKS]);
        }
        // channels between workers: e.g. 4 workers
        //
        //    <--- w0 <-----> w1 <-----> w2 <-----> w3 --->
        //  WtoW[3]   WtoW[0]    WtoW[1]    WtoW[2]    WtoW[3]
    }

    return 0;
}
