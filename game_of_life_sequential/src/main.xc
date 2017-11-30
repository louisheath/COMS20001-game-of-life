// Sequential

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <assert.h>

#define  IMHT 64                  //image height
#define  IMWD 64                  //image width
#define  NPKT IMWD/8              //number of packets in a row

#define  printAt 2

typedef unsigned char uchar;      //using uchar as shorthand

port p_scl = XS1_PORT_1E;         //interface ports to orientation
port p_sda = XS1_PORT_1F;

in port buttons = XS1_PORT_4E; //port to access xCore-200 buttons
out port leds = XS1_PORT_4F;   //port to access xCore-200 LEDs
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
    return x % n;
}

uchar pack(int index, uchar byte, uchar c) {    // pack c into packet 'byte' at index 'index'
    if (c == 0x00) {
        byte &= ~(1 << index);
    }
    else if (c == 0xFF) {
        byte |= (1 << index);
    }
    return byte;
}

int unpack(int index, uchar byte) {             // unpack bit from index 'index' of packet 'byte'
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
    printf( "DataInStream: Start...\n" );

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
      for ( int p = 0; p < NPKT; p++) {           // for every packet we can fit in the row
          for ( int x = 0; x < 8; x++) {          // for each of the eight bits to be packed
              packet = pack(x, packet, line[x + p*8]);
              //printf( "-%4.1d ", line[p*8 + x] ); //show image values
          }
          c_out <: packet;
          packet = 0x00;
      }
      //printf("\n")
    }

    //Close PGM image file
    _closeinpgm();

    printf("DataInStream: Complete\n");
    return;
}

/*
 * We need to ask Tilo if this array parameter is copying the array every time, and if so how to point
 */
int getNeighbours(int x, int y, uchar rowVal[NPKT][IMHT]) {
    // counter for number of alive neighbours
    int alive = 0;
    int b, p; // an x coordinate is the bth bit in the pth packet

    for (int i = -1; i < 2; i++) {
        int xN = mod(IMWD, x + i);  // x coordinate of each neighbour cell

        for (int j = -1; j < 2; j++) {
            int yN = mod(IMHT, y + j);

            if (j != 0 || i != 0) { // when j == i == 0, x == xN and y == yN. avoid.
                b = mod(8, xN);      // get index in block of 8 bits
                p = xN / 8;          // get in which block of 8 bits we're processing

                if (unpack(b, rowVal[p][yN]) == 1) alive++;; //unpack neighbour value and find out if it's alive
            }
        }
    }

    return alive;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend tilt, chanend time, in port b, out port LEDs)
{
    // run unit tests
    tests();

    // game state
    uchar val[NPKT][IMHT];

    // arrays for storing processed rows
    uchar prevRow[NPKT];
    uchar currRow[NPKT];

    // variables for hardware control
    int button = 0;           // button input from board/button listener
    uchar flashState = 0;     // state of flashing seperate green LED

    // variables for pausing
    int tilted = 0;           // tilt state
    int output = 0;           // whether game should be output
    uint32_t timeTaken;       // time taken to process image
    int numAlive = 0;         // number of alive cells at pause
    int i = 0;                // current iteration

    // prompt user for start button
    printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
    printf( "Waiting for start button...\n" );


    // wait for SW1 button input
    while (button != 14) {
        b :> button;
    }

    LEDs <: 4; // send green LED to be lit

    printf( "Processing...\n" );

    // Read input from DataInStream
    for(int y = 0; y < IMHT; y++ ) {
        for( int p = 0; p < NPKT; p++ ) {
            c_in :> val[p][y];
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

                    // reset number of alive cells counter
                    numAlive = 0;

                    // Look at neighbouring cells and work out the next state of each cell
                    for( int y = 0; y < IMHT; y++ ) {              // for every row
                        for( int p = 0; p < NPKT; p++ ) {          //   and the packets in them
                            // packet for new cell values, start with all alive
                            uchar newP = 0xFF;
                            numAlive += 8;

                            for ( int x = 0; x < 8; x++) {             // for every bit in the current packet
                                // get number of alive neighbours
                                int alive = getNeighbours(x + p*8, y, val);

                                // If currently alive
                                if (unpack(x, val[p][y]) == 1) {
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
                                val[p][y - 1] = prevRow[p];                      // update non-overlapping rows
                                // if we're at the last row, update it as the y loop is going to stop
                                if (y == IMHT) val[p][y] = currRow[p];
                            }
                            prevRow[p] = currRow[p];                                // move current processed row to previous row storage for writing next iteration
                        }
                    }

                    i++;
                    if (i == 2 || i == 100) {
                        output = 1;
                        break;
                    }

                    // flash green LED
                    flashState ^= 1;
                    LEDs <: flashState;

                    break;
            }
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

            printf("Status Report:\n"
                   " Number of rounds processed: %d\n"
                   " Current number of live cells: %d\n"
                   " Processing time elapsed: %dms\n", i, numAlive, timeTaken);

            // wait until we're untilted
            tilt :> int x;
            tilted = 0;
        }
        else {
            // tell DataOutStream to open an output file
            //   this is necessary to prevent the previous file being corrupted
            c_out <: (uchar) 1;
            // Send processed data to DataOutStream
            for(int y = 0; y < IMHT; y++ ) {
                for( int p = 0; p < NPKT; p++ ) { // for every packet
                    c_out <: val[p][y];           // send cell information to DataOutStream
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
        select {
            case c_in :> uchar x: // if distributer has a game state for outputting
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
                //printf( "DataOutStream: Line %d written...\n", y);
                }

                //Close the PGM image
                _closeoutpgm();

                printf( "DataOutStream: Complete\n");
                break;
        }
    }

    return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
// Orientation Sensor: Use the physical X-axis tilt of the board to trigger processing to be paused,
// and continued once the board is horizontal again, indicate a pausing state by lighting the red LED,
// print a status report when pausing starts to the console containing the number of rounds processed so far,
// the current number of live cells and the processing time elapsed after finishing image read-in
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
    uchar testState[NPKT][IMHT];
    for (int y = 0; y < IMHT; y++) {
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
         time;       // request time

    par {
        i2c_master(i2c, 1, p_scl, p_sda, 10);                                  //server thread providing orientation data
        orientation(i2c[0], tilt);                             //client thread reading orientation data
        DataInStream("64x64.pgm", inIO);                                     //thread to read in a PGM image
        DataOutStream("64x64out.pgm", outIO);                                //thread to write out a PGM image
        distributor(inIO, outIO, tilt, time, buttons, leds); // farmer
        checkTime(time);
    }

    return 0;
}
