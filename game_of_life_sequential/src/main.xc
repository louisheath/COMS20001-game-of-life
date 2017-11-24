// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 64                  //image height
#define  IMWD 64                  //image width
#define  NPKT IMWD/8              //number of packets in a row

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

// modulo function that works with negatives
int mod(int n,int x) {
  while (x < 0  || x > (n-1)) {
    if (x < 0) x += n;
    else x -= n;
  }
  return x;
}

uchar pack(int index, uchar byte, uchar c) {    // pack c into packet 'byte' at index 'index'
    if (c == 0x0) {
        byte = byte & ~(1 << index);
        index++;
    }
    else if (c == 0xFF) {
        byte = byte | (1 << index);
        index++;
    }
    return byte;
}

int unpack(int index, uchar byte) {             // unpack bit from index 'index' of packet 'byte'
    return (byte >> index) & 1;
}

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
                case req :> uchar x: // check to see if distributer wants the time
                    req <: timeTaken;
                    //printf("Timer terminated\n");
                    return;
                    break;
                case pause :> uchar x: // check to see if board is tilted and timing should pause.
                    paused = 1;
                    req <: timeTaken;
                    printf("Timer paused\n");
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
            case pause :> uchar x:
            paused = 0;
            t :> prev;
            req <: (uchar) 0;
            printf("Timer unpaused\n");
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
      for ( int p = 0; p < NPKT; p++) {         // for every packet we can fit in the row
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

//mod by n
int modn(int n, int x){
  while (x < 0 || x > (n - 1)){
      if (x < 0) x += n;
      else x -= n;
  }
  return x;
}

int getNeighbours(int x, int y, uchar rowVal[NPKT][IMHT]) {
    // variables used in finding neighbours
    int xRight  = mod(IMWD, x+1);
    int xLeft   = mod(IMWD, x-1);
    int yUp     = mod(IMHT, y-1);
    int yDown   = mod(IMHT, y+1);

    // coordinates for neighbours
    int nCoords[8][2] = {
            {xLeft, yUp},   {x, yUp},   {xRight, yUp},
            {xLeft, y},                 {xRight, y},
            {xLeft, yDown}, {x, yDown}, {xRight, yDown}
    };

    // store states of neighbours in array
    int neighbours[8];

    // e.g. 26th bit is going to be the 3rd bit of the 4th packet
    //      in this case b = 3, p = 4
    int b, p;
    for (int n = 0; n < 8; n++) {   // for each neighbour
        b = mod(8, nCoords[n][0]);
        p = nCoords[n][0] / 8;
        neighbours[n] = unpack(b, rowVal[p][nCoords[n][1]]);
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
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend c_control, in port b, out port LEDs, chanend reqTime) {

  uint32_t timeTaken;       // time taken to process image
  int button = 0;           // button input from board/button listener
  uchar flashState = 0;     // state of flashing seperate green LED

  uchar val[NPKT][IMHT];    //To store snapshot of current image
  uchar newVal[NPKT][IMHT]; //To store new snapshot of image
  int i = 0;                //Counter for the iterations
  int numAlive = 0;         // number of alive workers at current iteration

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for start button...\n" );

  // wait for SW1 button input
  while (button != 14) {
      b :> button;
  }

  LEDs <: 4; // send green LED to be lit

  printf( "Processing...\n" );

  for(int y = 0; y < IMHT; y++ ) {       // Read every packet into world array
      for( int p = 0; p < NPKT; p++ ) {
         c_in :> val[p][y];
      }
  }

  reqTime <: (uchar) 1; //trigger timer funtion

  // loop whilst SW2 hasn't been pressed
  while (button != 13) {
      select {
          case reqTime :> uint32_t timeTaken:
                LEDs <: (uchar) 8;
                printf("Status Report:\n "
                        "Number of rounds processed: %d\n "
                        "Current number of live cells: %d\n "
                        "Processing time elapsed: %d\n", i, numAlive, timeTaken);
                // wait until unpause
                reqTime :> uchar resume;
                break;
          default:
                // read button input, if any
                b :> button;

                // reset alive cell counter
                numAlive = 0;

                // flash processing LED via state change
                flashState = (1 ^ flashState);
                LEDs <: flashState;

                for( int y = 0; y < IMHT; y++ ) {   //go through all lines
                   for( int p = 0; p < NPKT; p++ ) { //go through each packet per line
                       // packet for new cell values, start with all alive
                       uchar newP = 0xFF;
                       numAlive = numAlive + 8;
                       for ( int x = 0; x < 8; x++ ) { // go through each pixel in the packet

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
                       newVal[p][y] = newP;
                   }
                 }

                for( int y = 0; y < IMHT; y++ ) {   //go through all lines
                    for( int p = 0; p < NPKT; p++ ) { //go through each pixel per line
                      val[p][y] = newVal[p][y];                   //transfer new pixels to old array
                    }
                }
                //printf( "\nOne processing round completed...\n" );

                //Increment iteration
                i++;
                break;
            }
      }

      reqTime <: (uchar) 0; // inform timer that it wants a return for time
      reqTime :> timeTaken; // store returned time

      printf("Processing: Complete in %dms\n", timeTaken);

      LEDs <: 2; // send blue LED to be lit

      // Send processed data to DataOutStream
      for(int y = 0; y < IMHT; y++ ) {
          for( int p = 0; p < NPKT; p++ ) { // for every packet
              c_out <: newVal[p][y];           // send cell information to DataOutStream
          }
      }
      LEDs <: 0; // send blue LED to be lit
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
          line[p*8 + x] = unpack(x, packet);
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
void orientation( client interface i2c_master_if i2c, chanend toDist, chanend toTimer) {
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
            //toDist <: 1;
            toTimer <: (uchar) 1;
        }
    }
    else if (x<10) {
        tilted = 0;
        //toDist <: 0;
        toTimer <: (uchar) 0;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    i2c_master_if i2c[1];              //interface to orientation

    char infname[] = "64x64.pgm";     //put your input image path here
    char outfname[] = "64x64out.pgm"; //put your output image path here
    chan c_inIO,        // DataStreamIn
         c_outIO,       // DataStreamOut
         c_control,     // Orientation sensor
         reqTime,       // Request time
         pauseTime;     // Orientation to Timer pause signal

    par {
        i2c_master(i2c, 1, p_scl, p_sda, 10);               //server thread providing orientation data
        orientation(i2c[0],c_control, pauseTime);           //client thread reading orientation data
        DataInStream(infname, c_inIO);                      //thread to read in a PGM image
        DataOutStream(outfname, c_outIO);                   //thread to write out a PGM image
        distributor(c_inIO, c_outIO, c_control, buttons, leds, reqTime);   //thread to coordinate work on image
        checkTime(reqTime, pauseTime);
    }

    return 0;
}
