// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width
#define  NWKS 4                   //number of workers

typedef unsigned char uchar;      //using uchar as shorthand

port p_scl = XS1_PORT_1E;         //interface ports to orientation
port p_sda = XS1_PORT_1F;

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
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
      printf( "-%4.1d ", line[ x ] ); //show image values
    }
    printf( "\n" );
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend worker[4])
{
   uchar val[IMWD][IMHT]; //To store snapshot of current image
   uchar updVal[IMWD][IMHT]; //To store snapshot of current image

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
  fromAcc :> int value;

  printf( "Processing...\n" );

  for(int y = 0; y < IMHT; y++ ) {   //go through all lines
      for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
          c_in :> val[x][y];                   //read the pixel value
      }
  }

  //divide work between workers
  for(int i = 0; i < NWKS; i++) {
      //printf("worker %d\n", i);
      for(int y = 0; y < ((IMHT / NWKS) + 2); y++ ) {   //go through all lines
            for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                //send over row of cells to new 2D array to be sent to workers
                //consider the two extra rows below and above worked set of rows so that all rules can be followed

                worker[i] <: val[x][mod(IMHT, (y + (i*(IMHT / NWKS)) - 1))];
            }
      }
  }

  for(int i = 0; i < NWKS; i++) {
      for(int y = 0; y < ((IMHT / NWKS)); y++ ) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line

              worker[i] :> updVal[x][y + (i*(IMHT / NWKS))];
          }
      }
  }

  for(int y = 0; y < IMHT; y++ ) {   //go through all lines
        for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            c_out<: updVal[x][y];
        }
  }

  printf( "\nOne processing round completed...\n" );
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise workers to work on a row of cells
//
/////////////////////////////////////////////////////////////////////////////////////////
void worker(chanend fromFarmer) {

    //store value of rows worker will work on
    uchar rowVal[IMWD][(IMHT / NWKS) + 2];
    uchar newVal[IMWD][(IMHT / NWKS)];

    for(int y = 0; y < ((IMHT / NWKS) + 2); y++ ) {   //go through all lines
        for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            //copy over row of cells to new 2D array to be worked
            fromFarmer :> rowVal[x][y];
            //printf( "-%4.1d ", rowVal[x][y] ); //show image values
        }
        //printf( "\n");
    }

    for( int y = 1; y < (IMHT / NWKS) + 1; y++ ) {   //go through all lines
        for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            //store neighbours in 1D array
            int xRight = mod(IMWD, x+1);
            int xLeft = mod(IMWD, x-1);
            int yUp = y-1;
            int yDown = y+1;

            uchar neighbours[8] = {(rowVal[xLeft][yUp]), (rowVal[x][yUp]), (rowVal[xRight][yUp]),
                                   (rowVal[xLeft][y]), (rowVal[xRight][y]),
                                   (rowVal[xLeft][yDown]), (rowVal[x][yDown]), (rowVal[xRight][yDown])};
            int alive = 0;
            for (int i = 0; i < 8; i++) {
                if (neighbours[i] == 0xFF) {
                    alive++;
                }
            }
            // If currently alive
            if (rowVal[x][y] == 0xFF) {
                // If number of alive neighbours isn't two or three, die. Else stay alive by default
                if (alive != 2 && alive != 3) {
                    newVal[x][y - 1] = 0x0;
                }
                else {
                    newVal[x][y - 1] = 0xFF;
                }
            }
            // Else cell is currently dead: if three alive neighbours resurrect
            else {
                if (alive == 3) {
                    newVal[x][y - 1] = 0xFF;
                }
                else newVal[x][y - 1] = 0x0;
            }
        }
    }

    for( int y = 0; y < (IMHT / NWKS); y++ ) {   //go through all lines
        for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            fromFarmer <: newVal[x][y]; //send some modified pixel out
            //printf( "-%4.1d ", newVal[x][y] ); //show image values
        }
        //printf( "\n"); //show image values
    }
    /*
    for( int y = 0; y < (IMHT / NWKS) + 2; y++ ) {   //go through all lines
        for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            rowVal[x][y] = rowVal[x][y];                   //transfer new pixels to old array
        }
    }*/
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
    for( int x = 0; x < IMWD; x++ ) {
      c_in :> line[ x ];
      printf( "-%4.1d ", line[ x ] ); //show image values
    }
    printf( "\n" );
    _writeoutline( line, IMWD );
    //printf( "DataOutStream: Line written...\n" );
  }

  //Close the PGM image
  _closeoutpgm();
  printf( "DataOutStream: Done...\n" );
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist) {
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

    //send signal to distributor after first tilt
    if (!tilted) {
      if (x>30) {
        tilted = 1 - tilted;
        toDist <: 1;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    /* SKELETON main():

    i2c_master_if i2c[1];               //interface to orientation

    char infname[] = "test.pgm";     //put your input image path here
    char outfname[] = "testout.pgm"; //put your output image path here
    chan c_inIO, c_outIO, c_control;    //extend your channel definitions here

    par {
        i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
        orientation(i2c[0],c_control);         //client thread reading orientation data
        DataInStream(infname, c_inIO);          //thread to read in a PGM image
        DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
        distributor(c_inIO, c_outIO, c_control);//thread to coordinate work on image
    }
    */

  i2c_master_if i2c[1];                          // interface to orientation sensor

  char infname[] = "test.pgm";                   // input image path
  char outfname[] = "testout.pgm";               // output image path
  chan c_inIO, c_outIO, c_control, workers[4];   // channel definitions

  par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);            //server thread providing orientation data
    orientation(i2c[0],c_control);                   //client thread reading orientation data
    DataInStream(infname, c_inIO);                   //thread to read in a PGM image
    DataOutStream(outfname, c_outIO);                //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control, workers);//thread to coordinate work on image
    //intialise 4 workers
    worker(workers[0]);
    worker(workers[1]);
    worker(workers[2]);
    worker(workers[3]);
  }

  return 0;
}
