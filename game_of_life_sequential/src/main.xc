// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width

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

//mod by n
int modn(int n, int x){
  while (x < 0 || x > (n - 1)){
      if (x < 0) x += n;
      else x -= n;
  }
  return x;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc)
{
  uchar val[IMWD][IMHT]; //To store snapshot of current image
  uchar newVal[IMWD][IMHT]; //To store new snapshot of image

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
  fromAcc :> int value;

  //Read in and do something with your image values..
  //This just inverts every pixel, but you should
  //change the image according to the "Game of Life"
  printf( "Processing...\n" );

  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
    for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
      c_in :> val[x][y];                   //read the pixel value
    }
  }

  while(1) {
      for( int y = 0; y < IMHT; y++ ) {   //go through all lines
         for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            //store neighbours in 1D array
            int xRight = mod(IMWD, x+1);
            int xLeft = mod(IMWD, x-1);
            int yUp = mod(IMHT, y-1);
            int yDown = mod(IMHT, y+1);

            uchar neighbours[8] = {(val[xLeft][yUp]), (val[x][yUp]), (val[xRight][yUp]),
                                   (val[xLeft][y]), (val[xRight][y]),
                                   (val[xLeft][yDown]), (val[x][yDown]), (val[xRight][yDown])};
            int alive = 0;
            for (int i = 0; i < 8; i++) {
                if (neighbours[i] == 0xFF) {
                    alive++;
                }
            }
            // If currently alive
            if (val[x][y] == 0xFF) {
                // If number of alive neighbours isn't two or three, die. Else stay alive by default
                if (alive != 2 && alive != 3) {
                    newVal[x][y] = 0x0;
                }
                else
                {
                    newVal[x][y] = 0xFF;
                }
            }
            // Else cell is currently dead: if three alive neighbours resurrect
            else {
                if (alive == 3) {
                    newVal[x][y] = 0xFF;
                }
                else newVal[x][y] = 0x0;
            }
         }
       }

      for( int y = 0; y < IMHT; y++ ) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            c_out <: newVal[x][y]; //send some modified pixel out
          }
      }

      for( int y = 0; y < IMHT; y++ ) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
            val[x][y] = newVal[x][y];                   //transfer new pixels to old array
          }
      }
      //printf( "\nOne processing round completed...\n" );
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
  int x = 0;

  //Open PGM file
  printf( "DataOutStream: Start...\n" );
  res = _openoutpgm( outfname, IMWD, IMHT );
  if( res ) {
    printf( "DataOutStream: Error opening %s\n.", outfname );
    return;
  }
  while (x < 100){
      //Compile each line of the image and write the image line-by-line
      if (x == 99) {
          for( int y = 0; y < IMHT; y++ ) {
            for( int x = 0; x < IMWD; x++ ) {
              c_in :> line[ x ];
              printf( "-%4.1d ", line[ x ] ); //show image values
            }
            printf( "\n" );
            _writeoutline( line, IMWD );
            //printf( "DataOutStream: Line written...\n" );
          }
          x++;
      }
      else{
          for( int y = 0; y < IMHT; y++ ) {
              for( int x = 0; x < IMWD; x++ ) {
                c_in :> line[ x ];
              }
              _writeoutline( line, IMWD );
              //printf( "DataOutStream: Line written...\n" );
          }
          x++;
      }
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

i2c_master_if i2c[1];               //interface to orientation

char infname[] = "test.pgm";     //put your input image path here
char outfname[] = "testout.pgm"; //put your output image path here
chan c_inIO, c_outIO, c_control;    //extend your channel definitions here

par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    orientation(i2c[0],c_control);        //client thread reading orientation data
    DataInStream(infname, c_inIO);          //thread to read in a PGM image
    DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control);//thread to coordinate work on image
}

return 0;
}
