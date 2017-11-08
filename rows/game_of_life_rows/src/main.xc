// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width
#define  numW 4

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

void worker(chanend f) {
}

int inDivision(int x, int div) {           // is row x in the given division of the world?
    if (div <= numW && div > 0) {
        int divHeight = IMHT / numW;       // TODO: extend this so it catches floats
        int inDiv = 0;

        int lower = mod(IMHT, ((div - 1) * divHeight) - 1);
        int upper = mod(IMHT, div * divHeight);

        if (x >= lower && x <=upper) inDiv = 1;

        return inDiv;
    }
    else fprintf(stderr, "Inputted division %d not in range 1-%d\n", div, numW);
}

// Farmer
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend workers[4])
{
  // world array
  uchar w[IMWD][IMHT];

  // Start up. DONT wait for tilt because I don't have the board
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
  //fromAcc :> int value;

  // Read in and populate world array
  printf( "Processing...\n" );
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
    for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
      c_in :> w[x][y];                    //read the pixel value
      c_out <: (uchar)( w[x][y] ^ 0xFF ); //send some modified pixel out
    }
  }

  // divide array for each worker, each getting four rows and the rows above and below
  uchar w0[IMWD][IMHT/numW +2],
        w1[IMWD][IMHT/numW +2],
        w2[IMWD][IMHT/numW +2],
        w3[IMWD][IMHT/numW +2];
  for( int y = 0; y < IMHT; y++ ) {       // go through all rows
    if (inDivision(y, 1)) {               // if row y is in division 1
      for( int x = 0; x < IMWD; x++ ) {
        if (y == IMHT-1) w0[x][y] = w[x][numW+1];  // first group is taking rows 0,1,2,3,4,15
        else w0[x][y] = w[x][y];                   // index 15 needs catching because array is only 6 tall
      }
    }
    if (inDivision(y, 2)) {
      for( int x = 0; x < IMWD; x++ ) {            // group two takes rows 3,4,5,6,7,8
        printf("x:% y:%d",x,y);
        w0[x][y - numW] = w[x][y];                 // so each y needs subtracting by numW to get 0,1,2,3,4,5
      }
    }
    if (inDivision(y, 3)) {
      for( int x = 0; x < IMWD; x++ ) {            // group three takes rows 7,8,9,10,11,12
        w0[x][y - 2*numW] = w[x][y];               // so each y needs subtracting by 2*numW to get 0,1,2,3,4,5
      }
    }
    if (inDivision(y, 4)) {
      for( int x = 0; x < IMWD; x++ ) {            // group four takes rows 11,12,13,14,15,0
        if (y == 0) w0[x][y] = w[x][numW+1];       // each y needs subtracting by 3*numW,
        else w0[x][y - 3*numW] = w[x][y];          //   0 needs to be caught and replaced with numW+1 (last index)
      }
    }
  }  // ^This is not DRY but writing in full for now to get it working

  printf( "\nOne processing round completed...\n" );
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
    }
    _writeoutline( line, IMWD );
    printf( "DataOutStream: Line written...\n" );
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
        //toDist <: 1;
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
chan c_inIO, c_outIO, c_control, workers[4];    //extend your channel definitions here

par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    orientation(i2c[0],c_control);        //client thread reading orientation data
    DataInStream(infname, c_inIO);          //thread to read in a PGM image
    DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control, workers);//thread to coordinate work on image
    worker(workers[0]);
    worker(workers[1]);
    worker(workers[2]);
    worker(workers[3]);
  }

  return 0;
}
