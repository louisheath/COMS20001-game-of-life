// COMS20001 - Game of Life: Jack Jones and Louis Heath
// (using the XMOS i2c accelerometer)
// Version 2 : Four workers

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width
#define  NWKS 4                   //number of workers

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0]:port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0]:port p_sda = XS1_PORT_1F;

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
// Distributor receives current world state from DataStreamIn,
//             divides world and distributes it to workers,
//             receives new cells from workers and builds new world,
//             and sends new world to DataStreamOut
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend worker[NWKS])
{
    uchar val[IMWD][IMHT];    // World state

    //Starting up and wait for tilting of the xCore-200 Explorer
    printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
    printf( "Waiting for Board Tilt...\n" );
    //fromAcc :> int value;

    printf( "Processing...\n" );

    for(int y = 0; y < IMHT; y++ ) {       // Read every pixel into world array
        for( int x = 0; x < IMWD; x++ ) {
           c_in :> val[x][y];
        }
    }

    // Divide work between workers
    for(int w = 0; w < NWKS; w++) {                         // for each of the workers
        for(int y = 0; y < ((IMHT / NWKS) + 2); y++ ) {     // for the portion of rows to be given to the worker
            for( int x = 0; x < IMWD; x++ ) {               // for every column
                // send cell values to the worker, who will combine them into a new array
                worker[w] <: val[x][mod(IMHT, (y + (w*(IMHT / NWKS)) - 1))];
            }
        }
    }

    // Receive processed cells from workers
    for(int w = 0; w < NWKS; w++) {                  // for each of the workers
        for(int y = 0; y < ((IMHT / NWKS)); y++ ) {   // for each row that will be used in new array (edge rows not returned)
            for( int x = 0; x < IMWD; x++ ) {          // for every column
               // receive cell values from worker and place into new world array
               worker[w] :> val[x][y + (w*(IMHT / NWKS))];
            }
        }
    }

    // Send processed data to DataOutStream
    for(int y = 0; y < IMHT; y++ ) {
        for( int x = 0; x < IMWD; x++ ) { // for every cell
            c_out <: val[x][y];          // send cell information to DataOutStream
        }
    }

    //printf( "\nOne processing round completed...\n" );
}

int getNeighbours(int x, int y, uchar rowVal[IMWD][IMHT / NWKS + 2]) {
    // variables used in finding neighbours
    int xRight = mod(IMWD, x+1);
    int xLeft = mod(IMWD, x-1);
    int yUp = y-1;
    int yDown = y+1;
    // store states of neighbours in array
    uchar neighbours[8] = {(rowVal[xLeft][yUp]),   (rowVal[x][yUp]),   (rowVal[xRight][yUp]),
                           (rowVal[xLeft][y]),                         (rowVal[xRight][y]),
                           (rowVal[xLeft][yDown]), (rowVal[x][yDown]), (rowVal[xRight][yDown])};
    // count number of alive neighbours
    int alive = 0;
    for (int i = 0; i < 8; i++) {
        if (neighbours[i] == 0xFF) alive++;
    }
    return alive;
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise workers to work on a row of cells
//
/////////////////////////////////////////////////////////////////////////////////////////
void worker(int id, chanend fromFarmer, chanend wLeft, chanend wRight)
{
    // number of rows worker is processing (excluding ghost rows)
    int load = IMHT / NWKS;
    // array of rows worker will work on
    uchar rowVal[IMWD][(IMHT / NWKS) + 2];
    // array for output rows
    uchar newVal[IMWD][(IMHT / NWKS)];
    // iteration counter
    int i = 0;

    // contruct array to work on
    for( int y = 0; y < (load + 2); y++ ) {     // for every row to be input
        for( int x = 0; x < IMWD; x++ ) {         // for every column
            // read in rows cell by cell from distributer to be worked on
            fromFarmer :> rowVal[x][y];
        }
    }

    while (i < 100) {
        i++;

        // Look at neighbouring cells and work out the next state of each cell
        for( int y = 1; y < load + 1; y++ ) {   // for every row excluding edge rows
            for( int x = 0; x < IMWD; x++ ) {            // for every column
                // set cell to alive by default
                newVal[x][y - 1] = 0xFF;

                // get number of alive neighbours
                int alive = getNeighbours(x, y, rowVal);
                // ^ How costly is it to send array as parameter? Is it worth it for cleaner code?

                // If currently alive
                if (rowVal[x][y] == 0xFF) {
                    // If number of alive neighbours isn't two or three, die. Else stay alive by default
                    if (alive != 2 && alive != 3)
                        newVal[x][y - 1] = 0x0;
                }
                // Else cell is currently dead: if not exactly three alive neighbours stay dead
                else if (alive != 3)
                    newVal[x][y - 1] = 0x0;
            }
        }

        // Update processed rows for use in next iteration
        for( int y = 0; y < load; y++ ) {                  // for every row excluding edge rows
            for( int x = 0; x < IMWD; x++ ) {              // for each column
                rowVal[x][y + 1] = newVal[x][y];           // update non-overlapping rows
            }
        }

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
           for ( int x = 0; x < IMWD; x++ )
               wLeft <: newVal[x][0];
           // 2. send to right
           for ( int x = 0; x < IMWD; x++ )
               wRight <: newVal[x][load - 1];
           // 3. receive from left
           for ( int x = 0; x < IMWD; x++ )
               wLeft :> rowVal[x][0];
           // 4. receive from right
           for ( int x = 0; x < IMWD; x++ )
               wRight :> rowVal[x][load + 1];
       }
       else {             // even numbered workers
           // 1. receive from right
           for ( int x = 0; x < IMWD; x++ )
               wRight :> rowVal[x][load + 1];
           // 2. receive from left
           for ( int x = 0; x < IMWD; x++ )
               wLeft :> rowVal[x][0];
           // 3. send to right
           for ( int x = 0; x < IMWD; x++ )
               wRight <: newVal[x][load - 1];
           // 4. send to left
           for ( int x = 0; x < IMWD; x++ )
               wLeft <: newVal[x][0];
       }
    }

    //printf("Worker %d iterations complete\n", id);

    // Send new cell states to farmer for combining
    for( int y = 0; y < load; y++ ) {             // for every row excluding edge rows
        for( int x = 0; x < IMWD; x++ ) {         // for each column
            fromFarmer <: newVal[x][y];           // send to farmer (distributer)
        }
    }
    //printf("Worker %d passed cells to distributer\n", id);
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

  i2c_master_if i2c[1];                          // interface to orientation sensor

  // Channel definitions
  chan c_inIO,     // DataStreamIn
       c_outIO,    // DataStreamOut
       c_control,  // Orientation sensor
       WtoD[NWKS], // Workers to Distributer
       WtoW[NWKS]; // Workers to Workers

  par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);              //server thread providing orientation data
    on tile[0] : orientation(i2c[0],c_control);                     //client thread reading orientation data
    on tile[0] : DataInStream("test.pgm", c_inIO);                  //thread to read in a PGM image
    on tile[0] : DataOutStream("testout.pgm", c_outIO);             //thread to write out a PGM image
    on tile[0] : distributor(c_inIO, c_outIO, c_control, WtoD);     //thread to coordinate work on image
    //initialise 4 workers
    on tile[1] : worker(0, WtoD[0], WtoW[3], WtoW[0]);
    on tile[1] : worker(1, WtoD[1], WtoW[0], WtoW[1]);
    on tile[1] : worker(2, WtoD[2], WtoW[1], WtoW[2]);
    on tile[1] : worker(3, WtoD[3], WtoW[2], WtoW[3]);
    // channels between workers: e.g. 4 workers
    //
    //    <--- w0 <-----> w1 <-----> w2 <-----> w3 --->
    //  WtoW[3]   WtoW[0]    WtoW[1]    WtoW[2]    WtoW[3]
  }

  return 0;
}
