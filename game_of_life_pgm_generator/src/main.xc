// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 1024                  //image height
#define  IMWD 1024                  //image width

typedef unsigned char uchar;      //using uchar as shorthand

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
        c_in :> line[x];
        //printf( "-%4.1d", line[x]); //show image values
    }
    printf( "processed line %d \n", y );
    _writeoutline( line, IMWD );
  }

  //Close the PGM image
  _closeoutpgm();

  printf( "DataOutStream: Complete\n");
  return;
}

// give dataOut cell values to put in the file, repeating the given layout tile
void work(chanend c_out) {
    uchar A = 0xFF; // alive
    uchar D = 0x0;  // dead

    uchar layout[8][8] = {
                {D, D, D, D, D, D, D, D},
                {D, D, D, D, D, D, D, D},
                {D, D, D, D, D, D, D, D},
                {D, D, D, A, D, D, D, D},
                {D, D, D, D, A, D, D, D},
                {D, D, A, A, A, D, D, D},
                {D, D, D, D, D, D, D, D},
                {D, D, D, D, D, D, D, D}
        };

    for ( int y = 0; y < IMHT; y++ ) {
        for ( int x = 0; x < IMWD; x++ ) {
            c_out <: layout[x % 8][y % 8];
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    chan c;

    par {
        DataOutStream("out.pgm", c);       //thread to write out a PGM image
        work(c);
    }

    return 0;
}
