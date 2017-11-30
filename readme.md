## Conway's Game of Life

Second year project implementing Game of Life concurrently for the xCORE-200 Explorer board.

This is a pair-programming project by [Louis Heath](https://github.com/louisheath) and [Jack Jones](https://github.com/jj16791).

### Contents

**game_of_life_parallel_rows**

Our concurrent implementation of Conway's Game of Life.

**game_of_life_sequential**

A sequential implementation of the same logic, to show the performance increase of concurrency.

**pgm_generator**

A program which creates .pgm game states, populated with infinitely surviving cell layouts. Often it is more practical to make a .png file and convert it.

**test_outputs**

Program outputs for sequential. "16, 50, 40.pgm" is the output of a 16x16 file after the 50th iteration, which took 40ms to process.

**libraries**

"lib_i2c", "lib_logging" and "lib_xassert" are all libraries provided with the code skeleton.
