#include "subsystems/misc.hpp"

#include <stdio.h>

/**
 * This solution needs to be rewritten really bad. 
 * 
 * We should really be using a seperate repo for each bot, regardless of how 
 * simliar they are since the bots can be changed at any time. 
 * 
 * Second, reading from a file each and every time we want to check which bot it
 * is very expensive. 
 * 
 * If we were going to go with a single repo for both turret bots, I would opt
 * for creating new makefile targets that adds a define to differentiate the 
 * two robots so that there is no runtime cost and so we don't need an SD card
 * just to differentiate the two.
 */
bool isSilva(){

    FILE* usd_file_read = fopen("/usd/TURRET_ID.txt", "r");
    char buf[50]; // This just needs to be larger than the contents of the file
    fread(buf, 1, 50, usd_file_read); // passing 1 because a `char` is 1 byte, and 50 b/c it's the length of buf
    int isGoldy = buf[0] == '1'; // buf[0] is the first character in the file, if it's a 1, the turret is goldy. If not, it's silva.
    fclose(usd_file_read); 
    return !isGoldy;
}
