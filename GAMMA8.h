/*file: GAMMA8.h     This is include file for a sketch
 *-----------------------------------------------------------------------------
 *
 * Gamma corrected lookup table
 * 
 * table remaps requsted light values to gamma-corrected light values. 
 * EG. 127, for 50% PWM, maps to 36 to give 50% light output.
 * 0 maps to 0 and 255 maps to 254. (255 is solid on, so 254 is used)
 * 
 */
#ifndef GAMMA8_H__  /* include guard */
#define GAMMA8_H__


const byte GAMMA8[256]  
{     /* 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15*/
/*  0*/  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,/*15*/
/* 16*/  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,/*31*/
/* 32*/  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,/*47*/
/* 48*/  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,/*63*/
/* 64*/  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,/*79*/
/* 80*/ 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,/*95*/
/* 96*/ 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,/*111*/
/*112*/ 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,/*127*/
/*128*/ 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,/*143*/
/*142*/ 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,/*159*/
/*160*/ 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,/*175*/
/*176*/ 90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,/*191*/
/*192*/115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,/*207*/
/*208*/144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,/*223*/
/*224*/177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,/*239*/
/*240*/215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,254 /*255*/
};


#endif /*  GAMMA8_H__
 ------------------------------------ GAMMA8.h EoF----------------------
*/
