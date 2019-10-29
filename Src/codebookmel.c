/* THIS IS A GENERATED FILE. Edit generate_codebook.c and its input */

/*
 * This intermediary file and the files that used to create it are under 
 * The LGPL. See the file COPYING.
 */

#include "defines.h"

  /* /media/xrj/软件/codec2/codec2/src/codebook/mel1.txt */
#ifdef __EMBEDDED__
static const float codes0[] = {
#else
static float codes0[] = {
#endif
  550,
  600,
  650,
  700,
  750,
  800,
  850,
  900
};
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel2.txt */
#ifdef __EMBEDDED__
static const float codes1[] = {
#else
static float codes1[] = {
#endif
  50,
  100,
  200,
  300
};
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel3.txt */
#ifdef __EMBEDDED__
static const float codes2[] = {
#else
static float codes2[] = {
#endif
  800,
  850,
  900,
  950,
  1000,
  1050,
  1100,
  1150,
  1200,
  1250,
  1300,
  1350,
  1400,
  1450,
  1500,
  1650
};
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel4.txt */
#ifdef __EMBEDDED__
static const float codes3[] = {
#else
static float codes3[] = {
#endif
  25,
  50,
  75,
  100,
  125,
  150,
  175,
  250
};
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel5.txt */
#ifdef __EMBEDDED__
static const float codes4[] = {
#else
static float codes4[] = {
#endif
  1350,
  1400,
  1450,
  1500,
  1550,
  1600,
  1650,
  1700
};
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel6.txt */
#ifdef __EMBEDDED__
static const float codes5[] = {
#else
static float codes5[] = {
#endif
  25,
  50,
  100,
  150
};

const struct lsp_codebook mel_cb[] = {
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel1.txt */
  {
    1,
    3,
    8,
    codes0
  },
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel2.txt */
  {
    1,
    2,
    4,
    codes1
  },
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel3.txt */
  {
    1,
    4,
    16,
    codes2
  },
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel4.txt */
  {
    1,
    3,
    8,
    codes3
  },
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel5.txt */
  {
    1,
    3,
    8,
    codes4
  },
  /* /media/xrj/软件/codec2/codec2/src/codebook/mel6.txt */
  {
    1,
    2,
    4,
    codes5
  },
  { 0, 0, 0, 0 }
};
