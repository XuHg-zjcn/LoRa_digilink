/*---------------------------------------------------------------------------*\

  FILE........: cohpsk_put_test_bits.c
  AUTHOR......: David Rowe
  DATE CREATED: April 2015

  Sinks a stream of test bits generated by cohpsk_get_test_bits,
  useful for testing coh psk mod and demod.  Returns 0 if BER < 0.02
  to facailtate Ctesting.

\*---------------------------------------------------------------------------*/


/*
  Copyright (C) 2015 David Rowe

  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License version 2.1, as
  published by the Free Software Foundation.  This program is
  distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "codec2_cohpsk.h"
#include "octave.h"

#define LOG_FRAMES 100

int main(int argc, char *argv[])
{
    FILE         *fin, *foct;
    char          rx_bits[COHPSK_BITS_PER_FRAME];
    int           state, i, nbits, bit_errors, nerrors;
    short         error_pattern[COHPSK_BITS_PER_FRAME];
    int           error_positions_hist[COHPSK_BITS_PER_FRAME], logframes;
    int           nerr_log[LOG_FRAMES];
    struct COHPSK *coh;

    for(i=0; i<COHPSK_BITS_PER_FRAME; i++)
        error_positions_hist[i] = 0;

    if (argc < 2) {
	fprintf(stderr, "usage: %s InputOneCharPerBitFile [OctaveLogFile]\n", argv[0]);
	exit(1);
    }

    if (strcmp(argv[1], "-") == 0) fin = stdin;
    else if ( (fin = fopen(argv[1],"rb")) == NULL ) {
	fprintf(stderr, "Error opening input file: %s: %s.\n",
         argv[1], strerror(errno));
	exit(1);
    }

    coh = cohpsk_create();

    foct = NULL;
    logframes = 0;
    if (argc == 3) {
        if ( (foct = fopen(argv[2],"wt")) == NULL ) {
            fprintf(stderr, "Error opening output Octave file: %s: %s.\n",
                    argv[2], strerror(errno));
	exit(1);
        }
    }

    for(i=0; i<COHPSK_BITS_PER_FRAME; i++)
        error_positions_hist[i] = 0;

    state = 0; nbits = 0; nerrors = 0;
    while (fread(rx_bits, sizeof(char), COHPSK_BITS_PER_FRAME, fin) ==  COHPSK_BITS_PER_FRAME) {

        cohpsk_put_test_bits(coh, &state, error_pattern, &bit_errors, rx_bits, 0);
        if (state == 1) {
            for(i=0; i<COHPSK_BITS_PER_FRAME; i++)
                error_positions_hist[i] += error_pattern[i];
           if (logframes < LOG_FRAMES)
                nerr_log[logframes++] = bit_errors;
            nerrors += bit_errors;
            nbits   += COHPSK_BITS_PER_FRAME;
        }

        if (fin == stdin) fflush(stdin);
    }

    if (foct != NULL) {
        octave_save_int(foct, "nerr_log_c", nerr_log, 1, logframes);
        octave_save_int(foct, "error_positions_hist_c", error_positions_hist, 1, COHPSK_BITS_PER_FRAME);
        fclose(foct);
    }

    fclose(fin);
    float ber = (float)nerrors/nbits;
    fprintf(stderr, "BER: %4.3f Nbits: %d Nerrors: %d\n", ber, nbits, nerrors);

    /* return code for Ctest - 0 for pass, 1 for fail, based on 2% BER */
    if (ber < 0.02)
        return 0;
    else
        return 1;
}

