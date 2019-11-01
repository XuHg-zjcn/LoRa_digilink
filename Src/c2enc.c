/*---------------------------------------------------------------------------*\

  FILE........: c2enc.c
  AUTHOR......: David Rowe
  DATE CREATED: 23/8/2010

  Encodes a file of raw speech samples using codec2 and outputs a file
  of bits.

\*---------------------------------------------------------------------------*/

/*
  Copyright (C) 2010 David Rowe

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

#include "codec2.h"
#include "c2file.h"
#include "fatfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "c2enc.h"
FIL fin,fout;
void c2enc(void)
{
    int            mode;
    void          *codec2;
    short         *buf;
    unsigned char *bits;
    int            nsam, nbit, nbyte;
    int            bit, byte,i;
    int            report_var = 0;
    int            eq = 0;
	  int            size_readed;
	  int            size_writed;
	  FRESULT res2;
	mode = CODEC2_MODE_450;

    f_open(&fin, "0:/in.raw", FA_READ);//Input file
	  f_open(&fout, "0:/o_w1.ca", FA_CREATE_ALWAYS | FA_WRITE);//output data for debug
    // Write a header if we're writing to a .c2 file

    codec2 = codec2_create(mode);
    nsam = codec2_samples_per_frame(codec2);
    nbit = codec2_bits_per_frame(codec2);
    buf = (short*)pvPortMalloc(nsam*sizeof(short));
    nbyte = (nbit + 7) / 8;
    bits = (unsigned char*)pvPortMalloc(nbyte*sizeof(char));
		
    //fprintf(stderr,"gray: %d softdec: %d\n", gray, softdec);
		printf("mode=%d,nsam=%d,nbit=%d\r\n",mode,nsam,nbit);
    //while(fread(buf, sizeof(short), nsam, fin) == (size_t)nsam) {
    f_read(&fin, buf, nsam*sizeof(short), (UINT*)&size_readed);
		size_writed=nbyte*sizeof(char);
		while(size_readed==nsam*sizeof(short) && size_writed==nbyte*sizeof(char)){
	      codec2_encode(codec2, bits, buf);//size_readed==nsam*sizeof(short) && size_readed!=0 && buf[0]!=0 && buf[0]!=a
			  //printf("0x%02x%02x%02x\r\n",bits[0],bits[1],bits[2]);
		    //printf("%d,%d,rd=%d\r\n",buf[0],buf[1],size_readed);
        f_write(&fout, bits, nbyte*sizeof(char), (UINT*)&size_writed);//write encoded data
		    f_read(&fin, buf, nsam*sizeof(short), (UINT*)&size_readed);//read input data
		}
		
	  /*for(int i=0;i<3;i++) {
			//printf("%d\n",xPortGetFreeHeapSize());
			f_read(&fin, buf, nsam*sizeof(short), (UINT*)&size_readed);
			codec2_encode_450(codec2, bits, buf);
			HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
      f_write(&fout, bits, nbyte*sizeof(char), (UINT*)&size_writed);
			HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
		}*/
	
  //fwrite(bits, sizeof(char), nbyte, fout);

	// if this is in a pipeline, we probably don't want the usual
        // buffering to occur

    f_close(&fin);
		f_close(&fout);
    codec2_destroy(codec2);

    vPortFree(buf);
    vPortFree(bits);
    
    //f_close(&fout);
}
