#ifndef __W25QXX_H__
#define __W25QXX_H__
#include "stm32f4xx_hal.h"


#define W25X_PageProgram		0x02 
#define W25X_ReadData			  0x03
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7
#define write_codebook(a,b,c) write_anybyte((uint8_t*)a,b*4,c*4)

#define newamp1vq_cb0 0
#define newamp1vq_cb1 20*512
#define newamp2vq_cb0 40*512
#define lsp_cbjvm0 40*512+41*500
#define lsp_cbjvm1 40*512+41*500+10*512
#define lsp_cbjvm2 40*512+41*500+15*512

float read_afloat(uint32_t address);
uint8_t read_abyte(uint32_t address);
void write_page(uint8_t *in, uint32_t start_address, uint16_t nbyte);
void write_anybyte(uint8_t *in, uint32_t WriteAddr, uint32_t nbyte);
void Tx_commad(uint8_t byte);
void erase(void);

#endif