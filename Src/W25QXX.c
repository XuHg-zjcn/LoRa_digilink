#include "stm32f4xx_hal.h"
#include "W25QXX.h"
extern SPI_HandleTypeDef hspi1;
uint32_t current_address = 0xFFFFFFFF;
float read_afloat(uint32_t address) {
	float ret;
	address *= 4;
	if(address != current_address) {
	  HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
		uint8_t address8[4];
		address8[0]=W25X_ReadData;
		address8[1]=address>>16;
		address8[2]=address>>8;
		address8[3]=address;
	  HAL_SPI_Transmit(&hspi1, address8, 4, 10000);
		current_address = address;
	}
	HAL_SPI_Receive(&hspi1, (uint8_t*)&ret, 4, 10000);
	current_address += 4;
	return ret;
}
uint8_t read_abyte(uint32_t address) {
	uint8_t ret;
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
	if(address != current_address) {
		uint8_t address8[4];
		address8[0]=W25X_ReadData;
		address8[1]=address>>16;
		address8[2]=address>>8;
		address8[3]=address;
	  HAL_SPI_Transmit(&hspi1, address8, 4, 10000);
		current_address = address;
	}
	HAL_SPI_Receive(&hspi1, (uint8_t*)&ret, 1, 10000);
	//HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
	current_address += 1;
	return ret;
}
uint8_t W25QXX_BUSY(void) //??W25Q16?????? ?????1
{
    uint8_t flag=0x05;
    HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &flag, 1, 10000);
    HAL_SPI_Receive(&hspi1, &flag, 1, 10000);
	  HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
    flag&=0x01;
    return flag;
}

void write_page(uint8_t *in, uint32_t start_address, uint16_t nbyte) {
	//start_address |= 0x02<<24;
	uint8_t address8[4];
	Tx_commad(W25X_WriteEnable);
	address8[0]=0x02;
	address8[1]=start_address>>16;
	address8[2]=start_address>>8;
	address8[3]=start_address;
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, address8, 4, 10000);
	HAL_SPI_Transmit(&hspi1, in, nbyte, 10000);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
	while(W25QXX_BUSY());
}
void write_anybyte(uint8_t *in, uint32_t WriteAddr, uint32_t nbyte) {
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256;		 	    
	if(nbyte<=pageremain)
		pageremain=nbyte;
	while(1)
	{	   
		write_page(in,WriteAddr,pageremain);
		if(nbyte==pageremain)
			break;
	 	else //NumByteToWrite>pageremain
		{
			in+=pageremain;
			WriteAddr+=pageremain;	

			nbyte-=pageremain;
			if(nbyte>256)
				pageremain=256;
			else
				pageremain=nbyte;
		}
	}
}
void Tx_commad(uint8_t byte) {
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &byte, 1, 10000);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
}
void erase(void) {
	Tx_commad(W25X_WriteEnable);
	Tx_commad(0x00);
	Tx_commad(W25X_ChipErase);
	while(W25QXX_BUSY());
}
