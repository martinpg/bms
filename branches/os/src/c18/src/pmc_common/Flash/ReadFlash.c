#include "flash.h"

#if defined (FLASH_V1) || defined (FLASH_V2) || defined (FLASH_V3) 
 /*********************************************************************
 Function:        	void ReadFlash(unsigned long startaddr, unsigned int num_bytes, unsigned char *flash_array)

 PreCondition:    	None
                  
 Input:           	startaddr - Strating address from which flash has to be read
			num_bytes - Number of bytes of flash to be read
			*flash_array - Pointer to array to which the flash has be read
 
 Output:          	Reads the flash content to array passed as pointer
 
 Side Effects:    	None
 
 Overview:        	The function reads flash for number of bytes passed as parameter from starting address 
                   
 Note:            	Non zero number of bytes has to be passed as parameter for num_bytes
 ********************************************************************/
void ReadFlash(unsigned long startaddr, unsigned int num_bytes, unsigned char *flash_array)
{

			TBLPTRU = ((unsigned char)startaddr>>16);						//Load the address to Address pointer registers
			TBLPTRH = (unsigned char)(((unsigned int)startaddr)>>8);	
			TBLPTRL	= ((unsigned char)startaddr);

		while(num_bytes--)
		{
			//*********** Table read sequence ******************************
			_asm	TBLRDPOSTINC _endasm
			*(flash_array + num_bytes) = TABLAT;
		}	

}


#endif

