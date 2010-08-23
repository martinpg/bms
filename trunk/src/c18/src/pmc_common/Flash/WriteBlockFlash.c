#include "flash.h"

#if defined (FLASH_V1) || defined (FLASH_V2) 
 /*********************************************************************
 Function:        	void WriteBlockFlash(unsigned long startaddr, unsigned int num_bytes, unsigned char *flash_array)

 PreCondition:    	None
                  
 Input:           	startaddr - Strating address from which flash has to be written
			num_blocks - Number of blocks 0f 64 bytes of flash to be written
			*flash_array - Pointer to array contents of which has to be written to flash
			
 Output:          	None
 
 Side Effects:    	Flash will be written in blocks of 64 bytes
 
 Overview:        	The function writes flash from starting address in terms of 64 bytes
			till end address or nearest multiple of 64. If number of bytes between strating 
			and end address in not in multiples of 64 bytes, write begins from address that is previous 
			nearest multiple of 64 and then excessive memory is erased and written with default value of 0xFF
                   
 Note:            	1. Necessary to erase flash block (1024 Bytes) exclusively in application before writing 
			   if application had written data into to this block of flash(after erasing followed by programming).
			2. Necessary to write interms block of 64 bytes
 ********************************************************************/
void WriteBlockFlash(unsigned long startaddr, unsigned char num_blocks, unsigned char *flash_array)
{
unsigned char write_byte=0,flag=0;

		startaddr =(unsigned long) ((unsigned char)(startaddr / 0x40)) *0x40;	//Calculate starting address of the block
		
		while(num_blocks--)
		{
				TBLPTRU = ((unsigned char)startaddr>>16);						//Load the address to Address pointer registers
				TBLPTRH = (unsigned char)(((unsigned int)startaddr)>>8);	
				TBLPTRL	= ((unsigned char)startaddr);
				
				write_byte = 64;
				while(write_byte--)
				{
					TABLAT = *(flash_array++);
					_asm  TBLWTPOSTINC 	_endasm
				
				}	

			  TBLPTRU = ((unsigned char)startaddr>>16);						//Load the address to Address pointer registers
			  TBLPTRH = (unsigned char)(((unsigned int)startaddr)>>8);	
			  TBLPTRL = ((unsigned char)startaddr);
			  //*********** Flash write sequence ***********************************
			  EECON1bits.WREN = 1;
			  if(INTCONbits.GIE)
			  {
				INTCONbits.GIE = 0;
				flag=1;
			  }		  
			  EECON2 = 0x55;
			  EECON2 = 0xAA;
			  EECON1bits.WR =1;
			  EECON1bits.WREN = 0 ; 
			  if(flag)
			  {
				INTCONbits.GIE = 1;	
				flag=0;
			  }
	  
			 startaddr = startaddr + 0x40;									//increment to one block of 64 bytes
		}
		
}


#endif

