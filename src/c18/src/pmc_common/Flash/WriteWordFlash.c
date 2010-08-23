#include "flash.h"

#if defined (FLASH_V2) 
 /*********************************************************************
 Function:        	void WriteWordFlash(unsigned long startaddr, unsigned int data)

 PreCondition:    	None
                  
 Input:           	startaddr - Strating address from which flash has to be written
			data - Data to be written into flash
						
 Output:          	None
 
 Side Effects:    	None
 
 Overview:        	The function writes word to flash 
                   
 Note:            	1. Necessary to erase flash block (1024 Bytes) exclusively in application before writing 
			   if application had written data into to this block of flash(after erasing followed by programming).
			2. Starting address has to be an even address else boundary mismatch will occur 
 ********************************************************************/
void WriteWordFlash(unsigned long startaddr, unsigned int data)
{

unsigned char flag=0;

				TBLPTRU = ((unsigned char)startaddr>>16);						//Load the address to Address pointer registers
				TBLPTRH = (unsigned char)(((unsigned int)startaddr)>>8);	
				TBLPTRL	= ((unsigned char)startaddr);
				
					TABLAT = (unsigned char) (data&0x00FF);
					_asm  TBLWTPOSTINC 	_endasm

					TABLAT =(unsigned char) ((data&0xFF00)>>8);
					_asm  TBLWT 	_endasm
	
			  TBLPTRU = ((unsigned char)startaddr>>16);						//Load the address to Address pointer registers
			  TBLPTRH = (unsigned char)(((unsigned int)startaddr)>>8);	
			  TBLPTRL	= ((unsigned char)startaddr);
		  
		  //*********** Flash write sequence ***********************************
				EECON1bits.WPROG = 1;	
				EECON1bits.WREN = 1;
				if(INTCONbits.GIE)
				{
					INTCONbits.GIE = 0;
					flag=1;
				}
				EECON2  = 0x55;
				EECON2  = 0xAA;
				EECON1bits.WR = 1;
				if(flag)
					INTCONbits.GIE = 1;
		
}


#endif

