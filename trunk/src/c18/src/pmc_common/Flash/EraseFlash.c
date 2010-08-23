#include "flash.h"

#if defined (FLASH_V1) || defined (FLASH_V2)  
 /*********************************************************************
 Function:        	void EraseFlash(unsigned long startaddr, unsigned long endaddr)

 PreCondition:    	None
                  
 Input:           	startaddr - Strating address from which flash has to be erased
			endaddr - End address till which flash has to be erased
 
 Output:          	None
 
 Side Effects:    	Flash will be erased in blocks of 1024 bytes
 
 Overview:        	The function erases flash from starting address in terms of 1024 bytes
			till end address or nearest multiple of 1024 
                   
 Note:            	1. If number of bytes between strating and end address in not in multiples 
			    of 1024 bytes, then excessive memory is erased upto nearest next 
			    multiple of 1024. 
			2. The starting and end address has to be in blocks of 1024 bytes 
			     else function will allign the address to nearest previous and next 
			     1024 byte alligned address respectively
 ********************************************************************/
void EraseFlash(unsigned long startaddr, unsigned long endaddr)
{
unsigned char flag=0;

		startaddr =(unsigned long) ((unsigned char)(startaddr / 0x400)) *0x400;	//Calculate starting address of the block
		
		while(startaddr<endaddr)
		{
			TBLPTRU = ((unsigned char)startaddr>>16);						//Load the address to Address pointer registers
			TBLPTRH = (unsigned char)(((unsigned int)startaddr)>>8);	
			TBLPTRL	= ((unsigned char)startaddr);
			//*********Flash Erase sequence*****************
			EECON1bits.WREN = 1;
			EECON1bits.FREE = 1;
			if(INTCONbits.GIE)
			{
				INTCONbits.GIE = 0;
				flag=1;
			}
			EECON2 = 0x55;
			EECON2 = 0xAA;
			EECON1bits.WR = 1;
			if(flag)
				INTCONbits.GIE = 1;
			
			startaddr = startaddr +0x400;
		}
}


#endif

