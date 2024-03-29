#include <p18cxxx.h>
#include <compare.h>

/********************************************************************
*    Function Name:  OpenECompare3                                  *
*    Return Value:   void                                           *
*    Parameters:     config: bit definitions to configure compare   *
*	 				 period: time period for the compare(match)		*	
*    Description:    This routine configures the compare for        *
*                    interrupt, output signal and compare period    *
*    Notes:          The bit definitions for config can be found    *
*                    in the compare.h file.  
********************************************************************/
#if defined (ECC_V8) || defined (ECC_V8_1)

void OpenECompare3(unsigned char config,unsigned int period)
{
	CCPR3L = period;       	// load ECAxL 
  	CCPR3H = (period >> 8);    // load ECAxH
  
	CCP3CON = config&0x0F;  // Configure capture

  //configure timer source for CCP
  CCPTMRS0 &= 0b00111111;
  CCPTMRS0 |= ((config&0b00110000)<<2); 
	
  if(config&0x80)
  {
    PIR4bits.CCP3IF = 0;   // Clear the interrupt flag
    PIE4bits.CCP3IE = 1;   // Enable the interrupt
  }
  
#ifndef CC4_IO_V2    
if((CCP3CON & 0x0f) != 0x0a)
	CM3_TRIS = 0;	
#endif	

 
}


#endif


