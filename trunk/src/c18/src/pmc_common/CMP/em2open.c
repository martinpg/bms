#include <p18cxxx.h>
#include <compare.h>

/********************************************************************
*    Function Name:  OpenECompare2                                  *
*    Return Value:   void                                           *
*    Parameters:     config: bit definitions to configure compare   *
*	 				 period: time period for the compare(match)		*	
*    Description:    This routine configures the compare for        *
*                    interrupt, output signal and compare period    *
*    Notes:          The bit definitions for config can be found    *
*                    in the compare.h file.  
********************************************************************/
#if defined (ECC_V8) || defined (ECC_V8_1)

void OpenECompare2(unsigned char config,unsigned int period)
{
	CCPR2L = period;       	// load ECAxL 
  	CCPR2H = (period >> 8);    // load ECAxH
  
	ECCP2CON = config&0x0F;  // Configure capture

  //configure timer source for CCP
  CCPTMRS0 &= 0b11000111;
  CCPTMRS0 |= ((config&0b01110000)>>1);
	
#if defined(CC4_IO_V2)
  if(config&0x80)
  {
    PIR2bits.CCP2IF = 0;   // Clear the interrupt flag
    PIE2bits.CCP2IE = 1;   // Enable the interrupt

  }
#else  
  if(config&0x80)
  {
    PIR3bits.CCP2IF = 0;   // Clear the interrupt flag
    PIE3bits.CCP2IE = 1;   // Enable the interrupt

  }
#endif  

#ifndef CC4_IO_V2 
if((ECCP2CON & 0x0f) != 0x0a)
	CM2_TRIS = 0;	
#endif	

}


#endif


