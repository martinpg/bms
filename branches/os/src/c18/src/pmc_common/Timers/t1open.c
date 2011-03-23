#include <p18cxxx.h>
#include <timers.h>

/********************************************************************
*    Function Name:  OpenTimer1                                     *
*    Return Value:   void                                           *
*    Parameters:     config: bit definitions to configure Timer1    *
*    Description:    This routine first resets the Timer1 regs      *
*                    to the POR state and then configures the       *
*                    interrupt, clock source and 8/16-bit mode.     *
*    Notes:          The bit definitions for config can be found    *
*                    in the timers.h file.                          *
********************************************************************/
#if defined (TMR_V1) || defined (TMR_V2) || defined (TMR_V3) ||\
    defined (TMR_V4) || defined (TMR_V5) 
void OpenTimer1(unsigned char config)
{
  T1CON = (0x7e & config); // Set everything except 8/16 mode, and
                            // don't start timer yet

  if( config & 0x40 )       // The 8/16 selection bit isn't in the
      T1CONbits.RD16 = 1;   // right place -- we have to move it
  else
      T1CONbits.RD16 = 0;

  TMR1H=0;             // Clear out timer registers
  TMR1L=0;
  PIR1bits.TMR1IF=0;

  if(config&0x80)      // Enable interrupts if selected
    PIE1bits.TMR1IE=1;
  else
    PIE1bits.TMR1IE=0;
  
  T1CONbits.TMR1ON = 1;  // Start Timer1
}

#endif

#if defined (TMR_V6)  || defined (TMR_V7) || defined (TMR_V7_1)
	 
void OpenTimer1(unsigned char config, unsigned char config1)
{
  T1GCON = (0XF3 & config1);
  
  T1CON = (0x7F & config) << 1 ;    
    	
  TMR1H=0;             // Clear out timer registers
  TMR1L=0;
  
  PIR1bits.TMR1IF=0;

  if(config&0x80)      // Enable interrupts if selected
    PIE1bits.TMR1IE=1;
  else
    PIE1bits.TMR1IE=0;
    
  if(T1GCONbits.TMR1GE)	
      T1GCONbits.T1GGO = 1;
  
  T1CONbits.TMR1ON = 1;  // Start Timer1
  
}
#endif 
	
	

