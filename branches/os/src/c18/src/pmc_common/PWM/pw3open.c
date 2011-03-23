#include <p18Cxxx.h>
#include <pwm.h>


/********************************************************************
*    Function Name:  OpenPWM3                                       *
*    Return Value:   void                                           *
*    Parameters:     period: PWM period                             *
*    Description:    This routine first resets the PWM registers    *
*                    to the POR state.  It then configures clock    *
*                    source.                                        *
********************************************************************/
#if defined (PWM_V3) 
void OpenPWM3( char period )
{
        CCP3CON=0b00001100;    //ccpxm3:ccpxm0 11xx=pwm mode

  		T2CONbits.TMR2ON = 0;  // STOP TIMER2 registers to POR state
  		PR2 = period;          // Set period
  		T2CONbits.TMR2ON = 1;  // Turn on PWM3
}

#elif defined (PWM_V4) 
void OpenPWM3( char period )
{
  CCP3CON=0b00001100;    //ccpxm3:ccpxm0 11xx=pwm mode

	if (T3CONbits.T3CCP2 == 0 && T3CONbits.T3CCP1 == 0)
  	{
     	T2CONbits.TMR2ON = 0;  // STOP TIMER2 registers to POR state
     	PR2 = period;          // Set period
     	T2CONbits.TMR2ON = 1;  // Turn on PWM3
  	}
  	else
  	{
     	T4CONbits.TMR4ON = 0;  // STOP TIMER4 registers to POR state
     	PR4 = period;          // Set period
     	T4CONbits.TMR4ON = 1;  // Turn on PWM3
  	}
}

#elif defined (PWM_V9) 
void OpenPWM3( char period )
{
  CCP3CON=0b00001100;    //ccpxm3:ccpxm0 11xx=pwm mode

	TRISGbits.TRISG0=0;

	if (T3CONbits.T3CCP2 == 0 && T3CONbits.T3CCP1 == 0)
  	{
     	T2CONbits.TMR2ON = 0;  // STOP TIMER2 registers to POR state
     	PR2 = period;          // Set period
     	T2CONbits.TMR2ON = 1;  // Turn on PWM3
  	}
  	else
  	{
     	T4CONbits.TMR4ON = 0;  // STOP TIMER4 registers to POR state
     	PR4 = period;          // Set period
     	T4CONbits.TMR4ON = 1;  // Turn on PWM3
  	}
}

#endif
