#include <p18cxxx.h>
#include <pwm.h>


/********************************************************************
*    Function Name:  ClosePWM5                                      *
*    Return Value:   void                                           *
*    Parameters:     void                                           *
*    Description:    This routine turns off PWM5.                   *
********************************************************************/
#if defined (PWM_V4) || defined (PWM_V9) || defined (PWM_V14) || defined (PWM_V14_1)
void ClosePWM5(void)
{
  
	CCP5CON=0;            // Turn off PWM5

}
#endif
