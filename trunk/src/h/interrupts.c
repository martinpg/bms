#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}

// High priority interrupt routine

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh ()
{
	if (PIR2bits.OSCFIF) {
		
	}	
  /*if (INTCONbits.TMR0IF)
    {                                   //check for TMR0 overflow
      INTCONbits.TMR0IF = 0;            //clear interrupt flag
      Flags.Bit.Timeout = 1;            //indicate timeout
      LATBbits.LATB0 = !LATBbits.LATB0; //toggle LED on RB0
    }
*/
}