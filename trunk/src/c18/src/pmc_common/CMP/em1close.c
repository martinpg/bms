#include <p18cxxx.h>
#include <compare.h>


/********************************************************************
*    Function Name:   CloseECompare1                                *
*    Return Value:    void                                          *
*    Parameters:      void                                          *
*    Description:     This routine disables the capture interrupt.  *
********************************************************************/
#if defined (ECC_V5)

void CloseECompare1(void)
{
   PIE2bits.ECCP1IE = 0;    // Disable the interrupt
   ECCP1CON=0x00;           // Reset the CCP module
}

#elif defined (ECC_V8) || defined (ECC_V8_1)

void CloseECompare1(void)
{
#if defined(CC4_IO_V2)
   PIE1bits.CCP1IE = 0;    // Disable the interrupt
#else  
   PIE3bits.CCP1IE = 0;    // Disable the interrupt
#endif
   ECCP1CON=0x00;           // Reset the CCP module
}

#endif
