// Status "register"
// LSB 0 - Active (@todo high speed mode) ( >1 mA)
//     1 - Shutdown (active low @todo)
//     2 - Charge (1), Discharge (0)
//	   3 - Soft fail (still running)
#define FAIL		0x02;
#define SOFT_FAIL	0x08;
unsigned char STATUS_REG;

// Error "codes"
//     1 - Undervoltage
//     2 - Overvoltage
//     3 - Overcurrent 
//     4 - Overtemperature
//     5 - 5V Brownout (@todo unimplemented)
//     6 - Unbalanced (@todo unimplemented)
//     7 - Remote shutdown (@todo unimplemented)
//     8 - ** This bit indicates an oscillator failure
//    16 - ** This bit indicates an EEPROM read failure
//    32 - ** This bit indicates a voltage reference failure
#define UNDERVOLT		0x01;
#define OVERVOLT		0x02;
#define OVERCURRENT		0x03; 
#define OVERTEMP		0x04;
#define	BROWNOUT		0x05;
#define UNBALANCED		0x06;
#define REMOTE_SHDN		0x07;

#define OSC_FAIL		0x08;
#define READ_FAIL		0x10;
#define REF_FAIL		0x20;

unsigned char ERROR;