// Status "register"
// LSB 0 - Active (@todo high speed mode) ( >1 mA)
//     1 - Shutdown (active low @todo)
//     2 - Charge (1), Discharge (0)
//	   3 - Soft fail (still running)
#define FAIL		0x02;
#define SOFT_FAIL	0x08;
unsigned char STATUS_REG;

// ERROR_REGL
//     1 - Undervoltage
//     2 - Overvoltage
//     3 - Overcurrent 
//     4 - Overtemperature
//     5 - 5V Brownout (@todo unimplemented)
//     6 - Unbalanced (@todo unimplemented)
//     7 - Remote shutdown (@todo unimplemented)
//     8 - RESERVED
//     9 - RESERVED
//    10 - RESERVED
//    11 - RESERVED
//    12 - RESERVED
//    13 - RESERVED
//    14 - RESERVED
//    15 - RESERVED

#define UNDERVOLT		0x01;
#define OVERVOLT		0x02;
#define OVERCURRENT		0x04; 
#define OVERTEMP		0x08;
#define	BROWNOUT		0x10;
#define UNBALANCED		0x20;
#define REMOTE_SHDN		0x40;

// ERROR_REGH
//     1 - Oscillator Failure
//     2 - EEPROM Read Failure
//     3 - Voltage Reference Failure
#define OSC_FAIL		0x01;
#define READ_FAIL		0x02;
#define REF_FAIL		0x04;

unsigned int ERROR_REGL, ERROR_REGH;