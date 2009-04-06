// Status "register"
// LSB 0 - Active (@todo high speed mode) ( >1 mA)
//     1 - Shutdown (active low @todo)
//     2 - Charge (1), Discharge (0) @todo
//	   3 - Soft fail (still running)
//     4 - RESERVED
//     5 - RESERVED
//     6 - RESERVED
//     7 - RESERVEd
//     8 - RESERVED
//     9 - RESERVED
//    10 - RESERVED
//    11 - RESERVED
//    12 - RESERVED
//    13 - RESERVED
//    14 - RESERVED
//    15 - RESERVED
#define SHDN		0x01
#define CHARGING	0x04
#define FAIL		0x02
#define SOFT_FAIL	0x08


// ERROR_REGL
//     0 - Undervoltage
//     1 - Overvoltage
//     2 - Overcurrent (charging)
//     3 - Overcurrent (discharging)
//     4 - Overtemperature
//     5 - 5V Brownout (@todo unimplemented)
//     6 - Unbalanced (@todo unimplemented)
//     7 - Remote shutdown (@todo unimplemented)

#define UNDERVOLT		0x01
#define OVERVOLT		0x02
#define OVERCURRENTIN	0x04
#define OVERCURRENTOUT	0x08
#define OVERTEMP		0x10
#define	BROWNOUT		0x20
#define UNBALANCED		0x40
#define REMOTE_SHDN		0x80

// ERROR_REGH
//     0 - Oscillator Failure
//     1 - EEPROM Read Failure
//     2 - Voltage Reference Failure
//     3 - Temperature Sensor Failure
//     4 - RESERVED
//     5 - RESERVED
//     6 - RESERVED
//     7 - RESERVED

#define OSC_FAIL		0x01
#define READ_FAIL		0x02
#define REF_FAIL		0x04
#define TEMP_FAIL		0x08