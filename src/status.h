/*
 * Status.h -Status header
 */
#ifndef STATUS_H
#define STATUS_H
 
// Status "register"
// LSB 0 - Active (@todo high speed mode) (>1 mA)
//     1 - Shutdown (active low @todo)
//     2 - Charge (1), Discharge (0) @todo
//	   3 - Sleep
//     4 - Fail
//     5 - Superuser
//     6 - RESERVED
//     7 - Constant Output

#define ACTIVE		0x01
#define SHDN		0x02
#define CHARGING	0x04
#define SLEEP		0x08
#define FAIL		0x10
#define SU			0x20
#define ALLMSGS		0x40

volatile unsigned char STATUS_REG;
volatile struct {
	unsigned sACT:1;
	unsigned sSDN:1;
	unsigned sCHG:1;
	unsigned sSLP:1;
	unsigned sFAI:1;
	unsigned sSU:1;
	unsigned :2;
} STATUS_REGbits;

#define UNDERVOLT		0x0001	// Undervoltage
#define OVERVOLT		0x0002	// Overvoltage
#define OVERCURRENTIN	0x0004	// Overcurrent (charging)
#define OVERCURRENTOUT	0x0008	// Overcurrent (discharging)
#define OVERTEMP		0x0010	// Overtemperature
#define	BROWNOUT		0x0020	// 5V Brownout (@todo implement)
#define UNBALANCED		0x0040	// Unbalanced (@todo implement)
#define REMOTE_SHDN		0x0080	// Remote shutdown (@todo implement)
#define OSC_FAIL		0x0100	// Oscillator Failure
#define READ_FAIL		0x0200	// EEPROM Read Failure
#define REF_FAIL		0x0400	// Voltage Reference Failure
#define TEMP_FAIL		0x0800	// Temperature
#define CURRENT_FAIL	0x1000	// Current sensor Failure

volatile unsigned int ERROR_REG;
volatile struct {
	unsigned UV:1;
	unsigned OV:1;
	unsigned OC:1;
	unsigned OD:1;
	unsigned OT:1;
	unsigned BO:1;
	unsigned UB:1;
	unsigned RS:1;

	unsigned OF:1;
	unsigned RF:1;
	unsigned VF:1;
	unsigned TF:1;
	unsigned CF:1;
	unsigned :3;
} ERROR_REGbits;

#endif /* STATUS_H */