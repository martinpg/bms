
#define uiSU_REQD		0x10	// bit 5 indicates SU must be enabled
#define uiBUFFER_SIZE	30


#define cmdSTATUS		0x01
#define cmdRCVMSGS		0x02
#define cmdCLOSE_RELAY	0x11
#define cmdOPEN_RELAY	0x12
#define cmdRED_ON		0x03
#define cmdRED_OFF		0x04
#define cmdGREEN_ON		0x05
#define cmdGREEN_OFF	0x06
#define cmdGET_VALUE	0x07
#define cmdSU			0x08
#define cmdSU_OFF		0x13
#define cmdVOLTS		0x09
#define cmdCURRENT		0x0A
#define cmdTEMP			0x0B


const unsigned char txtSTATUS[] = 		"status";
const unsigned char txtRCVMSGS[] =		"rmsg";
const unsigned char txtCLOSE_RELAY[] =	"on";
const unsigned char txtOPEN_RELAY[] =	"off";
const unsigned char txtRED_ON[] =		"r on";
const unsigned char txtRED_OFF[] =		"r off";
const unsigned char txtGREEN_ON[] =		"g on";
const unsigned char txtGREEN_OFF[] =	"g off";
const unsigned char txtGET_VALUE[] =	"get";
const unsigned char txtSU[] =			"su";
const unsigned char txtSU_OFF[] =		"nosu";
const unsigned char txtVOLTS[] = 		"volts";
const unsigned char txtCURRENT[] =		"current";

// Errors
