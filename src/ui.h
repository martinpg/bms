/*
 * UI.h - User Interface header
 */
#ifndef UI_H
#define UI_H

typedef struct {
	unsigned cmd:4;
	unsigned ops:3;
	unsigned su:1;
} command;

#define uiBUFFER_SIZE	24
#define uiCOMMAND_QSIZE	5

#define cmdSTATUS		0x01
#define cmdRCVMSGS		0x02
#define cmdRED_ON		0x03
#define cmdRED_OFF		0x04
#define cmdGREEN_ON		0x05
#define cmdGREEN_OFF	0x06
#define cmdGET_VALUE	0x07
#define cmdGET_VOLTS	0x17
#define cmdGET_CURRENT	0x27
#define cmdGET_TEMP		0x37
#define cmdCONV_RAW		0x08
#define cmdCONV_FLOAT	0x09
#define cmdSU			0x0A
#define cmdCLOSE_RELAY	0x0B
#define cmdOPEN_RELAY	0x0C
#define cmdSET_VALUE	0x0D
#define cmdSU_OFF		0x0E
#define cmdPRINT_MSG	0x0F
#define cmdRESET		0xFF

#define tlmVOLTS		'V'
#define tlmTEMP			'T'
#define tlmCURRENT		'I'

const unsigned char txtSTATUS[] = 		"status";
const unsigned char txtRCVMSGS[] =		"rmsgs";
const unsigned char txtCLOSE_RELAY[] =	"on";
const unsigned char txtOPEN_RELAY[] =	"off";
const unsigned char txtRED_ON[] =		"r on";
const unsigned char txtRED_OFF[] =		"r off";
const unsigned char txtGREEN_ON[] =		"g on";
const unsigned char txtGREEN_OFF[] =	"g off";
const unsigned char txtGET_VALUE[] =	"get";
const unsigned char txtSET_VALUE[] =	"set";
const unsigned char txtSU[] =			"su";
const unsigned char txtSU_OFF[] =		"nosu";
const unsigned char txtVOLTS[] = 		"volts";
const unsigned char txtCURRENT[] =		"current";
const unsigned char txtTEMP[] =			"temp";
const unsigned char txtMSGS[] =			"msg";
const unsigned char txtCONV_RAW[] = 	"convr";
const unsigned char txtCONV_FLOAT[] =	"convf";
const unsigned char txtRESET[] =		"reset";
const unsigned char txtCRLF[] = 		"\r\n";

// UI States
#define uiState_RESET			0x00
#define uiState_SYSTEM_PROMPT	0x01
#define uiState_WAIT_FOR_CMD	0x02
//#define uiState_PROCESS_CMD		0x03
#define uiState_WAIT_FOR_INPUT1	0x03
#define uiState_WAIT_FOR_INPUT2 0x04

#endif /* UI_H */
