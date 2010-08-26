#ifndef __UI_H
#define __UI_H
/*
 * UI.h - User Interface header
 */

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

// UI States
#define uiState_RESET			0x00
#define uiState_SYSTEM_PROMPT	0x01
#define uiState_WAIT_FOR_CMD	0x02
//#define uiState_PROCESS_CMD		0x03
#define uiState_WAIT_FOR_INPUT1	0x03
#define uiState_WAIT_FOR_INPUT2 0x04

unsigned char parseTlm( char* s );
command* parseCmd( char* s );

#endif /* __UI_H */
