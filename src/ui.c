#include <stddef.h>
#include "ui.h"

const unsigned char txtSTATUS =  		"status";
const unsigned char txtRCVMSGS =		"rmsgs";
const unsigned char txtCLOSE_RELAY =	"on";
const unsigned char txtOPEN_RELAY =		"off";
const unsigned char txtRED_ON =			"r on";
const unsigned char txtRED_OFF =		"r off";
const unsigned char txtGREEN_ON =		"g on";
const unsigned char txtGREEN_OFF =		"g off";
const unsigned char txtGET_VALUE =		"get";
const unsigned char txtSET_VALUE =		"set";
const unsigned char txtSU =				"su";
const unsigned char txtSU_OFF =			"nosu";
const unsigned char txtVOLTS = 			"volts";
const unsigned char txtCURRENT =		"current";
const unsigned char txtTEMP =			"temp";
const unsigned char txtMSGS =			"msg";
const unsigned char txtCONV_RAW = 		"convr";
const unsigned char txtCONV_FLOAT =		"convf";
const unsigned char txtRESET =			"reset";

command* parseCmd( char* s ) {
	static command parsedCmd;
	parsedCmd.cmd = 0;
	parsedCmd.ops = 0;
	parsedCmd.su = 0;
	if (strcmp(s, txtSTATUS) == 0) {
		// status
		parsedCmd.cmd = cmdSTATUS;
	} else if (strcmp(s, txtRCVMSGS) == 0) {
		parsedCmd.cmd = cmdRCVMSGS;
	} else if (strcmp(s, txtCLOSE_RELAY) == 0) {
		parsedCmd.cmd = cmdCLOSE_RELAY;
		parsedCmd.su = 1;
	} else if (strcmp(s, txtOPEN_RELAY) == 0) {
		parsedCmd.cmd = cmdOPEN_RELAY;
	} else if (strcmp(s, txtRED_ON) == 0) {
		parsedCmd.cmd = cmdRED_ON;
	} else if (strcmp(s, txtRED_OFF) == 0) {
		parsedCmd.cmd = cmdRED_OFF;
	} else if (strcmp(s, txtGREEN_ON) == 0) {
		parsedCmd.cmd = cmdGREEN_ON;
	} else if (strcmp(s, txtGREEN_OFF) == 0) {
		parsedCmd.cmd = cmdGREEN_OFF;
	} else if (strcmp(s, txtGET_VALUE) == 0) {
		parsedCmd.cmd = cmdGET_VALUE;
		parsedCmd.ops = 2;
	} else if (strcmp(s, txtSET_VALUE) == 0) {
		parsedCmd.cmd = cmdSET_VALUE;
		parsedCmd.ops = 2;
		parsedCmd.su = 1;
	} else if (strcmp(s, txtSU) == 0) {
		parsedCmd.cmd = cmdSU;
	} else if (strcmp(s, txtSU_OFF) == 0) {
		parsedCmd.cmd = cmdSU_OFF;
		parsedCmd.su = 1;
	} else if (strcmp(s, txtMSGS) == 0) {
		parsedCmd.cmd = cmdPRINT_MSG;
	} else if (strcmp(s, txtCONV_RAW) == 0) {
		parsedCmd.cmd = cmdCONV_RAW;
		parsedCmd.ops = 2;
	} else if (strcmp(s, txtCONV_FLOAT) == 0) {
		parsedCmd.cmd = cmdCONV_FLOAT;
		parsedCmd.ops = 2;
	} else if (strcmp(s, txtRESET) == 0) {
		parsedCmd.cmd = cmdRESET;
		parsedCmd.su = 1;
	}
	return &parsedCmd;
}

unsigned char parseTlm( char* s ) {
	if (strcmp(s, txtVOLTS) == 0) {
		return tlmVOLTS;
	} else if (strcmp(s, txtTEMP) == 0) {
		return tlmTEMP;
	} else if (strcmp(s, txtCURRENT) == 0) {
		return tlmCURRENT;
	}
	return NULL;
}