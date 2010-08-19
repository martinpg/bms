/*
 *
 * Battery Management System
 * 
 * Version 0.1
 *
 * Author:
 * Kevin McHale (k.mchale@me.com)
 *
 * Changelog:
 * ...is taken care of by SVN (for now)
 *
 */

// To-dos
//
// @todo implement EEPROM flushing
// @todo refactor code to be more than 1 file (main.c)
// @todo fix EEPROM implementation

#include <p18f2680.h>
#include <adc.h>
#include <delays.h>
#include <i2c.h>
#include <stdio.h>
//#include <math.h>
#include <usart.h>
#include <string.h>
#include <xlcd.h>

#include "status.h"
#include "main.h"
#include "ui.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "serial.h"

#pragma config OSC = IRCIO67 // internal oscillator
#pragma config FCMEN = OFF
#pragma config IESO = OFF
//#pragma config MCLRE = OFF // MCLR disabled
#pragma config PBADEN = OFF
#pragma config LVP = OFF
#pragma config XINST = ON // extended mode

float VRef;
float VRef0;
volatile unsigned char EEPROM_OFFSET;
volatile unsigned char eeFlush;
unsigned char ADC_MUX;
volatile unsigned int voltage[MAX_CELLS];
volatile unsigned int charge[MAX_CELLS]; // @todo save charge to EEPROM
volatile signed int current;
volatile float fVoltage[MAX_CELLS];
volatile signed int temp[MAX_CELLS];
volatile float fTemp[MAX_CELLS];
volatile unsigned char tempEnabled;
unsigned int OVERVOLT_LIMIT[MAX_CELLS];
unsigned int UNDERVOLT_LIMIT[MAX_CELLS];
unsigned int DISCHG_RATE_LIMIT; // Overcurrent (discharge) setpoint (mA)
unsigned int CHARGE_RATE_LIMIT; // Overcurrent (charge) setpoint (mA)
unsigned int CURRENT_THRES; // Threshold of charge / discharge (mA) // @todo implement
unsigned int TEMP_DISCHG_LIMIT; // Max discharge temperature
unsigned int TEMP_CHARGE_LIMIT; // Max charge temperature
unsigned int REF_LOW_LIMIT; // Reference too low (mV): ~3247 mV
unsigned int REF_HI_LIMIT; // Reference too high (mV): ~3349 mV
unsigned int REF0_LOW_LIMIT;
unsigned int REF0_HI_LIMIT;
const unsigned char uiPROMPT_SU[] = "% ";
const unsigned char uiPROMPT[] = "> ";
const unsigned char uiINPUT[] = "? ";

unsigned int i, j;
volatile unsigned int msgs;

// AFE and current calibration factors
cal cv[MAX_CELLS];
cal ci;

xTaskHandle xCurrent = NULL;
xTaskHandle xVoltage = NULL;
xTaskHandle xCheck = NULL;
xTaskHandle xTemp = NULL;
xTaskHandle xUI = NULL;
xComPortHandle xSerial = NULL;
volatile xSemaphoreHandle xADCSem = NULL;
volatile xQueueHandle qMsgs = NULL;
volatile xQueueHandle qEEWrite = NULL;

char initLCD( void );
void init( void ) {
	// Initialize clock
	OSCCON |= 0xF0;
	OSCTUNE |= 0x40; //enable pll for intosc

	// Initialize semaphores
	vSemaphoreCreateBinary(xADCSem);
	
	// Initialize queues
	qMsgs = xQueueCreate(10, sizeof(struct message *));
	vQueueAddToRegistry(qMsgs, "msgs");
	
	// Initialize global variables
	VRef = VDD;
	VRef0 = 0;
	EEPROM_OFFSET = 0x00;
	eeFlush = 0x00;
	ADC_MUX = ADC_VREF_DEFAULT;
	STATUS_REG = 0x00;
	ERROR_REG = 0x0000;
	tempEnabled = 0x00;
	DISCHG_RATE_LIMIT = 3000; // mA
	CHARGE_RATE_LIMIT = 6000; // mA
	CURRENT_THRES = 20; // mA
	TEMP_DISCHG_LIMIT = (int) 50 << 8;
	TEMP_CHARGE_LIMIT = (int) 35 << 8;
	REF_LOW_LIMIT = floatToInt(2.45); // V
	REF_HI_LIMIT = floatToInt(2.55); // V
	REF0_LOW_LIMIT = 0;
	REF0_HI_LIMIT = floatToInt(0.01); // V
	for (i = 0; i < MAX_CELLS; i++) {
		// default values (3.7V, 25C)
		//voltage[i] = floatToInt(3.7);
		voltage[i] = 0;
		fVoltage[i] = 0.0;
		temp[i] = 25 << 1;
		fTemp[i] = 25.0;
		OVERVOLT_LIMIT[i] = floatToInt(4.2); // V
		UNDERVOLT_LIMIT[i] = floatToInt(3.0); // V
	}
	current = 0;
	msgs = 0;
	
	// Set up digital I/O ports
	
	// 		PORTA:	0 - MUX Output
	//				1 - Current Sensor Output
	//				3 - Voltage Reference (+3.3V)
	//				2,4,5 - N.C.
	//				6,7 - Clock
	TRISA = 0xCB;
	
	//		PORTB:	0 - LED0
	//				1 - LED1
	//				2 - LCD RS / CANTX
	//				3 - LCD RW / CANRX
	//				4 - LCD DB4
	//				5 - LCD DB5
	//				6 - LCD DB6
	//				7 - LCD DB7
	TRISB = 0x08;
	
	//		PORTC:	0 - Address bit 0
	//				1 - Address bit 1
	//				2 - Address bit 2
	//				3 - I2C SCL clocknnn
	//				4 - I2C SDA data
	//				5 - Relay Enable (active high)
	//				6 - Serial TX
	//				7 - Serial RX
	TRISC = 0x98;
	
	// Set up interrupts
	//INTCON = 0x04; // Disable global interrupt, enables peripheral interrupt
	
	// Set up USART
	OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 0x67);
	//xSerial = xSerialPortInitMinimal(38400, (portBASE_TYPE)80);
	stdout = _H_USART;
}

void main( void ) {
	init();
	#ifdef DEBUG_CONSOLE
		printf("Initialization...\r\n");
	#endif
	setRedLED();
	
	// Open ADC port looking at VRef+ pin
	#ifdef DEBUG_CONSOLE
		printf("Checking Vref.");
	#endif	
	OpenADC(ADC_CONFIG, ADC_VREF_DEFAULT, ADC_15ANA);
	OpenI2C(MASTER, SLEW_OFF);
	SSPADD = 0x4f;
	ConvertADC();
	//initEEPROM();
	while (BusyADC()) { // wait for ADC to complete
		#ifdef DEBUG_CONSOLE
			printf(".");
		#endif
	}
	//OpenADC(ADC_CONFIG, ADV_V
	if (ReadADC() > REF_LOW_LIMIT && ReadADC() < REF_HI_LIMIT) {
		// Reference is good to go; use it.
		ADC_MUX = ADC_VREF_EXT1;
		VRef = VREF_DEFAULT;
		ERROR_REG &= ~REF_FAIL;
		#ifdef DEBUG_CONSOLE
			printf("GOOD!");
		#endif	
		clearRedLED();
	} else {
		// Reference is broken, use VDD
		ADC_MUX = ADC_VREF_DEFAULT;
		ERROR_REG |= REF_FAIL;
		VRef = VDD;
		#ifdef DEBUG_CONSOLE
			printf("FAIL(%x)", ReadADC());
		#endif	
		clearRedLED();
		//openRelay();
		//while(TRUE); // @todo new calibrated setpoints needed (store them in EEPROM)
	}
	#ifdef DEBUG_CONSOLE
		printf("\r\n");
		printf("Loading OS...");
	#endif		
	//xTaskCreate( tskCheckCurrent, ( const char * const ) "ichk", configMINIMAL_STACK_SIZE, NULL, prioDEFAULT, &xCurrent );
	xTaskCreate( tskCheckTemps, ( const char * const ) "tchk", configMINIMAL_STACK_SIZE, NULL, prioDEFAULT, &xTemp );
	//xTaskCreate( tskCheckVolts, ( const char * const ) "vchk", configMINIMAL_STACK_SIZE, NULL, prioDEFAULT, &xVoltage );
	xTaskCreate( tskCheck, ( const char * const ) "chk", configMINIMAL_STACK_SIZE, NULL, prioDEFAULT, &xCheck );
	xTaskCreate( tskUI, ( const char * const ) "ui", configMINIMAL_STACK_SIZE, NULL, prioDEFAULT, &xUI );
	#ifdef DEBUG_CONSOLE
		printf("scheduler started!\r\n");
	#endif	
	vTaskStartScheduler();
	#ifdef
		printf("OS Stopped!");
	#endif	
	while(1){
		// should never reach here
	}
}

char initTemp( unsigned char address ) {
	char retVal = 0;
	unsigned char writeByte;
	{
		IdleI2C();
		StartI2C();
		retVal = WriteI2C(0x90 | (address << 1));
		retVal |= WriteI2C(0);
		StopI2C();
	}
	return retVal;
}

char initLCD( void ) {
	char retVal = 0;
	OpenXLCD( FOUR_BIT & LINES_5X7 );
	while (!BusyXLCD());
	putrsXLCD("TEST!");
	return 0;
}

signed int readTemp( unsigned char address ) {
	unsigned int result = 0;
	if (!(tempEnabled & (1 << address)) >> address) { // check if enabled, if it is, don't select Ta reg again
		tempEnabled |= (initTemp(address) == 0) << address;
	}
	//portENTER_CRITICAL();
	{
		IdleI2C();
		StartI2C();
		WriteI2C(0x91 | (address << 1)); // read @ address [1001]000, @todo check for ack returned (-2)?
		result = ReadI2C();
		result <<= 8;
		AckI2C();
		result |= ReadI2C();
		NotAckI2C();
		StopI2C();
	}
	//portEXIT_CRITICAL();
	return result;
}

void failTemp( unsigned char address ) {
	// @todo add *which* temp failed, change EN status
	tempEnabled &= ~(1 << address);
}

float intToFloat( int x, unsigned int shift ) {
	// shift is where the decimal occurs (ie 1 represents to the left of the ones place)
	/*float result = 0;
	 char i;
	 for (i = 0; i < 16; i++) {
	 result += pow(2, i - shift) * (x & (0x01 << i)); // debug
	 }
	 return result;*/
	float result = (float)(x >> shift);
	result += ((float)(x & ((shift << 1) - 1))) / pow(2, shift);
	return result;
}

int floatToInt ( float x ) {
	return (int) ( ADC_BITS * ( x - VRef0 ) )  / ( VRef - VRef0 );
}

void checkVoltage( unsigned int x, unsigned int address ) {
	if (x > OVERVOLT_LIMIT[address]) {
		openRelay();
		ERROR_REG |= OVERVOLT;
		STATUS_REG |= FAIL;
	} else if (x < UNDERVOLT_LIMIT[address]) {
		openRelay();
		ERROR_REG |= UNDERVOLT;
		STATUS_REG |= FAIL;
	}
}

void checkCurrent( unsigned int x ) {
	if (x < CHARGE_RATE_LIMIT) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REG |= OVERCURRENTIN;
	} else if (x > DISCHG_RATE_LIMIT) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REG |= OVERCURRENTOUT;
	}
}

void checkTemp( signed int x ) {
	/*This function assumes that the discharge limit is greater than the charge
	 limit which should be the case for all battery chemistries.*/
	if (x > TEMP_DISCHG_LIMIT) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REG |= OVERTEMP;
	} else if (x > TEMP_CHARGE_LIMIT && STATUS_REG & CHARGING) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REG |= OVERTEMP;
	}
}

void openRelay( void ) {
	`	//_asm BCF PORTB, 0, 0 _endasm	
	//PORTC &= 0%11011111;
	//PORTCbits.RC5 = 0;
	pinRELAY = relayOPEN;
}

void closeRelay( void ) {
	//_asm BSF PORTB, 0, 0 _endasm	
	//PORTC |= 0%00100000;
	//PORTCbits.RC5 = 1;
	pinRELAY = relayCLOSED;
}

void setAddress( unsigned char address ) {
	if (address < MAX_CELLS) {
		pinADDRESS0 = pinADDRESS0 & 0x07 | address;
		//PORTC = (PORTC & 0x07) | address;
	} else {
		setAddress(0);
		// @todo shouldn't get here (improper call), generate error?
	}
}

void setGreenLED() {
	//PORTB |= 0x01;
	pinLED0 = ledON;
}

void setRedLED() {
	//PORTB |= 0x02;
	pinLED1 = ledON;
}

void clearGreenLED() {	
	//PORTB &= 0xFE;
	pinLED0 = ledOFF;
}

void clearRedLED() {
	//PORTB &= 0xFD;
	pinLED1 = ledOFF;
}

void toggleGreenLED() {
	pinLED1 = ~pinLED1;
	/*if (PORTB & 0x01) {
		clearGreenLED();	
	} else {
		setGreenLED();
	}*/
}

void toggleRedLED() {
	/*if ((PORTB & 0x02) >> 1) {
		clearRedLED();
	} else {
		setRedLED();
	}*/
	pinLED0 = !pinLED0;
}

float conv( float x, cal c ) {
	return x * c.g + c.b;
}

void writeWord( unsigned char address, unsigned int x ) {
	// @todo make this more robust -- not only taking ints, use sizeof
	unsigned int *ptr = &x;
	unsigned char i;
	for (i = 0; i < sizeof(x); i++) {
		eepromWrite(address++,*(ptr++));
	}
	/*
	 eepromWrite(address, (unsigned char)(0x00ff & x));
	 eepromWrite(address, (unsigned char)((0xff00 & x) >> 8));*/
}

unsigned int readWord( unsigned char address ) {
	unsigned int result;
	unsigned int *ptr = &result;
	unsigned char i;
	for (i = 0; i < 2; i++) {
		*(ptr++)=eepromRead(address++);
	}
	return result;
}

unsigned char eepromRead( unsigned char address ) {
	EEADRH = 0;
 	EEADR = address;
	EECON1 |= 0x01; // read
	while (EECON1bits.RD == 1);
	return EEDATA;
}

void eepromWrite( unsigned char address, unsigned char x ) {
	char oldGIE = INTCONbits.GIE;
	EECON1bits.WRERR = 0; // clear error
	PIR2bits.EEIF = 0; // clear write done flag
	EEADRH = 0;
	EEADR = address;
	EEDATA = x;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	INTCONbits.GIE = 0;	// disable interrupts
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1; // write
	while (PIR2bits.EEIF == 0);// wait to complete
	if (EECON1bits.WRERR) {
		ERROR_REG |= READ_FAIL;
		STATUS_REG |= FAIL;
		// @todo serious failure; deal with it
	}
	EECON1bits.WREN = 0; // disable writes
	INTCONbits.GIE = 0x01 & oldGIE;
}

void printDump( void ) {
	unsigned char k;
	printf("STATUS_REG=%#x\tERROR_REG=%#x\tEEPROM_OFFSET=%#x\r\n", STATUS_REG, ERROR_REG);
	printf("VRef0=%f\tVRef=%f\r\n", VRef0, VRef);
	printf("REF_LOW_LIM=%#x\tREF_HI_LIM=%#x\tREF0_LOW_LIM=%#x\tREF0_HI_LIM=%#x",
			REF_LOW_LIMIT, REF_HI_LIMIT, REF0_LOW_LIMIT, REF0_HI_LIMIT);
	printf("ADC_MUX=%#x\tI_THR=%d\tDISCHG_LIM=%d\tCHG_LIM=%d\r\n", 
			ADC_MUX, CURRENT_THRES, DISCHG_RATE_LIMIT, CHARGE_RATE_LIMIT);
	printf("I=%d\tci=%f,%f\r\n", current, ci.g, ci.b);
	for (k = 0; k < MAX_CELLS; k++) {
		printf("cell[%u]: v=%f(%#x)\tT=%f(%#x)\t\UV_LIM=%u\tOV_LIM=%u\tchg=%d\tcv=%f,%f\r\n", 
			k, fVoltage[k], voltage[k], fTemp[k], temp[k],UNDERVOLT_LIMIT[k],
			OVERVOLT_LIMIT[k], charge[k], cv[k].g, cv[k].b);
	}
	printf("msgs=%d\ti=%d\tj=%d\r\n");
}

void printMsgs( void ) {
	printf("printMsgs unimplemented.\r\n");
}

void tskCheckVolts( void *params ) {
	static unsigned char cell = 0;
	static message msg;
	#ifdef DEBUG_CONSOLE
		printf("Starting Voltage ADC task...\r\n");
	#endif
	msg.type = msgVOLTS;
	while (1) {
		if (xSemaphoreTake(xADCSem, 0) == pdTRUE) {
			SetChanADC(ADC_CHMUX);
			Delay10TCYx(ADC_MUX_DELAY); // @todo neessary?
			ConvertADC();
			taskYIELD();
			while (BusyADC()); // @todo don't block
			voltage[cell] = ReadADC();
			xSemaphoreGive(xADCSem);
			msg.raw = voltage[cell];
			fVoltage[cell] = conv(intToFloat(voltage[cell], 0), cv[cell]);
			if (cell < MAX_CELLS) {
				cell++;
			} else {
				cell = 0;
			}
			setAddress(cell);
			xQueueSend(qMsgs, (void *) &msg, (portTickType) 10);
		}
	}
}

void tskCheckTemps( void *params ) {
	unsigned char cell = 0;
	message msg;
	#ifdef DEBUG_CONSOLE
		printf("Starting Temperatures I2C task...\r\n");
	#endif
	msg.type = msgTEMP;
	while (1) {
		temp[cell] = readTemp(cell);
		fTemp[cell] = ((float) temp[cell]) / 2;
		msg.cell = cell;
		msg.raw = temp[cell];
		xQueueSend(qMsgs, (void *) &msg, (portTickType) 10);
		if (cell < MAX_CELLS) {
			cell++;
		} else {
			cell = 0;
		}
	}
}
void tskCheckCurrent( void *params ) {
	static message msg;
	#ifdef DEBUG_CONSOLE
		printf("Starting Current ADC task...\r\n");
	#endif
	msg.type = msgCURRENT;
	while (1) {
		if (xSemaphoreTake(xADCSem, 0) == pdTRUE) {
			SetChanADC(ADC_CHCURRENT);
			Delay10TCYx(ADC_MUX_DELAY); // @todo necessary?
			ConvertADC();
			taskYIELD();
			while(BusyADC()); // @todo don't block
			current = ReadADC();
			xSemaphoreGive(xADCSem);
			current *= ci.g;
			current += ci.b;
			msg.raw = current;
			xQueueSend(qMsgs, (void *) &msg, (portTickType) 10);
		}
	}
}

void tskWriteEEPROM( void *params ) {
	static unsigned char writesPending = 0;
	static eeWord word;
	while (1) {
		writesPending = uxQueueMessagesWaiting(qEEWrite);
		if (writesPending >= 2 || eeFlush) {
			if (xQueueReceive(qEEWrite, &word, 0)) {
				writeWord(word.address, word.data);
			}
		}
	}
}

unsigned char parse(char* s) {
	if (strcmp(s, txtSTATUS) == 0) {
		return cmdSTATUS;
	} else if (strcmp(s, txtRCVMSGS) == 0) {
		return cmdRCVMSGS;
	} else if (strcmp(s, txtCLOSE_RELAY) == 0) {
		return cmdCLOSE_RELAY;
	} else if (strcmp(s, txtOPEN_RELAY) == 0) {
		return cmdOPEN_RELAY;
	} else if (strcmp(s, txtRED_ON) == 0) {
		return cmdRED_ON;
	} else if (strcmp(s, txtRED_OFF) == 0) {
		return cmdRED_OFF;
	} else if (strcmp(s, txtGREEN_ON) == 0) {
		return cmdGREEN_ON;
	} else if (strcmp(s, txtGREEN_OFF) == 0) {
		return cmdGREEN_OFF;
	} else if (strcmp(s, txtGET_VALUE) == 0) {
		return cmdGET_VALUE;
	} else if (strcmp(s, txtSU) == 0) {
		return cmdSU;
	} else if (strcmp(s, txtSU_OFF) == 0) {
		return cmdSU_OFF;
	} else if (strcmp(s, txtVOLTS) == 0) {
		return cmdVOLTS;
	} else if (strcmp(s, txtCURRENT) == 0) {
		return cmdCURRENT;
	}
	return -1; // shouldn't get here
}

void tskUI( void *params ) {
	static unsigned char *prompt;
	unsigned char uiState = 0x00;
	typedef struct {
		unsigned char cmd;
		unsigned op:1;
		unsigned su:1;
		unsigned :6;
	} command;
	command cmd = {NULL, NULL, NULL};
	unsigned char buffer[uiBUFFER_SIZE];
	unsigned char *buf;
	#ifdef DEBUG_CONSOLE
		printf("Starting UI...");
	#endif
	buf = (void *) &buffer;
	prompt = (void *) &uiPROMPT;
	for (i = 0; i < uiBUFFER_SIZE; i++) {
		buffer[i] = 0; // @todo lcean this up
	}
	#ifdef DEBUG_CONSOLE
		printf("done!\r\n");
	#endif
	while (1) {
		switch(uiState) {
			case 0x00:
				// wait for reception of anything
				while (!DataRdyUSART());
				*buf = getcUSART();
				uiState |= (*buf != 0 && *buf != 0xff);
				break;
			case 0x01:
				// display prompt
				printf("\r\n%s", prompt);
				uiState = 0x02;
				break;
			case 0x02:
				// wait / receive command
				while (!DataRdyUSART());
				*buf = getcUSART();
				printf("%c", *buf);
				if (*buf == 0x0D) {
					*buf = 0x00;
					cmd.cmd = parse(&buffer);
					buf = (void *) &buffer;
					uiState = 0x03;
				} else if (*buf == 0x09) {
					// backspace
					printf(" %c", *buf); // erasure?
				} else if ((int) buf - (int) &buffer >= uiBUFFER_SIZE) {
					// buffer overflow; reset pointer and empty array
					buf = (void *) &buffer;
					for (i = 0; i < uiBUFFER_SIZE; i++) {
						buffer[i] = 0; // @todo clean this up
					}
					printf("\r\nInvalid command!\r\n");
					uiState = 0x01;
				} else {
					buf++;
				}
				break;
			case 0x03:
				uiState = 0x01;
				// process command
				if ((cmd.cmd & uiSU_REQD) >> 4 & STATUS_REGbits.sSU) {
					break;
					cmd.cmd = cmdSU_OFF;
					// @todo generate error
				}
				switch(cmd.cmd) {
					case cmdSTATUS:
						printDump();
						break;
					case cmdRCVMSGS:
						printMsgs();
						if (STATUS_REG & ALLMSGS == ALLMSGS) {
							STATUS_REG &= ~ALLMSGS;
						} else {
							STATUS_REG |= ALLMSGS;
						}
						break;
					case cmdCLOSE_RELAY:
						closeRelay();
						break;
					case cmdOPEN_RELAY:
						openRelay();
						break;
					case cmdRED_ON:
						setRedLED();
						break;
					case cmdRED_OFF:
						clearRedLED();
						break;
					case cmdGREEN_ON:
						setGreenLED();
						break;
					case cmdGREEN_OFF:
						clearGreenLED();
						break;
					case cmdGET_VALUE:
						uiState = 0x04;
						break;
					case cmdSU:
						prompt = (void *) &uiPROMPT_SU;
						STATUS_REGbits.sSU = 1;
						break;
					case cmdSU_OFF:
						prompt = (void *) &uiPROMPT;
						STATUS_REGbits.sSU = 0;
						break;
					default:
						printf("Unknown command!\r\n");
						uiState = 0x01;
						break;
				}
				break;
			case 0x04:
				printf("\r\nget what%s", uiINPUT);
				while (!DataRdyUSART());
				buf = (void *) &buffer;
				*buf = getcUSART();
				printf("%c", *buf);
				if (*buf == 0x0D) {
					*buf = 0x00;
					cmd.cmd = parse(&buffer);
					buf = (void *) &buffer;
					uiState = 0x05;
				} else if (*buf == 0x09) {
					// backspace
					printf(" %c", *buf); // erasure?
				} else if ((int) buf - (int) &buffer >= uiBUFFER_SIZE) {
					// buffer overflow; reset pointer and empty array
					buf = (void *) &buffer;
					for (i = 0; i < uiBUFFER_SIZE; i++) {
						buffer[i] = 0; // @todo clean this up
					}
					printf("\r\nUninterpretable (too long)!\r\n");
				} else {
					buf++;
				}
				while (!DataRdyUSART()); // wait for response
				// @todo break out this routine
				break;
			case 0x05:
				switch (cmd.cmd) {
					case cmdVOLTS:
						printf("\r\n\which cell%s", uiINPUT);
						while (!DataRdyUSART());
						buf = (void *) &buffer;
						*buf = getcUSART();
						printf("\r\n%f V\r\n", fVoltage[atoi(buffer)]);
						break;
					case cmdCURRENT:
						printf("\r\n%f mA\r\n", fVoltage[atoi(buffer)]);
						break;
					case cmdTEMP:
						printf("\r\n\which cell%s", uiINPUT);
						while (!DataRdyUSART());
						buf = (void *) &buffer;
						*buf = getcUSART();
						printf("\r\n%f %cC\r\n", fVoltage[atoi(buffer)], 0xf8);
						break;
					case cmdMSGS:
						printf("\r\n%u\r\n", msgs);
						break;	
					default:
						printf("\r\nUninterpretable!\r\n");
						break;
				}
				break;	
			default:
				uiState = 0x01;
				break;
		}
	}
}

void tskCheck( void *params ) {
	message msg;
	unsigned int fVal;
	unsigned char suf;
	#ifdef DEBUG_CONSOLE
		printf("Starting Checking task...\r\n");
	#endif
	while (1) {
		if (xQueueReceive(qMsgs, &msg, 100) == pdPASS) {
			msgs++;
			switch(msg.type) {
				case msgVOLTS:
					checkVoltage(voltage[msg.cell], msg.cell);
					fVal = fVoltage[msg.cell];
					suf = 'V';
					break;
				case msgTEMP:
					checkTemp(temp[msg.cell]);
					fVal = fTemp[msg.cell];
					suf = 'C';
					break;
				case msgCURRENT:
					checkCurrent(current);
					suf = 'A';
					break;
				default:
					break;
			}
			if (STATUS_REG & ALLMSGS == ALLMSGS) {				
				printf("%u:%f%s(%#x), ", msg.cell, fVal, suf, msg.raw);
			}
		} else {
			// @todo no messages in x time? bad sign. fail out
		}
		ClrWdt();
	}		
}

