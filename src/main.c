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
//#include <timers.h> //@todo implement for runtime
//#include <math.h>
#include <usart.h>
#include <string.h>
//#include <xlcd.h>

#include "main.h"
#include "xlcd.h"
#include "status.h"
#include "ui.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "serial.h"

const unsigned char uiINPUT = 			"? ";
const unsigned char uiPROMPT_SU = 		"% ";
const unsigned char uiPROMPT = 			"> ";
const unsigned char txtCRLF = 			"\r\n";

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
volatile xQueueHandle qMsgs = NULL; // @todo put this in the correct thread
volatile xQueueHandle qEEWrite = NULL; // @todo put this in the correct thread
volatile xQueueHandle qSerialTx = NULL; // put this in the correct thread
volatile xQueueHandle qCommands = NULL;

void init( void ) {
	unsigned char i = 0;
	// Initialize clock
	OSCCON |= 0xF0;
	OSCTUNE |= 0x40; //enable pll for intosc

	// Initialize semaphores
	vSemaphoreCreateBinary(xADCSem);
	
	// Initialize queues
	qMsgs = xQueueCreate(10, sizeof(struct message *));
	qSerialTx = xQueueCreate(20, sizeof(unsigned char));
	vQueueAddToRegistry(qMsgs, "msgs");
	vQueueAddToRegistry(qSerialTx, "tx");
	
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
	TEMP_DISCHG_LIMIT = (int) 50 << 1;
	TEMP_CHARGE_LIMIT = (int) 35 << 1;
	REF_LOW_LIMIT = floatToInt(2.45); // V
	REF_HI_LIMIT = floatToInt(2.55); // V
	REF0_LOW_LIMIT = 0;
	REF0_HI_LIMIT = floatToInt(0.01); // V
	for (i = 0; i < MAX_CELLS; i++) {
		// default values (3.7V, 25C)
		//voltage[i] = floatToInt(3.7);
		cv[i].g = 1.0f;
		cv[i].b = VRef0;
		voltage[i] = 0;
		fVoltage[i] = 0.0;
		temp[i] = 25 << 1;
		fTemp[i] = 25.0;
		OVERVOLT_LIMIT[i] = floatToInt(4.2); // V
		UNDERVOLT_LIMIT[i] = floatToInt(3.0); // V
	}
	current = 0;
	msgs = 0;

	// Set up digital tristate buffers on I/O ports
	setupMUXInput();
	setupCurrentInput();
	setupVRefInput();
	
	setupLEDOutputs();
	setupRelayOutput();
	// 		PORTA:	0 - MUX Output
	//				1 - Current Sensor Output
	//				3 - Voltage Reference (+3.3V)
	//				2,4,5 - N.C.
	//				6,7 - Clock
	//TRISA = 0xCB;
	
	//		PORTB:	0 - LED0
	//				1 - LED1
	//				2 - LCD RS / CANTX
	//				3 - LCD RW / CANRX
	//				4 - LCD DB4
	//				5 - LCD DB5
	//				6 - LCD DB6
	//				7 - LCD DB7
	//TRISB = 0x08;
	
	//		PORTC:	0 - Address bit 0
	//				1 - Address bit 1
	//				2 - Address bit 2
	//				3 - I2C SCL clock
	//				4 - I2C SDA data
	//				5 - Relay Enable (active high)
	//				6 - Serial TX
	//				7 - Serial RX
	//TRISC = 0x98;
	
	// @todo ensure I2C and serial is set up by module
	
	// Set up interrupts
	//INTCON = 0x04; // Disable global interrupt, enables peripheral interrupt
	
	// Set up USART
	OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 0x67);
	//xSerial = xSerialPortInitMinimal(38400, (portBASE_TYPE)80);
	stdout = _H_USART;
}

void setupMUXInput( void ) {
	// @todo fix this hack
	unsigned char i;
	for (i = 0; i < MAX_CELLS; i++) {
		#if tris_INPUT == 1
			trisMUX_ADDR0 |= tris_INPUT << i;
		#else
			trisMUX_ADDR0 &= !(!tris_INPUT << i);
		#endif
	} 
}

void setupCurrentInput( void ) {
	trisCURRENT = tris_INPUT;
}

void setupVRefInput( void ) {
	trisVREF = tris_INPUT;
}

void setupLEDOutputs( void ) {
	trisLED0 = tris_OUTPUT;
	trisLED1 = tris_OUTPUT;
}

void setupRelayOutput( void ) {
	trisRELAY = tris_OUTPUT;
}

void main( void ) {
	init();
	#ifdef DEBUG_CONSOLE
		printf("\r\n\r\nInitialization...\r\n");
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

void vConfigureTimerForRunTimeStats( void ) {
	T0CON = 0b00000111;
	TMR0L = 0;
	TMR0H = 0;
	T0CONbits.TMR0ON = TRUE;
	//( configCPU_CLOCK_HZ / 10000UL) - 1UL;
}

unsigned int vGetRunTime( void ) {
	return ((int) TMR0H << 8) | (int) TMR0L;
}

unsigned char checkConversions( void ) {
	unsigned char i;
	if (floatToInt(intToFloat(43, 0)) != 43) {
		return -1;
	}
	if (intToFloat(floatToInt(3.712), 0) != 3.712) {
		return -2;
	}
	for (i = 0; i < MAX_CELLS; i++) {
		if (cv[i].g == 0) {
			return -3;
		}
		// @todo implement
		// check this (conv(0.1432, cv[i])
	}
	if (ci.g == 0) {
		return -4;
	}
	return 0;
}

/*char initLCD( void ) {
	char retVal = 0;
	OpenXLCD( FOUR_BIT & LINES_5X7 );
	while (!BusyXLCD());
	putrsXLCD("TEST!");
	return 0;
}*/

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
`	latRELAY = relayOPEN;
}

void closeRelay( void ) {
	latRELAY = relayCLOSED;
}

void setAddress( unsigned char address ) {
	if (address < MAX_CELLS) {
		pinMUX_ADDR0 |= (address & 0x07);
		// @todo recheck this line
	} else {
		setAddress(0);
		// @todo shouldn't get here (improper call), generate error?
	}
}

void setGreenLED() {
	latLED0 = ledON;
}

void setRedLED() {
	latLED1 = ledON;
}

void clearGreenLED() {
	latLED0 = ledOFF;
}

void clearRedLED() {
	latLED1 = ledOFF;
}

void toggleGreenLED() {
	latLED1 = !latLED1;
}

void toggleRedLED() {
	latLED0 = !latLED0;
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
	printf("STATUS_REG=%#0x0\tERROR_REG=%#0x0\tEEPROM_OFFSET=%#0x0\r\n", STATUS_REG, ERROR_REG);
	printf("VRef0=%0.0f\tVRef=%0.0f\r\n", VRef0, VRef);
	printf("REF_LOW_LIM=%#0x0\tREF_HI_LIM=%#0x0\r\nREF0_LOW_LIM=%#0x0\tREF0_HI_LIM=%#x\r\n",
			REF_LOW_LIMIT, REF_HI_LIMIT, REF0_LOW_LIMIT, REF0_HI_LIMIT);
	printf("ADC_MUX=%#0x0\tI_THR=%d\tDISCHG_LIM=%d\tCHG_LIM=%d\r\n", 
			ADC_MUX, CURRENT_THRES, DISCHG_RATE_LIMIT, CHARGE_RATE_LIMIT);
	printf("I=%d\tci=%0.0f,%0.0f\r\n", current, ci.g, ci.b);
	for (k = 0; k < MAX_CELLS; k++) {
		printf("cell[%u]: v=%0.0f(%#x)\tT=%0.0f(%#x)\t\UV_LIM=%u\tOV_LIM=%u\tchg=%d\tcv=%0.0f,%0.0f\r\n", 
			k, fVoltage[k], voltage[k], fTemp[k], temp[k],UNDERVOLT_LIMIT[k],
			OVERVOLT_LIMIT[k], charge[k], cv[k].g, cv[k].b);
	}
	printf("msgs=%u", msgs);
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

void tskCheck( void *params ) {
	message msg;
	unsigned int fVal;
	unsigned char suf;
	#ifdef DEBUG_CONSOLE
		printf("Starting Checking task...\r\n");
	#endif
	while (1) {
		if (xQueueReceive(qMsgs, &msg, 100) == pdPASS) {
			if (STATUS_REGbits.sMSG) {
				command c;
				c.su = 0;
				c.ops = 0;
				c.cmd = cmdPRINT_MSG;
				xQueueSend(qCommands, &c, 0);
			}
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
		} else {
			// @todo no messages in x time? bad sign. fail out
		}
		ClrWdt();
	}		
}

void tskUI( void *params ) {
	static unsigned char *prompt;
	unsigned char uiState = uiState_RESET;
	command *cmd;
	message msg;
	unsigned char buffer[uiBUFFER_SIZE];
	unsigned char *buf;
	unsigned int i = 0;	
	#ifdef DEBUG_CONSOLE
		printf("Starting UI...");
	#endif
	buf = (void *) &buffer;
	prompt = (void *) &uiPROMPT;
	for (i = 0; i < uiBUFFER_SIZE; i++) {
		buffer[i] = 0; // @todo clean this up
	}
	if (qCommands == NULL) {
		qCommands = xQueueCreate(uiCOMMAND_QSIZE, sizeof(command));
		vQueueAddToRegistry(qCommands, "cmd");
	}
	#ifdef DEBUG_CONSOLE
		printf("done!\r\n");
	#endif
	while (1) {
		if (xQueueReceive(qCommands, cmd, 0) == pdPASS) {
			if (cmd->su & !STATUS_REGbits.sSU) {
				printf("su required");
				uiState = uiState_SYSTEM_PROMPT;
			} else {
				switch(cmd->cmd) {
					case cmdSTATUS:
						printDump();
						break;
					case cmdRCVMSGS:
						STATUS_REGbits.sMSG = !STATUS_REGbits.sMSG;
						break;
					case cmdPRINT_MSG:
						printf("%u - ", msgs);
						if (cmd->ops == 1) {
							printf("c%u @ ", msg.cell);
						}
						switch (msg.type) {
							case msgVOLTS:
								printf("%f V", fVoltage[msg.cell]);
								break;
							case msgCURRENT:
								printf("%u mA", current);
								break;
							case msgTEMP:
								printf("%f %cC", fTemp[msg.cell], 235);
								break;
							default:
								break;		
						}
						printf(txtCRLF);
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
						switch (cmd->ops) {
							case 2:
								printf("\r\nget what%s", uiINPUT);
								uiState = uiState_WAIT_FOR_INPUT1;
								break;
							case 1:
								printf("\r\nfor which cell%s", uiINPUT);
								uiState = uiState_WAIT_FOR_INPUT2;
								break;
							default:
								uiState = uiState_RESET;
								break;
						}
						break;
					case cmdCONV_RAW:
						switch (cmd->ops) {
							case 2:
								printf("\r\nconvert what%s", uiINPUT);
								uiState = uiState_WAIT_FOR_INPUT1;
								break;
							case 1:
								printf("\r\nfor which cell%s", uiINPUT);
								uiState = uiState_WAIT_FOR_INPUT2;
								break;
							default:
								uiState = uiState_RESET;
								break;
						}
						break;
					case cmdCONV_FLOAT:
						switch (cmd->ops) {
							case 2:
								printf("\r\nconvert what%s", uiINPUT);
								uiState = uiState_WAIT_FOR_INPUT1;
								break;
							case 1:
								printf("\r\nfor which cell%s", uiINPUT);
								uiState = uiState_WAIT_FOR_INPUT2;
								break;
							default:
								uiState = uiState_RESET;
								break;
						}
						break;
					case cmdSU:
						prompt = (void *) &uiPROMPT_SU;
						STATUS_REGbits.sSU = 1;
						break;
					case cmdSU_OFF:
						prompt = (void *) &uiPROMPT;
						STATUS_REGbits.sSU = 0;
						break;
					case cmdPRINT_MSG_LCD:
						// @todo implement
						break;	
					default:
						printf("Unknown command!");
						uiState = uiState_SYSTEM_PROMPT;
						break;
				}
			}
		} else {
			switch(uiState) {
				case uiState_RESET:
					// wait for reception of anything
					while (!DataRdyUSART());
					*buf = getcUSART();
					uiState |= (*buf != 0 && *buf != 0xff); //0x01 if not garbage
					break;
				case uiState_SYSTEM_PROMPT:
					// display prompt
					printf("\r\n%s", prompt);
					uiState = uiState_WAIT_FOR_CMD;
					break;
				case uiState_WAIT_FOR_CMD:
					// wait / receive command
					while (!DataRdyUSART());
					*buf = getcUSART();
					printf("%c", *buf);
					if (*buf == 0x0D) {
						*buf = 0x00;
						cmd = parseCmd(&buffer);
						if (cmd->cmd != 0)
							while (xQueueSend(qCommands, cmd, 20) != pdPASS);
						else if ((int) buf != (int) &buffer) 
							printf("Unknown command!");
						buf = (void *) &buffer;
						uiState = uiState_SYSTEM_PROMPT;
					} else if (*buf == 0x08) {
						// backspace
						printf(" %c", *buf); // erasure
						*buf = 0x00; // erasure in buffer
					} else if ((int) buf - (int) &buffer >= uiBUFFER_SIZE) {
						// buffer overflow; reset pointer and empty array
						buf = (void *) &buffer;
						for (i = 0; i < uiBUFFER_SIZE; i++) {
							buffer[i] = 0; // @todo clean this up
						}
						printf("\r\nInvalid command (too long)!\r\n");
						uiState = uiState_SYSTEM_PROMPT;
					} else {
						buf++;
					}
					break;			
				case uiState_WAIT_FOR_INPUT1:
					// wait for response to get / conv what?
					while (!DataRdyUSART());
					buf = (void *) &buffer;
					*buf = getcUSART();
					printf("%c", *buf);
					if (*buf == 0x0D) {
						*buf = 0x00;
						*buf = parseTlm(&buffer);
						switch (*buf) {
							case tlmVOLTS:
								msg.type = msgVOLTS;
								cmd->ops--;
								break;
							case tlmCURRENT:
								msg.type = msgCURRENT;
								cmd->ops -= 2;
								break;
							case tlmTEMP:
								msg.type = msgTEMP;
								cmd->ops--;
								break;
							default:
								break;
						}
						while (xQueueSend(qCommands, cmd, 20) != pdPASS);
						buf = (void *) &buffer;
						uiState = uiState_WAIT_FOR_INPUT2;
					} else if (*buf == 0x08) {
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
					break;
				case uiState_WAIT_FOR_INPUT2:
					// get / conv what?
					while (!DataRdyUSART());
					buf = (void *) &buffer;
					*buf = getcUSART();
					if (atoi(buf) < MAX_CELLS) {
						msg.cell = atoi(buf);
					} else {
						printf("\r\nout of range!\r\n");
						uiState = uiState_SYSTEM_PROMPT;
					}
				default:
					uiState = uiState_RESET;
					break;
			}
		}
	}
}
