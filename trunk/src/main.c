/*
 *
 * Battery Management System
 * 
 * Version 0.1
 *
 * Queen's University
 * Electrical and Computing Engineering
 * http://www.ece.queensu.ca/
 *
 * Authors:
 * Kevin McHale (k.mchale@me.com)
 *
 * Changelog:
 * ...is taken care of by SVN.
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
#include <math.h>
#include <i2c.h>
#include <stdlib.h>
#include "status.h"

#define TRUE	1
#define FALSE	0
#define ADC_RESOLUTION		1024 // 10 bits
#define	VDD					5000 // mV
#define VREF_DEFAULT		3300 // mV
#define MAX_CELLS 			4
#define MAX_TEMP_FAILS		2
#define TEMP_CONFIG_REG		0x00
#define ADC_CONFIG			ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_16_TAD
#define ADC_VREF			ADC_CH3 & ADC_INT_ON & ADC_REF_VDD_VSS
#define ADC_CHMUX			ADC_CH0
#define ADC_CHCURRENT		ADC_CH1
#define ADC_MUX_DELAY		1
/*#define ADC_CURRENT			ADC_CH1 & ADC_INT_ON & ADC_REF_VREFPLUS_VSS
 #define ADC_MUX				ADC_CH0 & ADC_INT_ON & ADC_REF_VREFPLUS_VSS
 #define ADC_CURRENT_NOREF	ADC_CH0 & ADC_INT_ON & ADC_REF_VDD_VSS
 #define ADC_MUX_NOREF		ADC_CH0 & ADC_INT_ON & ADC_REF_VDD_VSS*/

void init(void);
void main(void);
void setAddress(unsigned char address);
void toggleRedLED(void);
void setRedLED(void);
void clearRedLED(void);
void toggleGreenLED(void);
void setGreenLED(void);
void clearGreenLED(void);
void increaseCount(void);
void openRelay(void);
void closeRelay(void);
void checkVoltage(unsigned int x, unsigned int address);
void checkCurrent(unsigned int x);
void checkTemp(signed int x);
void interruptHandlerHigh (void);
void writeWord(unsigned char address, unsigned int x);
unsigned int readWord(unsigned char address);
void eepromWrite(unsigned char address, unsigned char x);
unsigned char eepromRead(unsigned char address);
void initEEPROM(void);
signed int readTemp(unsigned char address);
void initTemp(unsigned char address);
void reset(void);
void failTemp(unsigned char address);

unsigned int VRef = VDD;
unsigned char EEPROM_OFFSET = 0x00;
unsigned char CURRENT_CELL = 0;
unsigned char ADC_CURRENT = 0;
unsigned char ADC_MUX = 0;
unsigned int voltage[MAX_CELLS];
unsigned int current = 0;
unsigned int MAX_TEMP_FAIL = 2; // failures tolerated
signed int temp[MAX_CELLS];
unsigned char STATUS_REG = 0x00;
unsigned char ERROR_REGL = 0x00;
unsigned char ERROR_REGH = 0x0;
unsigned char tempFailCount = 0;
unsigned char tempEnable = 0x00;

// Default set points (saved to EEPROM if erased)
//unsigned int OVERVOLT_LIMIT = 0x035C; // Overvoltage setpoint
//unsigned int UNDERVOLT_LIMIT = 0x0266; // Undervoltage setpoint
unsigned int OVERVOLT_LIMIT[MAX_CELLS] = {0x023E, 0x0265, 0x0259, 0x0257}; //@todo only works for first amp
unsigned int UNDERVOLT_LIMIT[MAX_CELLS] = {0x013F, 0x0155, 0x0150, 0x014F}; //@todo enter new values into spreadsheet; these are broken
unsigned int DISCHG_RATE_LIMIT = 0x02AA; // Overcurrent (discharge) setpoint (mA)
unsigned int CHARGE_RATE_LIMIT = 0x03C1; // Overcurrent (charge) setpoint (mA)
unsigned int CURRENT_THRES	= 10; // Threshold of charge / discharge (mA) // @todo implement
unsigned int TEMP_DISCHG_LIMIT = 0x2A00; // Max discharge temperature
unsigned int TEMP_CHARGE_LIMIT = 0x1E00; // Max charge temperature
unsigned int REF_LOW_LIMIT = 0x0298; // Reference too low (mV): ~3247 mV
unsigned int REF_HI_LIMIT = 0x02AD; // Reference too high (mV): ~3349 mV

// Voltage input stage calibration factors
//float gv[MAX_CELLS]; // gain (V/V)
//int bv[MAX_CELLS]; // bias (mV)
// Current input calibration factors
//unsigned int gi = 200; // Sensitivity (mA/V)
//unsigned int bi = 2500; // Q-point (mV)

void init(void) {
	unsigned char i;
	// Initialize clock
	// OSCCON<6:4>=111 or 110
	// FOSC3:FOSC0=1001 or 1000
	// 0b011100?0
	OSCCON |= 0xF2;
	OSCTUNE |= 0x40; //enable pll for intosc
	//SSPADD = 0x13;
	
	// Initialize global variables
	CURRENT_CELL = 0;
	EEPROM_OFFSET = 0x00;
	STATUS_REG = 0x00;
	ERROR_REGL = 0x00;
	ERROR_REGH = 0x00;
	current = 0;
	VRef = VDD;
	
	// Set up digital I/O ports
	
	// 		PORTA:	0 - MUX Output
	//				1 - Current Sensor Output
	//				3 - Voltage Reference (+3.3V)
	//				2,4,5 - N.C.
	//				6,7 - Clock
	TRISA = 0xCB;
	
	//		PORTB:	0 - LED0
	//				1 - LED1
	//				2 - CANTX
	//				3 - CANRX
	//				4-7 - N.C.
	TRISB = 0x08;
	
	//		PORTC:	0 - Address bit 0
	//				1 - Address bit 1
	//				2 - Address bit 2
	//				3 - I2C SCL clock
	//				4 - I2C SDA data
	//				5 - Relay Enable (active high)
	//				6 - Serial TX
	//				7 - Serial RX
	TRISC = 0x80;
	
	// Set up interrupts
	/*INTCON = 0x04; // Disable global interrupt, enables peripheral interrupt
	 
	 // Set up I2C bus
	 /*SSPSTAT = 0x80; // Disable SMBus & slew rate control*/
	SSPADD = 0xff;
	 // Initialize default cal factor arrays
	 
	for (i = 0; i < MAX_CELLS; i++) {
		voltage[i] = 0;
		temp[i] = 0;
	}
}

void initEEPROM(void) {
	unsigned char i;
	// Set up EEPROM
	EEPROM_OFFSET = eepromRead(0x00);
	if (EEPROM_OFFSET == 0xFF || EEPROM_OFFSET == 0x00) {
		// Setting up EEPROM for the first time
		EEPROM_OFFSET = 0x02;
		for (i = 0; i < MAX_CELLS; i++) {
			writeWord(EEPROM_OFFSET, UNDERVOLT_LIMIT[i]);
			EEPROM_OFFSET += 2;
			writeWord(EEPROM_OFFSET, OVERVOLT_LIMIT[i]);
			EEPROM_OFFSET += 2;
		}
		writeWord(EEPROM_OFFSET, CHARGE_RATE_LIMIT);
		EEPROM_OFFSET += 2;
		writeWord(EEPROM_OFFSET, DISCHG_RATE_LIMIT);
		EEPROM_OFFSET += 2;
		writeWord(EEPROM_OFFSET, CURRENT_THRES);
		EEPROM_OFFSET += 2;
		writeWord(EEPROM_OFFSET, TEMP_DISCHG_LIMIT);
		EEPROM_OFFSET += 2;
		writeWord(EEPROM_OFFSET, TEMP_CHARGE_LIMIT);
		EEPROM_OFFSET += 2;
		writeWord(EEPROM_OFFSET, REF_LOW_LIMIT);
		EEPROM_OFFSET += 2;
		writeWord(EEPROM_OFFSET, REF_HI_LIMIT);
		EEPROM_OFFSET += 2;
		eepromWrite(0x00, EEPROM_OFFSET);
	} else {
		EEPROM_OFFSET = 0x02;
		for (i = 0; i < MAX_CELLS; i++) {
			UNDERVOLT_LIMIT[i] = readWord(EEPROM_OFFSET);
			EEPROM_OFFSET += 2;
			OVERVOLT_LIMIT[i] = readWord(EEPROM_OFFSET);
			EEPROM_OFFSET += 2;
		}
		CHARGE_RATE_LIMIT = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		DISCHG_RATE_LIMIT = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		CURRENT_THRES = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		TEMP_DISCHG_LIMIT = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		TEMP_CHARGE_LIMIT = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		REF_LOW_LIMIT = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		REF_HI_LIMIT = readWord(EEPROM_OFFSET);
		EEPROM_OFFSET += 2;
		if (EEPROM_OFFSET != readWord(0x00)) {
			// EEPROM problem. existing config doesn't match code version
			writeWord(0x00, 0xFF); // reset EEPROM
			initEEPROM();
		}
	}
}

void main(void) {
	unsigned char i ;
	init();
	setRedLED();
	
	// Open ADC port looking at VRef+ pin
	OpenADC(ADC_CONFIG, ADC_VREF, ADC_15ANA);
	OpenI2C(MASTER, SLEW_OFF);
	SSPADD = 0x28;
	ConvertADC();
	initEEPROM();
	/*for (i = 0; i < MAX_CELLS; i++) {
		initTemp(i);
	}*/
	while (BusyADC()); // wait for ADC to complete
	if (ReadADC() > REF_LOW_LIMIT && ReadADC() < REF_HI_LIMIT) {
		// Reference is good to go; use it.
		ADC_MUX = ADC_CH0 & ADC_INT_ON & ADC_REF_VREFPLUS_VSS;
		VRef = VREF_DEFAULT;
		clearRedLED();
	} else {
		// Reference is broken, use VDD
		ADC_MUX = ADC_CH0 & ADC_INT_ON & ADC_REF_VDD_VSS;
		STATUS |= SOFT_FAIL;
		ERROR_REGH |= REF_FAIL;
		VRef = VDD;
		openRelay();
		//while(TRUE); // @todo new calibrated setpoints needed (store them in EEPROM)
	}
	setGreenLED();
	while (TRUE) {
		SetChanADC(ADC_CHCURRENT);
		Delay10TCYx(ADC_MUX_DELAY); // @todo do we need this?
		ConvertADC();
		while (BusyADC()) {
			temp[CURRENT_CELL] = readTemp(CURRENT_CELL);
			//checkTemp(temp[CURRENT_CELL]);
		}
		current = ReadADC();
		SetChanADC(ADC_CHMUX);
		/*checkCurrent(current); // @debug*/
		Delay10TCYx(ADC_MUX_DELAY); // @todo do we need this?
		while (BusyADC()) {
			//temp[CURRENT_CELL] = readTemp(CURRENT_CELL);
			//checkTemp(temp[CURRENT_CELL]);
		}
		voltage[CURRENT_CELL] = ReadADC();
		checkVoltage(ReadADC(), CURRENT_CELL);
		increaseCount();
		setAddress(CURRENT_CELL);
	}
	// ClearWDT();
	//_asm CLEARWDT _endasm
	// @todo watchdog and clear it and stuff
	// NOP sled:
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	// should never get here
	reset();
	setRedLED();
	clearGreenLED();
	// after main method completes, helper file calls it again
}

void initTemp(unsigned char address) {
	char error = 0;
	IdleI2C();
	StartI2C();
	while (SSPCON2bits.SEN);
	IdleI2C();
	error |= WriteI2C(0x90 | (address << 1));
	IdleI2C();
	error |= WriteI2C(0x01); // select CONFIG register
	IdleI2C();
	error |= WriteI2C(TEMP_CONFIG_REG);
	if (error) {
		failTemp(address);
	} else {
		tempEnable |= 1 << address;
	}
	IdleI2C();
	StopI2C();
	while (SSPCON2bits.PEN);
}

signed int readTemp(unsigned char address) {
	unsigned int result = 0;
	/*if (!(tempEnable & (1 << address)) >> address) { // check if enabled, if it is, don't select Ta reg again
		//initTemp(address);
		tempEnable |= 1 << address;
	}*/
	if ((1 << address) & tempEnable == (1 << address)) {
		tempEnable |= 1 << address;
	}
	IdleI2C();	// make sure bus is idle
	StartI2C();	// initiate START bus condition
	IdleI2C();
	if (WriteI2C(0x90 | (address  << 1) | 1) == 0) {
		IdleI2C();
		result = ReadI2C();
		result <<= 8;
		IdleI2C();
		AckI2C();
		IdleI2C();
		result |= ReadI2C();
		IdleI2C();
		NotAckI2C();
		IdleI2C();
	} else {
		failTemp(address);
	}
	StopI2C();
	/*
	IdleI2C();
	result = ReadI2C();
	//result &= 0x7f; // mask sign bit?
	result <<= 8;
	IdleI2C();
	AckI2C();
	IdleI2C();
	result |= ReadI2C();
	IdleI2C();
	//error |= getsI2C(&result, 2);
	//error |= getsI2C(result, 1);
	//getsI2C(*data, 2); // grab length bytes from bus	result = ReadI2C() << 8;	AckI2C();	IdleI2C();	result |= ReadI2C();	NotAckI2C(); // send EOD bus condition
	NotAckI2C();
	IdleI2C();
	StopI2C();*/
	return result;
}

void failTemp(unsigned char address) {
	// @todo add *which* temp failed, change EN status
	tempEnable &= ~(1 << address);
	tempFailCount++;
	if (tempFailCount > MAX_TEMP_FAILS) {
		ERROR_REGH |= TEMP_FAIL;
		STATUS_REG |= FAIL | SHDN;
		openRelay();
	}
}

void writeByteI2C(unsigned char controlByte, unsigned char address, unsigned char data) {
	IdleI2C(); // make sure bus is idle
	StartI2C(); // initiate START bus condition
	while ( SSPCON2bits.SEN ) {
		// wtf @todo
	} // wait until start condition is over 
	WriteI2C( controlByte ); // write 1 byte - R/W bit should be 0
	IdleI2C(); // ensure module is idle
	WriteI2C(0%1001 << 3 | address); // write address byte to EEPROM
	IdleI2C(); // ensure module is idle
	WriteI2C ( data );// Write data byte to EEPROM
	IdleI2C();                          // ensure module is idle
	StopI2C();                          // send STOP condition
	while ( SSPCON2bits.PEN ){
		// wtf @todo
	}          // wait until stop condition is over 
	while (EEAckPolling(controlByte)) {
		// wtf @todo
	}  //Wait for write cycle to complete
}

float intToFloat(int x, unsigned int shift) {
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

int floatToInt(auto float x) {
	//return x * ADC_RESOLUTION / intToFloat(Vref);
	return (int) x;
}

void checkVoltage(unsigned int x, unsigned int address) {
	if (x > OVERVOLT_LIMIT[address]) {
		openRelay();
		ERROR_REGL |= OVERVOLT;
		STATUS_REG |= FAIL;
		setRedLED();
	} else if (x < UNDERVOLT_LIMIT[address]) {
		openRelay();
		ERROR_REGL |= UNDERVOLT;
		STATUS_REG |= FAIL;
		setRedLED();
	}
}

void checkCurrent(unsigned int x) {
	if (x < CHARGE_RATE_LIMIT) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REGL |= OVERCURRENTIN;
		setRedLED();
	} else if (x > DISCHG_RATE_LIMIT) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REGL |= OVERCURRENTOUT;
		setRedLED();
	}
}

void checkTemp(signed int x) {
	/*This function assumes that the discharge limit is greater than the charge
	 limit which should be the case for all battery chemistries.*/
	if (x > TEMP_DISCHG_LIMIT) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REGL |= OVERTEMP;
		setRedLED();
	} else if (x > TEMP_CHARGE_LIMIT && STATUS_REG & CHARGING) {
		openRelay();
		STATUS_REG |= FAIL;
		ERROR_REGL |= OVERTEMP;
		setRedLED();
	}
}

void openRelay(void) {
	`	//_asm BCF PORTB, 0, 0 _endasm	
	PORTC &= 0%11011111;
}

void closeRelay(void) {
	//_asm BSF PORTB, 0, 0 _endasm	
	PORTC |= 0%00100000;
}

void setAddress(unsigned char address) {
	if (address < MAX_CELLS) {
		PORTC = (PORTC & 0x07) | address;
	} else {
		setAddress(0);
		// @todo shouldn't get here (improper call), generate error?
	}
}

void increaseCount(void) {
	if (CURRENT_CELL < MAX_CELLS - 1) {
		CURRENT_CELL++;
	} else {
		// wrap
		CURRENT_CELL = 0;
	}
}

void setGreenLED() {
	PORTB |= 0x01;
}

void setRedLED() {
	PORTB |= 0x02;
}

void clearGreenLED() {	
	PORTB &= 0xFE;
}

void clearRedLED() {
	PORTB &= 0xFD;
}

void toggleGreenLED() {
	if (PORTB & 0x01) {
		clearGreenLED();	
	} else {
		setGreenLED();
	}
}

void toggleRedLED() {
	if ((PORTB & 0x02) >> 1) {
		clearRedLED();
	} else {
		setRedLED();
	}
}

void writeWord(unsigned char address, unsigned int x) {
	// @todo make this more robust -- not only taking ints, use sizeof
	unsigned char *ptr = &x;
	char i;
	for (i = 0; i < sizeof(x); i++) {
		eepromWrite(address++,*(ptr++));
	}
	/*
	 eepromWrite(address, (unsigned char)(0x00ff & x));
	 eepromWrite(address, (unsigned char)((0xff00 & x) >> 8));*/
}

unsigned int readWord(unsigned char address) {
	unsigned int result;
	unsigned char *ptr = &result;
	char i;
	for (i = 0; i < 2; i++) {
		*(ptr++)=eepromRead(address++);
	}
	return result;
}

unsigned char eepromRead(unsigned char address) {
	EEADRH = 0;
 	EEADR = address;
	EECON1 |= 0x01; // read
	while (EECON1bits.RD == 1);
	return EEDATA;
}

void eepromWrite(unsigned char address, unsigned char x) {
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
		ERROR_REGH |= READ_FAIL;
		STATUS_REG |= SOFT_FAIL;
	}
	EECON1bits.WREN = 0; // disable writes
	INTCONbits.GIE = 0x01 & oldGIE;
}

void reset(void) {
	STATUS_REG = 0x00;
	ERROR_REGL = 0x00;
	ERROR_REGH = 0x00;
	tempFailCount = 0;
	tempEnable = 0x00;
	openRelay();
}