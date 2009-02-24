//
//
// Main Routine (main loop)
//
//
// @todo implement EEPROM flushing

#include <p18f2680.h>
#include <adc.h>
#include <delays.h>
#include <math.h>
#include <status.h>
#include <i2c.h>

void init(void);
void main(void);
void setAddress(unsigned char address);
void toggleLED(unsigned char led);
void setLED(unsigned char led);
void clearLED(unsigned char led);
void increaseCount(void);
void openRelay(void);
void closeRelay(void);
void checkVoltage(float x);
void interruptHandlerHigh (void);
void writeWord(unsigned char address, unsigned int x);
unsigned int readWord(unsigned char address);
void eepromWrite(unsigned char address, unsigned char x);
unsigned char eepromRead(unsigned char address);
unsigned char probeSensors(void);
//#define samples 	1; // Samples to keep

unsigned int MAX_CELLS = 8;
unsigned int VRef;
// const unsigned int samples = 1;
unsigned char EEPROM_START;
unsigned char CURRENT_CELL;
unsigned int voltage[8];
unsigned int temp[8];
unsigned int current;

// Default set points (saved to EEPROM if erased)
unsigned int OVERVOLT_LIMIT = 4.2; // Overvoltage setpoint (volts)
unsigned int UNDERVOLT_LIMIT = 3.0; // Undervoltage setpoint (volts)
unsigned int DISCHG_RATE_LIMIT = 6.0; // Overcurrent (discharge) setpoint (amps)
unsigned int CHARGE_RATE_LIMIT = 6.0; // Overcurrent (charge) setpoint (amps)
unsigned int TEMP_DISCHG_LIMIT = 85.0; // Max discharge temperature
unsigned int TEMP_CHARGE_LIMIT = 60.0; // Max charge temperature
unsigned int CURRENT_THRES	= 0.01; // Amps, Threshold of charge / discharge
unsigned int REF_LOW_LIMIT = 2.4; // Volts, Reference too low
unsigned int REF_HI_LIMIT = 2.6; // Volts, Reference too high

unsigned char STATUS_REG;
unsigned char ERROR_REGL;
unsigned char ERROR_REGH;

void init(void) {
	// Initialize clock
	OSCCON = 0x42;
	//OSCTUNE = 0x00; // set to 31 kHz

	// Initialize global variables
	CURRENT_CELL = 0;
	STATUS_REG = 0x01; // Operating (>1ma)
	current = 0;
	VRef = (int)(5.000); // Vdd

	// Set up digital I/O ports
	TRISA = 0xCB;
	TRISB &= 0xC9;
	TRISC &= 0x80;
	
	setAddress(0);

	// Set up interrupts
	INTCON = 0x4; // Disable global interrupt, enables peripheral interrupt
	INTCON2 = 0x00;
	RCONbits.IPEN = 0; // disable priority interrupts
	PIE1 = 0x00;
	PIE2 = 0x8C;
	PIE3 = 0xA0;
	//INTCONbits.GIEH = 1; // enable interrupts
	INTCONbits.GIE = 0;
	// @todo clear all interrupts?

	// Set up EEPROM
	EEPROM_START = eepromRead(0x00);
	
	if (EEPROM_START == 0xFF || EEPROM_START == 0) {
		// Setting up EEPROM for the first time
		writeWord(0x01, UNDERVOLT_LIMIT);
		writeWord(0x03, OVERVOLT_LIMIT);
		writeWord(0x05, CHARGE_RATE_LIMIT);
		writeWord(0x07, DISCHG_RATE_LIMIT);
		writeWord(0x09, TEMP_CHARGE_LIMIT);
		writeWord(0x0B, TEMP_DISCHG_LIMIT);
		writeWord(0x0D, CURRENT_THRES);
		writeWord(0x0F, REF_LOW_LIMIT);
		writeWord(0x11, REF_HI_LIMIT);
		EEPROM_START = 0x13;
		eepromWrite(0x00, EEPROM_START);
	} else {
		eepromWrite(0x00, EEPROM_START);
		UNDERVOLT_LIMIT = readWord(0x01);
		OVERVOLT_LIMIT = readWord(0x03);
		CHARGE_RATE_LIMIT = readWord(0x05);
		DISCHG_RATE_LIMIT = readWord(0x07);
		TEMP_CHARGE_LIMIT = readWord(0x09);
		TEMP_DISCHG_LIMIT = readWord(0x0B);
		CURRENT_THRES = readWord(0x0D);
		REF_LOW_LIMIT = readWord(0x0F);
		REF_HI_LIMIT = readWord(0x11);
	}

	// Enable High / Low voltage detect (for Vdd)
	//HLVDCON = 0x3E; // 4.48V - 4.69V brownout, interrupt enabled

	// enable communications busses, store last reset (RCON), etc
}

void main(void) {
	init();
	// Open ADC port looking at VRef+ pin
	OpenADC(ADC_FOSC_8 & 
			ADC_RIGHT_JUST &
			ADC_16_TAD, 
			ADC_CH3 &
			ADC_INT_ON &
			ADC_REF_VDD_VSS, 
			ADC_15ANA);
	OpenI2C(MASTER, SLEW_OFF);
	Delay10TCYx(5); // delay 50 cycles to do A/D
	ConvertADC();
	setLED(1);
	while (BusyADC()); // wait for ADC to complete
	clearLED(1);
	if (ReadADC() > REF_LOW_LIMIT && ReadADC() < REF_HI_LIMIT) {
		// Reference is good to go; use it.
		OpenADC(ADC_FOSC_8 &
				ADC_RIGHT_JUST &
				ADC_16_TAD,
				ADC_CH0 &
				ADC_INT_ON &
				ADC_REF_VREFPLUS_VSS,
				ADC_15ANA);
			Delay10TCYx(5); // delay 50 cyc
			VRef = 2.5; //int((2.5)*2^16);
		// @todo change register to operating when it actually starts to
	} else {
		// Reference is broken, use VDD
		STATUS |= SOFT_FAIL;
		ERROR_REGH |= REF_FAIL;
		VRef = 5.0;
	}

	while (1) {
		toggleLED(0); // 1 cycle
		/*StartI2C();
		if (WriteI2C((0b1001 << 3) + (CURRENT_CELL << 1) | 1) == -1) {
			ERROR_REGH |= TEMP_FAIL;
		}
		temp[CURRENT_CELL] = ReadI2C() << 8;
		AckI2C();
		temp[CURRENT_CELL] |= ReadI2C();
		StopI2C();*/
		if (BusyADC() == 0) {
			clearLED(1);
			voltage[CURRENT_CELL] = VRef * ReadADC();
			increaseCount();
			setAddress(CURRENT_CELL);
			Delay10TCYx(1); // delay after switching
			setLED(1);
			ConvertADC();
		}
		//_asm CLEARWDT _endasm
	// @todo watchdog and clear it and stuff
	}
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//_asm nop _endasm
	//closeRelay(); // should never get here
}

float intToFloat(int x) {
	return ((float) x) / (2^10);
}

int floatToInt(auto float x) {
	return 0; //@todo
}

char setOV(float x) {
	if (x < 5 && x > 0) {
		OVERVOLT_LIMIT = x;
		return 0;
	}
	return 1;
}

char setUV(float x) {
	if (x < 5 && x > 0) {
		UNDERVOLT_LIMIT = x;
		return 0;
	}
	return 1;
}

char setOC(float x) {
	// 10 Amp limit on relay (hardware limitation)
	if (fabs(x) < 10) {
		DISCHG_RATE_LIMIT = fabs(x);
		return 0;
	}
	return 1;
}

char setOD(float x) {
	if (fabs(x) < 10) {
		CHARGE_RATE_LIMIT = fabs(x);
		return 0;
	}
	return 1;
}

void checkVoltage(float x) {
	if (x > OVERVOLT_LIMIT) {
		STATUS_REG |= FAIL;
		openRelay();
	} else if (x < UNDERVOLT_LIMIT) {
		STATUS_REG |= FAIL;
		openRelay();
	}
}

void checkCurrent(float x) {
	if (x < CHARGE_RATE_LIMIT) {
		STATUS_REG |= FAIL;
		openRelay();
	} else if (x > DISCHG_RATE_LIMIT) {
		STATUS_REG |= FAIL;
		openRelay();
	}
}

void checkTemp(float x) {
	if (x > TEMP_CHARGE_LIMIT) {
		STATUS_REG |= FAIL;
		openRelay();
	} else if (x < TEMP_DISCHG_LIMIT) {
		STATUS_REG |= FAIL;
		openRelay();
	}
}

void openRelay(void) {
	//PORTB &= 0b11111110;
  //PORTC &= 0b11011111;
	_asm BCF PORTB, 6, 0 _endasm
}

void closeRelay(void) {
	//PORTB |= 0b00100000;
	_asm BSF PORTB, 6, 0 _endasm
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
	if (CURRENT_CELL < MAX_CELLS) {
		CURRENT_CELL++;
	} else {
		CURRENT_CELL = 0;
	}
}

void setLED(unsigned char led) {
  PORTA |= 1 << (led + 4);
}

void clearLED(unsigned char led) {
	PORTA &= ~(1 << (led + 4));
}

void toggleLED(unsigned char led) {
	if (PORTA & (1 << (led + 4))) { // if on, turn off
		clearLED(led);
	} else {
		setLED(led);
	}
}

void writeWord(unsigned char address, unsigned int x) {
	unsigned char *ptr = &x;
	char i;
	for (i = 0; i < 2; i++) {
		eepromWrite(address++,*(ptr++));
	}
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

#pragma code interruptVectorHigh = 0x08

void interruptVectorHigh (void) {
	_asm goto interruptHandlerHigh _endasm
}

#pragma code
#pragma interrupt interruptHandlerHigh save

void interruptHandlerHigh(void) {
	if (PIR2bits.OSCFIF) {
		STATUS_REG |= SOFT_FAIL;
		ERROR_REGH |= OSC_FAIL;
		PIR2bits.OSCFIF = 0;
		// clear?
	} else if (PIR1bits.ADIF) {
		// ADC interrupt (not needed?
		PIR1bits.ADIF = 0;
	} else if (PIR2bits.BCLIF) {
		// Bus collision
		PIR2bits.BCLIF = 0;
		// @todo busErrors++;
	} else if (PIR3bits.ERRIF) {
		// error in CAN module
		PIR3bits.ERRIF = 0;
		// @todo if CAN is actually being used, shutdown
	}
}