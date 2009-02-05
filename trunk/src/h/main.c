#include <p18f2680.h>
#include <adc.h>
#include <delays.h>
#include <math.h>

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
unsigned char eepromRead(unsigned char address);
void eepromWrite(unsigned char address, unsigned char x);
unsigned int floatToInt(float x);
float intToFloat(unsigned int x);

//#define MAX_CELLS	8;
//#define VREF		5; // Reference voltage
//#define samples 	1; // Samples to keep

// Error "codes"
//     1 - Undervoltage
//     2 - Overvoltage
//     3 - Overcurrent 
//     4 - Overtemperature
//     5 - 5V Brownout (@todo unimplemented)
//     6 - Unbalanced (@todo unimplemented)
//     7 - Remote shutdown (@todo unimplemented)
//     8 - ** This bit indicates an oscillator failure
#define UNDERVOLT		0x01;
#define OVERVOLT		0x02;
#define OVERCURRENT		0x03; 
#define OVERTEMP		0x04;
#define	BROWNOUT		0x05;
#define UNBALANCED		0x06;
#define REMOTE_SHDN		0x07;

#define OSC_FAIL		0x08;
#define READ_FAIL		0x10;
#define VREF_FAIL		0x20;

unsigned char ERROR;

const int MAX_CELLS = 8;
unsigned int VRef;
// const unsigned int samples = 1;

unsigned char CURRENT_CELL;
unsigned int voltage[8];
unsigned int temp[8];
unsigned int current;
unsigned char EEPROM_START;

// Status "register"
// LSB 0 - Active (@todo high speed mode) ( >1 mA)
//     1 - Shutdown (active low @todo)
//     2 - Charge (1), Discharge (0)
//	   3 - Soft fail (still running)
#define FAIL		0x02;
#define SOFT_FAIL	0x08;
unsigned char STATUS_REG;

// Default set points (saved to EEPROM if erased)
unsigned int OVERVOLT_LIMIT = 4.2; // Overvoltage setpoint (volts)
unsigned int UNDERVOLT_LIMIT = 3.0; // Undervoltage setpoint (volts)
unsigned int DISCHG_RATE_LIMIT = 6.0; // Overcurrent (discharge) setpoint (amps)
unsigned int CHARGE_RATE_LIMIT = 6.0; // Overcurrent (charge) setpoint (amps)
unsigned int TEMP_DISCHG_LIMIT = 85.0; // Max discharge temperature
unsigned int TEMP_CHARGE_LIMIT = 60.0; // Max charge temperature
unsigned int CURRENT_THRES	= 0.01; // Amps

void init(void) {
	// Initialize global variables
	unsigned char i;
	CURRENT_CELL = 0;
	STATUS_REG = 0x01; // Operating (>1ma)
	current = 0;
	VRef = floatToInt(5.000);

	// Set up digital I/O ports
	TRISA = 0xFF;
	TRISB &= 0x1E; // Use B[5..7] for address bits, B[0] for relay
	TRISC &= 0xF0; // Use C[0..3] for LEDs
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
		EEPROM_START = 0x10;
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
	}

	// Enable High / Low voltage detect (for Vdd)
	//HLVDCON = 0x3E; // 4.48V - 4.69V brownout, interrupt enabled

	// enable communications busses, store last reset (RCON), etc
}

void main(void) {
	init();	
	// @todo check vref to see if its not broken first!
	OpenADC(ADC_FOSC_8 & 
			ADC_RIGHT_JUST &
			ADC_16_TAD, 
			ADC_CH0 &
			ADC_INT_ON &
			ADC_REF_VDD_VSS, 
			ADC_15ANA);
	Delay10TCYx(5); // delay for 50 cycles?
	ConvertADC();
	setLED(1);
	// @todo put vref in eeprom

	while (1) {
		toggleLED(0); // 1 cycle
		// ADC_XX_TAD where XX is 20,16,12,8,6,4,2,0 T_ad
		// OpenADC(ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_EXT, 0);
		if (BusyADC() == 0) {
			clearLED(1);
			voltage[CURRENT_CELL] = VRef * ReadADC();
			increaseCount();
			setAddress(CURRENT_CELL);
			// @todo above line or below switch statement
			switch (CURRENT_CELL) {
				case 0:
					SetChanADC(ADC_CH0);
					break;
				case 1:
					SetChanADC(ADC_CH1);
					break;
				case 2:
					SetChanADC(ADC_CH2);
					break;
				case 3:
					SetChanADC(ADC_CH4);
					break;
				default:
					SetChanADC(ADC_CH8);
					break;
			}
			Delay10TCYx(5); // delay after switching
			setLED(1);
			ConvertADC();
		}
	// @todo watchdog and clear it and stuff
	}
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
	PORTB &= 0b11111110;
}

void closeRelay(void) {
	PORTB |= 0b00000001;
}

void setAddress(unsigned char address) {
	if (address < MAX_CELLS) {
		PORTB = (PORTB & 0x1F) | (address << 5);
	} else {
		setAddress(0);
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
	PORTC = PORTC | (1 << led);
}

void clearLED(unsigned char led) {
	PORTC = PORTC & (0 << led);
}

void toggleLED(unsigned char led) {
	if (PORTC & (1 << led)) { // if on, turn off
		clearLED(led);
	} else {
		setLED(led);
	}
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
		ERROR |= OSC_FAIL;
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