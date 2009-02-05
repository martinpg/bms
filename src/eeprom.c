#include <p18f2680.h>
#include <status.h>

void writeWord(unsigned char address, unsigned int x);
unsigned int readWord(unsigned char address);
void eepromWrite(unsigned char address, unsigned char x);
unsigned char eepromRead(unsigned char address);

unsigned char EEPROM_START;

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
	while (PIR2bits.EEIF == 0); // wait to complete
	if (EECON1bits.WRERR) {
		ERROR |= READ_FAIL;
		STATUS |= SOFT_FAIL;
	}
	EECON1bits.WREN = 0; // disable writes
	INTCONbits.GIE = oldGIE; // re-enable interrupts
}