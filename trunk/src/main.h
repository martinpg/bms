/**
 *
 * main.h - Header for the main.c file
 * Author Kevin McHale (k.mchale@me.com)
 *
 */
 
#define DEBUG_CONSOLE	// comment out this line to remove console debugging messages
 
#define TRUE	1
#define FALSE	0
#define ADC_BITS			1024 // 10 bits
#define	VDD					5.0 // Volts
#define VREF_DEFAULT		2.5 // Volts
#define MAX_CELLS 			4
#define MAX_TEMP_FAILS		2 // failures tolerated
#define TEMP_CONFIG_REG		0x60
#define ADC_CONFIG			ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_16_TAD
#define ADC_VREF_DEFAULT	ADC_CH3 & ADC_INT_ON & ADC_REF_VDD_VSS
#define ADC_VREF_EXT1		ADC_CH3 & ADC_INT_ON & ADC_REF_VREFPLUS_VSS
#define ADC_VREF_EXT2		ADC_CH3 & ADC_INT_ON & ADC_REF_VREFPLUS_VREFMINUS
#define ADC_CHMUX			ADC_CH0
#define ADC_CHCURRENT		ADC_CH1
#define ADC_MUX_DELAY		1
#define SAMPLE_BUFFER		10
#define SERIAL_BUFFER		24
#define CONFIG_VERSION		1

#define prioLOW				0
#define prioMED				1
#define prioHIGH			2
#define prioCRITICAL		3
#define prioDEFAULT			( prioMED )

#define pinRELAY			PORTCbits.RC5
#define pinADDRESS0			PORTC
#define pinLED0				PORTBbits.RB0
#define pinLED1				PORTBbits.RB1

#define relayCLOSED			1
#define relayOPEN			0
#define ledON				1
#define ledOFF				0

typedef struct {
	float g;
	float b;
} cal;

#define msgDWELL	0
#define msgVOLTS	1
#define msgCURRENT	2
#define msgTEMP		3

typedef struct {
	unsigned char type;
	unsigned int raw;
	unsigned char cell;
} message;

typedef struct {
	unsigned char address;
	unsigned int data;
} eeWord;

void init( void );
void main( void );
void setAddress( unsigned char address );
void toggleRedLED( void );
void setRedLED( void );
void clearRedLED( void );
void toggleGreenLED( void );
void setGreenLED( void );
void clearGreenLED( void );
void increaseCount( void );
void openRelay( void );
void closeRelay( void );
void checkVoltage( unsigned int x, unsigned int address );
void checkCurrent( unsigned int x );
void checkTemp( signed int x );
void interruptHandlerHigh ( void );
void writeWord( unsigned char address, unsigned int x );
unsigned int readWord( unsigned char address );
void eepromWrite( unsigned char address, unsigned char x );
unsigned char eepromRead( unsigned char address );
void initEEPROM( void );
signed int readTemp( unsigned char address );
char initTemp( unsigned char address );
void failTemp( unsigned char address );
int convVolts( int x, unsigned int i );
int floatToInt( float x );
int convVolts( int x , unsigned int i );
float intToFloat( int , unsigned int );
int floatToInt ( float x );
float conv( float x, cal c );
void tskCheckCurrent( void *params );
void tskCheckVolts( void *params );
void tskCheckTemps( void *params );
void tskCalculate( void *params );
void tskCheck( void *params );
void tskSerial( void *params );
void isr_low ( void );
void low_interrupt ( void );
void tskUI( void *params );
