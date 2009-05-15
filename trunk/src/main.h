/**
 *
 * main.h - Header for the main.c file
 * Author Kevin McHale (k.mchale@me.com)
 *
 */
#define TRUE	1
#define FALSE	0
#define ADC_BITS			1024 // 10 bits
#define	VDD					5 // Volts
#define VREF_DEFAULT		2.5 // Volts
#define MAX_CELLS 			4
#define MAX_TEMP_FAILS		2
#define TEMP_CONFIG_REG		0x60
#define ADC_CONFIG			ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_16_TAD
#define ADC_VREF_DEFAULT	ADC_CH3 & ADC_INT_ON & ADC_REF_VDD_VSS
#define ADC_VREF_EXT1		ADC_CH3 & ADC_INT_ON & ADC_REF_VREFPLUS_VSS
#define ADC_VREF_EXT2		ADC_CH3 & ADC_INT_ON & ADC_REF_VREFPLUS_VREFMINUS
#define ADC_CHMUX			ADC_CH0
#define ADC_CHCURRENT		ADC_CH1
#define ADC_MUX_DELAY		1
#define SAMPLE_BUFFER		10

typedef struct {
	float g;
	float b;
} cal;

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
void reset( void );
void failTemp( unsigned char address );
int convVolts( int x, unsigned int i );
float intToFloat( int x );
int floatToInt( float x );
int convVolts( int x , unsigned int i );
float intToFloat( int x );
int floatToInt ( float x );
float conv( float x, cal c );