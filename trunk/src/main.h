#define TRUE	1
#define FALSE	0
#define ADC_RESOLUTION		1024 // 10 bits
#define	VDD					5000 // mV
#define VREF_DEFAULT		3300 // mV
#define MAX_CELLS 			4
#define MAX_TEMP_FAILS		2
#define TEMP_CONFIG_REG		0x60
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
char initTemp(unsigned char address);
void reset(void);
void failTemp(unsigned char address);
