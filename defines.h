//
//  defines.h
//  TWI_Slave
//
//  Created by Ruedi Heimlicher on 09.10.2016.
//
//

#ifndef defines_h
#define defines_h

#define TEST 1
#define RAUM		"WERKSTATT SPI"

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

/*
// Atmega328
#define SDAPIN		4 // PORT C
#define SCLPIN		5
*/

// Atmega644
#define SDAPIN		0 // PORT C
#define SCLPIN		1

//#define TWI_WAIT_BIT		3
#define TWI_OK_BIT         4
#define WDTBIT             7

#define LAMPEBIT           0
#define OFENBIT            1


#define SERVOPORT          PORTD // Ausgang fuer Servo
#define SERVODDR           DDRD  // Ausgang fuer Servo

#define SERVOPIN0          4 // OC1A: Impuls für Servo 0
#define SERVOPIN1          5 // OC1B: Impuls für Servo 1
#define SERVOENABLEPIN     6 // Enable fuer Servos, Active H

// ADC Eingang
#define ADCPORT            PORTA
#define ADCDDR             DDRA


#define SOLEVORLAUFPIN           1
#define SOLERUECKLAUFPIN         2
#define SPEICHERPIN              4


//Bit- Definitionen fuer Slave
#define LAMPEEIN     4
#define LAMPEAUS     5

#define OFENEIN      6
#define OFENAUS      7

/*
#define TASTE1			38
#define TASTE2			46
#define TASTE3			54
#define TASTE4			72
#define TASTE5			95
#define TASTE6			115
#define TASTE7			155
#define TASTE8			186
#define TASTE9			205
#define TASTEL			225
#define TASTE0			235
#define TASTER			245
*/

// VCC = 2.50
#define TASTE1			12
#define TASTE2			21
#define TASTE3			32
#define TASTE4			49
#define TASTE5			69
#define TASTE6			92
#define TASTE7			118
#define TASTE8			140
#define TASTE9			154
#define TASTEL			167
#define TASTE0			178
#define TASTER			200



#define TESTPORT PORTB
#define TESTDDR  DDRB
#define TESTPIN  PINB

#define TEST_PIN  7



#define TASTATURPORT PORTA
#define TASTATURDDR  DDRA

#define TASTATURPIN	0
//#define BUZZERPIN		2

#define SOLEVORLAUFBYTE       0   // Byte fuer SOLEVORLAUF
#define SOLERUECKLAUFBYTE		1	// Byte fuer Soleruecklauf
#define SPEICHERBYTE			   2	// Byte fuer Speichertemperatur 


// TWI
#define STATUS			3	// Byte fuer Status

#define STROMHH      4 // Bytes fuer Stromdaten
#define STROMH       5
#define STROML       6

#define SPIERR       7

#define SLAVE_IN_PORT         PORTA		// Eingang fuer Slave
#define SLAVE_IN_DDR          DDRA		// DDR fuer Slave
#define SLAVE_IN_PIN          PINA  // PIN fuer Slave
#define SPI_PIN               6

#define SLAVE_OUT_PORT         PORTC		// Ausgangsport fuer Slave
#define SLAVE_OUT_DDR          DDRC		// DDR fuer Slave
#define SLAVE_OUT_PIN          PINC  // PIN fuer Slave

#define BUZZER_PORT         PORTA		// Ausgangsport fuer Slave
#define BUZZER_DDR          DDRA		// DDR fuer Slave
#define BUZZER_PIN         7

#define ALARM_IN_PORT         PORTB		// Eingang fuer Slave
#define ALARM_IN_DDR          DDRB		// DDR fuer Slave
#define ALARM_IN_PIN          PINB  // PIN fuer Slave

#define EINGANG0BIT			2	// PIN 2 von PORT B als Eingang
#define TIEFKUEHLALARM_PIN	4	// PIN  als Eingang fuer TiefkuehlAlarn
#define WASSERALARM_PIN		5	//  als Eingang fuer Wasseralarm


#define ALARMBIT		6	// Bit 7 von Status
#define MANUELLBIT		7	// Bit 7 von Status

#define MANUELL_PIN		7	// Bit 7 von Status


#define LOOPLEDDDR		DDRD
#define LOOPLEDPORT		PORTD
#define LOOPLED			7

#define STARTDELAYBIT	0
#define HICOUNTBIT		1


#endif /* defines_h */
