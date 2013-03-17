//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __Ruedi_Heimlicher__ 2009. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "twislave.c"
#include "lcd.c"

#include "adc.c"

//***********************************
//Werkstatt							*
#define SLAVE_ADRESSE 0x64 //		*
//									*
//***********************************
#define RAUM		"WERKSTATT\0"

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4 // PORT C
#define SCLPIN		5

#define TWI_WAIT_BIT		3
#define TWI_OK_BIT		4
#define WDTBIT		7
#define SLAVEPORT	PORTD		// Ausgang fuer Slave
#define LAMPEBIT		0
#define OFENBIT	1

#define SERVOPORT	PORTD			// Ausgang fuer Servo
#define SERVOPIN0 7				// Impuls f�r Servo
#define SERVOPIN1 6				// Enable fuer Servo, Active H

// Definitionen fuer mySlave
//#define LAMPEEIN 4
//#define LAMPEAUS 5

// Definitionen fuer Slave
#define LAMPEEIN 0
#define LAMPEAUS 1

#define OFENEIN 2
#define OFENAUS 3


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
#define TASTATURPORT PORTC

#define TASTATURPIN	3
#define POTPIN			0
#define BUZZERPIN		2

#define INNEN			0	// Bit fuer Innentemperatur
#define TEMP1			1	// Bit fuer Temperatur 1
#define TEMP2			2	// Bit fuer Temperatur 2

#define STATUS					3	// Byte fuer Status

#define EINGANG0PIN			2	// PIN 2 von PORT B als Eingang 
#define TIEFKUEHLALARMPIN	3	// PIN 3 von PORT B als Eingang fuer TiefkuehlAlarn
#define WASSERALARMPIN		4	// PIN 4 von PORT B als Eingang fuer Wasseralarm 

#define MANUELL		7	// Bit 7 von Status 
#define MANUELLPIN	4	//Pin 3 von PORT D fuer Anzeige Manuell


#define LOOPLEDPORT		PORTD
#define LOOPLED			4
#define TWILED				5

#define STARTDELAYBIT	0
#define HICOUNTBIT		1


void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);
void lcd_puts(const char *s);
volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
uint16_t                EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit


uint8_t                 EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t                 EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

void delay_ms(unsigned int ms);

volatile uint8_t        Lampestatus=0x00;
static volatile uint8_t Radiatorstatus=0x00;


volatile uint16_t       Servotakt=20;					//	Abstand der Impulspakete
volatile uint16_t       Servopause=0x00;				//	Zaehler fuer Pause
volatile uint16_t       Servoimpuls=0x00;				//	Zaehler fuer Impuls
volatile uint8_t        Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
volatile uint8_t        ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
volatile uint8_t        ServoimpulsdauerSpeicher=0;		//	Speicher  fuer Servoimpulsdauer
volatile uint8_t        Potwert=45;
volatile uint8_t        TWI_Pause=1;
volatile uint8_t        ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
uint8_t                 ServoimpulsNullpunkt=23;
uint8_t                 ServoimpulsSchrittweite=10;
uint8_t                 Servoposition[]={23,33,42,50,60};
volatile uint16_t       ADCImpuls=0;

uint8_t                 EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events




uint8_t Tastenwahl(uint8_t Tastaturwert)
{
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



void slaveinit(void)
{
 	DDRD |= (1<<LAMPEEIN);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<LAMPEAUS);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<OFENEIN);		//Pin 2 von PORT D als Ausgang fuer OFENEIN
 	DDRD |= (1<<OFENAUS);		//Pin 3 von PORT D als Ausgang fuer OFENAUS
	DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
	DDRD |= (1<<DDD5);		//Pin 5 von PORT D als Ausgang fuer Manuell
 	DDRD |= (1<<SERVOPIN1);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	DDRD |= (1<<SERVOPIN0);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	
	
	DDRB &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang f�r Taste 1
	PORTB |= (1<<PB0);	//Pull-up

	//DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang f�r Taste 2
	//PORTB |= (1<<PB1);	//Pull-up
	
	//TIEFKUEHLALARMPIN
	DDRB &= ~(1<<TIEFKUEHLALARMPIN);	//Pin 3 von PORT B als Eingang f�r Tiefkuehlalarm
	PORTB |= (1<<TIEFKUEHLALARMPIN);	//Pull-up

	//WASSERALARMPIN
   
	DDRB &= ~(1<<WASSERALARMPIN);	//Pin 4 von PORT B als Eingang f�r Wasseralarm
	PORTB |= (1<<WASSERALARMPIN);	//Pull-up
	

	//LCD
	DDRB |= (1<<LCD_RSDS_PIN);		//Pin 5 von PORT B als Ausgang fuer LCD
 	DDRB |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	DDRB |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

	// TWI vorbereiten
	TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang f�r SDA
	TWI_PORT |= (1<<SDAPIN); // HI
	
	TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang f�r SCL
	TWI_PORT |= (1<<SCLPIN); // HI

	SlaveStatus=0;
	SlaveStatus |= (1<<TWI_WAIT_BIT);
	
	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC 	
	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer ADC 	
	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
	PORTC |= (1<<DDC3); //Pull-up


	
	
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//Timer fuer Servo	
	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//R�cksetzen des Timers
	
}

void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 

ISR (TIMER0_OVF_vect)
{ 
	ADCImpuls++;
	Servopause++;
	//lcd_clr_line(1);

	//lcd_gotoxy(10,1);
	//lcd_puts("Tim\0");
	//delay_ms(400);
	//lcd_cls();
	//lcd_clr_line(0);
	//lcd_gotoxy(0,1);
	//lcd_puts("Stop Servo\0");
	//lcd_puts(" TP\0");
	//lcd_putint1(TWI_Pause);
	//	Intervall 64 us, Overflow nach 16.3 ms
	if (Servopause==3)	// Neues Impulspaket nach 48.9 ms
	{

		if (TWI_Pause)
		{
//			lcd_gotoxy(19,0);
//			lcd_putc(' ');
			timer2(Servoimpulsdauer);	 // setzt die Impulsdauer
			if (SERVOPORT &  (1<<SERVOPIN1)) // Servo ist ON
			{
				SERVOPORT |= (1<<SERVOPIN0); // Schaltet Impuls an SERVOPIN0 ein
			}
			SERVOPORT |= (1<<5);// Kontrolle auf PIN D5
		}
		Servopause=0;
	}
}


ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR2=0;
		SERVOPORT &= ~(1<<SERVOPIN0);//	SERVOPIN0 zuruecksetzen
		SERVOPORT &= ~(1<<5);// Kontrolle auf PIN D5 OFF
		//delay_ms(800);
		//lcd_clr_line(1);
		
}




void main (void) 
{
	slaveinit();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	//init_twi_slave (SLAVE_ADRESSE);
	//uint16_t ADC_Wert= readKanal(0);
	sei();
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts(RAUM);
	
	SLAVEPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
	SLAVEPORT &= ~(1<<LAMPEAUS);//	LAMPEAus sicher low
	SLAVEPORT |= (1<<LAMPEAUS);
	delay_ms(30);
	SLAVEPORT &= ~(1<<LAMPEAUS);

	SLAVEPORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
	SLAVEPORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
	SLAVEPORT |= (1<<OFENAUS);
	delay_ms(30);
	SLAVEPORT &= ~(1<<OFENAUS);

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	uint8_t Servowert=0;
	uint8_t Servorichtung=1;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	uint8_t Schalterposition=0;
	timer0();
	
	initADC(TASTATURPIN);
	
	wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	


	uint8_t twierrcount=0;
	LOOPLEDPORT |=(1<<LOOPLED);
	
	delay_ms(800);
	uint8_t tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);


	//	Zaehler fuer Wartezeit nach dem Start
	uint16_t startdelay0=0x00AF;
	//uint16_t startdelay1=0;

	//Zaehler fuer Zeit von (SDA || SCL = LO)
	uint16_t twi_LO_count0=0;
	uint16_t twi_LO_count1=0;
	uint8_t StartStatus=0x00; //status

	//Zaehler fuer Zeit von (SDA && SCL = HI)
	uint16_t twi_HI_count0=0;

	uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0); 
	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
	/*
	eepromWDT_Count0: Zaehler der wdt-Resets mit restart. 
	
	eepromWDT_Count1: Zaehler fuer neuen wdt-Reset. Wenn wdt anspricht, wird der Zaheler erhoeht.
	Beim Restart wird bei anhaltendem LO auf SDA oder SCL gewartet.
	Wenn SCL und SDA beide HI sind, wird der Zaehler auf den Wert von eepromWDT_Count0 gesetzt 
	und der TWI-Slave gestartet.
	
	*/

	// Ankuendigen, dass schon ein wdt erfolgte
	if (!(eepromWDT_Count0==eepromWDT_Count1))
	{
		lcd_gotoxy(18,1);
		lcd_puts("W\0");

	}







	while (1)
	{	
		wdt_reset();
				//Blinkanzeige
		loopcount0++;
		if (loopcount0==0xAFFF)
		{
			loopcount0=0;
			LOOPLEDPORT ^=(1<<LOOPLED);
			//lcd_gotoxy(0,0);
			//lcd_puts("wdt\0");
			//lcd_puthex(tempWDT_Count);
			//delay_ms(10);
			
		}

		
		/**	Beginn Startroutinen	***********************/
			// wenn Startbedingung vom Master:  TWI_slave initiieren
		if (SlaveStatus & (1<<TWI_WAIT_BIT)) 
		{
			if ((TWI_PIN & (1<<SCLPIN))&&(!(TWI_PIN & (1<<SDAPIN))))// Startbedingung vom Master: SCL HI und SDA LO
			{
			init_twi_slave (SLAVE_ADRESSE);
			sei();
			SlaveStatus &= ~(1<<TWI_WAIT_BIT);
			SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
			
			// StartDelayBit zuruecksetzen
			
			}
		}
	
		/**	Ende Startroutinen	***********************/
		
		
		/***** rx_buffer abfragen **************** */
		//rxdata=0;
		
		//***************
		//	Servo
		//***************
		
		if ((SlaveStatus & (1<<TWI_OK_BIT)) &&(rxdata) && !(SlaveStatus & (1<< MANUELL)))	//Daten von TWI liegen vor und Manuell ist OFF
		{
         
				//PORTD |=(1<<PD3);

			{
				if (rxbuffer[3] < 6)
				{					
						if (Servorichtung && (Servowert<4))// vorw�rts
						{
							Servowert++;
							if (Servowert==4)
							{
							Servorichtung=0;
							}
						}
						else if (Servowert)
						{
							Servowert--;
							if (Servowert==0)
							{
							Servorichtung=1;
							}
						}
						
					/*
					lcd_gotoxy(0,12);
					lcd_puts("R:\0");
					lcd_putint2(Servorichtung);
					lcd_puts(" W:\0");
					lcd_putint2(Servowert);
					*/
					
					
					Servowert=rxbuffer[3];
					
					ServoimpulsdauerPuffer=Servoposition[Servowert];
						
				}
				
				//lcd_gotoxy(0,0);
				//lcd_puts("I:\0");
				//lcd_putint2(Servoimpulsdauer);
				//lcd_gotoxy(8,0);
				//lcd_gotoxy(0,1);
				//lcd_puts(" P:\0");
				//lcd_putint2(ServoimpulsdauerPuffer);
				
				//lcd_puts(" S:\0");
				//lcd_putint2(ServoimpulsdauerSpeicher);
				//lcd_gotoxy(19,0);
				//lcd_puts(" O:\0");
				//lcd_putint1(ServoimpulsOK);
				SERVOPORT &= ~(1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo aus
				
				if (!(ServoimpulsdauerPuffer==Servoimpulsdauer))	//	neuer Wert ist anders als aktuelle Impulsdauer
				{
					if (ServoimpulsdauerPuffer==ServoimpulsdauerSpeicher)	// neuer Wert ist schon im Speicher
					{
						ServoimpulsOK++;	//	Zaehler der gleichen Impulse incr
					}
					else
					{
						ServoimpulsdauerSpeicher=ServoimpulsdauerPuffer;
						ServoimpulsOK=0;	//	Zaehler der gleichen Impulse zuruecksetzen
						
					}
					
				}//
				else
				{
					ServoimpulsOK=0;	//Ausreisser
				}
				
				if (ServoimpulsOK>3)	//	neuer Wert ist sicher
				{
					SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
					
					if (ServoimpulsdauerSpeicher>Servoimpulsdauer)
					{
						Servoimpulsdauer=ServoimpulsdauerSpeicher+2; //	Etwas weiter im UZ drehen
						delay_ms(800);
						//Servoimpulsdauer=ServoimpulsdauerSpeicher-2;
						//delay_ms(400);
						
					}
					else
					{
						Servoimpulsdauer=ServoimpulsdauerSpeicher-2; //	Etwas weiter gegen UZ drehen
						delay_ms(800);
						//Servoimpulsdauer=ServoimpulsdauerSpeicher+2;
						//delay_ms(400);
						
					}
					
					Servoimpulsdauer=ServoimpulsdauerSpeicher;
//					lcd_gotoxy(17,0);
//					lcd_puts("S\0");
//					lcd_putint1(Servowert);

					ServoimpulsOK=0;
				}//	neuer Wert ist sicher

				
				
			}
			
			//RingD2(2);
			//delay_ms(20);
			
			Lampestatus=rxbuffer[0];
			lcd_gotoxy(12,0);
			lcd_puts("St:\0");
			lcd_puthex(Lampestatus);
			//delay_ms(20);
			/*
			// TWI_NEW_BIT wird in twislave-ISR gesetzt, wenn alle Daten aufgenommen sind
			
			if (SlaveStatus & (1<<TWI_NEW_BIT))
			{
				SlaveStatus &= ~(1<<TWI_NEW_BIT); // Die Aktionen sollen nur einmal ausgeloest werden
				
			
			*/
			// Lampe
         
			if ( Lampestatus  & (1<<LAMPEBIT)) // PIN 0
				{
					//delay_ms(1000);
					//Lampe ein
					//lcd_gotoxy(19,1);
					//lcd_puts("I\0");
					SLAVEPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
					SLAVEPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
					SLAVEPORT |= (1<<LAMPEEIN);
					delay_ms(30);
					SLAVEPORT &= ~(1<<LAMPEEIN);
		//			PORTD |= (1<<2);
					//lcd_gotoxy(15,1);
					//lcd_puts("ON \0");
				}
				else
				{
					//delay_ms(1000);
					//Lampe aus
					//lcd_gotoxy(19,1);
					//lcd_puts("0\0");
					
					SLAVEPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
					SLAVEPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
					SLAVEPORT |= (1<<LAMPEAUS);
					delay_ms(30);
					SLAVEPORT &= ~(1<<LAMPEAUS);
			//		PORTD &= ~(1<<2);
					//lcd_gotoxy(15,1);
					//lcd_puts("OFF\0");

				}
				
         // Ofen
         
         Radiatorstatus=rxbuffer[1];
         //if ( Slavestatus  & (1<<OFENBIT)) // PIN 1
			if ( Radiatorstatus & 0x03) // // Ofen ein
				{
					//delay_ms(1000);
					//Ofen ein
					lcd_gotoxy(19,1);
					lcd_putc('I');
					SLAVEPORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
					SLAVEPORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
					SLAVEPORT |= (1<<OFENEIN); // Impuls an ON
					delay_ms(30);
					SLAVEPORT &= ~(1<<OFENEIN);
					//lcd_gotoxy(15,1);
					//lcd_puts("ON \0");
				}
				else
				{
					//delay_ms(1000);
					//Ofen aus
					lcd_gotoxy(19,1);
					lcd_putc('0');
					
					SLAVEPORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
					SLAVEPORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
					SLAVEPORT |= (1<<OFENAUS); // Impuls an OFF
					delay_ms(30);
					SLAVEPORT &= ~(1<<OFENAUS);
				//	PORTD &= ~(1<<2);
					//lcd_gotoxy(15,1);
					//lcd_puts("OFF\0");

				}
			
	//****************************
	//	tx_buffer laden
	//****************************	
			
				//lcd_clr_line(1);
				// Temperatur lesen
				uint16_t tempBuffer=0;
				initADC(INNEN);
				tempBuffer=readKanal(INNEN);
				//lcd_gotoxy(0,1);
				//lcd_puts("T\0");
				//lcd_putint(tempBuffer>>2);
				//lcd_put_tempbis99(tempBuffer>>2);
				
				txbuffer[INNEN]=(uint8_t)(tempBuffer>>2);// Vorlauf
				//txbuffer[INNEN]=(uint8_t)(readKanal(INNEN)>>2);// Vorlauf
				//lcd_gotoxy(0,1);
				//lcd_puts("V\0");

	/*
				initADC(TEMP1);
				
				uint16_t tempBuffer1=readKanal(TEMP1);
				
				lcd_gotoxy(8,1);
				lcd_puts("R\0");
				lcd_put_tempbis99(tempBuffer1>>2);

				txbuffer[TEMP1]=(uint8_t)(tempBuffer1>>2);// Ruecklauf
				
				
				initADC(TEMP2);
				tempBuffer=readKanal(TEMP2);
				txbuffer[TEMP2]=(uint8_t)(tempBuffer>>2); // Aussen
				lcd_gotoxy(8,0);
				lcd_puts("A\0");
				lcd_put_tempbis99(tempBuffer>>2);
				
	*/
				//
				//	Kuehltruhe abfragen
				//
				if (PINB & (1<< TIEFKUEHLALARMPIN)) // HI, Alles OK
				{
					txbuffer[STATUS] &= ~(1<< TIEFKUEHLALARMPIN); // TIEFKUEHLALARMBit zuruecksetzen
				}
				else 
				{
					txbuffer[STATUS] |= (1<< TIEFKUEHLALARMPIN);	// TIEFKUEHLALARMBit setzen
				}
				
				//
				//	Wasseralarm abfragen
				//
				if (PINB & (1<< WASSERALARMPIN)) // HI, Alles OK
				{
					txbuffer[STATUS] &= ~(1<< WASSERALARMPIN); // WASSERALARMBit zuruecksetzen
				}
				else 
				{
					txbuffer[STATUS] |= (1<< WASSERALARMPIN);	// WASSERALARMBit setzen
				}
				
				//lcd_gotoxy(17,1);
				//lcd_puts("TKA\0");
	/*			
				if (PINB & (1<< EINGANG0PIN))
				{
				lcd_puts("OFF\0");
				}
				else
				{
				lcd_puts("ON \0");
				}
			
	*/		
			
			rxdata=0;               // TWI erledigt
			//PORTD &= ~(1<<PD3);

		}
		
		
		
		if (!(PINB & (1<<PB0))) // Taste 0
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P0 Down\0");

			if (! (TastenStatus & (1<<PB0))) //Taste 0 war nich nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<PB0);
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount ++;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
			
				if (Tastencount >= Tastenprellen)
				{
						/* initialize the LCD */
					lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
					lcd_puts("WERKSTATT\0");
					
					if (Servowert<4)
					{
					Servowert++;
					Servoimpulsdauer=Servoposition[Servowert];
					
					}
					/*
					if (Servoimpulsdauer<61)
					{
						Servoimpulsdauer++;
						SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
						lcd_gotoxy(0,1);
						//lcd_puts("P0 down  \0");
						lcd_putint2(Servoimpulsdauer);
						
					}
					*/
					Tastencount=0;
					TastenStatus &= ~(1<<PB0);
				}
			}//else
			
		}
	
			
		if (!(PINB & (1<<PB1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P1 Down\0");

			if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PB1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);

			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");

				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
				
				if (Servowert > 0)
					{
					Servowert--;
					Servoimpulsdauer=Servoposition[Servowert];
					
					}

					/*
					if (Servoimpulsdauer>19)
					{
						Servoimpulsdauer--;
						SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 setzen: Servo ein
					
					lcd_gotoxy(0,1);
					lcd_putint2(Servoimpulsdauer);
					}
					*/
					Tastencount=0;
					TastenStatus &= ~(1<<PB1);
				}
			}//	else
			
		}
		
		/* ******************** */
//		initADC(TASTATURPIN);
//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
		
		if (Tastenwert>23)
		{
			/*
			 0: 
			 1: Uhr ein
			 2: 
			 3: Uhr aus
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=50)
			{
				
				 //lcd_clr_line(1);
				 //lcd_gotoxy(8,1);
				 //lcd_puts("T:\0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				
				// lcd_gotoxy(18,1);
				 //lcd_putint2(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0://
					{ 
						
						SlaveStatus |= (1<<MANUELL);	// MANUELL ON, 7
						PORTD |= (1<<MANUELLPIN);
						SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo ein
						//Schalterposition=0;
						//Servoimpulsdauer=Servoposition[Schalterposition];
					}break;
						
					case 1:	//	Uhr ein
					{ 
						if (SlaveStatus & (1<<MANUELL))
						{
							SLAVEPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
							SLAVEPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
							SLAVEPORT |= (1<<LAMPEEIN);
							delay_ms(20);
							SLAVEPORT &= ~(1<<LAMPEEIN);
						}
					}break;
						
					case 2://
					{ 
						
						uint16_t tempBuffer=0;
						initADC(INNEN);
						tempBuffer=readKanal(INNEN);
						lcd_gotoxy(0,1);
						lcd_puts("V\0");
						//lcd_putint(tempBuffer>>2);
						lcd_put_tempbis99(tempBuffer>>2);
						
						
					}break;
						
					case 3: //	Lampe aus
					{ 
						if (SlaveStatus & (1<<MANUELL))
						{
							SLAVEPORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
							SLAVEPORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
							SLAVEPORT |= (1<<LAMPEAUS);
							delay_ms(20);
							SLAVEPORT &= ~(1<<LAMPEAUS);
						}
					}break;
						
					case 4://
					{ 
						if ((SlaveStatus & (1<<MANUELL)) &&  Schalterposition)
						{
							Schalterposition--;
							Servoimpulsdauer=Servoposition[Schalterposition];
						}
						
					}break;
						
					case 5://
					{ 
						//Slavestatus |= (1<<MANUELL);	// MANUELL ON
						//PORTD |= (1<<PD3);
						//SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo ein
						if (SlaveStatus & (1<<MANUELL))
						{
						Schalterposition=0;
						Servoimpulsdauer=Servoposition[Schalterposition];
						}
						
					}break;
						
					case 6://
					{ 
						if ((SlaveStatus & (1<<MANUELL)) && (Schalterposition<4))
						{
							Schalterposition++;
							Servoimpulsdauer=Servoposition[Schalterposition];
						}
					}break;
						
					case 7://
					{ 
						if (Servoimpulsdauer>Servoposition[0])
						{
							Servoimpulsdauer--;
							//lcd_gotoxy(0,16);
							//lcd_putint2(Servoimpulsdauer);
						}
						
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9://
					{ 
						if (Servoimpulsdauer<Servoposition[4])
						{
							Servoimpulsdauer++;
							//lcd_gotoxy(0,2);
							//lcd_putint2(Servoimpulsdauer);
						}
					}break;
						lcd_gotoxy(8,1);
						lcd_puts("P:\0");
						lcd_putint2(Schalterposition);

					case 10://
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12:
					{
						
						SlaveStatus &= ~(1<<MANUELL); // MANUELL OFF
						SERVOPORT &= ~(1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo aus
						PORTD &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}//	if Tastenwert
		
		
	}//while


// return 0;
}
