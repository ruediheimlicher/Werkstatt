/*
 *  web_SPI.c
 *  WebServer
 *
 *  Created by Sysadmin on 11.Oktober.09.
 *  Copyright 2009 Ruedi Heimlicher. All rights reserved.
 *
 */

#include <avr/io.h>
/*
// defines fuer Atmega328p

#define OSZIPORT		PORTC
#define OSZIPORTDDR	DDRC
#define OSZIPORTPIN	PINC
#define PULSA			0

#define OSZILO OSZIPORT &= ~(1<<PULSA)
#define OSZIHI OSZIPORT |= (1<<PULSA)
#define OSZITOGG OSZIPORT ^= (1<<PULSA)

#define SPI_CONTROL_DDR			DDRD						// DDR fuer Soft-SPI
#define SPI_CONTROL_PORT		PORTD						// Port fuer Soft-SPI
#define SPI_CONTROL_PORTPIN	PIND						// Port-Pin fuer Soft-SPI

// ************************************************
// Modifizierte Belegung fuer Betrieb mit Webserver
// ************************************************

#define SPI_CONTROL_MOSI		PORTD0					// Ausgang fuer Daten zum Slave
#define SPI_CONTROL_MISO		PORTD1					// Eingang fuer Daten vom Slave
#define SPI_CONTROL_SCK			PORTD2					// Ausgang fuer CLK
#define SPI_CONTROL_CS_HC		PORTD3					// Ausgang CS fuer Slave
*/

// defines fuer Atmega644p

#define OSZIPORT		PORTC
#define OSZIPORTDDR	DDRC
#define OSZIPORTPIN	PINC
#define PULSA			2
#define PULSB			3

#define OSZILO OSZIPORT &= ~(1<<PULSA)
#define OSZIHI OSZIPORT |= (1<<PULSA)
#define OSZITOGG OSZIPORT ^= (1<<PULSA)

#define SPI_CONTROL_DDR			DDRB						// DDR fuer Soft-SPI
#define SPI_CONTROL_PORT		PORTB						// Port fuer Soft-SPI
#define SPI_CONTROL_PORTPIN	PINB						// Port-Pin fuer Soft-SPI

// ************************************************
// Modifizierte Belegung fuer Betrieb mit Webserver
// ************************************************

#define SPI_CONTROL_MOSI		PORTB0					// Ausgang fuer Daten zum Slave
#define SPI_CONTROL_MISO		PORTB1					// Eingang fuer Daten vom Slave
#define SPI_CONTROL_SCK			PORTB2					// Ausgang fuer CLK
#define SPI_CONTROL_CS_HC		PORTB3					// Ausgang CS fuer Slave





#define SPI_CLK_HI SPI_CONTROL_PORT |= (1<<SPI_CONTROL_SCK)
#define SPI_CLK_LO SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_SCK)

// ************************************************

// ************************************************
// defines fuer spistatus

#define SPI_SHIFT_BIT			0	// SPI einleiten
#define SPI_STATUS0_BIT			1
#define TWI_WAIT_BIT				2	// TWI soll warten
#define DATA_RECEIVE_BIT		3
#define TWI_STOP_REQUEST_BIT	4	// TWI Stop  anmelden
#define WRITE_CONFIRM_BIT		5	// Meldung von HomeCentral, dass EEPROM-Write OK ist
#define STATUS_CONFIRM_BIT		6	// Status 0 ist n Master geschickt
#define SPI_DATA_READY_BIT		7	// EEPROM-Daten fuer HomeServer sind bereit

// ************************************************
volatile uint16_t								errCounter=0;
static volatile uint8_t						ONCounter=0x00;
static volatile uint8_t						OFFCounter=0x00;
static volatile uint8_t						OutCounter=0x00;
static volatile uint8_t						SendOKCounter=0x00;
static volatile uint8_t						SendErrCounter=0x00;
static volatile uint8_t						IncompleteCounter=0x00;
//static volatile uint16_t					TimeoutCounter=0x00;
static volatile uint8_t					SPI_ErrCounter=0x00;
//static volatile uint16_t					resetcounter=0x00; // counter fuer Dauer reset-meldeimPULSA vom Master
// ************************************************
// defines fuer spistatus
#define ACTIVE_BIT				0
#define STARTDATEN_BIT			1
#define ENDDATEN_BIT				2
#define SUCCESS_BIT				3
#define LB_BIT						4
#define HB_BIT						5
#define ERR_BIT					6

// ************************************************
// defines fuer pendenzstatus
#define SEND_STATUS0_BIT		0	// Ankuendigen, dass in web-Schlaufe die Confirm-Status0-page geschickt wird
#define RESETDELAY_BIT			7	// Anzeige, dass ein Hardware-Reset im Gang ist.
#define RESETREPORT           6 // Anzeige, dass ein reset erfolgte. Meldung an homecentral schicken


#define CS_HC_PASSIVE			SPI_CONTROL_PORT |= (1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist HI
#define CS_HC_ACTIVE				SPI_CONTROL_PORT &= ~(1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist LO 

#define out_PULSE_DELAY			200								// Pause bei shift_byte

#define SPI_BUFSIZE				8							// Anzahl Bytes

// Ausgang:
volatile uint8_t					outbuffer[SPI_BUFSIZE];	// buffer fuer die Ausgangsdaten
volatile uint8_t					out_startdaten;			// Startdaten fuer Ausgang
volatile uint8_t					out_enddaten;				// Enddaten fuer Ausgang
volatile uint8_t					out_hbdaten;
volatile uint8_t					out_lbdaten;

// Eingang
volatile uint8_t					inbuffer[SPI_BUFSIZE];	// buffer fuer die Eingangsdaten
volatile uint8_t					in_startdaten;				// Startdaten fuer Eingang
volatile uint8_t					in_enddaten;				// Enddaten fuer Eingang
volatile uint8_t					in_hbdaten;
volatile uint8_t					in_lbdaten;

 volatile uint8_t             spistatus=0;				// Status der Uebertragung
static volatile uint8_t			ByteCounter=0;				// aktuelle Bytenummer


volatile uint8_t  webspistatus = 0;

volatile uint8_t  cronstatus = 0; // regelung mit cronjobs


volatile uint8_t pendenzstatus = 0;


uint8_t SPI_shift_out_byte(uint8_t out_byte);

void Init_SPI_Master(void) 
{ 
	SPI_CONTROL_DDR |= ((1<<SPI_CONTROL_MOSI)|(1<<SPI_CONTROL_SCK)|(1<<SPI_CONTROL_CS_HC));	// Set MOSI , SCK , and SS output 
	SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);  // SCK HI
   SPI_CONTROL_PORT |=(1<<SPI_CONTROL_CS_HC); // CS HI

	SPI_CONTROL_DDR &= ~(1<<SPI_CONTROL_MISO);																// MISO Eingang
	SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MISO);																	// HI


} 

void Clear_SPI_Master(void)
{
	SPI_CONTROL_PORT |= ((0<<SPI_CONTROL_MOSI)|(0<<SPI_CONTROL_SCK)|(0<<SPI_CONTROL_CS_HC));	// Set MOSI , SCK , and SS LO 

   
}
// https://www.mikrocontroller.net/topic/13208 crazy horse
void write_spi (unsigned char out_byte)
{
   
   //msb first
   unsigned char loop, mask;
   for (loop=0,mask=0x80;loop<8;loop++, mask=mask>>1)
   {
      SPI_CONTROL_PORT &=~(1<<SPI_CONTROL_SCK);
      if (out_byte & mask)
      {
         SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MOSI); // MOSI HI
      }
      else
      {
        SPI_CONTROL_PORT &=~(1<<SPI_CONTROL_MOSI); // MOSI HI
      }
      SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);				// Takt HI
   }
   SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);				// Takt HI
}


uint8_t SPI_shift_out_byte(uint8_t out_byte)
{
   uint8_t in_byte=0;
   uint8_t delayfaktor=1;
   
   uint8_t i=0;
   for(i=0; i<8; i++)
   {
      // Vorbereiten: Master legt DataBit auf MOSI
      
      if (out_byte & 0x80) // aktuelles MSB, wird fortlaufend nach links geschoben
      {
         /* this bit is high */
         SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MOSI); // MOSI HI
      }
      else
      {
         /* this bit is low */
         SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MOSI); // MOSI LO
      }
      _delay_us(delayfaktor*out_PULSE_DELAY);
      
      // Vorgang beginnt: Takt LO, Slave legt Data auf MISO
      
      SPI_CONTROL_PORT &=~(1<<SPI_CONTROL_SCK);
      _delay_us(delayfaktor*out_PULSE_DELAY);
      
      // Slave lesen von MISO
      if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MISO))	// Bit vom Slave ist HI
      {
         in_byte |= (1<<(7-i));
      }
      else
      {
         in_byte &= ~(1<<(7-i));
      }
      _delay_us(delayfaktor*out_PULSE_DELAY);
      SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);				// Takt HI
      
      out_byte = out_byte << 1;									//	Byte um eine Stelle nach links schieben
      _delay_us(delayfaktor*out_PULSE_DELAY);
   } // for i
   _delay_us(delayfaktor*out_PULSE_DELAY);
   
   SPI_CONTROL_PORTPIN |= (1<<SPI_CONTROL_MISO);
   return in_byte;
}





