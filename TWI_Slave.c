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
#include <stdlib.h>

#include "defines.h"
#include "twislave.c"
#include "lcd.c"
#include "web_SPI.c"
#include "adc.c"
#include "ds18x20.c"

//***********************************
//Werkstatt							*
#define SLAVE_ADRESSE 0x64 //		*
//#define SLAVE_ADRESSE 0x62 //		*

//									*
//***********************************

//#define SLAVE_ADRESSE 0x99



void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);
void lcd_puts(const char *s);
volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
//uint16_t                EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit


uint8_t                 EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t                 EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset


volatile uint8_t        Lampestatus=0x00;
static volatile uint8_t Radiatorstatus=0x00;

volatile uint16_t          Servostellung=0;					//	Abstand der Impulspakete
//volatile uint16_t       Servotakt=20;					//	Abstand der Impulspakete
//volatile uint16_t       Servopause=0x00;				//	Zaehler fuer Pause
//volatile uint16_t       Servoimpuls=0x00;				//	Zaehler fuer Impuls
//volatile uint8_t        Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
//volatile uint8_t        ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
//volatile uint8_t        ServoimpulsdauerSpeicher=0;		//	Speicher  fuer Servoimpulsdauer
//volatile uint8_t        Potwert=45;
volatile uint8_t           TWI_Pause=1;
//volatile uint8_t        ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
//uint8_t                 ServoimpulsNullpunkt=23;
uint8_t                    ServoimpulsSchrittweite=10;
uint16_t                 Servoposition[]={1000,1250,1500,1750,2000,1750,1500,1250};

volatile uint16_t       ADCImpuls=0;

uint8_t                 EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events


// SPI

//#define CS_HC_PASSIVE			SPI_CONTROL_PORT |= (1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist HI
//#define CS_HC_ACTIVE				SPI_CONTROL_PORT &= ~(1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist LO

#define out_PULSE_DELAY			200								// Pause bei shift_byte

volatile uint16_t EventCounter=0;

volatile uint16_t       timer0counter0=0;					//
volatile uint16_t       timer0counter1=0;

//#define MAXSENSORS 2
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;

volatile uint8_t       testcounter=0;
volatile uint8_t       spicounter=0;
volatile uint8_t       adccounter=0;

volatile uint16_t spannungA=0;
volatile uint16_t spannungB=0;
volatile uint8_t Tastenwert=0;


void SPI_shift_out(void)
{
   
   uint8_t byteindex=0;
   in_startdaten=0;
   in_enddaten=0;
   uint8_t delayfaktor=16; // war 2
   
   in_lbdaten=0;
   in_hbdaten=0;
   lcd_gotoxy(19,1);
   lcd_putc(' ');
  // OSZILO;
   
   SPI_CLK_HI; // Clk sicher HI
   delay_ms(1);
   CS_HC_ACTIVE; // CS LO fuer Slave: Beginn Uebertragung
   //delay_ms(1);
   _delay_us(delayfaktor*out_PULSE_DELAY);
   //OSZILO;
   in_startdaten=SPI_shift_out_byte(out_startdaten);
   //OSZIHI;
   _delay_us(delayfaktor*out_PULSE_DELAY);
   in_lbdaten=SPI_shift_out_byte(out_lbdaten);
   
   _delay_us(delayfaktor*out_PULSE_DELAY);
   in_hbdaten=SPI_shift_out_byte(out_hbdaten);
   
   _delay_us(delayfaktor*out_PULSE_DELAY);
   for (byteindex=0;byteindex<SPI_BUFSIZE;byteindex++)
   {
      _delay_us(delayfaktor*out_PULSE_DELAY);
      inbuffer[byteindex]=SPI_shift_out_byte(outbuffer[byteindex]);
      //
   }
   _delay_us(delayfaktor*out_PULSE_DELAY);
   
   // Enddaten schicken: Zweiercomplement von in-Startdaten
   uint8_t complement = ~in_startdaten;
   
   in_enddaten=SPI_shift_out_byte(complement);
   
   _delay_us(delayfaktor*out_PULSE_DELAY);
   CS_HC_PASSIVE; // CS HI fuer Slave: Uebertragung abgeschlossen
   _delay_us(out_PULSE_DELAY);
   
   SPI_CLK_HI; // Clk sicher HI
   
   /*
   lcd_gotoxy(5,1);
   lcd_putint(out_startdaten);
   lcd_putc('*');
   lcd_putint(complement);
   lcd_putc('*');
   lcd_putint(in_enddaten);
   */
   lcd_gotoxy(19,1);
   
   if (out_startdaten + in_enddaten==0xFF)
   {
      lcd_putc('+');
      
   }
   else
   {
      lcd_putc('-');
      errCounter++;
      SPI_ErrCounter++;
    }
   
   
   //	lcd_gotoxy(17,3);
   //	lcd_puthex(errCounter & 0x00FF);
  // OSZIHI;
   spicounter++;
   
}

// end SPI

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
   TESTDDR |= (1<<TEST_PIN);
   TESTPORT |= (1<<TEST_PIN);
   
   
    SLAVE_OUT_DDR |= (1<<LAMPEEIN);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON
    SLAVE_OUT_DDR |= (1<<LAMPEAUS);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
    SLAVE_OUT_DDR |= (1<<OFENEIN);		//Pin 2 von PORT D als Ausgang fuer OFENEIN
    SLAVE_OUT_DDR |= (1<<OFENAUS);		//Pin 3 von PORT D als Ausgang fuer OFENAUS
   /*
   SERVODDR |= (1<<SERVOPIN1);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
    SERVODDR |= (1<<SERVOPIN0);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
    */
   
   //SLAVE_IN_DDR &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang fŸr Taste 1
   //SLAVE_IN_PORT |= (1<<PB0);	//Pull-up
   
   //SLAVE_IN_DDR &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr Taste 2
   //SLAVE_IN_PORT |= (1<<PB1);	//Pull-up
   
   //TIEFKUEHLALARM_PIN
   ALARM_IN_DDR &= ~(1<<TIEFKUEHLALARM_PIN);	//Pin 3 von PORT B als Eingang fŸr Tiefkuehlalarm
   ALARM_IN_PORT |= (1<<TIEFKUEHLALARM_PIN);	//Pull-up
   
   //WASSERALARM_PIN
   
   ALARM_IN_DDR &= ~(1<<WASSERALARM_PIN);	//Pin 4 von PORT B als Eingang fŸr Wasseralarm
   ALARM_IN_PORT |= (1<<WASSERALARM_PIN);	//Pull-up
   
   //Manuell
   
   SLAVE_IN_DDR &= ~(1<<MANUELL_PIN);	//Pin als Eingang fŸr Manuell-Taste
   SLAVE_IN_PORT |= (1<<MANUELL_PIN);	//Pull-up
  
   
   LOOPLEDDDR |= (1<<LOOPLED);
   
   //LCD
   LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_A_PIN);	//Pin als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin als Ausgang fuer LCD
   
   // TWI vorbereiten
   TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang fŸr SDA
   TWI_PORT |= (1<<SDAPIN); // HI
   
   TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang fŸr SCL
   TWI_PORT |= (1<<SCLPIN); // HI
   
   ADCDDR &= ~(1<<ADC_A_PIN);
   ADCDDR &= ~(1<<ADC_B_PIN);
   
   TASTATURDDR &= ~(1<<TASTATURPIN);
   
   SlaveStatus=0;
   SlaveStatus |= (1<<TWI_WAIT_BIT);
   
   BUZZER_DDR |= (1<<BUZZER_PIN);
   
   OSZIPORTDDR |=(1<<PULSA);
   OSZIPORT |= (1<<PULSA);

}



// provisorisch fuer timing
void timer0 (void)
{
   // Timer fuer Servo
   //	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
   TCCR0B |= (1<<CS02);
   TCCR0B |= (1<<CS00);
   //8-Bit Timer, Timer clock = system clock/256
   //TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
   
   //	TCCR0 |= (1<<CS02);					//Takt 4 MHz /256 Intervall 64 us
   
   TIFR0 |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK0 |= (1<<TOIE0);			//Overflow Interrupt aktivieren
   TCNT0 = 0x00;					//RŸcksetzen des Timers
   
   //	SERVOPORT |= (1<<CONTROL_B);//	CONTROL_B setzen
}



ISR (TIMER0_OVF_vect)
{
   
   timer0counter0++;
   if (SlaveStatus & (1<<ALARMBIT))
   {
      BUZZER_PORT ^= (1<<BUZZER_PIN);
   }
   if (timer0counter0 >= 0x00FF)
   {
      if (TEST)
      {
      lcd_gotoxy(16,0);
      lcd_putint(timer0counter1&0xFF);
      }
      
      timer0counter0=0;
      timer0counter1++;
      //if (timer0counter1 >= 0xAFFF)
      {
         
         //timer0counter1;
//         SlaveStatus |= (1<<TWI_OK_BIT);
      }
   }
}

void timer1(void)
{
   
   SERVODDR |= (1<<SERVOPIN0);
/*
   TCCR1A = (1<<WGM10)|(1<<COM1A1)   // Set up the two Control registers of Timer1.
   |(1<<COM1B1);             // Wave Form Generation is Fast PWM 8 Bit,
   TCCR1B = (1<<WGM12)|(1<<CS12)     // OC1A and OC1B are cleared on compare match
   |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.
   
   OCR1A = 63;                       // Dutycycle of OC1A = 25%
   //OCR1B = 127;                      // Dutycycle of OC1B = 50%

   return;
   */
   // https://www.mikrocontroller.net/topic/83609
   OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 1000;
   OCR1A = Servoposition[2];
   //OCR1B = 0x0FFF;
   ICR1 = 0xC3FF;          // Pulsabstand 50 ms  0x9FFF: 40ms

  // TCCR1A |= (1<<COM1A0);
   TCCR1A |= (1<<COM1A1); // clear on compare match
   TCCR1A |= (1<<WGM11);
   
   TCCR1B |= (1<<WGM12);
   TCCR1B |= (1<<WGM13);
   
   TCCR1B |= (1<<CS11);
  // TCCR1B |= (1<<CS10);
   
   
   
 //  TIMSK |= (1<<OCIE1A) | (1<<TICIE1); // OC1A Int enablad
}



void timer2 (void)
{
   //----------------------------------------------------
   // Set up timer 0 to generate interrupts @ 1000Hz
   //----------------------------------------------------
   TCCR2A = _BV(WGM20);
   TCCR2B |= (1<<CS20) | (1<<CS22);
   OCR2A = 0x2;
   TIMSK2 |= (1<<OCIE2A);
   
   
   // Timer fuer Exp
   //	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
   //	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
   
   //Timer fuer Servo
   /*
    TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
    
    TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
    TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
    TCNT0 = 0x00;					//RŸcksetzen des Timers
    */
}

ISR(TIMER2_COMPA_vect)
{
   
   TCNT2=0;
   if(EventCounter < 0x8FFF)// am Zaehlen, warten auf beenden von TWI // 0x1FF: 2 s
   {
      
   }
   else // Ueberlauf, TWI ist beendet,SPI einleiten
   {
      
      EventCounter =0;
      //lcd_gotoxy(0,0);
      //lcd_puthex(webspistatus);
      if (!(webspistatus & (1<<TWI_WAIT_BIT)))        // TWI soll laufen
      {
         //     if (cronstatus & (1<<CRON_HOME)) // eventuell nach xxx verschieben
         {
            webspistatus |= (1<<SPI_SHIFT_BIT);         // shift_out veranlassen
            //         cronstatus &=  ~Ê(1<<CRON_HOME); // nur ein shift-out nach cron-Request
         }
      }
      
      if (webspistatus & (1<<TWI_STOP_REQUEST_BIT))	// Gesetzt in cmd=2: Vorgang Status0 von HomeServer ist angemeldet
      {
         webspistatus |= (1<<SPI_STATUS0_BIT);			// STATUS 0 soll noch an Master gesendet werden.
         
         webspistatus &= ~(1<<TWI_STOP_REQUEST_BIT);	//Bit zuruecksetzen
      }
      
      
      if (webspistatus & (1<<SPI_STATUS0_BIT))
      {
         webspistatus |= (1<<TWI_WAIT_BIT);				// SPI/TWI soll in der naechsten schleifen nicht mehr ermoeglicht werden
         pendenzstatus |= (1<<SEND_STATUS0_BIT);		// Bestaetigung an Homeserver schicken, dass Status 0 angekommen ist. In cmd=10 zurueckgesetzt.
         
         webspistatus &= ~(1<<SPI_STATUS0_BIT);			//Bit zuruecksetzen
      }
      // xxx
      
   }
   
   EventCounter++;
   
}

uint8_t search_sensors(void)
{
   uint8_t i;
   uint8_t id[OW_ROMCODE_SIZE];
   uint8_t diff, nSensors;
   
   
   ow_reset();
   
   nSensors = 0;
   
   diff = OW_SEARCH_FIRST;
   while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS )
   {
      DS18X20_find_sensor( &diff, &id[0] );
      
      if( diff == OW_PRESENCE_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("No Sensor found\0" );
         
         delay_ms(800);
         lcd_clr_line(1);
         break;
      }
      
      if( diff == OW_DATA_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("Bus Error\0" );
         break;
      }
      lcd_gotoxy(4,1);
      
      for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      {
         //lcd_gotoxy(15,1);
         //lcd_puthex(id[i]);
         
         gSensorIDs[nSensors][i] = id[i];
         //delay_ms(100);
      }
      
      nSensors++;
   }
   
   return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void)
{
   
   gTemp_measurementstatus=0;
   if ( DS18X20_start_meas(NULL) != DS18X20_OK)
   {
      gTemp_measurementstatus=1;
   }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
   uint8_t i;
   uint8_t subzero, cel, cel_frac_bits;
   for ( i=0; i<gNsensors; i++ )
   {
      
      if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
                             &cel, &cel_frac_bits) == DS18X20_OK )
      {
         gTempdata[i]=cel*10;
         gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
         if (subzero)
         {
            gTempdata[i]=-gTempdata[i];
         }
      }
      else
      {
         gTempdata[i]=0;
      }
   }
}


// Code 1_wire end


volatile uint8_t testwert=17;

void main (void)
{
 
   slaveinit();

   //PORT2 |=(1<<PC4);
   //PORTC |=(1<<PC5);
   //init_twi_slave (SLAVE_ADRESSE);
   //uint16_t ADC_Wert= readKanal(0);
   sei();
   
   /* initialize the LCD */
   //LCD
   LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_A_PIN);	//Pin als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin als Ausgang fuer LCD

   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   lcd_puts("Guten Tag\0");
   delay_ms(1000);
   lcd_cls();
   lcd_puts(RAUM);
   
   
   
   uint8_t TastaturCount=0;
   //uint8_t Servowert=0;
   //uint8_t Servorichtung=1;
   
   uint16_t TastenStatus=0;
   uint16_t Tastencount=0;
   uint16_t Tastenprellen=0x003;
   uint8_t Schalterposition=0;
   
   // Timer fuer SPI-
   //timer2();
   
   //	initADC(TASTATURPIN);
   
   //	wdt_enable(WDTO_2S);
   
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;

   
   Init_SPI_Master();
   
   
#pragma mark DS1820 init
   // DS1820 init-stuff begin
   uint8_t i=0;
   uint8_t nSensors=0;
   ow_reset();
   gNsensors = search_sensors();
   
   delay_ms(100);
   lcd_gotoxy(0,0);
   lcd_puts("Sens: \0");
   lcd_puthex(gNsensors);
   if (gNsensors>0)
   {
      lcd_clr_line(1);
      start_temp_meas();
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }
   // DS1820 init-stuff end

   
   
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
      //lcd_gotoxy(18,1);
      //lcd_puts("W\0");
   }
   timer0();
   timer1();
   lcd_cls();
   
   initADC(TASTATURPIN);
   
   
#pragma mark while
   while (1)
   {
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      if (((loopcount0 & 0xFF)==0) && (SlaveStatus & (1<<MANUELLBIT)))
      {
   
         //OSZILO;
         //uint16_t tastaturadc=(readKanal(TASTATURPIN));
         Tastenwert = ((readKanal_raw(TASTATURPIN))>>2);
         //Tastenwert++;
         //OSZIHI;
   
      }
      //
      if (loopcount0==0xBFFF)
      {
 //        OSZILO;
 //        Tastenwert=(readKanal_raw(TASTATURPIN)>>2);
 //       OSZIHI;
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //BUZZER_PORT ^= (1<<BUZZER_PIN);
         loopcount1++;
         //lcd_gotoxy(10,0);
         //lcd_putc('A');
         //lcd_puts("rx");
         //lcd_putint(loopcount1);
         if (loopcount1 >= 0x0F)
         {
            

            
            loopcount1=0;
            Servostellung++;
            OCR1A = Servoposition[Servostellung %8];
            //lcd_gotoxy(0,0);
            //lcd_puts("rx");
            //lcd_putint(rxdata);
            //lcd_gotoxy(10,1);
            //lcd_putc('s');
            //lcd_putint(SlaveStatus);

           spannungA = (readKanal(ADC_A_PIN));
           spannungB = (readKanal(ADC_B_PIN));
            
            adccounter++;
            if (TEST || (!(TESTPIN & (1<<TEST_PIN))))
            {
               rxdata=1;
               SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON, Simulation Startroutine
               

            }
            
 

         }
         
         
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
            lcd_gotoxy(16,0);
            lcd_puts(" TWI");
            init_twi_slave (SLAVE_ADRESSE);
            sei();
            SlaveStatus &= ~(1<<TWI_WAIT_BIT);
            
            SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
            
            // StartDelayBit zuruecksetzen
         }
         
         else
         {
            lcd_gotoxy(16,0);
            lcd_puts("WAIT");

         }
         /*
         //lcd_gotoxy(16,1);
         //lcd_puts("WAIT");
        */
      }
      if (TEST || (!(TESTPIN & (1<<TEST_PIN))))
      {
         SlaveStatus &= ~(1<<TWI_WAIT_BIT); // simulation TWI
         SlaveStatus |= (1<<TWI_OK_BIT);
      }

      
  //    lcd_gotoxy(10,1);
  //    lcd_puts("X");
      

      ;
      //lcd_gotoxy(10,1);
      //lcd_putc('B');
      /**	Ende Startroutinen	***********************/
      
      /***** rx_buffer abfragen **************** */
      //rxdata=0;
      
      //***************
      //	Test
      if (TEST)
      {
      //   rxdata=1;
      }
      //SlaveStatus |= (1<<TWI_OK_BIT);
      
      // end test
      //***************
      
 #pragma mark TWI_OK
      if ((SlaveStatus & (1<<TWI_OK_BIT)) &&(rxdata) && !(SlaveStatus & (1<<MANUELLBIT)))	//Daten von TWI liegen vor und MANUELLBIT ist OFF
      {
         //OSZILO;
         lcd_gotoxy(0,1);
         lcd_putint(spannungA>>2);
         
         lcd_putc(' ');
         lcd_putint12(spannungB);
         //lcd_putc('T');
         //lcd_putint(Tastenwert);

         //lcd_gotoxy(10,1);
         //lcd_putc('C');
         testcounter++;
         //lcd_gotoxy(0,1);
         //lcd_putint(testcounter);
         //lcd_putc(' ');
         //lcd_putint(spicounter);
         
         if (TEST || (!(TESTPIN & (1<<TEST_PIN))))
         {
            SlaveStatus &= ~(1<<TWI_OK_BIT); // simulation TWI
         }
         /*
          
          if (rxbuffer[3] < 6)
          {
          if (Servorichtung && (Servowert<4))// vorwŠrts
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
          
          //RingD2(2);
          //delay_ms(20);
          */
         Lampestatus=rxbuffer[0];
         //lcd_gotoxy(12,0);
         //lcd_puts("L:\0");
         //lcd_puthex(Lampestatus);
         //delay_ms(20);
         
         /*
          // TWI_NEW_BIT wird in twislave-ISR gesetzt, wenn alle Daten aufgenommen sind
          
          if (SlaveStatus & (1<<TWI_NEW_BIT))
          {
          SlaveStatus &= ~(1<<TWI_NEW_BIT); // Die Aktionen sollen nur einmal ausgeloest werden
          */
         
   #pragma mark Lampe
         // Lampe
         
         if (TEST)
         {
            //Lampestatus  |= (1<<LAMPEBIT);
            Radiatorstatus |= (1<< OFENBIT);
         }
         lcd_gotoxy(8,0);
         lcd_putc('L');
         
         if ( Lampestatus  & (1<<LAMPEBIT)) // Bit 0
         {
            //delay_ms(1000);
            //Lampe ein
            SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
            SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
            SLAVE_OUT_PORT |= (1<<LAMPEEIN);
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);
            lcd_putc('1');
            
            //lcd_gotoxy(15,1);
            //lcd_puts("ON \0");
         }
         else
         {
            //delay_ms(1000);
            //Lampe aus
            //lcd_gotoxy(19,1);
            //lcd_putc('0');
            
            SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
            SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
            SLAVE_OUT_PORT |= (1<<LAMPEAUS);
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);
            lcd_putc('0');
            
            //lcd_gotoxy(15,1);
            //lcd_puts("OFF\0");
            
         }
         
         // Ofen
         
         Radiatorstatus=rxbuffer[1];
         
         if (TEST)
         {
            /*
             SLAVE_OUT_PORT |= (1<<LAMPEAUS); // Impuls an OFF
             delay_ms(30);
             SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);
             delay_ms(200);
             SLAVE_OUT_PORT |= (1<<LAMPEEIN); // Impuls an ON
             delay_ms(30);
             SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);
             */
            
         }
         lcd_gotoxy(10,0);
         lcd_putc('R');
         
 #pragma mark Ofen
         //if ( Slavestatus  & (1<<OFENBIT)) // Bit 1
         if ( Radiatorstatus & (1<<OFENBIT)) // //Bit 1 Ofen ein
         {
            //delay_ms(1000);
            //Ofen ein
            
            //lcd_putc('I');
            SLAVE_OUT_PORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
            SLAVE_OUT_PORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
            SLAVE_OUT_PORT |= (1<<OFENEIN); // Impuls an ON
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<OFENEIN);
            //lcd_gotoxy(15,1);
            lcd_putc('1');
         }
         else
         {
            //delay_ms(1000);
            //Ofen aus
            //lcd_putc('0');
            SLAVE_OUT_PORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
            SLAVE_OUT_PORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
            SLAVE_OUT_PORT |= (1<<OFENAUS); // Impuls an OFF
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<OFENAUS);
            //lcd_gotoxy(15,1);
            lcd_putc('0');
         }
         
         
         
         // ****************************
         //	tx_buffer laden
         // ****************************
         
         //lcd_clr_line(1);
         
         // Thermomenter analog:
         // Temperatur lesen
         // uint16_t tempBuffer=0;
         // initADC(INNEN);
         // tempBuffer=readKanal(INNEN);
         //lcd_gotoxy(0,1);
         //lcd_puts("T\0");
         //lcd_putint(tempBuffer>>2);
         //lcd_put_tempbis99(tempBuffer>>2);
         
         //  txbuffer[INNEN]=(uint8_t)(tempBuffer>>2);// Vorlauf
#pragma mark Sensors
         // Temperatur messen mit DS18S20
         if (gNsensors) // Sensor eingeseteckt
         {
            start_temp_meas();
            delay_ms(800);
            read_temp_meas();
            uint8_t line=0;
            //Sensor 1
            lcd_gotoxy(0,line);
            lcd_puts("T:     \0");
            if (gTempdata[0]/10>=100)
            {
               lcd_gotoxy(3,line);
               lcd_putint((gTempdata[0]/10));
            }
            else
            {
               lcd_gotoxy(2,line);
               lcd_putint2((gTempdata[0]/10));
            }
            
            lcd_putc('.');
            lcd_putint1(gTempdata[0]%10);
         }
         txbuffer[INNEN]=2*((gTempdata[0]/10)& 0x00FF);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
         // Halbgrad addieren
         if (gTempdata[0]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
         {
            txbuffer[INNEN] +=1;
         }
         
 #pragma mark Kuehltruhe/Wasser
         //
         //	Kuehltruhe abfragen
         //
         if (ALARM_IN_PIN & (1<<TIEFKUEHLALARM_PIN)) // HI, Alles OK
         {
            txbuffer[STATUS] &= ~(1<<TIEFKUEHLALARM_PIN); // TIEFKUEHLALARM_PIN zuruecksetzen Bit 3
            lcd_gotoxy(17,1);
            lcd_putc('-');
         }
         else
         {
            txbuffer[STATUS] |= (1<<TIEFKUEHLALARM_PIN);	// TIEFKUEHLALARM_PIN setzen
            lcd_gotoxy(17,1);
            lcd_putc('t');
         }
         
         //
         //	Wasseralarm abfragen
         //
         if (ALARM_IN_PIN & (1<<WASSERALARM_PIN)) // HI, Alles OK
         {
            txbuffer[STATUS] &= ~(1<<WASSERALARM_PIN); // WASSERALARM_PIN zuruecksetzen
            lcd_gotoxy(18,1);
            lcd_putc('-');
            
         }
         else
         {
            txbuffer[STATUS] |= (1<<WASSERALARM_PIN);	// WASSERALARM_PIN setzen
            lcd_gotoxy(18,1);
            lcd_putc('w');
            
         }
         
         //lcd_gotoxy(17,1);
         //lcd_puts("TKA\0");
         
         
         /*
          if (PINB & (1<<EINGANG0BIT))
          {
          lcd_puts("OFF\0");
          }
          else
          {
          lcd_puts("ON \0");
          }
          */
         // lcd_gotoxy(0,1);
         // lcd_putint(rxbuffer[0]);
         rxdata=0;               // TWI erledigt
         //OSZIHI;
         
#pragma mark SPI
         /***** SPI: Daten von SPI_Slave_Strom abfragen **************** */
         //out_enddaten = 0xA4;
         
         inbuffer[0]=0;
         inbuffer[1]=0;
         inbuffer[2]=0;
         
         testwert++;
         
         outbuffer[0] = testwert;
         outbuffer[1] = 0;//testwert;
         outbuffer[2] = 0;//testwert;
         
         // if (TEST)
         {
            //lcd_gotoxy(0,1);
            //lcd_puts("SPI");
            out_startdaten = 0xA1;
            out_hbdaten = 0xA2;
            out_lbdaten = 0xA3;
            //lcd_gotoxy(0,1);
            //lcd_putint16(gTempdata[0]);
            
            lcd_gotoxy(0,2);
            lcd_puts("oS \0");
            lcd_putint(outbuffer[0]);
            lcd_putc('*');
            //lcd_putint(outbuffer[1]);
            //lcd_putc('*');
            //lcd_putint(outbuffer[2]);
            lcd_putc('s');
            lcd_putint(out_startdaten);
            
         }
         OSZILO;
         
#pragma mark SPI_shift_out
         //****************************
      
         SPI_shift_out(); // delayfaktor 2: 80ms aktueller delayfaktor 16: 150ms
         //****************************
         OSZIHI;
         //    if (TEST)
         {
            lcd_gotoxy(0,3);
            lcd_puts("iS \0");
            lcd_putint(inbuffer[0]);
            lcd_putc('*');
            lcd_putint(inbuffer[1]);
            lcd_putc('*');
            lcd_putint(inbuffer[2]);
            lcd_putc('*');
            lcd_putint(in_startdaten);
            
         }
         
         lcd_gotoxy(13,0);
         lcd_putint(SPI_ErrCounter);

         lcd_gotoxy(12,2);
         lcd_putint16(inbuffer[0]+0xFF*inbuffer[1]);
         
         txbuffer[STROML]= inbuffer[0]; // L: byte 4
         txbuffer[STROMH] = inbuffer[1]; // H: byte 5
         txbuffer[STROMHH] = inbuffer[2]; // HH: byte 6
         
         /***** End SPI**************** */
         
         //****************************
         //	end tx_buffer laden
         //****************************
         //OSZIHI;
      }
      else
      {
         /*
         lcd_gotoxy(10,1);
         lcd_puts("st\0");
         lcd_putint(SlaveStatus);
         lcd_putc(' ');
         lcd_putc('D');
          */
      }
      
     
      if (!(SLAVE_IN_PIN & (1<<MANUELL_PIN))) // Taste 0 gedrueckt
      {
        // lcd_gotoxy(12,1);
        // lcd_puts("P0");
         
         if (! (TastenStatus & (1<<MANUELLBIT))) //Taste 0 war noch nicht gedrueckt
         {
            //RingD2(5);
            TastenStatus |= (1<<MANUELLBIT);
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
               TastenStatus ^= (1<<MANUELLBIT);
               if (SlaveStatus & (1<<MANUELLBIT))
               {
                  SlaveStatus &= ~(1<<MANUELLBIT); // MANUELLBIT OFF
                  lcd_gotoxy(19,3);
                  lcd_putc('-');
                  
               }
               else
               {
                  SlaveStatus |= (1<<MANUELLBIT); // MANUELLBIT ON
                  lcd_gotoxy(19,3);
                  lcd_putc('M');
                  
               }

               Tastencount=0;
               TastenStatus &= ~(1<<MANUELLBIT);
               
            }
         }//else
         
      }
      else
      {

      }
      
      /* ******************** */
      
      
#pragma mark Tastatur
      //lcd_putc('X');
      //Tastenwert=0;
      //		initADC(TASTATURPIN);
      //		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
      
      //lcd_gotoxy(3,1);
      //lcd_putint(Tastenwert);
      
      if (Tastenwert>5)
      {
         /*
          0:
          1: Uhr ein
          2:
          3: Uhr aus
          4: Schalterpos -
          5: MANUELLBIT ein
          6: Schalterpos +
          7:
          8:
          9:
          
          12: MANUELLBIT aus
          */
         
         TastaturCount++;
         if (TastaturCount>=10)
         {
            //lcd_clr_line(1);
            //lcd_gotoxy(6,1);
            //lcd_puts("T:\0");
            //lcd_putint(Tastenwert);
            uint8_t Taste=Tastenwahl(Tastenwert);
            
            lcd_gotoxy(10,1);
            //lcd_putint(Tastenwert);
            //lcd_putc(' ');
            lcd_putint2(Taste);
            //delay_ms(600);
            // lcd_clr_line(1);
            
            
            TastaturCount=0;
            //Tastenwert=0x00;
            //				uint8_t pos=0;
            //				lcd_gotoxy(18,1);
            //				lcd_putint2(Taste);
            
            //Taste = 0x0F; //sicher nichts
            switch (Taste)
            {
               case 0://
               {
                  SlaveStatus ^= (1<<MANUELLBIT);
                  if (SlaveStatus & (1<<MANUELLBIT))
                  {
                     //SlaveStatus &= ~(1<<MANUELLBIT); // MANUELLBIT OFF
                     lcd_gotoxy(19,3);
                     lcd_putc('-');

                  }
                  else
                  {
                     //SlaveStatus |= (1<<MANUELLBIT); // MANUELLBIT ON
                     lcd_gotoxy(19,3);
                     lcd_putc('M');

                  }

                  //SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo ein
                  //Schalterposition=0;
                  //Servoimpulsdauer=Servoposition[Schalterposition];
               }break;
                  
               case 1:	//	Uhr ein
               {
                  if (SlaveStatus & (1<<MANUELLBIT))
                  {
                   }
               }break;
                  
               case 2://
               {
                  
                  
                  
               }break;
                  
               case 3: //	Lampe aus
               {
                  if (SlaveStatus & (1<<MANUELLBIT))
                  {
                   }
               }break;
                  
               case 4://
               { 
                  if ((SlaveStatus & (1<<MANUELLBIT)) &&  Schalterposition)
                  {
                     Schalterposition--;
                     //Servoimpulsdauer=Servoposition[Schalterposition];
                  }
                  
               }break;
                  
               case 5://
               { 
                  //Slavestatus |= (1<<MANUELLBIT);	// MANUELLBIT ON
                  //PORTD |= (1<<PD3);
                  //SERVOPORT |= (1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo ein
                  if (SlaveStatus & (1<<MANUELLBIT))
                  {
                     Schalterposition=0;
                     //Servoimpulsdauer=Servoposition[Schalterposition];
                  }
                  
               }break;
                  
               case 6://
               { 
                  if ((SlaveStatus & (1<<MANUELLBIT)) && (Schalterposition<4))
                  {
                     Schalterposition++;
                     //Servoimpulsdauer=Servoposition[Schalterposition];
                  }
               }break;
                  
               case 7://
               { 
                  //if (Servoimpulsdauer>Servoposition[0])
                  {
                     //Servoimpulsdauer--;
                     //lcd_gotoxy(0,16);
                     //lcd_putint2(Servoimpulsdauer);
                  }
                  
               }break;
                  
               case 8://
               { 
                  
               }break;
                  
               case 9://
               { 
                  //if (Servoimpulsdauer<Servoposition[4])
                  {
                     //Servoimpulsdauer++;
                     //lcd_gotoxy(0,2);
                     //lcd_putint2(Servoimpulsdauer);
                  }
               }break;
                  //lcd_gotoxy(8,1);
                  //lcd_puts("P:\0");
                  //lcd_putint2(Schalterposition);
                  
               case 10://
               { 
                  
               }break;
                  
               case 11://
               { 
                }break;
                  
               case 12:
               {
                  
                   //SERVOPORT &= ~(1<<SERVOPIN1);//	SERVOPIN1 zuruecksetzen: Servo aus
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
