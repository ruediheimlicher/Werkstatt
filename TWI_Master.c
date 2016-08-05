//
//  TWI_Master.c
//  TWI_Master
//
//  Created by Sysadmin on 19.03.08.
//  Copyright Ruedi Heimlicher 2008. All rights reserved.
//



#include <avr/io.h>
# include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <ctype.h>

# include  "TWI_Master.h"
//# include "Funktionen.c"
# include "twimaster.c"
# include "lcd.c"
# include "err.c"
# include "adc.c"

# include "slaves.c"
# include "display.c"
#include "datum.c"
#include "version.c"
//#include "web_SPI.c"
#include "SPI_slave.c"
//#include "ip_arp_udp_tcp.c"
//#include "websrv_help_functions.c"
//#include "enc28j60.h"
#include "net.h"

#include "rtc.c"


/* ************************************** */
volatile uint8_t rxdata =0;
static uint8_t  webtaskflag =0;



/* *** SPI *********************************** */
uint8_t testCounterON =0;
uint8_t testCounterOFF =0;
uint8_t loopCounterTWI =0;// Anzahl TWI-Loops
uint8_t loopCounterSPI =0;// Anzhl SPI-Loops
/* *** end SPI *********************************** */

// Importierte Daten vom Webserver
//volatile					uint8_t WebRxDaten[twi_buffer_size];
//volatile					uint8_t WebRxStartDaten;

//Daten fuer den Export zum Webserver
//static volatile		uint8_t WebTxDaten[twi_buffer_size];
//																					static volatile		uint8_t WebTxStartDaten;

// Importierte Daten vom Webserver
//static volatile		uint8_t EEPROMRxDaten[twi_buffer_size];
//static volatile		uint8_t EEPROMRxStartDaten;

//Daten fuer den Export zum Webserver
static volatile		uint8_t EEPROMTxDaten[twi_buffer_size];
static volatile		uint8_t EEPROMTxStartDaten;

// Importierte Daten vom Webserver
//static volatile		uint8_t SolarRxDaten[twi_buffer_size];
//static volatile		uint8_t SolarRxStartDaten;

//Daten fuer den Export zum Webserver
//static volatile		uint8_t SolarTxDaten[twi_buffer_size];
//static volatile		uint8_t SolarTxStartDaten;


static volatile		uint8_t hbyte;
static volatile		uint8_t lbyte;


static volatile		uint8_t aktuelleDatenbreite=8;

static volatile		uint8_t Testposition=0;;
static volatile		uint8_t alteposition=0;

static volatile      uint8_t Write_Device;
static volatile      uint8_t Read_Device;


// TWI-Fehlercodes
static volatile uint8_t twistatus =0;

// Kontakt zum eeprom
static volatile uint8_t eepromstatus =0;
static volatile uint8_t pwmstatus =0;


static volatile uint8_t Write_Err;
static volatile uint8_t Read_Err;
static volatile uint8_t EEPROM_Err;


static volatile uint8_t Echo=0;
static volatile uint8_t olderrcounter=0;
/* ************************************** */
//static char d[3];
//static char* key1;
//static char* sstr;
//char VarString[64];
//static char DataString[48];
//static char EEPROM_String[96];

static char SolarString[48];

//static char d[3];
//static char* key1;
//static char* sstr;


//#define test 0


#define SCLPIN		0
#define SDAPIN		1

#define WEB_ON		1
#define IOW_TYP		24

#define PRELL 2

#define HEIZUNG		0
#define WERKSTATT		1
#define WOZI			2
#define BUERO			3
#define LABOR			4
#define OG1				5
#define OG2				6
#define ESTRICH		7

#define FEHLERBYTE	24		//Beginn der Fehlermeldungen


#define buffer_size			8

#define ERRTASK				0xA0	// Fehlertask an Webserver schicken, soll Eintrag ins Log veranlassen
#define ERR_UHR				0xA0



#define NULLTASK				0xB0	// Nichts tun
#define STATUSTASK			0xB1	// Status des TWI aendern
#define STATUSCONFIRMTASK	0xB2	// Statusaendern bestaetigen

#define EEPROMREPORTTASK	0xB4	// Daten von EEPROM an HomeServer schicken
#define EEPROMCONFIRMTASK	0xB5  // Quittung an HomeCentral senden
#define EEPROMRECEIVETASK	0xB6	// Start-Adresse von EEPROM empfangen
#define EEPROMWRITETASK		0xB7	// auf EEPROM schreiben
#define EEPROMREADTASK		0xB8	// von EEPROM lesen
#define EEPROMSENDTASK		0xB9	// von EEPROM lesen

#define PWMREADTASK        0xBA  // PWM-Daten vom EEPROM lesen und an Zielraum schicken (synchronisierung)

#define DATATASK				0xC0	// Normale Loop im Webserver

#define SOLARTASK				0xC1	// Data von solar

#define MASTERERRTASK		0xC7	// Fehlermeldung vom Master senden




// defines fuer Alarm
#define TIEFKUEHLALARM        3
#define WASSERALARMKELLER     4
#define WASSERALARMESTRICH    1

// defines fuer spistatus



#define SPI_SHIFT_IN_OK_BIT	6

// defines fuer BUS-Status
#define SPI_SENDBIT				0
#define TWI_CONTROLBIT			1				// Statusbit fuer TWI
#define WEB_CONTROLBIT			2				// Statusbit fuer Web

// Uhr

volatile uint8_t  uhrstatus =0;

// defines fuer uhrstatus
#define SYNC_OK		0	// Uhr ist synchronisiert
#define SYNC_WAIT		1	// Uhr ist  wartet auf Synchronisation
#define SYNC_READY	2	// DCF77 hat gueltiges Datum 
#define SYNC_CHECK	3	// DCF77 soll Datum bereitstellen, Anzahl korrekte Daten abwarten
#define SYNC_NULL		4	// Uhr ist undefiniert, wartet auf Synchronisation (nach restart)
#define SYNC_NEW		5	// erste Synchrinisation nach Reset, noch keine gueltige Zeit

volatile uint8_t  DCF77_counter =0; // Anzahl gueltige Datumspakete in Folge
#define MIN_SYNC		2	// Anzahl gueltige Daten fuer Synchronisation


// defines fuer EEPROMstatus
#define PWM_READ     0  // Daten fuer PWM lesen


#define TASTE1 38
#define TASTE2 46
#define TASTE3 54
#define TASTE4 72
#define TASTE5 95
#define TASTE6 115
#define TASTE7 155
#define TASTE8 186
#define TASTE9 205
#define TASTEL 225
#define TASTE0 235
#define TASTER 245

#define STARTDELAY 0x0FF
//#define STARTDELAY 0

#define WOCHENPLANBREITE 0x40;

#define START_BYTE_DELAY	12		// Timerwert fuer Start-Byte
#define BYTE_DELAY			12		// Timerwert fuer Data-Byte


// defines fuer PWM
#define PWM_CODEBIT        0     //Es folgt ein TWI-Paket mit den Positionsdaten fuer den PWM. Diese werden in den Array Servoposition geladen
#define PWM_SCHALTERBIT    1     // Byte 7 enthaelt Schalterposition, die eingestellt werden soll
#define PWM_POSITIONBIT    2     // Byte 7 enthaelt Impulslaenge fuer PWM


uint8_t Raum_Thema=0x00;			//	Bit 4-7: Thema		Bit 0-3: Raum
uint8_t Objekt_Wochentag=0;		//	Bit 4-7: Objekt	Bit 0-3: Wochentag
uint8_t Stunde_Minute=0;
uint8_t Menu_Ebene=0;

//uint8_t ByteCounter=0;

unsigned char mcusr_mirror __attribute__ ((section (".noinit"))); // Watchdog ausschalten 

void get_mcusr(void) \
      __attribute__((naked)) \
      __attribute__((section(".init3"))); 
    
	 void get_mcusr(void) 
    { 
     mcusr_mirror = MCUSR; 
     MCUSR = 0; 
    wdt_disable(); 
    } 

/* ***** EEPROM-Stuff ********************************* */
uint8_t EEMEM ip0;
uint8_t EEMEM ip1;
uint8_t EEMEM ip2;
uint8_t EEMEM ip3;
uint8_t EEMEM ipstring[10];

/* ************************************** */


/* ************************************** */

// the password string (only the first 5 char checked), (only a-z,0-9,_ characters):
static char *errmsg; // error text

static volatile uint8_t Temperatur;

/* ************************************** */



void timer0(void);



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


void tempbis99(uint16_t temperatur,char*tempbuffer)
{
	char buffer[8]={};
	//uint16_t temp=(temperatur-127)*5;
	uint16_t temp=temperatur*5;
	
	//itoa(temp, buffer,10);
	
	r_itoa16(temp,buffer);
	
	//lcd_puts(buffer);
	//lcd_putc('*');
	
	//char outstring[7]={};
	
	tempbuffer[6]='\0';
	tempbuffer[5]=' ';
	tempbuffer[4]=buffer[6];
	tempbuffer[3]='.';
	tempbuffer[2]=buffer[5];
	if (abs(temp)<100)
	{
		tempbuffer[1]=' ';
		
	}
	else 
	{
		tempbuffer[1]=buffer[4];
		
	}		
	tempbuffer[0]=buffer[0];
}

void tempAbMinus20(uint16_t temperatur,char*tempbuffer)
{	
	char buffer[8]={};
	int16_t temp=(temperatur)*5;
	temp -=200;
	char Vorzeichen=' ';
	if (temp < 0)
	{
		Vorzeichen='-';
	}
	
	r_itoa16(temp,buffer);
	//		lcd_puts(buffer);
	//		lcd_putc(' * ');
	
	//		char outstring[7]={};
	
	tempbuffer[6]='\0';
	//outstring[5]=0xDF; // Grad-Zeichen
	tempbuffer[5]=' ';
	tempbuffer[4]=buffer[6];
	tempbuffer[3]='.';
	tempbuffer[2]=buffer[5];
	if (abs(temp)<100)
	{
		tempbuffer[1]=Vorzeichen;
		tempbuffer[0]=' ';
	}
	else
	{
		tempbuffer[1]=buffer[4];
		tempbuffer[0]=Vorzeichen;
	}
	//		lcd_puts(outstring);
}





/*
 Der Buffer, in dem die empfangenen Daten gespeichert werden. Der Slave funktioniert ähnlich  wie ein normales
 Speicher-IC [I2C-EEPROM], man sendet die Adresse, an die man schreiben will, dann die Daten. Die interne Speicher-Adresse
 wird dabei automatisch hochgezählt
 */
volatile uint8_t rxbuffer[buffer_size];
//volatile uint8_t rxstartbuffer=0;
//volatile uint8_t WebRxDaten[tag_data_size];
//static volatile uint8_t WebRxStartDaten=0;

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];//={0,0,0,0,0,0,0,0};
//volatile uint8_t txstartbuffer=0;
uint8_t senderfolg=0;

volatile uint8_t  test =0; // Anzahl gueltige Datumspakete in Folge
volatile uint16_t  testcounterL =0; // Zähler fuer timer0: Takt fuer SPI bei test
volatile uint16_t  testcounterH =0;

static volatile uint8_t StartDaten;

/*Der Buffer fuer die Output-Daten des Webservers.*/
//volatile uint8_t WebTxDaten[eeprom_buffer_size];
//volatile uint8_t WebTxDaten[data_buffer_size];
// Buffer fuer den Output-Code des Pakets
//volatile uint8_t WebTxStartDaten;

/*Der Buffer fuer die Output-Daten des EEPROMS.*/
volatile uint8_t EEPROMTXdaten[eeprom_buffer_size];
// Buffer fuer den Output-Code des Pakets
volatile uint8_t EEPROMTXStartdaten;



/*Der Buffer fuer die Input-Daten der Heizung.*/
volatile uint8_t HeizungRXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten der Heizung.*/
volatile uint8_t HeizungTXdaten[buffer_size];

/*Der Buffer fuer die Input-Daten des Labors*/
volatile uint8_t LaborRXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten des Labors*/
volatile uint8_t LaborTXdaten[buffer_size];

/*Der Buffer fuer die Input-Daten des Bueros*/
volatile uint8_t BueroRXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten des Bueros*/
volatile uint8_t BueroTXdaten[buffer_size];

/*Der Buffer fuer die Input-Daten der Werkstatt*/
volatile uint8_t WerkstattRXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten der Werkstatt*/
volatile uint8_t WerkstattTXdaten[buffer_size];


/*Der Buffer fuer die Input-Daten des WoZi*/
volatile uint8_t WoZiRXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten der Werkstatt*/
volatile uint8_t WoZiTXdaten[buffer_size];

/*Der Buffer fuer die Input-Daten des OG1*/
volatile uint8_t OG1RXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten des OG1*/
volatile uint8_t OG1TXdaten[buffer_size];

/*Der Buffer fuer die Input-Daten des OG2*/
//volatile uint8_t OG2RXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten des OG2*/
volatile uint8_t OG2TXdaten[buffer_size];

/*Der Buffer fuer die Input-Daten des Estrichs*/
volatile uint8_t EstrichRXdaten[buffer_size];
/*Der Buffer fuer die Output-Daten des Estrichs*/
volatile uint8_t EstrichTXdaten[buffer_size];


static volatile uint8_t min=0;
static volatile  uint8_t std=0;
static volatile  uint8_t tag=0;
	
static volatile uint8_t oldmin=0;
static volatile  uint8_t oldstd=0;
static volatile  uint8_t oldtag=0;


uint16_t			Brennerzeit=0;

uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit
//uint8_t LaborDaten[8]={};
//uint8_t HeizungDaten[8]={};
//uint8_t EstrichDaten[8]={};

//Status Receive TWI
static volatile uint8_t LeseStatus=0;

//Status Transmit TWI
static volatile uint8_t SchreibStatus=0;

//Status WEB
volatile uint8_t WebStatus=0x00; //	Webserver abfragen


//Status MANUELL
//volatile uint8_t TWI_Status=0x00;//		Anfangsposition: TWI ist OFF

//Status Loop
volatile uint8_t BUS_Status=0x00;//		Anfangspos: TWI OFF, wird nach Startdelay eingeschaltet, WEB ist am Anfang OFF


#pragma mark Signal Status
//Status Timer
volatile uint8_t SIGNAL_Status=19;
volatile uint8_t RW=0; //lesen oder schreiben

//	SIGNAL_Count
volatile uint8_t SIGNAL_Count=0; // Anzahl Interrupts bis TWI abfragen




char* wochentagstring[] = {"MO","DI","MI","DO","FR","SA","SO"};

uint8_t AnzeigeWochentag=0; //


const char Heizung0[]  PROGMEM ="Status\0";
const char Heizung1[]  PROGMEM ="Plan\0";
const char Heizung2[]  PROGMEM ="Tag\0";
const char Heizung3[]  PROGMEM ="Nacht\0";
const char Heizung4[]  PROGMEM ="Heizung 4\0";
const char Heizung5[]  PROGMEM ="Heizung 5\0";
const char Heizung6[]  PROGMEM ="Heizung 6\0";
const char Heizung7[]  PROGMEM ="Heizung 7\0";
PGM_P const HeizungTable[] PROGMEM ={Heizung0, Heizung1, Heizung2, Heizung3, Heizung4, Heizung5, Heizung6, Heizung7};

const char Werkstatt0[] PROGMEM = "Status\0";
const char Werkstatt1[] PROGMEM = "Plan\0";
const char Werkstatt2[] PROGMEM = "WS 2\0";
const char Werkstatt3[] PROGMEM = "WS 3\0";
const char Werkstatt4[] PROGMEM = "WS 4\0";
const char Werkstatt5[] PROGMEM = "WS 5\0";
const char Werkstatt6[] PROGMEM = "WS 6\0";
const char Werkstatt7[] PROGMEM = "WS 7\0";
PGM_P const WerkstattTable[] PROGMEM = {Werkstatt0, Werkstatt1, Werkstatt2, Werkstatt3, Werkstatt4, Werkstatt5, Werkstatt6, Werkstatt7};


const char WoZi0[] PROGMEM = "Status\0";
const char WoZi1[] PROGMEM = "Plan\0";
const char WoZi2[] PROGMEM = "WoZi 2\0";
const char WoZi3[] PROGMEM = "WoZi 3\0";
const char WoZi4[] PROGMEM = "WoZi 4\0";
const char WoZi5[] PROGMEM = "WoZi 5\0";
const char WoZi6[] PROGMEM = "WoZi 6\0";
const char WoZi7[] PROGMEM = "WoZi 7\0";
//PGM_P  const WoZiTable[] PROGMEM = {WoZi0, WoZi1, WoZi2, WoZi3, WoZi4, WoZi5, WoZi6, WoZi7};


const char Buero0[] PROGMEM = "Status\0";
const char Buero1[] PROGMEM = "Plan\0";
const char Buero2[] PROGMEM = "Buero 2\0";
const char Buero3[] PROGMEM = "Buero 3\0";
const char Buero4[] PROGMEM = "Buero 4\0";
const char Buero5[] PROGMEM = "Buero 5\0";
const char Buero6[] PROGMEM = "Buero 6\0";
const char Buero7[] PROGMEM = "Buero 7\0";
//PGM_P  const BueroTable[] PROGMEM = {Buero0, Buero1, Buero2, Buero3, Buero4, Buero5, Buero6, Buero7};

const char Labor0[] PROGMEM = "Status\0";
const char Labor1[] PROGMEM = "Plan\0";
const char Labor2[] PROGMEM = "Labor 2\0";
const char Labor3[] PROGMEM = "Labor 3\0";
const char Labor4[] PROGMEM = "Labor 4\0";
const char Labor5[] PROGMEM = "Labor 5\0";
const char Labor6[] PROGMEM = "Labor 6\0";
const char Labor7[] PROGMEM = "Labor 7\0";
//PGM_P  const LaborTable[] PROGMEM = {Labor0, Labor1, Labor2, Labor3, Labor4, Labor5, Labor6, Labor7};

const char OG_10[] PROGMEM = "Status\0";
const char OG_11[] PROGMEM = "Plan\0";
const char OG_12[] PROGMEM = "OG_12\0";
const char OG_13[] PROGMEM = "OG_13\0";
const char OG_14[] PROGMEM = "OG_15\0";
const char OG_15[] PROGMEM = "OG_16\0";
const char OG_16[] PROGMEM = "OG_16\0";
const char OG_17[] PROGMEM = "OG_17\0";
//PGM_P  const OG_1Table[] PROGMEM = {OG_10, OG_11, OG_12, OG_13, OG_14, OG_15, OG_16, OG_17};

const char OG_20[] PROGMEM = "Status\0";
const char OG_21[] PROGMEM = "Plan\0";
const char OG_22[] PROGMEM = "OG2 2\0";
const char OG_23[] PROGMEM = "OG2 3\0";
const char OG_24[] PROGMEM = "OG2 4\0";
const char OG_25[] PROGMEM = "OG2 5\0";
const char OG_26[] PROGMEM = "OG2 6\0";
const char OG_27[] PROGMEM = "OG2 7\0";
//PGM_P  const OG_2Table[] PROGMEM = {OG_20, OG_21, OG_22, OG_23, OG_24, OG_25, OG_26, OG_27};

const char Estrich0[] PROGMEM = "Status\0";
const char Estrich1[] PROGMEM = "Plan\0";
const char Estrich2[] PROGMEM = "Estrich 2\0";
const char Estrich3[] PROGMEM = "Estrich 3\0";
const char Estrich4[] PROGMEM = "Estrich 4\0";
const char Estrich5[] PROGMEM = "Estrich 5\0";
const char Estrich6[] PROGMEM = "Estrich 6\0";
const char Estrich7[] PROGMEM = "Estrich 7\0";
//PGM_P  const EstrichTable[] PROGMEM = {Estrich0, Estrich1, Estrich2, Estrich3, Estrich4, Estrich5, Estrich6, Estrich7};


const char Raum0[] PROGMEM = "Heizung\0";
const char Raum1[] PROGMEM = "Werkstatt\0";
const char Raum2[] PROGMEM = "WoZi\0";
const char Raum3[] PROGMEM = "Buero\0";
const char Raum4[] PROGMEM = "Labor\0";
const char Raum5[] PROGMEM = "OG 1\0";
const char Raum6[] PROGMEM = "OG 2\0";
const char Raum7[] PROGMEM = "Estrich\0";
PGM_P const RaumTable[] PROGMEM = {Raum0, Raum1, Raum2, Raum3, Raum4, Raum5, Raum6, Raum7};

PGM_P const P_MenuTable[] PROGMEM = {Heizung0, Heizung1, Heizung2, Heizung3, Heizung4, Heizung5, Heizung6, Heizung7,
	Werkstatt0, Werkstatt1, Werkstatt2, Werkstatt3, Werkstatt4, Werkstatt5, Werkstatt6, Werkstatt7,
	WoZi0, WoZi1, WoZi2, WoZi3, WoZi4, WoZi5, WoZi6, WoZi7,
	Buero0, Buero1, Buero2, Buero3, Buero4, Buero5, Buero6, Buero7,
	Labor0, Labor1, Labor2, Labor3, Labor4, Labor5, Labor6, Labor7,
	OG_10, OG_11, OG_12, OG_13, OG_14, OG_15, OG_16, OG_17,
	OG_20, OG_21, OG_22, OG_23, OG_24, OG_25, OG_26, OG_27,
	Estrich0, Estrich1, Estrich2, Estrich3, Estrich4, Estrich5, Estrich6, Estrich7};


const char Titel[] PROGMEM = "HomeCentral\0";
const char Name[] PROGMEM = "Ruedi Heimlicher\0";
const char Adresse[] PROGMEM = "Falkenstrasse 20\0";
const char Ort[] PROGMEM = "8630 Rueti\0";
PGM_P const P_StartTable[] PROGMEM = {Titel, Name, Adresse, Ort};

uint8_t EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events
uint8_t EEMEM TWI_ErrCount;	// Akkumulierte TWI Restart Events



#define HEIZUNG_BRENNER 0xF1
#define HEIZUNG_BRENNER_EIN 0x01
#define HEIZUNG_BRENNER_AUS 0x00

//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void BlinkC(uint8_t  PIN, uint8_t anz);

extern unsigned char i2c_readAck(void);
extern unsigned char i2c_readNak(void);


uint8_t Hex2Int(char *s) 
{ 
	long res; 
	char *Chars = "0123456789ABCDEF", *p; 
	
	if (strlen(s) > 8) 
	/* Error ... */ ; 
	
	for (res = 0L; *s; s++) { 
		if ((p = strchr(Chars, toupper(*s))) == NULL) 
		/* Error ... */ ; 
		res = (res << 4) + (p-Chars); 
	} 
	
	return res; 
} 

/*
 ISR (TIMER0_COMPA_vect) 
 { 
 
 if ((SIGNAL_Count > SIGNAL_Status))//&& (Ein0==0)) //Tastendauer ist nicht ein
 { 
 
 delay_ms(20);
 
 }
 else 
 {
 SIGNAL_Count++;
 }
 
 }
 */

//ISR (TIMER0_OVF_vect) 
 



/*
 void timer0()
 {
 OSZIALO;
 delay_ms(5);
 
 
 //----------------------------------------------------
 // Set up timer 0 to generate interrupts @ 1000Hz
 //----------------------------------------------------
 TCCR0A = _BV(WGM01);
 TCCR0B = _BV(CS00) | _BV(CS02);
 OCR0A = 0xFF;
 TIMSK0 = _BV(OCIE0A);
 }
 */


void timer0 (void) 
{ 	
//	OSZIALO;
	//delay_ms(1);
	//OSZIAHI;
	//err_gotoxy(0,10);
	//err_puts("timer0\0");
	TCCR0A |=(1 << WGM01);
	TCCR0B |= (1<<CS00)|(1<<CS02);	//Takt /1024
	//TCCR0B |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	//	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64
	
   //OCR0A=0xFF;
	
	
	//TIFR0 |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	//TIMSK0 |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	//	TIMSK0 |= OCIE0A;					// Clear Timer on Compare Match, CTC
	TCNT0=0x00;						//Rücksetzen des Timers
	
}

ISR (TIM0_OVF_vect)
{
   testcounterL++;
   if (testcounterL ==0) // hochzaehlen
   {
      testcounterH++;
   }
   
}

#pragma mark TIMER2
void timer2 (uint8_t wert) 
{ 
	err_puts("timer2\0");
	//OSZIALO;
	TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22);	//Takt /1024	Intervall 32 us
	
	TCCR2A = 0;
	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
	
	//OC2 akt
	//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC
	
	TIFR2 |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren
	
	//TCCR2A = 0x00;					//Zaehler zuruecksetzen
	TCNT2=0;
	OCR2A = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 



void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag)
{
	eeprom_write_block((void*)ablauf, tag, 24);
}


void setTagplan(uint8_t *Daten)
{
	/*
	 erste halbe Stunde:		Cx	2 bits links
	 zweite halbe Stunde:	3x	2 bits rechts
	 ganze Stunde			Fx	alle 4 bits
	 Schalterstellung: Bit 3
	 Schalter ein			x8
	 Schalter aus			x0
	 */	
	//uint8_t tagblock[24];
	uint8_t i=0;
	for (i=0;i<buffer_size;i++)
	{
		if ((i<7)||(i>22))
		{
			Daten[i]= 0x00;
		}
		else
		{
			Daten[i]= 0xC8; // 200  ganze Stunden, 6 - 21 Uhr
		}
		
	}
	
	Daten[7]=0x48;		// 72
	Daten[22]=0x88;		//136
	
	Daten[9]=0x88;	//	Beginn Pause 9.30
	Daten[10]=0x08;
	Daten[11]=0x08;
	Daten[12]=0x48;	//	Ende Pause	12.30
	/*
	 for (i=0;i<buffer_size;i++)
	 {
	 lcd_gotoxy(0,1);
	 delay_ms(10);
	 lcd_putint(i);
	 lcd_gotoxy(5,1);
	 delay_ms(10);
	 lcd_putint(Daten[i]);
	 delay_ms(800);
	 }
	 */
}

void masterinit(void)
{

	DDRB &= ~(1<<5); // Pin 5 von Port B als Eingang fuer Servotest 0
	PORTB |= (1<<5); //HI

	DDRB &= ~(1<<6); // Pin 6 von Port B als Eingang fuer Servotest 1
	PORTB |= (1<<6); //HI

	DDRB &= ~(1<<7); // Pin 7 von Port B als Eingang fuer Servotest 2
	PORTB |= (1<<7); //HI
	
	
	
	
	
// 	DDRA &= ~(1<<DDA0);	//Pin 0 von PORT A als Eingang fuer Tastatur 	
//	PORTA |= (1<<DDA0); //Pull-up
	
	// TWI
	DDRC |= (1<<0);	//Pin 0 von PORT C als Ausgang (SCL)
	PORTC |= (1<<0);	//	ON
	DDRC |= (1<<1);	//Pin 1 von PORT C als Ausgang (SDA)
	PORTC |= (1<<1);	//	ON
	
	
	// erst in HS nach Ablauf Startddelay
	//	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer TWI SDA
	//	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer TWI SCL
	
	
	DDRB &= ~(1<<DDB0);	//Pin 0 von PORT B als Eingang fuer Test
	PORTB |= (1<<DDB0); //Pull-up
	
	DDRB &= ~(1<<DDB1);	//Pin 1 von PORT B als Eingang fuer Taster 1
	PORTB |= (1<<DDB1); //Pull-up
	
	DDRC |= (1<<2);	//Pin 2 von PORT C als Ausgang fuer TWI-Anzeige
	PORTC |= (1<<2); // ON
	DDRC |= (1<<TWI_CONTROLPIN);	//Pin 3 von PORT C als Ausgang fuer TWI-Anzeige
	PORTC |= (1<<TWI_CONTROLPIN); // ON
	
	//fuer Blinky
	
	DDRC |= (1<<LOOPLEDPIN);	//Pin 2 von PORT C als Ausgang fuer Loopcount
	
	
	
	//*********************************************************************
	//	Definitionen LCD
	//	Definitionen in lcd.h
	//*********************************************************************
	//	char* wochentag[] = {"MO","DI","MI","DO","FR","SA","SO"};
	//volatile int count=0;
	
	//volatile uint8_t Schaltposition=0;
	
	
	DDRC |= (1<<LCD_RSDS_PIN); //PIN 5 von PORT B als Ausgang fuer LCD_RSDS_PIN
	DDRC |= (1<<LCD_ENABLE_PIN); //PIN 6 von PORT B als Ausgang fuer LCD_ENABLE_PIN
	DDRC |= (1<<LCD_CLOCK_PIN); //PIN 7 von PORT B als Ausgang fuer LCD_CLOCK_PIN
	
	
	DDRA |= (1<<ERR_RSDS_PIN); //PIN 5 von PORT A als Ausgang fuer ERR_RSDS_PIN
	DDRA |= (1<<ERR_ENABLE_PIN); //PIN 6 von PORT A als Ausgang fuer ERR_ENABLE_PIN
	DDRA |= (1<<ERR_CLOCK_PIN); //PIN 7 von PORT A als Ausgang fuer ERR_CLOCK_PIN
	
	//DDRD= 0xFF;
	
	SRDDR |= (1<<SR_CLK_PIN);// PIN fuer Ausgang Clock fuer SR
	SRPORT |= (1<<SR_CLK_PIN);// HI
	
	SRDDR |= (1<<SR_LOAD_PIN);// PIN fuer Ausgang Enable fuer SR
	SRPORT |= (1<<SR_LOAD_PIN);// HI
	
	SRDDR &= ~(1<<SR_DATA_PIN);// PIN fuer Eingang Daten von SR
	SRPORT |= (1<<SR_DATA_PIN);// HI
	
	
}

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

void ServoTimerInit(void)
{
	/*
	 TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
	 //	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	 //	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64
	 TIFR0 |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	 TIMSK2|= (1<<TOIE0);			//Overflow Interrupt aktivieren
	 TCNT0=0x00;						//Rücksetzen des Timers
	 
	 TCCR2 |= (1<<CS00)|(1<<CS02);	//Takt /1024
	 //	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	 //	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64
	 TIFR |= (1<<TOV2); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	 TIMSK |= (1<<TOIE2);			//Overflow Interrupt aktivieren
	 TCNT2=0x00;						//Rücksetzen des Timers
	 
	 */	
}//ServoTimerInit









void setTWI_Status_LED(uint8_t status)
{
	if (status)
	{
		//lcd_puts("ON");
		// TWI-PIN  einschalten
		PORTC |= (1<<TWI_CONTROLPIN);
	}
	else
	{
		//lcd_puts("OFF");
		
		PORTC &= ~(1<<TWI_CONTROLPIN); // TWI-Bit  ausschalten
	}
	
}

void initOSZI(void)
{
	OSZIPORTDDR |= (1<<PULSA);
	OSZIPORT |= (1<<PULSA);	
	OSZIAPORTDDR |= (1<<PULSB);
	OSZIPORT |= (1<<PULSB);

}

void readSR (void)
{
	// Schalterstellung in SR laden
	SRPORT &= ~(1<<SR_LOAD_PIN); // PL> LO
	_delay_us(10);
	SRPORT |= (1<<SR_LOAD_PIN); // PL> HI
	_delay_us(10);
	uint8_t i=0;
	Write_Device=0;
	Read_Device=0x00;
	//Daten aus SR schieben
	for (i=0;i<16;i++)
	{
		uint8_t pos=15-i;
		// Bit lesen
		if (SRPIN & (1<<SR_DATA_PIN)) // PIN ist Hi, OFF
		{
			//Bit is HI
			if (i<8) // Byte 1
			{
				Write_Device |= (0<<(i)); // auf Device i soll geschrieben werden
			}
			else 
			{
				Read_Device |= (0<<(i-8));	// von Device i soll gelesen werden
			}
			
			
		}
		else 
		{
			//Bit is LO
			if (i<8) // Byte 1
			{
				Write_Device |= (1<<i); // auf Device i soll geschrieben werden
			}
			else 
			{
				Read_Device |= (1<<(i-8));	// von Device i soll gelesen werden
			}
			
		}
		
		// SR weiterschieben
		
		SRPORT &= ~(1<<SR_CLK_PIN); // CLK LO
		_delay_us(10);
		SRPORT |= (1<<SR_CLK_PIN); // CLK HI, shift
		_delay_us(10);
		
	} // for i
	
}

uint8_t UhrAbrufen (void)
{
	/*
	Knackpunkte:
	min, std, tag
	neueZeit
	*/
	//uint8_t i=0;
	//lcd_clr_line(1);
	/*
	lcd_gotoxy(0,1);
	lcd_puts("U\0");
	lcd_putc(' ');
	lcd_gotoxy(1,1);
	*/
		
	//	DCF-Uhr lesen
	uint8_t DCF77erfolg=0;
	DCF77erfolg=SlavedatenLesen(DCF77_ADRESSE,(void*)DCF77daten);
	
	return DCF77erfolg;
	// end Uhr lesen
	
}


uint8_t PWMDatenAbrufen (void)
{
	/*
    Daten fuer PWM aus EEPROM abrufen und
    */
	//uint8_t i=0;
	//lcd_clr_line(1);
	/*
    lcd_gotoxy(0,1);
    lcd_puts("PWM\0");
    lcd_putc(' ');
    lcd_gotoxy(1,1);
    */
   
	//	DCF-Uhr lesen
	uint8_t PWMerfolg=0;
	PWMerfolg=SlavedatenLesen(EEPROM_WOCHENPLAN_ADRESSE,(void*)DCF77daten);
	
	return PWMerfolg;
	// end Uhr lesen
	
}


uint8_t RTC_Abrufen (void)
{
	/*
    Zeit.minute = DCF77daten[0];
    Zeit.stunde = DCF77daten[1];
    Zeit.kalendertag = DCF77daten[2];
    Zeit.kalendermonat = DCF77daten[3];
    Zeit.kalenderjahr = DCF77daten[4];
    Zeit.wochentag = DCF77daten[5]-1;// DCF77 ist 1-basiert
    
    
    */
	/*
    struct time
    {
    uint8_t sekunde;
    uint8_t minute;
    uint8_t stunde;
    uint8_t kalendertag;
    uint8_t wochentag;
    uint8_t kalendermonat;
    uint8_t kalenderjahr;
    };
    */
   
	//	RTC-Uhr lesen
	//uint8_t data;
	uint8_t RTCerfolg=0;
	
	/*
    // Sekunden lesen: Register 0
    RTCerfolg= DS1307Read(0x00,&data);
    if (RTCerfolg)
    {
    return RTCerfolg;
    }
    
    uint8_t sekunde = ((data & 0x70)>>4)*10 + (data & 0x0F);
    
    // Minuten lesen: Register 1
    RTCerfolg= DS1307Read(0x01,&data);
    if (RTCerfolg)
    {
    return RTCerfolg;
    }
    uint8_t minute=((data & 0xF0)>>4)*10 + (data & 0x0F);
    
    
    
    // Stunde lesen: Register 2
    RTCerfolg= DS1307Read(0x02,&data);
    if (RTCerfolg)
    {
    return RTCerfolg;
    }
    uint8_t stunde=((data & 0xF0)>>4)*10 + (data & 0x0F);;
    */
	uint8_t sekunde=0;
	uint8_t minute=0;
	uint8_t stunde=0;
	
	
	
	RTCerfolg=read_Zeit(&sekunde, &minute, &stunde);
	
	if (test)
	{
		err_gotoxy(0,0);
		err_putint2(stunde);
		err_putc(':');
		err_putint2(minute);
      err_gotoxy(0,4);
      err_putc(' ');
      err_gotoxy(0,4);
      err_putc('>');
      err_puthex(RTCerfolg);
	}
   err_gotoxy(0,0);
   err_puthex(RTCerfolg);
   
	
	RTCdaten[0]=minute;
	RTCdaten[1]=stunde;
	
   
	uint8_t wochentag;
	uint8_t tagdesmonats;
	uint8_t monat;
	uint8_t jahr;
	
	RTCerfolg=Read_Datum(&wochentag,&tagdesmonats,&monat,&jahr);
	
	
	
   /*
    err_gotoxy(0,1);
    //err_puthex(RTCerfolg);
    //err_gotoxy(3,1);
    err_putint2(tagdesmonats);
    err_putc(':');
    err_putint2(monat);
    err_putc(':');
    err_putint2(jahr);
    err_putc(':');
    err_putint2(wochentag);
    */
   
   if (test)
   {
      /*
      err_gotoxy(6, 1);
      switch (wochentag)
      {
         case 1:
            err_puts("MO\0");
            break;
         case 2:
            err_puts("DI\0");
            break;
         case 3:
            err_puts("MI\0");
            break;
         case 4:
            err_puts("DO\0");
            break;
         case 5:
            err_puts("FR\0");
            break;
         case 6:
            err_puts("SA\0");
            break;
         case 7:
            err_puts("SO\0");
            break;
            
      }//switch wochentag
       */
   }

   err_gotoxy(2,0);
   err_puthex(RTCerfolg);

	RTCdaten[2]=tagdesmonats;
	RTCdaten[3]=monat;
	RTCdaten[4]=jahr;
	RTCdaten[5]=wochentag;
	
	return RTCerfolg;
	// end Uhr lesen
	
}



int main (void)
{
	//JTAG deaktivieren (datasheet 231)
	MCUCR |=(1<<7);
	MCUCR |=(1<<7);
	cli();
	uint8_t ch = MCUSR;
	MCUSR &= ~(1<<WDRF);
	MCUSR = 0;
	
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0x00;
	
	// Check if the WDT was used to reset
	//if (! (ch &  _BV(EXTRF))) // if its a not an external reset...
	{
		//
	}
	
	//uint16_t plen;
	int8_t cmd;
	int8_t send_cmd=0;
	
	
	//*********************************************************************
    
   
	//*********************************************************************
	
	masterinit();
	
	uint8_t TastaturCount=0;
	Brennerzeit=0;
	uint8_t i=0;
	for (i=0;i<8;i++)
	{
		//LaborTXdaten[i]=0;
		//LaborRXdaten[i]=0;
		DCF77daten[i]=0x00;
		txbuffer[i] = 0;
	}
	//WebTxStartDaten = 0x00;
	char startbuffer[16];
	
	// i2c_init(); //in Startzaehlschleife
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	delay_ms(50);
	//	lcd_CGRAMInit_A();
	
	// Zeichensatz laden
	lcd_CGRAMInit_Titel();
	
	delay_ms(50);
	lcd_puts("Guten Tag\0");
	delay_ms(800);
	lcd_cls();
	lcd_puts("Master ready\0");
	
	err_initialize(ERR_FUNCTION_8x2, ERR_CMD_ENTRY_INC, ERR_CMD_ON);
	err_puts("Err ready\0");
	delay_ms(800);
	
	
	//	initADC(0);
	uint8_t Tastenwert=0;
	//	uint8_t Servowert=0;
	
	lcd_cls();
	err_cls();
	/*
	lcd_gotoxy(14,0);
	lcd_puts("V:\0");
	lcd_puts(VERSION);
	delay_ms(800);
	*/
//	lcd_gotoxy(19,1); 
//	lcd_putc(165);	//	Punkt an letzter Stelle
	
	Raum_Thema=0x00;
	
	
	Zeit.minute=0;
	Zeit.stunde=0;
	Zeit.wochentag=0;
	uint8_t AnzeigeWochentag=0;
	
	volatile uint8_t neueZeit=0;
	
	
	uint16_t startdelay=STARTDELAY;
	
	//startdelay=0;
	
	uint8_t UhrLesen=0;
	uint8_t Taste=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x1F;
	
	//uint8_t web_request=0;
	
	
	
	volatile uint16_t loopcount0=0;
	volatile uint16_t loopcount1=0;
	
	uint8_t twi_LO_count0=0;
	uint8_t twi_LO_count1=0;
	
	uint8_t twi_HI_count0=0;
	//uint16_t twi_HO_count1=0;
	
	uint8_t SPI_Call_count0=0;
	
	uint8_t twi_Call_count0=0;	//	Anzahl TWI-Calls in einer Schleife
	uint8_t twi_Reply_count0=0;	//	Anzahl TWI-Replies in einer Schleife
	uint8_t twi_Stat_count=0;	//	Anzahl Resets nach erfolglosen TWI-Aufrufen
	uint8_t twierrcount=0;		//	Anzahl TWI-Resets
	
	
	uint8_t tempWDT_Count=0;
	/*	
	 // Bisherige Watchdog-Resets lesen
	 tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);;	// Bisherige Watchdog-Resets
	 
	 if (tempWDT_Count==0xFF) // Erster Start nach neuer programmierung, zuruecksetzen
	 {
	 tempWDT_Count=0;
	 eeprom_write_byte(&WDT_ErrCount,tempWDT_Count);		// Anzahl reset
	 delay_ms(5);
	 }
	 */	
	
	//delay_ms(300);
	
	
	//Websr
	// ******************
	ByteCounter=0xFF;
	//WebTxStartDaten = 0x00;
	//SolarTxStartDaten = 0x00;
	/*
	 Bus_Status:
	 Bit 0:
	 Bit 1:	TWI_CONTROLBIT
	 Bit 2:
	 
	 
	 */
	/******************************************************************/	
	// set the clock speed to "no pre-scaler" (8MHz with internal osc or 
	// full external speed)
	// set the clock prescaler. First write CLKPCE to enable setting of clock the
	// next four instructions.
	
	// Software-Umstellung der CPU-Frequenz auf 8 MHz
	
	//	CLKPR=(1<<CLKPCE); // change enable
	//	CLKPR=0; // "no pre-scaler"
	_delay_loop_1(0); // 60us
	
	
	
	
	
	uint8_t j;
	for (j=0;j<SPI_BUFSIZE;j++)
	{
		outbuffer[j]=0;
		inbuffer[j]=0;
	}
   /*
	outbuffer[0]=' ';
	outbuffer[1]='H';
	outbuffer[2]='o';
	outbuffer[3]='m';
	outbuffer[4]='e';
	outbuffer[5]=' ';
	*/
	out_startdaten='+';
	BUS_Status |=  (1<<TWI_CONTROLBIT);			// TWI am Anfang einschalten

	lcd_clr_line(0);
	lcd_puts("InitSPI_Slave\0");
	delay_ms(10);

	InitSPI_Slave();
	lcd_puts("OK\0");
	delay_ms(10);
	/******************************************************************/
	
	//lcd_gotoxy(18,1); 
	//lcd_putc('-');	//	Erste Runde, Strich an letzter Stelle
	//timer0();
	/******************************************************************/
	uhrstatus=0;
	uhrstatus |= (1<<SYNC_NULL); // Uhr undef, warten auf DCF77
	
	initOSZI();
   
   if (PINB & (1<<0))
   {
      
      test=0;
      
   }
   else
   {
      test=1;

   }
   
   /******************************************************************/

	/*** Hauptschleife															***/
	
	/******************************************************************/
#pragma mark while
	lcd_clr_line(0);
	//lcd_puts("Los!\0");
	delay_ms(10);
	while (1)
	{
		
		// Startfunktion: SCL und SDA pruefen
		if (startdelay==STARTDELAY)
		{
			//err_gotoxy(19,1); 
			//err_putc('+');	//	Erste Runde, + an letzter Stelle
			//lcd_puthex(startdelay);
			delay_ms(2);
			err_gotoxy(19,1); 
			err_putc(' ');	//	Erste Runde, Strich an letzter Stelle weg
			
		}
		
		wdt_reset();
		
		//	Startroutine noch im Gang: TWI beide Pins HI also Rückwaertszaehlen
		if( startdelay && ((PINC & (1<<SCLPIN) && (PINC & (1<<SDAPIN)))))	// SCL UND SDA ist HI
		{
			if (startdelay==1) // Letzter Durchlauf vor einschalten
			{
				DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer TWI SDA
				PORTC |= (1<<DDC1); //Pull-up
				DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer TWI SCL
				PORTC |= (1<<DDC0); //Pull-up
				BUS_Status |=(1<<TWI_CONTROLBIT);		// TWI ON
				BUS_Status &=~(1<<WEB_CONTROLBIT);		// WEB OFF
				lcd_clr_line(0);
				err_gotoxy(0,1); 
				err_puts("Start\0");	//	Erste Runde, Strich an letzter Stelle weg
#pragma mark RTC init				
				
            i2c_init();
				
				delay_ms(50);
				rtc_init();
				delay_ms(10);
				uint8_t res=0;
				
				res=rtc_write_Control(1);
				
				// stunde, minute, sekunde
				res=rtc_write_Zeit(14,38,0);// uint8_t stunde, uint8_t minute, uint8_t sekunde

            delay_ms(10);
				/*
				if (res)
				{
					err_gotoxy(0,1);
					err_puts("Z-\0");
				}
				else {
					err_gotoxy(0,1); 
					err_puts("Z+\0");
				}
				*/
				// Datum: 1 = Montag
				res=rtc_write_Datum(6,23,4,15);// uint8_t wochentag, uint8_t tagdesmonats, uint8_t monat, uint8_t jahr
				delay_ms(10);
				
				if (res)
				{
               err_gotoxy(0,1);
					err_puts("     \0");

					err_gotoxy(0,1);
					err_puts("D-\0");
				}
				else
            {
               err_gotoxy(0,1);
					err_puts("     \0");

					err_gotoxy(0,1);
					err_puts("D+\0");
				}
				
				wdt_reset();
				sei();
				
				
			}
			
			startdelay--;
			
		}
		else
		{
			//err_gotoxy(10,1); 
			//err_puthex(startdelay);
		}
      
      #pragma mark Standardloop start
		
		loopcount0++;
		if (loopcount0 >= 0x00FF)
		{
			LOOPLEDPINPORT ^=(1<<LOOPLEDPIN); // Blink-LED
			
			loopcount0=0;
			loopcount1++;
			//lcd_gotoxy(0,0);
			//lcd_puts("lpcnt1 \0");
			//lcd_puthex(loopcount1);
			if (startdelay==0)
			{
				//				timer2(0xAF);
				//				sei();
				//SPI_shift();
				
			}
			
			if (loopcount1>=0x2F)
			{
            
				//lcd_gotoxy(2,1);
				//lcd_puts("Wechsel \0");
				//lcd_puthex(loopcount1);
				loopcount1=0;
			}
			
		}
      
      if (test) // Start der TWI-Routinen ohne Webserver
      {
         if ((testcounterH) >>= 4)
         {
            err_gotoxy(16,0);
            err_puthex((testcounterH) >>= 4);
         }
         if (testcounterH > 0xFFF)
         {
            spistatus |= (1<<SPI_SHIFT_IN_OK_BIT);
            err_gotoxy(18,0);
            err_puts("T\0");
            testcounterH=0;
         }
         
      }

		
		
		//	Checken, ob SCL oder SDA längere Zeit low sind. 
		//	In diesem Fall zuerst TWI reseten, bei längerem Fehler reset des Prozessors
		
		
		// Startroutinen sind abgelaufen oder ganz am Anfang  
		// SDA und SCL sind laengere Zeit nicht gleichzeitig HI: Fehlersituation
		
		if (((startdelay==0)||(startdelay==STARTDELAY))&& (((!(PINC & (1<<SDAPIN))) && PINC & (1<<SCLPIN)) ) )// SDA ist LO und SCL ist HI (warten auf Ack)
      {
         err_gotoxy(15,1);
         err_puts("ERR\0");
         
         /*
          
          1. TWI Modul am Master abschalten.
          2. Am Master SDA als Input und SCL als Output konfigurieren.
          3. SCL im TWI Takt so lange toggeln bis SDA wieder high ist
          4. TWI wieder initialisieren und weiter machen ...
          
          Du mußt also das HW-TWI abschalten und per SW-I2C einen SCL-Puls
          generieren und dann versuchen, Stop zu senden. Erst dann ist der Slave
          wieder adressierbar.
          */
         
         // Zaehlen, wieviele Runden der Fehler dauert
         twi_LO_count0++;
         
         // nach einer Anzahl Runden  die zweite Anzahl twi_LO_count1 inkrementieren
         if (twi_LO_count0 >=0xAFF)
         {
            twi_LO_count0=0;
            twi_LO_count1++;
            err_gotoxy(0,1);
            err_puts("lc1 \0");
            err_puthex(twi_LO_count1);
            
            // Erste Grenze von twi_LO_count1 erreicht, also erneut stop senden
            if (twi_LO_count1>=0x0F) //
            {
               //					PORTC |= (1<<TWICOUNTPIN); //TWI-LED ON
               
               // TWI-Fehler inkrementieren
               twierrcount++;
               outbuffer[30]=twierrcount;
               // Wenn Fehler andauert, TWI neu starten
               if (twi_LO_count1 >=0xAF) //
               {
                  err_gotoxy(12,0);
                  err_puts("deb\0");
                  TWBR =0;
                  TWCR =0;
                  
                  uint8_t deb=i2c_debloc();
                  err_puthex(deb);
                  delay_ms(10);
                  TWI_DDR &= ~(1<<SCL_PIN);	// SCL-Pin wieder als EINgang
                  TWI_PORT |= (1<<SCL_PIN);  // HI
                  
                  delay_ms(10);
                  i2c_init();
                  rtc_init();
                  i2c_stop();
                  wdt_reset();
               }
               //err_gotoxy(10,1);
               //err_puts("st\0");
               //err_puthex(twi_LO_count1);
               //delay_ms(10);
               
            }
         }
         
      }
      
		else // alles in Ordnung, Fehler zuruecksetzen
		{
			//		PORTC &= ~(1<<TWICOUNTPIN); //TWI-LED OFF
			twi_LO_count0=0;
			twi_LO_count1=0;
			wdt_reset();
			//err_gotoxy(15,1);
			//err_puts("OK \0");
						//err_puthex(tempWDT_Count);
						//delay_ms(1000);

		}
		
#pragma mark SPI		
		
		/* *** SPI begin **************************************************************/
		
		//err_gotoxy(19,0);
		//err_putc('-');
		
		// ***********************
		if (SPI_CONTROL_PORTPIN & (1<< SPI_CONTROL_CS_HC)) // SPI ist Passiv
		{
			// ***********************
			/*
			 Eine Uebertragung hat stattgefunden. 
			 Die out-Daten sind auf dem Webserver.			 
			 Die in-Daten vom Webserver sind geladen.
			 Sie muessen noch je nach in_startdaten ausgewertet werden.
			 */
			 			
			// ***********************
			SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO); // MISO ist HI in Pausen
			
			#pragma mark PASSIVE

			if (spistatus &(1<<ACTIVE_BIT)) // Slave ist neu passiv geworden. Aufraeumen, Daten uebernehmen
			{
				
				wdt_reset();
				SPI_Call_count0++;
				// Eingang von Interrupt-Routine, Daten von Webserver
				err_gotoxy(19,0);
				err_putc(' ');
				
				// in lcd verschoben
				lcd_clr_line(2);
				lcd_gotoxy(0,2);
				
				// Eingang anzeigen
				lcd_puts("iW \0");
				lcd_puthex(in_startdaten);
				lcd_putc(' ');
				lcd_puthex(in_hbdaten);
				lcd_puthex(in_lbdaten);
				lcd_putc(' ');
				uint8_t j=0;
				for (j=0;j<4;j++)
				{
					//lcd_putc(' ');
					lcd_puthex(inbuffer[j]);
					//err_putc(inbuffer[j]);
				}
				OutCounter++;
				
				// Uebertragung pruefen
				
				//lcd_gotoxy(6,0);
				//lcd_puts("bc:\0");
				//lcd_puthex(ByteCounter);
            
            err_gotoxy(0,0);
				err_puts("      \0");

            
				err_gotoxy(19,0);
				err_putc(' ');
				err_gotoxy(19,0);
				if (ByteCounter == SPI_BUFSIZE-1) // Uebertragung war vollstaendig
				{
					if (out_startdaten + in_enddaten==0xFF)
					{
						err_putc('+');
						spistatus |= (1<<SUCCESS_BIT); // Bit fuer vollstaendige und korrekte  Uebertragung setzen
						lcd_gotoxy(19,0);
						lcd_putc(' ');
						//lcd_clr_line(3);
						//err_gotoxy(0,1);
						//err_puthex(loopCounterSPI++);
						//err_puts("OK \0");
                  
						//err_puthex(out_startdaten + in_enddaten);
						//					if (out_startdaten==0xB1)
						{
							SendOKCounter++;
						}
						spistatus |= (1<<SPI_SHIFT_IN_OK_BIT);
					}
					else 
					{
						spistatus &= ~(1<<SUCCESS_BIT); // Uebertragung fehlerhaft, Bit loeschen
						err_putc('-');
						err_clr_line(1);
						err_gotoxy(0,1);
						err_puts("ER1\0");
                  err_putc(' ');
						err_puthex(out_startdaten);
						err_puthex(in_enddaten);
						err_putc(' ');
						err_puthex(out_startdaten + in_enddaten);
                  
                  spistatus &= ~(1<<SPI_SHIFT_IN_OK_BIT);
 						{
							SendErrCounter++;
						}
						//errCounter++;
					}
					
				}
				else 
				{
					spistatus &= ~(1<<SUCCESS_BIT); //  Uebertragung unvollstaendig, Bit loeschen
					err_clr_line(0);
					err_gotoxy(0,0);
					err_puts("ER2\0");
					err_putc(' ');
               err_puthex(out_startdaten);
               err_puthex(in_enddaten);
               err_putc(' ');
               err_puthex(out_startdaten + in_enddaten);

					//delay_ms(100);
					//errCounter++;
					IncompleteCounter++;
                spistatus &= ~(1<<SPI_SHIFT_IN_OK_BIT);
				}
				
				//lcd_gotoxy(11, 1);							// Events zahelen
				//lcd_puthex(OutCounter);
				/*						
				 lcd_puthex(SendOKCounter);
				 lcd_puthex(SendErrCounter);
				 lcd_puthex(IncompleteCounter);
				 */				
				/*
				 lcd_gotoxy(0,0);
				 lcd_putc('i');
				 lcd_puthex(in_startdaten);
				 lcd_puthex(complement);
				 lcd_putc(' ');
				 lcd_putc('a');
				 lcd_puthex(out_startdaten);
				 lcd_puthex(in_enddaten);
				 lcd_putc(' ');
				 lcd_putc('l');
				 lcd_puthex(in_lbdaten);
				 lcd_putc(' ');
				 lcd_putc('h');
				 lcd_puthex(in_hbdaten);
				 out_hbdaten++;
				 out_lbdaten--;
				 
				 lcd_putc(out_startdaten);
				 */
				/*
				 lcd_gotoxy(0,0);
				 lcd_puthex(inbuffer[9]);
				 lcd_puthex(inbuffer[10]);
				 lcd_puthex(inbuffer[11]);
				 lcd_puthex(inbuffer[12]);
				 lcd_puthex(inbuffer[13]);
				 */
				//lcd_gotoxy(13,0);								// SPI - Fehler zaehlen
				//lcd_puts("ERR    \0");
				//lcd_gotoxy(17,0);
				//lcd_puthex(errCounter);
				
				// Bits im Zusammenhang mit der Uebertragung zuruecksetzen. Wurden in ISR gesetzt
				spistatus &= ~(1<<ACTIVE_BIT);		// Bit 0 loeschen
				spistatus &= ~(1<<STARTDATEN_BIT);	// Bit 1 loeschen
				spistatus &= ~(1<<ENDDATEN_BIT);		// Bit 2 loeschen
				spistatus &= ~(1<<SUCCESS_BIT);		// Bit 3 loeschen
				spistatus &= ~(1<<LB_BIT);				// Bit 4 loeschen
				spistatus &= ~(1<<HB_BIT);				// Bit 5 loeschen
				
				// aufraeumen
				out_startdaten=0x00;
				out_hbdaten=0;
				out_lbdaten=0;
				for (i=0;i<SPI_BUFSIZE;i++)
				{
					outbuffer[i]=0;
				}
				
				/*
				lcd_gotoxy(0,0);				// Fehler zaehlen
				lcd_puts("IC   \0");
				lcd_gotoxy(2,0);
				lcd_puthex(IncompleteCounter);
				lcd_gotoxy(5,0);
				lcd_puts("TW   \0");
				lcd_gotoxy(7,0);
				lcd_puthex(TWI_errCounter);
				
				lcd_gotoxy(5,1);
				lcd_puts("SE   \0");
				lcd_gotoxy(7,1);
				lcd_puthex(SendErrCounter);
				*/
			} // if Active-Bit
				
#pragma mark HomeCentral-Tasks 
		
		} // neu Passiv
		
		
		// letzte Daten vom Webserver sind in inbuffer und in in_startdaten, in_lbdaten, in_hbdaten
		
				
		else						// (IS_CS_HC_ACTIVE) 
		{
			if (!(spistatus & (1<<ACTIVE_BIT))) // CS ist neu aktiv geworden, Active-Bit 0 ist noch nicht gesetzt
			{
				// Aufnahme der Daten vom Webserver vorbereiten
				uint8_t j=0;
				in_startdaten=0;
				in_enddaten=0;
				in_lbdaten=0;
				in_hbdaten=0;
				for (j=0;j<SPI_BUFSIZE;j++)
				{
					inbuffer[j]=0;
				}
				
				spistatus |=(1<<ACTIVE_BIT); // Bit 0 setzen: neue Datenserie
				spistatus |=(1<<STARTDATEN_BIT); // Bit 1 setzen: erster Wert ergibt StartDaten
				
				bitpos=0;
				ByteCounter=0;
				//timer0(); // Ueberwachung der Zeit zwischen zwei Bytes. ISR setzt bitpos und ByteCounter zurueck, loescht Bit 0 in spistatus
				
				// Anzeige, das  rxdata vorhanden ist
				lcd_gotoxy(19,0);
				lcd_putc('$');

				
				
				
				// SPI-Buffer vorwaertsschalten
				/*
				uint8_t wert0=spibuffer[15];
				for(i=15;i>0;i--)
				{
					spibuffer[i]=spibuffer[i-1];
				}
				spibuffer[0]=wert0;
				*/
				
				/*
				// SPI-Buffer rueckwaertsschalten
				
				uint8_t wert0=spibuffer[0];
				for(i=0;i<15;i++)
				{
					spibuffer[i]=spibuffer[i+1];
				}
				spibuffer[15]=wert0;
				*/
				
			}//		if (!(spistatus & (1<<ACTIVE_BIT)))
		}//											(IS_CS_HC_ACTIVE) 
		
		/* *** SPI end **************************************************************/
	
		#pragma mark Webserver-Tasks
		
      /* *****************************************************************/
      
      /* *** SPI Daten auswerten *****************************************/
      
      /* *****************************************************************/
      
		if (spistatus & (1<<SPI_SHIFT_IN_OK_BIT))	// Shift-Bit ist nach 'neu PASSIVE' gesetzt, Datentausch ist erfolgt und OK
		{
			spistatus &= ~(1<<TWI_ERR_BIT);	// Bit fuer Fehler zuruecksetzen
			BUS_Status &= ~(1<<SPI_SENDBIT);	// sendbit in BUS_status zuruecksetzen, wird gesetzt, 
			{
				if (test)
				{
					//err_gotoxy(12,0);
					//err_puts("Los\0");
					in_startdaten=DATATASK;
				}
				//	******************************
				// Tasks mit Webserver behandeln
				//	******************************
				
				spistatus &= ~(1<<TWI_ERR_BIT);
				//lcd_clr_line(1);
				
				// in-startdaten anzeigen
				/*
				lcd_gotoxy(6,0);
				lcd_puts("      \0");
				lcd_gotoxy(6,0);
				lcd_puts("Task \0");
				lcd_puthex(SendOKCounter);
				*/
				
				lcd_gotoxy(0,0);
				lcd_puts("      \0");
				lcd_gotoxy(0,0);
				lcd_puts("in\0");
				lcd_putc(' ');
				//lcd_puthex(spistatus);
				//lcd_puthex(BUS_Status);
				lcd_puthex(in_startdaten);
				//delay_ms(100);
            outbuffer[42] = in_startdaten;
            
				switch (in_startdaten) // Daten vom Webserver, liegen am Anfang der Schleife bereit 
				{
					case NULLTASK: // B0
					{
						
						
					}break;
						
					case STATUSTASK:	// B1: in_hbdaten enthaelt Status
					{
						//Statustask abfragen, bei Status 0 TWI deaktivieren: Keine Stoerungen duch SPI-Aufrufe des Timers
						
						lcd_clr_line(1);
						lcd_gotoxy(0,1);
						lcd_puts("iS \0");
						
						lcd_puthex(in_startdaten);
						lcd_putc(' ');
						
						//Empfangene Angaben vom Status
						lcd_puthex(in_hbdaten);
						lcd_puthex(in_lbdaten);
						lcd_puts("         \0");
                  
                  
						if (in_hbdaten == 0x01)// TWI solll wieder eingeschaltet werden
						{
							BUS_Status |=  (1<<TWI_CONTROLBIT);		// TWI ON
							setTWI_Status_LED(1);
							out_hbdaten = 1;
							out_lbdaten = 1;
							outbuffer[1]=1; // Status an Webserver bestaetigen
							
							testCounterON++;
							//BUS_Status &= ~(1<<WEB_CONTROLBIT);		// WEB OFF
						
						}
						else if (in_hbdaten == 0x00)
						{
							BUS_Status &= ~(1<<TWI_CONTROLBIT);		// TWI OFF
						
							// Bestaetigung fuer Webserver
							out_startdaten= STATUSCONFIRMTASK; // B2  // an Webserver melden, dass TWI OFF ist
							
							setTWI_Status_LED(0);
							out_hbdaten = 0;
							out_lbdaten = 0;

							outbuffer[0]=0;	 // Status an Webserver bestaetigen
							
							testCounterOFF++;
							//BUS_Status |=  (1<<WEB_CONTROLBIT);		// WEB  ON
						}
						
						err_gotoxy(0,0);
						err_puts("oS \0");
						
						err_puthex(out_startdaten);
						err_putc(' ');
						
						//Angaben vom Status an Webserver
						err_puthex(out_hbdaten);
						err_puthex(out_lbdaten);
						
						if (test)
						{
							err_putc(' ');
							err_puthex(testCounterON);
							err_puthex(testCounterOFF);
						}
						// SPI senden veranlassen
						BUS_Status |= (1<<SPI_SENDBIT);
						in_startdaten=0;
						in_hbdaten=0;
						in_lbdaten=0;
					}break;
						
						
					case EEPROMREADTASK: // B8
					{
						// Auftrag vom Webserver, die Daten im EEPROM an hb, lb zu lesen 
						// und  im spi_buffer an den Webserver zurueckzuschicken
												
						// Experiment: Controlbit loeschen, um Lesen von SR zu verhindern. Wird mit B1 wieder gesetzt
                  
                  BUS_Status &= ~(1<<TWI_CONTROLBIT);		// TWI OFF
						
                  
                  
                  lbyte=in_lbdaten;
						hbyte=in_hbdaten;
						
						uint8_t readerfolg =0;
						
						// EEPROMTXdaten: Array mit den Daten des EEPROMS, wird zum Webserver geschickt
						// hbyte, lbyte: Adresse im EEPROM, geschickt vom Webserver
						readerfolg = EEPROMTagLesen(0xA0, (void*)EEPROMTXdaten, hbyte, lbyte);
						if (readerfolg==0)
						{
							
							//TCNT0 =0x00;
							//SIGNAL_Count=0;
							
							// verschoben in lcd
							lcd_gotoxy(0,3);
							lcd_puts("rE \0");
							
							//Empfangene Angaben vom EEPRPOM
							lcd_puthex(in_startdaten);
							lcd_putc(' ');
							
							lcd_puthex(in_hbdaten);
							lcd_puthex(in_lbdaten);
							lcd_putc(' ');
							lcd_puthex(EEPROMTXdaten[0]);
							lcd_puthex(EEPROMTXdaten[1]);	
							lcd_puthex(EEPROMTXdaten[2]);
							lcd_puthex(EEPROMTXdaten[3]);
							
							lcd_gotoxy(19,3);
							lcd_putc('>');
							//err_puts("E+\0");
							
							// Gelesene Daten zum Schicken an den Webserver vorbereiten
							
							out_startdaten=EEPROMREPORTTASK; // B4
							out_lbdaten=lbyte;
							out_hbdaten=hbyte;
							
							//err_gotoxy(6,1);
							
							// EEPROM-Daten in outbuffer schreiben
							for(i=0;i<8;i++)
							{
								outbuffer[i]=EEPROMTXdaten[i];
							}

                     for(i=16;i<8;i++)
							{
								outbuffer[i]=EEPROMTXdaten[i];
							}

							//delay_ms(1000);
							aktuelleDatenbreite=eeprom_buffer_size;
							
							// SPI senden veranlassen
							BUS_Status |= (1<<SPI_SENDBIT);
							
							// erledigt
							//							EEPROMRXStartdaten=0;
							lbyte=0;
							hbyte=0;
							
							
						}
						else
						{
							// SPI senden verhindern
							spistatus |= (1<<TWI_ERR_BIT);
							
							err_gotoxy(19,0);
							err_putc('-');
							//err_puts("E-\0");
							//err_puthex(readerfolg);
							
                     for(i=0;i<8;i++)
							{
								outbuffer[i]=13;
							}

						}
						
						//	BUS_Status |=(1<<TWI_CONTROLBIT);		// TWI wieder ON				
						//EEPROMRXStartdaten=0;
						in_startdaten=0;
						in_hbdaten=0;
						in_lbdaten=0;
						
						
					}break;
						
					case EEPROMWRITETASK:
					{
						/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */
						// Daten ins EEPROM schreiben
						/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */
						
						// Auftrag vom Webserver, die Daten im in_buffer an Adresse hb, lb ins EEPROM zu schreiben
						
						// Experiment: Controlbit loeschen, um Lesen von SR zu verhindern. Wird mit B1 wieder gesetzt
                  
                  BUS_Status &= ~(1<<TWI_CONTROLBIT);		// TWI OFF
                  
                  
                  EEPROMTXStartdaten=EEPROMWRITETASK; // B7
						lbyte=in_lbdaten;
						hbyte=in_hbdaten;
						
                  // Kontrollausgabe
                  outbuffer[33] = 0x3A;
                  outbuffer[34] = lbyte;
                  outbuffer[35] = hbyte;
                  
						uint8_t i=0;
						for(i=0;i<8;i++)
						{
							EEPROMTXdaten[i]=inbuffer[i];
                     outbuffer[36+i] = inbuffer[i];
                     
							//			err_gotoxy(3,1);
							// err_puthex(EEPROMTXdaten[i]);
							//			err_putc(' ');
							//delay_ms(2);
						}
						
						lcd_gotoxy(0,1);
						lcd_puts("wE \0");
						
						lcd_puthex(in_startdaten);
						lcd_putc(' ');
						
						//Empfangene Angaben vom EEPRPOM
						lcd_puthex(in_hbdaten);
						lcd_puthex(in_lbdaten);
						lcd_putc(' ');
						lcd_puthex(EEPROMTXdaten[0]);
						lcd_puthex(EEPROMTXdaten[1]);
						lcd_puthex(EEPROMTXdaten[2]);
						lcd_puthex(EEPROMTXdaten[3]);
						
						
						uint8_t eepromerfolg=0;
						
						// EEPROMTXdaten: Array mit den Daten fuer das EEPROM, vom Webserver 
						// hbyte, lbyte: Adresse im EEPROM, geschickt vom Webserver
						
                  //Daten ins EEPROM schreiben
                  
						eepromerfolg=EEPROMTagSchreiben(0xA0,(void*)EEPROMTXdaten,hbyte ,lbyte);
						//err_gotoxy(19,0);
						//err_puts("erf\0");
						//err_puthex(eepromerfolg);
						
						// erledigt
						
						EEPROMTXStartdaten=0;
						
						// Quittung senden: Daten fuer naechstes Paket von SPI laden
						
                                                         // Im Moment nicht verwendet. Webserver fragt nicht nach
						
						if (eepromerfolg==0) // alles ok
						{
							err_gotoxy(19,1);
							err_putc('<');
							
							out_startdaten= EEPROMCONFIRMTASK; // B5
							outbuffer[0]=EEPROMCONFIRMTASK;
							
							// SPI senden veranlassen
							BUS_Status |= (1<<SPI_SENDBIT);
							
							outbuffer[1]=1;
							err_gotoxy(19,1);
							err_putc('^');
							
						}
						else
						{
							// SPI senden verhindern
							spistatus |= (1<<TWI_ERR_BIT);

							outbuffer[1]=2;
							lcd_gotoxy(19,1);
							lcd_putc('!');
							
						}
						
						lbyte=0;
						hbyte=0;
						in_startdaten=0;
						//delay_ms(200);
						
					}break;

#pragma mark PWMREADTASK
               case PWMREADTASK:
               {
                  EEPROMTXStartdaten=PWMREADTASK; // BA
                  lbyte=in_lbdaten;
						hbyte=in_hbdaten;
						
						uint8_t readerfolg =0;
						
						// EEPROMTXdaten: Array mit den Daten des EEPROMS, wird zum Webserver geschickt
						// hbyte, lbyte: Adresse im EEPROM, geschickt vom Webserver
						readerfolg = EEPROMTagLesen(0xA0, (void*)EEPROMTXdaten, hbyte, lbyte);
						if (readerfolg==0)
						{
							lcd_gotoxy(0,3);
							lcd_puts("rP \0");
							
							//Empfangene Angaben vom EEPRPOM
							lcd_puthex(in_startdaten);
							lcd_putc(' ');
							
							lcd_puthex(in_hbdaten);
							lcd_puthex(in_lbdaten);
							lcd_putc(' ');
							lcd_puthex(EEPROMTXdaten[0]); //PWM-Daten
							lcd_puthex(EEPROMTXdaten[1]);
							lcd_puthex(EEPROMTXdaten[2]);
							lcd_puthex(EEPROMTXdaten[3]);
							
                     uint8_t pwmwriteerfolg=0;
                     pwmstatus |= (1<<(hbyte/2)); // raum ist hbyte/2                  
                     
                  }
               }break;
						
						
					case DATATASK:
						
					//default:						// DATATASK
					{

#pragma mark Lese- und Schreibstatus setzen
						//	******************************
						//	*	HomeCentral-Tasks setzen
						//	******************************
						//	Wer soll schreiben?
						//SchreibStatus |= (1<< HEIZUNG); // in Uhr-Abfrage gesetzt
						//SchreibStatus |= (1<< WERKSTATT);
						//SchreibStatus |= (1<< BUERO);
						//SchreibStatus |= (1<< WOZI);
						//SchreibStatus |= (1<< LABOR);
						//SchreibStatus |= (1<< OG1);
						//SchreibStatus |= (1<< OG2);
						//SchreibStatus |= (1<< ESTRICH);
						
						
						// Wer soll lesen?
						//LeseStatus |= (1<< HEIZUNG);
						//LeseStatus |= (1<< WERKSTATT);
						//LeseStatus |= (1<< BUERO);
						//LeseStatus |= (1<< WOZI);
						//LeseStatus |= (1<< LABOR);
						//LeseStatus |= (1<< OG1);
						//LeseStatus |= (1<< OG2);
						//LeseStatus |= (1<< ESTRICH);
						
#pragma mark readSR						
						Read_Device=0;
						Write_Device=0;
						
						//err_gotoxy(5,1);
						//Zaehler fuer TWI-Loop
						//err_puthex(loopCounterTWI++);
						lcd_gotoxy(15,0);
//						lcd_puts("U\0");
						lcd_putc(' ');
						OSZIBLO;
						//err_gotoxy(0,0);

						//err_puts("U\0");
                  
                  // ++++++++++++++++++++++++++++++++
                  // TEST
                  // ++++++++++++++++++++++++++++++++
                  if (test)
                  {
                     
                     uint8_t RTC_erfolg=1;
                     uint8_t versuche=0;
                     // RTC lesen
                     while (RTC_erfolg && versuche<0x0F)
                     {
                        RTC_erfolg = RTC_Abrufen();
                        err_gotoxy(4,0);
                        err_puts("R1\0");
                        //err_putc(' ');
                        err_puthex(RTC_erfolg);
                        //err_putc(' ');
                        err_puthex(versuche);
                        versuche++;
                     }
                     
                     
                     SchreibStatus=0;
                     LeseStatus=0;


                     uhrstatus &= ~(1<<SYNC_READY);
                     uhrstatus |= (1<<SYNC_OK);
                     uhrstatus &= ~(1<<SYNC_NEW);                 // TWI soll jetzt Daten senden

                   }
						else
                  {
                     
#pragma mark Uhr
                     //TWBR=48;
                     if (!(uhrstatus & (1<<SYNC_NULL)))     // Nach reset, rtc noch nicht abfragen
                        
                     {
                        uint8_t versuche=0;
                        uint8_t RTC_erfolg=1;
                        
                        //         RTC_erfolg = RTC_Abrufen();
                        while (RTC_erfolg && versuche<0x0F)
                        {
                           RTC_erfolg = RTC_Abrufen();
                           err_gotoxy(4,0);
                           err_puts("R1\0");
                           //err_putc(' ');
                           err_puthex(RTC_erfolg);
                           //err_putc(' ');
                           err_puthex(versuche);
                           versuche++;
                        }
                        
                        //OSZIBHI;
                        
                        if (RTC_erfolg)				// Fehler, aussteigen
                        {
                           SchreibStatus=0;
                           LeseStatus=0;
                           err_gotoxy(4,0);
                           err_puts("RTC1\0");
                           err_putc('*');
                           err_puthex(RTC_erfolg);
                           //err_putc('!');
                           
                           outbuffer[6] = RTC_erfolg;
                           //break;							// aktuelle Schlaufe verlassen
                           
                        }
                        else
                        {
                           err_gotoxy(13,0);
                           err_putc(' ');
                           err_putc(' ');
                           err_putc(' ');
                           err_gotoxy(19,0);
                           err_putc('+');
                           
                        }
                     }
                     
                     
                     lcd_gotoxy(16,0);
                     lcd_putc('+');
                     
                     lcd_putint2(DCF77daten[1]);
                     //lcd_putc(':');
                     //lcd_putint2(DCF77daten[0]);
                     
                     
#pragma mark Synchronisation
                     
                     
                     // Synchronisation
                     
                     
                     //sync start
                     // alle 60 Min: Warten starten
                     if ((((min/30)&&(min%30==0)&&(std<23))||(uhrstatus & (1<<SYNC_NULL)))&& (!(uhrstatus & (1<<SYNC_WAIT))))
                     {
                         {
                           uhrstatus &= ~(1<<SYNC_OK);
                           uhrstatus |= (1<<SYNC_WAIT); // Beginn Sync, Warten starten
                           DCF77_counter=0; // Zaehler fuer korrekte Daten
                           
                           err_gotoxy(4,0);
                           err_puts("            \0");
                           
                           lcd_gotoxy(0,1);
                           lcd_puts("S     \0");
                        }
                        
                     }
                     
                     
                     if (uhrstatus & (1<<SYNC_WAIT)) // Warten auf genuegende Anzahl korrekter Daten
                     {
                        //uint8_t newminute=0;
                        uint8_t DCF77_erfolg = UhrAbrufen(); // DCF-Uhr abrufen
                        if (DCF77_erfolg) // Fehler
                        {
                           lcd_gotoxy(16, 1);
                           err_puts("DCF\0");
                           err_putc('!');
                           DCF77_counter=0; // zurueck auf Feld 1. Uhr hat einen Fehler, Synchronisation neu starten
                        }
                        else // Wert ist OK
                        {
                           lcd_gotoxy(16, 1);
                           lcd_putc('+');
                           lcd_putint2(DCF77daten[0]);         // Minuten
                           // erste Synchronisation?
                           if (uhrstatus & (1<<SYNC_NULL))     // Nach reset, noch keine oldmin ...
                           {
                              // startwerte setzen
                              oldmin=DCF77daten[0];
                              oldstd=DCF77daten[1];
                              oldtag=DCF77daten[2];
                              uhrstatus &= ~(1<<SYNC_NULL);
                              uhrstatus |= (1<<SYNC_NEW);         // TWI soll noch keine Daten uebertragen
                           }
                           else if (!(oldmin == DCF77daten[0]))   // minute hat sich geaendert
                           {
                              //lcd_gotoxy(0,1);
                              //lcd_puts("       \0");
                              
                              lcd_gotoxy(1,1);
                              lcd_putc('1');                      // Anzeige, dass Synch laeuft
                              
                              // Kontrolle, ob Daten korrekt
                              
                              if ((oldstd==DCF77daten[1])&&(oldtag==DCF77daten[2])) // stunde und tag sind noch gleich
                              {
                                 lcd_gotoxy(1,1);
                                 lcd_putc('2');
                                 if ( DCF77daten[0]==(oldmin+1)) // naechste minute, Aenderung korrekt
                                 {
                                    lcd_gotoxy(1,1);
                                    lcd_putc('3');
                                    
                                    DCF77_counter++;
                                    
                                     
                                    lcd_putint1(DCF77_counter);
                                    oldmin=DCF77daten[0];
                                    if (DCF77_counter >= MIN_SYNC) // genuegende Anzahl korrekte Daten
                                    {
                                       lcd_gotoxy(1,1);
                                       lcd_putc('4');
                                       lcd_putc(' ');
                                       DCF77_counter =0;
                                       uhrstatus |= (1<<SYNC_READY); // Synchronisation ausloesen
                                       uhrstatus &= ~(1<<SYNC_WAIT); // WAIT zuruecksetzen
                                    }
                                 }
                                 else // fehler
                                 {
                                    DCF77_counter =0;                // Counter zuruecksetzen
                                    
                                    oldmin=DCF77daten[0];
                                    oldstd=DCF77daten[1];
                                    oldtag=DCF77daten[2];
                                    
                                 }
                                 
                              } //if (oldstd==DCF77daten[1])&&(oldtag=DCF77daten[2])
                              else // fehler
                              {
                                 DCF77_counter =0;
                                 oldmin=DCF77daten[0];
                                 oldstd=DCF77daten[1];
                                 oldtag=DCF77daten[2];
                                 
                              }
                              
                           }
                           
                           
                           
                        } // end DCF77erfolg=0
                        
                     } // end (uhrstatus & (1<<SYNC_WAIT)
                     
                     // TODO: Bei fehlgeschlagener Synchronisation Uhr unverändert lassen. Eventuell mit ODER SYNC_READY und SYNC_NEW und SYNC_OK
                     
                     if (uhrstatus & (1<<SYNC_READY)) // Uhr ist wieder bereit
                     {
                        uint8_t res=0;
                        res=rtc_write_Zeit(DCF77daten[1], DCF77daten[0],0); // stunde, minute, sekunde
                        
                        res=rtc_write_Datum(DCF77daten[5], DCF77daten[2], DCF77daten[3],DCF77daten[4]);
                        uhrstatus &= ~(1<<SYNC_READY);
                        uhrstatus |= (1<<SYNC_OK);
                        uhrstatus &= ~(1<<SYNC_NEW);                 // TWI soll jetzt Daten senden
                        
                        lcd_gotoxy(0,1);
                        lcd_puts("S:\0");
                        if (res)
                        {
                           lcd_puts("SYNC\0");
                           lcd_puts("err:\0");
                           lcd_puthex(res);
                        }
                        else
                        {
                           
                           lcd_putint2(DCF77daten[1]);
                           lcd_putc(':');
                           lcd_putint2(DCF77daten[0]);
                           lcd_putc(':');
                           lcd_putint1(DCF77daten[5]-1);
                        }
                        lcd_gotoxy(16, 1);
                        lcd_puts("   \0");
                        
                        uint8_t RTC_erfolg = RTC_Abrufen();
                        if (RTC_erfolg)                           // Fehler, aussteigen
                        {
                           SchreibStatus=0;
                           LeseStatus=0;
                           err_gotoxy(10,0);
                           err_puts("RTC2\0");
                           err_putc(' ');
                           err_puthex(RTC_erfolg);
                           err_putc('!');
                           
                           outbuffer[6] = RTC_erfolg;
                           // aktuelle Schlaufe verlassen
                        }
                        
                        
                     }
                     // Ende Synchronisation
                  } // if NOT test
						
                  
                  outbuffer[46] = RTCdaten[1]; // stunde
                  outbuffer[47] = RTCdaten[0]; // minute
                  outbuffer[45] = RTCdaten[5]; // wochentag
                  
                  outbuffer[40] = RTCdaten[2]; // tagdesmonats
                  
                  outbuffer[41] = RTCdaten[3] & 0x0F; //  monat
                  
                  uint8_t jahrab2010 = RTCdaten[4]  -10; // Jahr ab 2010
                  jahrab2010 <<=4; // bit
                  outbuffer[41] |= jahrab2010  ;

                  // ++++++++++++++++++++++++++++++++
                  // End NOT TEST
                  // ++++++++++++++++++++++++++++++++						
						
						readSR();				// Liste der abzufragenden Slaves lesen
						
						// Nur abfragen, wenn TWI laeuft
						if (BUS_Status & (1<<TWI_CONTROLBIT))
						{
							LeseStatus=Read_Device;
							SchreibStatus=Write_Device;
						}
						else 
						{
							SchreibStatus=0; 
							LeseStatus=0;
						}
                  
                  outbuffer[43] = LeseStatus;
                  outbuffer[44] = SchreibStatus;

						
						err_clr_line(1);
						err_gotoxy(12,1);
						err_putc('R');
						//err_putc(' ');
						//err_puthex(Read_Device);
						err_puthex(LeseStatus);
						err_putc(' ');
						
						err_putc('W');
						//err_putc(' ');
						//err_puthex(Write_Device);
						err_puthex(SchreibStatus);
						
						// Version anzeigen
						
						err_gotoxy(0,1);
						err_puts("V:\0");
						err_puts(VERSION);
                  
						delay_ms(64);
						in_startdaten=0;
						
						//	******************************
						//	*	HomeCentral-Slaves abfragen, sofern Schreib- oder Lesestatus gesetzt ist
						//	******************************
						
#pragma mark Lese- und Schreibstatus abarbeiten
						
						Write_Err=0;
						Read_Err=0;
						EEPROM_Err=0;
						
						if ((SchreibStatus || LeseStatus)&& (!(uhrstatus & (1<<SYNC_NEW))))		// Uhr nicht gerade am Synchronisieren
                     // && (twi_HI_count0 >= 0x02))//&&(TastaturCount==0))
						{
							OSZIALO;
							BUS_Status |= (1<<SPI_SENDBIT);
							out_startdaten= DATATASK;//	C0: Daten sammeln fuer den Webserver
							
							
							// Versuch 12.08: Verblassen des 2*20-LCD verhindern
							//err_initialize(ERR_FUNCTION_8x2, ERR_CMD_ENTRY_INC, ERR_CMD_ON);
							
							twi_Call_count0=0;
							twi_Reply_count0=0;
							twi_Stat_count=0; //	Stat_count zurücksetzen
							
							{
								//OSZIAHI;
								// Ausgangsdaten reseten
								out_hbdaten=0;
								out_lbdaten=0;
								
								uint8_t i=0;
								for (i=0 ; i<SPI_BUFSIZE; i++) 
								{
									//outbuffer[i]=0;
								}
								//err_gotoxy(15,0);
								//err_putc('U');
								
								wdt_reset();
								//twi_Reply_count0++; // Call ist OK
								//lcd_clr_part(0,12,19);
								//lcd_gotoxy(12,0);
								//lcd_gotoxy(15,0);
								//lcd_put_zeit(DCF77daten[0],DCF77daten[1]);
								
								min= RTCdaten[0];
								std= RTCdaten[1];
								tag= RTCdaten[2];
								
								lcd_gotoxy(17,0);
								lcd_putint2(min);
								/*								
								 min= DCF77daten[0];
								 std= DCF77daten[1];
								 tag= DCF77daten[5];
								 lcd_gotoxy(17,0);
								 lcd_putint2(min);
								 */								
                        //uint8_t StundenCode=0;
								
								//	Brennerlaufzeit addieren
								if ((min > Zeit.minute) || ((min ==0)&&(std==0)) || (std> Zeit.stunde) ) //neue Minute oder neue Stunde oder neuer Tag
								{
									uint8_t synchfehler=0;
									//						err_gotoxy(9,0);
									//						err_puts("A\0");
									//delay_ms(200);
									//err_clr_line(1);
									
									
									Zeit.minute = RTCdaten[0];
									Zeit.stunde = RTCdaten[1];
									Zeit.kalendertag = RTCdaten[2];
									Zeit.kalendermonat = RTCdaten[3];
									Zeit.kalenderjahr = RTCdaten[4];
									Zeit.wochentag = RTCdaten[5]-1;	// RTC, DCF77: Montag=1; EEPROM: Montag=0
									
									// Daten fuer Sync nachfuehren
									if (!(uhrstatus & (1<<SYNC_WAIT)))
									{
										oldmin=RTCdaten[0];
										oldstd=RTCdaten[1];
										oldtag=RTCdaten[2];
									}
									/*
									 Zeit.minute = DCF77daten[0];
									 Zeit.stunde = DCF77daten[1];
									 Zeit.kalendertag = DCF77daten[2];
									 Zeit.kalendermonat = DCF77daten[3];
									 Zeit.kalenderjahr = DCF77daten[4];
									 Zeit.wochentag = DCF77daten[5]-1;// DCF77 ist 1-basiert ab Sonntag
									 */
									
									synchfehler=DCF77daten[6];
									//outbuffer[29]=DCF77daten[6];
									
									neueZeit=1; //	Zeit in Räumen aktualisieren
									// 29.9.08						
									if ((Menu_Ebene & 0xF0)==0) // Oberste Ebene, AnzeigeWochentag aktualisieren
									{
										AnzeigeWochentag=Zeit.wochentag;
										
									}
									
									/*
									 lcd_clr_part(0,0,14);
									 if (Zeit.wochentag < 8)
									 {
									 lcd_put_wochentag(Zeit.wochentag);
									 }
									 lcd_putc(' ');
									 if (Zeit.kalendertag < 32)
									 {
									 lcd_putint2(Zeit.kalendertag);
									 }
									 lcd_putc('.');
									 if (Zeit.kalendermonat < 13)
									 {
									 lcd_putint2(Zeit.kalendermonat);
									 }
									 lcd_putc('.');
									 if (Zeit.kalenderjahr<99)
									 {
									 lcd_putint2(Zeit.kalenderjahr);
									 }
									 //lcd_gotoxy(15,1);
									 //lcd_put_zeit(Zeit.minute,Zeit.stunde);
									 */
									
									
									
									// Uhr-Fehler anzeigen
									// err_clr_part(0,13,19);
									// err_puts("S \0");
									// err_putint(synchfehler);
									
									//err_gotoxy(14,0);
									//err_puts("w \0");
									//err_puthex(tempWDT_Count);
									
								} //Zeit vorwaertsstellen
								wdt_reset();
								// end Uhr lesen
								
								UhrLesen=0;
								HeizungRXdaten[6]=0x00;
								HeizungRXdaten[7]=0x00;
#pragma mark Heizung					
								//	*************
								//	*	Heizung
								//	*************
								OSZIAHI;
								delay_ms(1);
								OSZIALO;
								uint8_t HeizungStundencode=0;
								uint8_t RinneStundencode=0;
								if (SchreibStatus & (1<< HEIZUNG))	//schreiben an Heizung
								{
									delay_ms(2);
									/*
									 Byte 0:	Heizungsstatus 
									 
									 Bit 0,1: Brenner ON/OFF
									 Bit 2		Halbstunde
									 Bit 3:	Mode Tag/Nacht-Einstellung
									 
									 */
									//err_gotoxy(0,1);
									//err_puts("H\0");
									
									uint8_t tagblock[buffer_size];
									uint8_t Stundencode=0;
									txbuffer[0]=0;
									
									// Brenner-Tagplan lesen
									
									wdt_reset();
									
									// code fuer Objekt 0 aus EEPROM lesen, 8 bytes
									// EEPROM_WOCHENPLAN_ADRESSE: A0
                           // HEIZUNG: 0
                           // Objekt: 0
                           // Tag:Zeit.wochentag
                           // xyz > uint_t16 code = Raum*100 + objekt*10 + Tag

                           
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock, HEIZUNG, 0, Zeit.wochentag);
									//OSZIAHI;
									delay_ms(1);
									//OSZIALO;
									//err_puthex(erfolg);
									wdt_reset();
									if (erfolg==0)
									{
										
										Stundencode=Tagplanwert(tagblock, Zeit.stunde); // Stundencode fuer aktuelle Stunde
										
										HeizungStundencode=Tagplanwert(tagblock, Zeit.stunde);
										
                              //err_gotoxy(0,1);
										//err_puts("            \0");
										
										//err_gotoxy(0,0);
										//err_putc('W');
										//err_puthex(Zeit.wochentag);
										//err_putc(':');
										//err_putc('H');
										//err_puthex(HeizungStundencode);
										//			01 zweite halbe Stunde ON
										//			10	erste halbe Stunde ON
										//			11	ganze Stunde ON
										//err_putc(':');
										
										
										//Echo = HeizungStundencode;
										
										HeizungStundencode &= 0x03; // Bit 0 und 1 filtern
										//outbuffer[27] = HeizungStundencode;
										
										
										//err_puts(" c\0");
										//err_puthex(Stundencode);
										txbuffer[0]=0;
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde  |_
											{
												
												txbuffer[0]=(Stundencode >=2); //Werte 2: erste halbe Std, 3: ganze Std auf FULL Wert 0: Brenner RED/OFF
												// 
												// Bit 3 fuer Anzeige der Halbstunde ist Null
											}break;
												
											case 1: // zweite halbe Stunde _|
											{
												txbuffer[0]=((Stundencode ==1)||(Stundencode==3)); //Werte 1: zweite halbe Std, 3: ganze Std auf FULL Wert 0: Brenner RED/OFF
												HeizungStundencode |= (1<<3);	// Bit 3 fuer Anzeige der Halbstunde ist 1
											}break;
												
											default:
											{
												txbuffer[0]=0; // 7.4.11
												
											}break;
										}//switch
										
										//err_putc('B');
										//err_puthex(txbuffer[0]);
										
										// An Heizung wird nur der jeweilige Wert fuer die Uhr weitergegeben: 
										// 
										outbuffer[28] = txbuffer[0];
										
										// HeizungStundencode wird an HomeServer weitergegeben
										
										//err_gotoxy(7,1);
										//err_puthex(txbuffer[0]);
									}//erfolg ==0
									else
									{
										//							err_gotoxy(1,1);
										//							err_puts("X\0");	//	Wochentag lesen
										// SPI senden verhindern
										
										spistatus |= (1<<TWI_ERR_BIT);
										EEPROM_Err |= (1<<HEIZUNG);
										
									}
									
									// end Objekt 0
									
									// Code fuer Objekt 1 lesen: Tag/Nacht-Mode der Heizung
									
									uint8_t tagblock1[buffer_size];
									uint8_t Stundencode1=0;	
									uint8_t obj1erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock1, HEIZUNG, 1, Zeit.wochentag);
									//OSZIAHI;
									delay_ms(1);
									//OSZIALO;
									if (obj1erfolg==0) // EEPROM erfolgreich gelesen
									{
										Stundencode1=Tagplanwert(tagblock1, Zeit.stunde);
										
										//				outbuffer[27] = Stundencode1; // auskomm 9.4.11
										//Simulation
										
										//Stundencode1=Zeit.minute % 5;
										txbuffer[3]=0;
										Stundencode1 &= 0x03;	// Bit 0 und 1 filtern
										switch (Stundencode1)
										{
											case 0: // Grundstellung, Tag ON, Nacht OFF
												txbuffer[3] = 0x01; // Schalterposition 1
												break;
												
											case 1: // Tag ON, Nacht Red
												txbuffer[3] = 0x02; // Schalterposition 2
												break;
												
											case 2:
												txbuffer[3] = 0x04; // Schalterposition 4
												break;
												
												
										}// switch Stundencode1
										
										
										// Test
										if  (test)
										{
											Testposition = (PINB & 0xE0); // Bit 5,6,7 von Port B
											Testposition >>= 5;
											//err_clr_line(0);
											//delay_ms(100);
											//err_gotoxy(0,0);
											//err_puts("pos:\0");
											
											//err_puthex(Testposition);
											txbuffer[3]=Testposition;										
										}
										
										Stundencode1 <<=4;			// Bit 4 bis 5 in Heizungsstundencode fuer Mode
										
										HeizungStundencode |= Stundencode1;
										
										
										
										
									} // erfolg==0
									else
									{
										// SPI senden verhindern
										spistatus |= (1<<TWI_ERR_BIT);
										EEPROM_Err |= (1<<HEIZUNG);
									}
									
									// end Objekt 1
									
									// Code fuer Objekt 2 lesen: Dachrinnenheizung
									
									uint8_t tagblock2[buffer_size];
									uint8_t obj2erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock2, HEIZUNG, 2, Zeit.wochentag);
									//OSZIAHI;
									delay_ms(1);
									//OSZIALO;
									if (obj2erfolg==0) // EEPROM erfolgreich gelesen
									{
										RinneStundencode=Tagplanwert(tagblock2, Zeit.stunde);
										
										//	outbuffer[30] = RinneStundencode; // auskomm 7.4.11
										
										//err_gotoxy(0,1);
										//err_puts("         \0");
										//err_gotoxy(0,1);
										//err_puts("Ri:\0");
										//err_puthex(RinneStundencode);
										RinneStundencode &= 0x03;	// Bit 0 und 1 filtern fuer outbuffer[5]
										
										//err_putc(' ');
										//err_puthex(RinneStundencode);
                              
                              // Code fuer Rinne an Heizung schicken
										if (RinneStundencode)
										{
											txbuffer[2] = 0x01;
										}
										else
										{
											txbuffer[2] = 0x00;
										}
										
										
										
									}
									else
									{
										// SPI senden verhindern
										spistatus |= (1<<TWI_ERR_BIT);
										EEPROM_Err |= (1<<HEIZUNG);
									}
                           
                           /*
                            Byte 6 lesen
                            Kann code enthalten, der das Verhalten des Slave steuert.
                            PWM_CODEBIT gesetzt:
                            Es folgt ein TWI-Paket mit den Positionsdaten fuer den PWM
                            Diese werden in den Array Servoposition geladen
                            PWM_SCHALTERBIT gesetzt:
                            Byte 7 enthaelt Schalterposition, die eingestellt werden soll
                            PWM_POSITIONBIT gesetzt:
                            Byte 7 enthaelt Impulslaenge fuer PWM
                            
                            */
                           
                           /*
                            PWM lesen
                            */
 									uint8_t tagblock3[buffer_size];
									uint8_t obj3erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock3, HEIZUNG, 3, 0);
									//OSZIAHI;
									delay_ms(1);
                          
                           if (obj3erfolg==0) // EEPROM erfolgreich gelesen
									{
                              
										//err_gotoxy(0,1);
										//err_puts("         \0");
										//err_gotoxy(0,1);
										//err_puts("PWM:\0");
                              uint8_t pwmcode = tagblock3[0];
										//err_puthex(pwmcode);
                              pwmcode = tagblock3[1];
                              //err_puthex(pwmcode);
                              pwmcode = tagblock3[2];
                              //err_puthex(pwmcode);
                              
                           }
                           else
                           {
                              
                           }
                           
                           /*
                            end PWM
                            */
									
									// end Objekt 2
									//err_gotoxy(0,8);
									//err_puts("pos:\0");
									//err_puthex(txbuffer[3]);
									
									
									
									wdt_reset();
									twi_Call_count0++;
									
                           // txbuffer an Zeizung schicken
                           erfolg=SlaveSchreiben(HEIZUNG_ADRESSE);
									
                           //OSZIAHI;
									delay_ms(1);
									//OSZIALO;
									//err_puthex(txbuffer[3]);
									wdt_reset();
									if (erfolg)
									{
										//err_clr_part(1,9,19);
										//err_puts("w Hz err\0");
										//err_puthex(erfolg);					
										//err_gotoxy(1,1);
										//err_puts("Y\0");	//	Schreiben
										Write_Err |= (1<<HEIZUNG);
										spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
										
									}
									
									txbuffer[0]=0;
									txbuffer[2]=0;
                           
                           
									
									SchreibStatus &= ~(1<< HEIZUNG); // Flag zuruecksetzen
									Write_Device &= ~(1<< HEIZUNG); // Flag zuruecksetzen
								}
								
								//OSZIAHI;
								delay_ms(1);
								//OSZIALO;
								if (LeseStatus & (1<< HEIZUNG))	//lesen von Heizung
								{
									delay_ms(2);
									//						err_gotoxy(0,1);
									//						err_puts("H\0");
									wdt_reset();
									twi_Call_count0++;
									uint8_t Heizungerfolg=SlavedatenLesen(HEIZUNG_ADRESSE,HeizungRXdaten);
									//OSZIAHI;
									delay_ms(1);
									//OSZIALO;
									
									wdt_reset();
									
									
									//err_putint1(adcerfolg);
									if (Heizungerfolg) // Fehler
									{
										Read_Err |= (1<<HEIZUNG);
										spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										//							err_gotoxy(2,1);
										//							err_puts("1\0");						
										twi_Reply_count0++;
										// Echo zeigen
										/*
										err_gotoxy(7,0);
										err_putc(':');
										err_putc('E');
										err_puts("  \0");
										err_gotoxy(9,0);
										Echo = HeizungRXdaten[6];
										err_puthex(Echo);
										*/
									}
									
									
									//	Bit 0, 1	TagWert fuer Stunde
									//	Bit 3 fuer aktuelle Lage innerhalb der Stunde einfuegen: 0: erste halbe Stunde 1: zweite halbe Stunde
									wdt_reset();
									//err_gotoxy(4,1);
									//err_puts("D3");
									//err_puthex(HeizungRXdaten[3]);
									
									//err_gotoxy(12,1);
									//err_puts("Ri\0");
									//err_puthex(RinneStundencode);
									RinneStundencode &= 0x03;	// Bit 0 und 1 filtern fuer outbuffer[5]
									//err_puthex(RinneStundencode);
									
									
									// Verschieben auf Bit 6,7
									RinneStundencode <<=6; 
									
									uint8_t dataerfolg=0;
									wdt_reset();
									if (dataerfolg)
									{
										i2c_stop();
										//delay_ms(800);
									}
									else
									{
										//								err_gotoxy(17,1);
										//								err_puts("1\0");						
										//					twi_Reply_count0++;
									}
									
#pragma mark outbuffer										
									//out_startdaten=DATATASK;
									outbuffer[0] = (HEIZUNG << 5);					// Bit 5-7: Raumnummer
									outbuffer[0] |= (Zeit.stunde & 0x1F);			//	Bit 0-4: Stunde, 5 bit
									outbuffer[1] = (0x01 << 6);						// Bits 6,7: Art=1
									outbuffer[1] |= Zeit.minute & 0x3F;				// Bits 0-5: Minute, 6 bit
									outbuffer[2] = HeizungRXdaten[0];				//	Vorlauf
									
									outbuffer[3] = HeizungRXdaten[1];				//	Rücklauf
									outbuffer[4] = HeizungRXdaten[2];				//	Aussen
									outbuffer[5] = 0;
									outbuffer[5] |= HeizungRXdaten[3];				//	Brennerstatus Bit 2
									outbuffer[5] |= HeizungStundencode;				// Bit 4, 5 gefiltert aus Tagplanwert von Brenner und Mode
									outbuffer[5] |= RinneStundencode;				// Bit 6, 7 gefiltert aus Tagplanwert von Rinne
									
									outbuffer[6] = 0x00;                       // eventuell err von RTC
									outbuffer[7] = 0x00;	// offen
									
									//	outbuffer[29] = Echo;// 7.4.11 auskomm.
									//outbuffer[23] |= Zeit.minute & 0x3F;				// Bits 0-5: Minute, 6 bit
									RinneStundencode=0;
									LeseStatus &= ~(1<< HEIZUNG);// Flag zuruecksetzen
									Read_Device &= ~(1<< HEIZUNG);// Flag zuruecksetzen
								}
								err_gotoxy(0,1);
								err_puts("         \0");
								
#pragma mark Werkstatt					
								//	*****************
								//	**	Werkstatt
								//	*****************
								//OSZIAHI;
								//delay_ms(1);
								//OSZIALO;
								
								if (SchreibStatus & (1<< WERKSTATT))	//schreiben an Werkstatt
								{	
									delay_ms(2);
									//						err_gotoxy(4,1);
									//						err_puts("W\0");
                           /*
                            Status byte 18
                            bit 0:   Lampe
                            bit 1:   Ofen
                            
                            
                            */
                           uint8_t werkstattstatus=0;
									uint8_t Werkstatttagblock[buffer_size];
									uint8_t Stundencode=0;
									txbuffer[0]=0;
									wdt_reset();
									uint8_t erfolg=0;
									
									// Objekt 0: Lampe
									
									erfolg = WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, Werkstatttagblock, WERKSTATT, 0, Zeit.wochentag);
									//err_puthex(erfolg);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(Werkstatttagblock, Zeit.stunde);
										//err_puts(" c\0");
										//err_puthex(Stundencode);
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												if (Stundencode >=2)	//	Werte 2, 3: Lampe FULL Wert 0: Lampe OFF
												{
													txbuffer[0] |= (1<< 0); // Bit 0 setzen
                                       werkstattstatus |= (1<< 0);
												}
												else
												{
													txbuffer[0] &= ~(1<< 0); // Bit 0 zuruecksetzen
												}
											}break;
												
											case 1: // zweite halbe Stunde
											{
												
												if ((Stundencode ==1)||(Stundencode==3))//Werte 1, 3: Brenner auf FULL Wert 0: Lampe OFF
												{
													txbuffer[0] |= (1<< 0); // Bit 0 setzen
                                       werkstattstatus |= (1<< 0);
												}
												else
												{
													txbuffer[0] &= ~(1<< 0); // Bit 0 zuruecksetzen
												}
												
											}break;
										}//switch
										
									}//erfolg
									else
									{
										EEPROM_Err |= (1<<WERKSTATT);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									
									// Objekt 1: Ofen
									txbuffer[1]=0;
									erfolg = WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, Werkstatttagblock, WERKSTATT, 1, Zeit.wochentag);
									//err_puthex(erfolg);
									wdt_reset();
									if (erfolg==0)
									{
										int OfenStundencode=Tagplanwert(Werkstatttagblock, Zeit.stunde);
                              OfenStundencode &= 0x03;	// Bit 0 und 1 filtern fuer TXdaten[1]
                              
                              
                              if (OfenStundencode) // Stundenwert ist >0, Ofen ein
                              {
                                 //txbuffer[0] |= (1<< 1); // Bit 1 setzen
                                 werkstattstatus |= (1<< 1);
                              }
                              else
                              {
                                // txbuffer[0] &= ~(1<< 1); // Bit 1 zuruecksetzen
                              }
                              
                              txbuffer[1]= OfenStundencode;
									}//erfolg
									else
									{
										//err_gotoxy(5,1);
										//err_puts("X\0");
										//err_putint1(txbuffer[0]);
										//err_clr_part(1,9,19);
										//err_puts("wl H er\0");
										//err_puthex(erfolg);
										//delay_ms(800);
										EEPROM_Err |= (1<<WERKSTATT);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									// end Objekt 1						

									wdt_reset();
									twi_Call_count0++;
									erfolg=SlaveSchreiben(WERKSTATT_ADRESSE);
									wdt_reset();
									if (erfolg)
									{
										Write_Err |= (1<<WERKSTATT);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									txbuffer[0]=0;
                           txbuffer[1]=0;
#pragma mark PWM
                           /*
                            PWM lesen
                            */
 									uint8_t tagblock3[buffer_size];
                           // EEPROM_WOCHENPLAN_ADRESSE, tagblock, raum, objekt, Wochentag
									uint8_t obj3erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock3, HEIZUNG, 3, 0);
									//OSZIAHI;
									delay_ms(1);
                           
                           if (obj3erfolg==0) // EEPROM erfolgreich gelesen
									{
										//err_gotoxy(10,0);
										//err_puts("         \0");
										//err_gotoxy(10,0);
										//err_puts("PWM:\0");
                              uint8_t pwmcode = tagblock3[0];
										//err_puthex(pwmcode);
                              pwmcode = tagblock3[1];
                              //err_puthex(pwmcode);
                              pwmcode = tagblock3[2];
                              //err_puthex(pwmcode);
                              
                              
                           }
                           else
                           {
                              
                           }
                           outbuffer[18] = werkstattstatus;
                           
                           
                        SchreibStatus &= ~(1<< WERKSTATT);
								}
								
								if (LeseStatus & (1<< WERKSTATT))	//lesen von Werkstatt
								{
									delay_ms(2);
									//err_gotoxy(5,1);
									//err_puts("W\0");
									
									wdt_reset();
									twi_Call_count0++;
									uint8_t werkstatterfolg=SlavedatenLesen(WERKSTATT_ADRESSE,WerkstattRXdaten);
									wdt_reset();
									
									err_puthex(WerkstattRXdaten[3]);
									
									if (werkstatterfolg)
									{
										//err_gotoxy(6,1);
										//err_puts("Z\0");
										Read_Err |= (1<<WERKSTATT);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
										if (WerkstattRXdaten[3] & (1<<TIEFKUEHLALARM))
										{
											outbuffer[31] |= (1<<TIEFKUEHLALARM); // Alarmbit setzen
                                 
										}
										else 
										{
											outbuffer[31] &= ~(1<<TIEFKUEHLALARM); // Alarmbit zuruecksetzen
										}
										
										if (WerkstattRXdaten[3] & (1<<WASSERALARMKELLER))
										{
											outbuffer[31] |= (1<<WASSERALARMKELLER); // Alarmbit setzen
										}
										else 
										{
											outbuffer[31] &= ~(1<<WASSERALARMKELLER); // Alarmbit zuruecksetzen
										}
										
										
										
									} // lesen ok
									
									
									
									
									LeseStatus &= ~(1<< WERKSTATT);
								}
#pragma mark Buero					
								//	*****************
								//	*	Buero
								//	*****************
								
								if (SchreibStatus & (1<< BUERO))	//schreiben an Buero
								{
                           /*
                            Status byte 22
                            bit 0:   Lampe
                            bit 1:   Ofen
                            
                            
                            */

                           uint8_t buerostatus=0;
									delay_ms(2);
									//err_gotoxy(8,1);
									//err_puts("B\0");
									//err_puts("  \0"); // Codes loeschen
									PORTC |= (1<<TWICOUNTPIN);
									uint8_t BueroTagblock[buffer_size];
									uint8_t Stundencode=0;
									//err_clr_line(0);
									//err_gotoxy(12,1);
									//err_puts("WB:\0");
									//err_putint1(Zeit.wochentag);
									
									// Buero-Tagplan lesen
									
									//err_clr_part(0,0,10);
									//err_puts("WT B rd\0");
									wdt_reset();
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, BueroTagblock, BUERO, 0, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(BueroTagblock, Zeit.stunde);
										wdt_reset();
										
										//	BueroTXdaten[0]: Bit 0: Uhr ein	Bit 1: Uhr aus
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												BueroTXdaten[0] = (Stundencode >=2); //Werte 2, 3: ON Wert 0: OFF
												
											
                                 }break;
												
											case 1: // zweite halbe Stunde
											{
												BueroTXdaten[0] = ((Stundencode ==1)||(Stundencode==3)); //Werte 1, 3: ON Wert 0: OFF
											}break;
										}//switch
										if (BueroTXdaten[0])
                              {
                                 buerostatus |= (1<<0);
                              }
                              
										// BueroTXdaten Test
                              /*
										if (Zeit.minute&2) // Minuten ungerade
										{
											BueroTXdaten[0] |= (1<<0); // Bit 0 setzen
										}
										else
										{
											BueroTXdaten[0] &= ~(1<<0); // Bit 0 reseten
										}
										*/
										//
										
										//err_putint1(BueroTXdaten[0]);
										//err_putc(' ');
										
										
										//	Servo
										if ((Zeit.minute % 10) >5)
										{
											BueroTXdaten[3]=Zeit.minute % 5;
										}
										else
										{
											BueroTXdaten[3]=5-Zeit.minute%5;
										}
										
										
										
									}//erfolg
									else
									{
										
										//err_gotoxy(9,1);
										//err_puts("X\0");
										//err_clr_part(1,9,19);
										//err_puts("wl B er\0");
										//err_puthex(erfolg);
										//delay_ms(800);
										EEPROM_Err |= (1<<BUERO);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									
									uint8_t pos=0x00;
									pos=(PINB & 0x03)>>4; // Bit 0, 1
									//pos >>4;
									//pos |= (1<<LOOPCOUNTPIN);
									//pos |= (1<<TWICOUNTPIN);
									
									pos=Zeit.minute%5;
									//					err_gotoxy(12,1);
									//					err_puthex(pos);
									BueroTXdaten[3]=pos;
									//delay_ms(40);
									/*
									 switch (pos)
									 {
									 case 0:
									 BueroTXdaten[3]=20;
									 break;
									 
									 case 1:
									 BueroTXdaten[3]=30;
									 break;
									 
									 case 2:
									 BueroTXdaten[3]=40;
									 break;
									 
									 case 3:
									 BueroTXdaten[3]=50;
									 break;
									 
									 default:
									 BueroTXdaten[3]=60;
									 }//switch pos
									 */
									
									//					err_putc(' ');
									//					err_putint2(BueroTXdaten[3]);
									
									uint8_t bueroerfolg=0;
									
									//err_clr_part(0,0,10);
									//err_puts("Br wr\0");
									wdt_reset();
									twi_Call_count0++;
									bueroerfolg=SlavedatenSchreiben(BUERO_ADRESSE, BueroTXdaten);
									wdt_reset();
									if (bueroerfolg)
									{
										
										//err_gotoxy(10,1);
										//err_puts("Y\0");
										//err_clr_part(1,9,19);
										//err_puts("w Br err\0");
										//err_puthex(bueroerfolg);
										//delay_ms(80);
										Write_Err |= (1<<BUERO);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									//					err_gotoxy(12,1);
									//					err_puts("WB\0");
									//err_gotoxy(12,0);
									//					err_puthex(bueroerfolg);
									
									SchreibStatus &= ~(1<< BUERO);
									delay_ms(100);
									PORTC &= ~(1<<TWICOUNTPIN);
                           outbuffer[22] = buerostatus;
								}
                        
                        
								//lesen von Buero
                        
                        
								if (LeseStatus & (1<< BUERO))
								{
									delay_ms(2);
									err_gotoxy(8,1);
									err_puts("B\0");
									err_puts("  \0");
									PORTC |= (1<<TWICOUNTPIN);
									
									//err_clr_line(1);
									//lcd_gotoxy(12,3);
									//err_puts("RB:\0");
									//delay_ms(50);
									//uint8_t BuerolesenDaten[buffer_size];
									uint8_t Buerolesenerfolg=0;
									
									//err_clr_part(0,0,10);
									//err_puts("Br rd\0");
									wdt_reset();
									twi_Call_count0++;
									Buerolesenerfolg=SlavedatenLesen(BUERO_ADRESSE,BueroRXdaten);
									wdt_reset();
									
									if (Buerolesenerfolg)
									{
										//err_gotoxy(10,1);
										//err_puts("Z\0");
										
										//err_clr_part(1,9,19);
										//err_puts("r Br err\0");
										//err_puthex(Buerolesenerfolg);
										//delay_ms(800);
										Read_Err |= (1<<BUERO);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									
									/*
									 // for (i=0;i<8;i++)
									 {
									 i=1;
									 err_gotoxy(13,0);
									 err_putint1(i);
									 err_putc(' ');
									 err_puthex(Buerodaten[i]);
									 delay_ms(600);
									 
									 }
									 */
									
									//err_clr_line(0);
									//err_puts("R B:\0");
									//err_puthex(BueroRXdaten[0]);
									//				err_puthex(BueroRXdaten[1]);
									//				err_puthex(BueroRXdaten[2]);
									//				err_puthex(BueroRXdaten[3]);
									//delay_ms(400);
									//err_puthex(LeseStatus);
                           
                           outbuffer[23] = BueroRXdaten[1]; // Temp
                           
                           
                           lcd_gotoxy(10,0);
                           lcd_puthex(BueroRXdaten[7]);
									LeseStatus &= ~(1<< BUERO);
									delay_ms(100);
									PORTC &= ~(1<<TWICOUNTPIN);
									
								}
								
#pragma mark Wozi					
								//	*****************
								//	*	WOZI
								//	*****************
								if (SchreibStatus & (1<< WOZI))	//schreiben an Wozi
								{
									delay_ms(2);
									//	err_gotoxy(12,1);
									//						err_puts("W\0");
									
									//delay_ms(2);
                           uint8_t wozistatus=0;
									uint8_t WoziTagblock[buffer_size];
									uint8_t Stundencode=0;
									WoZiTXdaten[0]=0;
									
									// WoZi-Tagplan lesen
									wdt_reset();
									
									// Lampe lesen
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, WoziTagblock, WOZI, 0, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(WoziTagblock, Zeit.stunde);
										//outbuffer[29] |= Stundencode; // 9.4.11
										
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												
												WoZiTXdaten[0]=(Stundencode >=2); //Werte 2, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
												
											}break;
												
											case 1: // zweite halbe Stunde
											{
												WoZiTXdaten[0]=((Stundencode ==1)||(Stundencode==3)); //Werte 1, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
											}break;
										}//switch
										if (WoZiTXdaten[0])
                              {
                                 wozistatus |= (1<<0);
                              }
										//outbuffer[29] |=WoZiTXdaten[0];
										// Test
										//				WoZiTXdaten[0]=Zeit.minute%2;
										//
										
										//							err_gotoxy(13,1);
										//							err_putint1(LaborTXdaten[0]);
										
										
									}//erfolg
									else
									{
										EEPROM_Err |= (1<<WOZI);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									
									wdt_reset();
									twi_Call_count0++;
									
									//begin Schreiben Objekt 1
									// Code fuer Objekt 1 lesen: Radiator
									WoZiTXdaten[1] = 0;
									uint8_t tagblock1[buffer_size];
									uint8_t obj1erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock1, WOZI, 1, Zeit.wochentag);
									if (obj1erfolg==0) // EEPROM erfolgreich gelesen
									{
										int RadiatorStundencode=Tagplanwert(tagblock1, Zeit.stunde);
										
                              RadiatorStundencode &= 0x03;	// Bit 0 und 1 filtern fuer WoZiTXdaten[1]
										WoZiTXdaten[1] = RadiatorStundencode;
										
									}
									else
									{
										EEPROM_Err |= (1<<WOZI);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
                           if (WoZiTXdaten[1])
                           {
                              wozistatus |= (1<<1);
                           }

									//end Schreiben Objekt 1
									
									uint8_t wozierfolg=0;
                           
									wozierfolg=SlavedatenSchreiben(WOZI_ADRESSE,  WoZiTXdaten);
									
									wdt_reset();
									//WoZiTXdaten[4]=45;
									if (wozierfolg)
									{
										Write_Err |= (1<<WOZI);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									/*
                            
									 */
									SchreibStatus &= ~(1<< WOZI);
                           outbuffer[20] = wozistatus;
								}
                        
                        // Lesen von Wozi
								
								if (LeseStatus & (1<< WOZI))	//lesen von Wozi
								{
									delay_ms(2);
									wdt_reset();
									twi_Call_count0++;
									uint8_t wozierfolg=SlavedatenLesen(WOZI_ADRESSE, (void*)WoZiRXdaten);
									wdt_reset();
									if (wozierfolg)
									{
										Read_Err |= (1<<WOZI);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
										uint8_t pos=0x00;
										pos=(PINC & 0x30)>>4; // Bit 4, 5
										switch (pos)
										{
											case 0:
												WoZiTXdaten[4]=0;
												break;
												
											case 1:
												WoZiTXdaten[4]=35;
												break;
												
											case 2:
												WoZiTXdaten[4]=50;
												break;
												
											case 3:
												WoZiTXdaten[4]=65;
												break;
												
										}//switch pos
										//		WebTxDaten[7]= WoZiRXdaten[1];	// Innentemperatur
										outbuffer[21]= WoZiRXdaten[1];
                              outbuffer[7]= WoZiRXdaten[1];// Innentemperatur
									}
									
									LeseStatus &= ~(1<< WOZI);
								}
								
#pragma mark Labor					
								//	*****************
								//	*	Labor
								//	*****************
								if (SchreibStatus & (1<< LABOR))	//schreiben an Labor
								{
									//						err_gotoxy(12,1);
									//						err_puts("L\0");
									delay_ms(2);
									//delay_ms(2);
                           uint8_t laborstatus=0;
									uint8_t LaborTagblock[buffer_size];
									uint8_t Stundencode=0;
									LaborTXdaten[0]=0;
									
									// Labor-Tagplan lesen
									//err_clr_part(0,0,10);
									//err_puts("WT L rd\0");
									wdt_reset();
                           
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, LaborTagblock, LABOR, 0, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(LaborTagblock, Zeit.stunde);
                              
                              
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												
												LaborTXdaten[0]=(Stundencode >=2); //Werte 2, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
												
											}break;
												
											case 1: // zweite halbe Stunde
											{
												LaborTXdaten[0]=((Stundencode ==1)||(Stundencode==3)); //Werte 1, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
											}break;
										}//switch
										
										// Test
										//				LaborTXdaten[0]=Zeit.minute%2;
										//
										
										//							err_gotoxy(13,1);
										//							err_putint1(LaborTXdaten[0]);
										
										
									}//erfolg
									else
									{
										EEPROM_Err |= (1<<LABOR);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
                           
                           if (LaborTXdaten[0])
                           {
                              laborstatus |= (1<<0);
                           }

									// Radiator lesen
                           LaborTXdaten[1]=0;
									wdt_reset();
									erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, LaborTagblock, LABOR, 1, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										int RadiatorStundencode=Tagplanwert(LaborTagblock, Zeit.stunde);
                              RadiatorStundencode &= 0x03;	// Bit 0 und 1 filtern fuer TXdaten[1]
                              LaborTXdaten[1] = RadiatorStundencode;
										
									}//erfolg
									else
									{
										EEPROM_Err |= (1<<LABOR);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
                           if (LaborTXdaten[1])
                           {
                              laborstatus |= (1<<1);
                           }
								
									// end Radiator lesen
									
									uint8_t laborerfolg=0;
									
									//err_clr_part(0,0,10);
									//err_puts("LB wr\0");
									wdt_reset();
									twi_Call_count0++;
									laborerfolg=SlavedatenSchreiben(LABOR_ADRESSE,  (void*)LaborTXdaten);
									wdt_reset();
									LaborTXdaten[4]=45;
									if (laborerfolg)
									{
										//err_gotoxy(13,1);
										//err_puts("Y\0");
										//err_clr_part(1,9,19);
										//err_puts("w Lb err\0");
										//err_puthex(laborerfolg);
										//delay_ms(800);
										Write_Err |= (1<<LABOR);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									
									SchreibStatus &= ~(1<< LABOR);
								}
								
								if (LeseStatus & (1<< LABOR))	//lesen von Labor
								{
									delay_ms(2);
									//						err_gotoxy(12,1);
									//						err_puts("L\0");
									//uint8_t laborerfolg=SlaveLesen(LABOR_ADRESSE);
									//err_clr_part(0,0,10);
									//err_puts("LB rd\0");
									wdt_reset();
									twi_Call_count0++;
									uint8_t laborerfolg=SlavedatenLesen(LABOR_ADRESSE, LaborRXdaten);
									wdt_reset();
									if (laborerfolg)
									{
										//err_gotoxy(14,1);
										//err_puts("Z\0");
										//err_clr_part(1,9,19);
										//err_puts("r Lb err\0");
										//err_puthex(laborerfolg);
										//delay_ms(800);
										Read_Err |= (1<<LABOR);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
										/*
										 for (i=0;i<8;i++)
										 {
										 LaborRXdaten[i]=rxbuffer[i];
										 rxbuffer[i]=0;
										 }
										 */
										LaborTXdaten[4]=45;
										uint8_t pos=0x00;
										pos=(PINC & 0x30)>>4; // Bit 4, 5
										//pos >>4;
										//err_gotoxy(7,1);
										//err_puthex(pos);
										//delay_ms(40);
										switch (pos)
										{
											case 0:
												LaborTXdaten[4]=0;
												break;
												
											case 1:
												LaborTXdaten[4]=35;
												break;
												
											case 2:
												LaborTXdaten[4]=50;
												break;
												
											case 3:
												LaborTXdaten[4]=65;
												break;
												
										}//switch pos
									}
									
									LeseStatus &= ~(1<< LABOR);
								} //	Labor
								
#pragma mark OG1					
								//	*****************
								//	*	OG1
								//	*****************
								if (SchreibStatus & (1<< OG1))	//schreiben an OG1
								{
									delay_ms(2);
									//	err_gotoxy(12,1);
									//						err_puts("O\0");
									
									//delay_ms(2);
									uint8_t OG1Tagblock[buffer_size];
									uint8_t Stundencode=0;
									OG1TXdaten[0]=0;
									//err_clr_line(1);
									//err_gotoxy(0,1);
									//err_puts("WD:\0");
									//err_putint1(Zeit.wochentag);
									
									// WoZi-Tagplan lesen
									//err_clr_part(0,0,10);
									//err_puts("WT L rd\0");
									wdt_reset();
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, OG1Tagblock, OG1, 0, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(OG1Tagblock, Zeit.stunde);
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												
												OG1TXdaten[0]=(Stundencode >=2); //Werte 2, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
												
											}break;
												
											case 1: // zweite halbe Stunde
											{
												OG1TXdaten[0]=((Stundencode ==1)||(Stundencode==3)); //Werte 1, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
											}break;
										}//switch
										
										// Test
										//				OG1TXdaten[0]=Zeit.minute%2;
										//
										
										//							err_gotoxy(13,1);
										//							err_putint1(OG1TXdaten[0]);
										
										
									}//erfolg
									else
									{
										//err_clr_part(1,9,19);
										//err_gotoxy(13,1);
										//err_puts("X\0");
										//err_puts("wl L er\0");
										//err_puthex(erfolg);
										//delay_ms(800);
										EEPROM_Err |= (1<<OG1);
										spistatus |= (1<<TWI_ERR_BIT);
									}
									
									uint8_t og1erfolg=0;
									
									//err_clr_part(0,0,10);
									//err_puts("LB wr\0");
									wdt_reset();
									twi_Call_count0++;
									
									og1erfolg=SlavedatenSchreiben(OG1_ADRESSE,  OG1TXdaten);
									
									wdt_reset();
									//OG1TXdaten[4]=45;
									if (og1erfolg)
									{
										//err_gotoxy(13,1);
										//err_puts("Y\0");
										//err_clr_part(1,9,19);
										//err_puts("w Lb err\0");
										//err_puthex(laborerfolg);
										//delay_ms(800);
										Write_Err |= (1<<OG1);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									/*
									 */
									SchreibStatus &= ~(1<< OG1);
								}
								
								if (LeseStatus & (1<< OG1))	//lesen von OG1
								{
									delay_ms(2);
									//						err_gotoxy(12,1);
									//						err_puts("L\0");
									//uint8_t laborerfolg=SlaveLesen(LABOR_ADRESSE);
									//err_clr_part(0,0,10);
									//err_puts("LB rd\0");
									wdt_reset();
									twi_Call_count0++;
									uint8_t og1erfolg=SlavedatenLesen(OG1_ADRESSE, OG1RXdaten);
									wdt_reset();
									if (og1erfolg)
									{
										//err_gotoxy(14,1);
										//err_puts("Z\0");
										//err_clr_part(1,9,19);
										//err_puts("r Lb err\0");
										//err_puthex(laborerfolg);
										//delay_ms(800);
										Read_Err |= (1<<OG1);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
										uint8_t pos=0x00;
										pos=(PINC & 0x30)>>4; // Bit 4, 5
										//pos >>4;
										//pos |= (1<<LOOPCOUNTPIN);
										//err_gotoxy(7,1);
										//err_puthex(pos);
										//delay_ms(40);
										
										//err_gotoxy(0,1);
										//err_putc('T');
										//err_puthex(OG1RXdaten[0]);
										//err_puthex(OG1RXdaten[1]);
										//err_puthex(WoZiRXdaten[2]);
										//err_puthex(WoZiRXdaten[3]);
										//err_puthex(WoZiRXdaten[4]);
										//err_puthex(WoZiRXdaten[5]);
										//err_puthex(WoZiRXdaten[6]);
										//err_puthex(WoZiRXdaten[7]);
										//WebTxDaten[7]= WoZiRXdaten[1];	// Innentemperatur
										
									}
									
									LeseStatus &= ~(1<< OG1);
								}
								
#pragma mark OG2					
								//	*****************
								//	*	OG2
								//	*****************
								if (SchreibStatus & (1<< OG2))	//schreiben an OG2
								{
									delay_ms(2);
									uint8_t OG2Tagblock[buffer_size];
									uint8_t Stundencode=0;
									OG2TXdaten[0]=0;
									wdt_reset();
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, OG2Tagblock, OG2, 0, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(OG2Tagblock, Zeit.stunde);
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												
												OG2TXdaten[0]=(Stundencode >=2); //Werte 2, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
												
											}break;
												
											case 1: // zweite halbe Stunde
											{
												OG2TXdaten[0]=((Stundencode ==1)||(Stundencode==3)); //Werte 1, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
											}break;
										}//switch
										
										// Test
										//				OG2TXdaten[0]=Zeit.minute%2;
										//
										
										//							err_gotoxy(13,1);
										//							err_putint1(OG2TXdaten[0]);
										
										
									}//erfolg
									else
									{
										EEPROM_Err |= (1<<OG2);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									
									uint8_t og2erfolg=0;
									wdt_reset();
									//					twi_Call_count0++;
									
									og2erfolg=SlavedatenSchreiben(OG2_ADRESSE,  OG2TXdaten);
									
									wdt_reset();
									if (og2erfolg)
									{
										Write_Err |= (1<<OG2);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										//						twi_Reply_count0++;
									}
									/*
									 */
                           /*
                            PWM lesen
                            */
 									uint8_t tagblock3[buffer_size];
									uint8_t obj3erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock3, HEIZUNG, 3, 0);
									//OSZIAHI;
									delay_ms(1);
                           
                           if (obj3erfolg==0) // EEPROM erfolgreich gelesen
									{
										//err_gotoxy(0,1);
										//err_puts("         \0");
										//err_gotoxy(0,1);
										//err_puts("PWM:\0");
                              uint8_t pwmcode = tagblock3[0];
										//err_puthex(pwmcode);
                              pwmcode = tagblock3[1];
                              //err_puthex(pwmcode);
                              pwmcode = tagblock3[2];
                              //err_puthex(pwmcode);
                              
                           }
                           else
                           {
                              
                           }

                           
                           
									SchreibStatus &= ~(1<< OG2);
								}	
								
								if (LeseStatus & (1<< OG2))	//lesen von OG2
								{
									delay_ms(2);
									LeseStatus &= ~(1<< OG2);
								}
								
#pragma mark Estrich					
								//	*****************
								//	*	Estrich
								//	*****************
								if (SchreibStatus & (1<< ESTRICH))	//schreiben an Estrich
								{
									delay_ms(2);
									uint8_t EstrichTagblock[buffer_size];
									uint8_t Stundencode=0;
									EstrichTXdaten[0]=0;
									wdt_reset();
									uint8_t erfolg=WochentagLesen(EEPROM_WOCHENPLAN_ADRESSE, EstrichTagblock, ESTRICH, 0, Zeit.wochentag);
									wdt_reset();
									if (erfolg==0)
									{
										Stundencode=Tagplanwert(EstrichTagblock, Zeit.stunde);
										switch (Zeit.minute/30)
										{
											case 0: // erste halbe Stunde
											{
												
												EstrichTXdaten[0]=(Stundencode >=2); //Werte 2, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
												
											}break;
												
											case 1: // zweite halbe Stunde
											{
												EstrichTXdaten[0]=((Stundencode ==1)||(Stundencode==3)); //Werte 1, 3: Brenner auf FULL Wert 0: Brenner RED/OFF
											}break;
										}//switch
										
										
										
									}//erfolg
									else
									{
										EEPROM_Err |= (1<<ESTRICH);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									
									uint8_t estricherfolg=0;
									
									//err_clr_part(0,0,10);
									//err_puts("E wr\0");
									wdt_reset();
									//					twi_Call_count0++;
									twi_Call_count0++;
									estricherfolg=SlavedatenSchreiben(ESTRICH_ADRESSE,  EstrichTXdaten);
									
									wdt_reset();
									if (estricherfolg)
									{
										Write_Err |= (1<<ESTRICH);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										twi_Reply_count0++;
									}
									
									SchreibStatus &= ~(1<< ESTRICH);
								}
                        
                        
								//lesen von Estrich
                        
                        
								if (LeseStatus & (1<< ESTRICH))
								{
									
									delay_ms(2);
									wdt_reset();
									twi_Call_count0++;
									uint8_t estricherfolg=SlavedatenLesen(ESTRICH_ADRESSE, (void*)EstrichRXdaten);
									wdt_reset();
									if (estricherfolg)
									{
										Read_Err |= (1<<ESTRICH);
                              spistatus |= (1<<TWI_ERR_BIT);
									}
									else
									{
										BUS_Status |=(1<<SPI_SENDBIT);
										twi_Reply_count0++;
										
										
										if (EstrichRXdaten[6] & (1<<7)) // Bit 7 von [6] 
										{
											outbuffer[31] |= (1<<WASSERALARMESTRICH); // Alarmbit setzen
										}
										else 
										{
											outbuffer[31] &= ~(1<<WASSERALARMESTRICH); // Alarmbit zuruecksetzen
										}
										
										//err_gotoxy(13,0);
										//err_putc('E');
										//err_putc('+');
										//delay_ms(400);
										
										//pos >>4;
										//err_gotoxy(7,1);
										//err_puthex(pos);
										//delay_ms(40);
										
										//lcd_gotoxy(6,1);
										//lcd_putc('E');
										//lcd_puthex(EstrichRXdaten[0]);
										//lcd_puthex(EstrichRXdaten[1]);
										//err_puthex(EstrichRXdaten[2]);
										//err_puthex(EstrichRXdaten[3]);
										//err_puthex(EstrichRXdaten[4]);
										//err_puthex(EstrichRXdaten[5]);
										//err_puthex(EstrichRXdaten[6]);
										//err_puthex(EstrichRXdaten[7]);
									}
									
									
									// outbuffer von Estrich schreiben
									
									for (i=0 ; i<8; i++) 
									{
										//		outbuffer[i]=EstrichRXdaten[i];			// Fuer Test: Daten ab Byte 0 von outbuffer
										outbuffer[estrich +i]=EstrichRXdaten[i]; // Daten ab Byte 'estrich' von outbuffer Byte 9
									}
                           lcd_gotoxy(0,3);
                           lcd_putc('E');
                           lcd_puthex(EstrichRXdaten[5]);
                           lcd_puthex(EstrichRXdaten[6]);
                           lcd_puthex(EstrichRXdaten[7]);
									
									LeseStatus &= ~(1<< ESTRICH); // erledigt
								}
								
							} // DCF77-erfolg==0
							
							//	Kontrolle: Labor schreiben: Schalter ein-aus nach jeder Minute
							//if (DCF77daten[0]!=LaborDaten[8])//letzter Minutenwert war anders
							{
								//					LaborDaten[0]=DCF77daten[0]% 2;		// ON bei geraden Minuten
								//					LaborDaten[1]=DCF77daten[0]% 2+1;	// OFF bei ungeraden Minuten
								//					LaborDaten[8]= DCF77daten[0];
							}
							
							if ((Menu_Ebene & 0xF0)>0)	//	Anzeigen wenn Menu_Ebene >0 
							{
								//err_gotoxy(10,0);
								//err_puts("Mn \0");
								//err_puthex(Menu_Ebene);
								//err_puts("   \0");
								wdt_reset();
								displayRaum(Raum_Thema, AnzeigeWochentag, Zeit.stunde, Menu_Ebene);
								wdt_reset();
								//err_gotoxy(15,0);
								//err_puts(" ok\0");
								//		lcd_gotoxy(10,3);
								//		lcd_puthex(Menu_Ebene);
								// Doppelpunkt am Schluss der Zeile blinken lassen
								lcd_gotoxy(19,1); 
								lcd_putc(':');
								
							}
							else
							{
								// Punkt am Schluss der Zeile blinken lassen
								//lcd_gotoxy(19,1); 
								//					lcd_putc(165);
								
							}
							/*
							 err_gotoxy(15,1);
							 err_puthex(SchreibStatus);
							 err_gotoxy(18,1);
							 err_puthex(LeseStatus);
							 delay_ms(1000);
							 */
							
							//err_clr_part(0,0,9);
							
							//				err_puts("TWI \0");
#pragma mark Fehlerbehandlung Start
							
							
							// TWI-Fehler angeben
							
														
							 err_gotoxy(0,1);
							 err_puts("          \0");
							 
							 err_gotoxy(0,1);
							 err_puts("E:\0");
							 err_putc('R');
							 err_puthex(Read_Err);
							 
							 err_putc('W');
							 err_puthex(Write_Err);
							 err_putc('E');
							 err_puthex(EEPROM_Err);
							 delay_ms(1000);
							if (twi_Call_count0==twi_Reply_count0) // alles OK
							{
								//err_puts("OK\0");
								twi_Call_count0=0;
								twi_Reply_count0=0;
								twi_Stat_count=0; //	Stat_count zurücksetzen
								
							}
							else
							{
								outbuffer[FEHLERBYTE]=Read_Err;		// Byte 24
								outbuffer[FEHLERBYTE+1]=Write_Err;
								outbuffer[FEHLERBYTE+2]=EEPROM_Err;
                        
                        for (int i=44;i<48;i++)
                        {
                           //outbuffer[i]= i;
                        }
                        
                        for (int i=0;i<8;i++)
                        {
                           //outbuffer[i+36]= EEPROMTXdaten[i];
                        }

								//	Warten auf nächsten Timerevent
								SchreibStatus=0;
								LeseStatus=0;	
								
								//err_puthex(twi_Call_count0);
								//err_putc(' ');
								//err_puthex(twi_Reply_count0);
								twi_Call_count0=0;
								twi_Reply_count0=0;
								
								
								
								i2c_stop();
								//TWI zuruecksetzen;
								
								TWCR =0;
								
								delay_ms(1000);
								
								//	TWI neu starten
								i2c_init();
								
								// delay einfuegen
								uint8_t reply_delay=0x0F;
								
								if (tempWDT_Count && (1<<6))// || (tempWDT_Count && (1<<6)))// zweiter wdt-Reset 
								{
									reply_delay = 2*reply_delay;	// laenger warten
								}
								
								twi_Stat_count++;
								err_gotoxy(18,1);
								//err_puts("rep err\0");
								err_puthex(twi_Stat_count);
								if (twi_Stat_count>4)		// debloc
								{
									//err_gotoxy(14,1);
									//err_puts("debl \0");
									i2c_debloc();
									
								}
								
								if (twi_Stat_count>reply_delay) // reset
								{
									//WebTxStartDaten= MASTERERRTASK;
									//WebTxDaten[0]=MASTERERRTASK;
									
									
									/*
									 tempWDT_Count=eeprom_read_byte(&WDT_ErrCount); //letzte gespeicherte wdt-Anzahl
									 
									 err_gotoxy(14,1);
									 err_puts("wrp \0");
									 err_puthex(tempWDT_Count);
									 delay_ms(1000);
									 tempWDT_Count++;
									 tempWDT_Count |= (1<<6); // Reply-Bit setzen
									 tempWDT_Count &= ~(1<<7); // wdt-Bit reseten
									 eeprom_write_byte(&WDT_ErrCount,tempWDT_Count);
									 cli();
									 wdt_enable (WDTO_2S);
									 while (1);
									 */
								}
							}
							
							// Daten schreiben
							
							
							// 10.12.09: Bedingung zur Vermeidung von Null-Datenserien
							
  //                   if (outbuffer[2]+outbuffer[3]+outbuffer[4])// Vorlauf, Ruecklauf und Aussen sind nie Null
							{
								spistatus |= (1<<SPI_SHIFT_IN_OK_BIT);
								BUS_Status |= (1<<SPI_SENDBIT);				
							}
							
							/*				
							 outbuffer[0]=' ';
							 outbuffer[1]='T';
							 outbuffer[2]='W';
							 outbuffer[3]='I';
							 outbuffer[4]=' ';
							 */
							
							lbyte=0;
							hbyte=0;
							in_startdaten=0;
							
							delay_ms(10); // Schwierigkeiten in Shift ohne Aufruf von clr_line
							
							Read_Device=0;
							Write_Device=0;
							LeseStatus=0;
							SchreibStatus=0;
							
							OSZIAHI;
							
						}//	if ((SchreibStatus || LeseStatus))
						
					}break; // default: DATATASK
						
				}  // switch in_startdaten
				
			} 
			
			// **************************************************
			
			// *** Fehlerbehandlung *********************************		
			
			if ((BUS_Status & (1<<SPI_SENDBIT)))
			{
				// Fehlerbehandlung
				if (spistatus & (1<<TWI_ERR_BIT))
				{
					// Fehlermeldung an Webserver schicken
					out_startdaten=ERRTASK;	// A0
					out_lbdaten=13;
					out_hbdaten=13;
               
               outbuffer[0]=Read_Err;
               outbuffer[1]=Write_Err;
               outbuffer[2]=EEPROM_Err;
					
					spistatus &= ~(1<<TWI_ERR_BIT); // Err-Bit zuruecksetzen, wird bei Fehlern im TWI gesetzt
				}
				else 
				{

				}
			}
			
         
			BUS_Status &= ~(1<<SPI_SENDBIT); // Send-Bit zuruecksetzen
			
			//TCNT0 =0x00;
			//SIGNAL_Count=0;
			
			
			// Shift fuer diese Runde erledigt
			spistatus &= ~(1<<SPI_SHIFT_IN_OK_BIT);
		} // if (spistatus & (1<<SPI_SHIFT_IN_OK_BIT))	
		
      
		
		
		// **************************************************
		
		wdt_disable();
		
		/* *** SPI end **************************************************************/
		
	#pragma mark Taste 0	
		
		wdt_reset();
	
      /*
		// TWI mit Taste toggeln
		if (!(PINB & (1<<PORTB0))) // Taste 0 TWI Ein/Aus
		{
			//err_gotoxy(12,0);
			//err_puts("P0 Down\0");
			//wdt_reset();
			if (! (TastenStatus & (1<<PORTB0))) //Taste 0 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PORTB0);
				Tastencount=0;
				//err_gotoxy(8,0);
				//err_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount ++;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				wdt_reset();
				if (Tastencount >= Tastenprellen)
				{
					err_gotoxy(8,0);
					err_puts("P0\0");
					//err_putint(Tastencount);
					err_putc(' ');
					err_puthex(BUS_Status);
					//err_putc('v');
					
					if (BUS_Status & (1<<TWI_CONTROLBIT)) //  TWI ist gesetzt > loeschen
					{
						//err_putc('1');
						//BUS_Status |= (1<<2); // WEB request
						
						//BUS_Status &= ~(1<<3); // TWI wird OFF, Bit 1 auf L setzen
						BUS_Status &= ~(1<<TWI_CONTROLBIT);
						LeseStatus=0;
						SchreibStatus=0;
						PORTC &= ~(1<<TWI_CONTROLPIN); // TWI-LED OFF
						
						//web_request &= ~(1<<7);
						//err_putc('n');
						//err_puthex(BUS_Status);
					}
					else
					{
						
						BUS_Status |= (1<<TWI_CONTROLBIT);
						PORTC |= (1<<TWI_CONTROLPIN); // TWI-LED ON
						
					}
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<PORTB0);
					
				}
			}//else
			
		}
		*/
		wdt_reset();
		
		if (!(PINB & (1<<PB1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P1 Down\0");
			
			if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht gedrueckt
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
					i2c_stop();
					i2c_debloc();
					LeseStatus=0;
					SchreibStatus=0;

					Tastencount=0;
					TastenStatus &= ~(1<<PB1);
					
					
				}
			}//	else
			
		} // Taste 1
		
		
		/* ************************* 
		 */
		
		// 3.12.		WEB_Status |= (1<<0);
		/* ************************* 
		 */
		/*
		 wdt_reset();
		 if (startdelay==STARTDELAY-1)
		 {
		 
		 lcd_gotoxy(18,3); 
		 lcd_putc('7');
		 
		 //delay_ms(300);
		 lcd_gotoxy(18,3); 
		 lcd_putc(' ');
		 
		 }
		 
		 if (startdelay==STARTDELAY)
		 {
		 lcd_gotoxy(17,3); 
		 lcd_putc('G');	//	Punkt 
		 err_gotoxy(17,1); 
		 err_putc('G');	//	Punkt 
		 
		 delay_ms(300);
		 }
		 */
		//err_gotoxy(16,1);
		//err_puts("I\0");
		//err_puthex(WEB_Status);
		//err_puts("T\0");
		//err_puthex(TWI_Status);
		
		uint8_t dataerfolg=0;
		
		wdt_reset();
		
		//BUS_Status |= (1<<TWI_CONTROLBIT);
		
		
		// Tastaturabfrage Start mit TWI, ohne WEB-Request, Startdelay abgezählt
			
		if ((!(BUS_Status & (1<<WEB_CONTROLBIT))) && (BUS_Status & (1<<TWI_CONTROLBIT))  && (startdelay==0))	
		{
			
			//err_gotoxy(4,1);
			//err_puts("TWI ");
			
			/************ SCL SDA checken *************/
			
			if (PINC & (1<<0) && PINC & (1<<1)) 	// SCL oder SDA sind beide high
			{
				if (twi_HI_count0< 0xFF)
				{
					twi_HI_count0++; // Zeit messen, waehrend der beide HI sind
				}
			}
			/*************************/
			neueZeit=0;
			//err_gotoxy(8,0);
			//err_putc('i');
			//err_puthex(BUS_Status);
			
			//			err_gotoxy(19,0);
			//			err_puts("T\0");
			
			
			wdt_reset();
			
			
			
			
#pragma mark Tasten 
			
			
			
		/*
			Tastenwert=0;
			initADC(tastatur_kanal);
			//err_gotoxy(10,1);
			//err_puts("TR \0");
			
			Tastenwert=(uint8_t)(readKanal(tastatur_kanal)>>2);
			*/
         Tastenwert=0;
			//err_puthex(Tastenwert);
			
			//			closeADC();
			
			//uint8_t Taste=-1;
			//		Tastenwert/=8;
			//		Tastenwert*=3;
			//		err_clr_line(1);
			//		err_gotoxy(0,1);
			//		err_puts("Taste \0");
			//		err_putint(Taste);
			//		delay_ms(200);
			
			
			
			if (Tastenwert>23) // ca Minimalwert der Matrix
			{
				//			wdt_reset();
				/*
				 0: Wochenplaninit
				 1: IOW 8* 2 Bytes auf Bus laden
				 2: Menu der aktuellen Ebene nach oben
				 3: IOW 2 Bytes vom Bus in Reg laden
				 4: Auf aktueller Ebene nach rechts (Heizung: Vortag lesen und anzeigen)									
				 5: Ebene tiefer
				 6: Auf aktueller Ebene nach links (Heizung: Folgetag lesen und anzeigen)									
				 7: 
				 8: Menu der aktuellen Ebene nach unten
				 9: DCF77 lesen
				 
				 12: Ebene höher
				 */
				TastaturCount++;
				if (TastaturCount>=8)	//	Prellen
				{
					
					//err_clr_line(1);
					//err_gotoxy(0,1);
					//err_puts("ADC:\0");
					//err_putint(Tastenwert);
					
					Taste=Tastenwahl(Tastenwert);
					
					//err_gotoxy(12,1);
					//err_puts("T:\0");
					//err_gotoxy(14,1);
					//err_putint2(Taste);
					/*
					 err_putint2(Taste);
					 //delay_ms(1200);
					 //err_clr_line(1);
					 */
					TastaturCount=0;
					Tastenwert=0x00;
					//				uint8_t i=0;
					
					//			uint8_t inByte0=0;
					//			uint8_t inByte1=0;
					
					uint8_t inBytes[4]={};
					//			uint8_t pos=0;
					//			lcd_gotoxy(12,0);
					//			lcd_put_wochentag(DCF77buffer[5]);
					//			lcd_gotoxy(15,0);
					//			lcd_put_zeit(DCF77buffer[0],DCF77buffer[1]);
					
					//			Taste=0xff;
					
					
					//Taste=4;
					
					switch (Taste)
					{
						case 0://WochenplanInit
						{ 
							break;
							// Blinken auf C2
							
						}
							break;
							
							
						case 1:
						{
							//
						}
							break;
							
						case 2:											//	Menu vorwaertsschalten	
						{
							lcd_cls();
							/*
							 err_gotoxy(0,1);
							 err_puts("M:\0");
							 err_putint2(Menu_Ebene & 0xF0);
							 err_gotoxy(10,1);
							 err_puts("T:\0");
							 err_putint2(Taste);
							 //					delay_ms(1000);
							 */
							switch (Menu_Ebene & 0xF0)					//	Bits 7-4, Menu_Ebene
							{
								case 0x00:								//	oberste Ebene, Raeume
								{
									//	(Raum_Thema & 0xF0): Bit 7-4, Pos in Raum-Liste
									if ((Raum_Thema & 0xF0)<0x70)	//	Ende der Raum-Liste noch nicht erreicht
										
									{
										Raum_Thema += 0x10;			//	Naechster Raum
									}
									else
									{
										lcd_gotoxy(14,0);
										lcd_puts(">>\0");				//Ende der Liste
										delay_ms(800);
										lcd_gotoxy(14,0);
										lcd_puts("  \0");
									}
									//err_cls();
									//err_gotoxy(10,1);
									//err_puts("Main:\0");
									//err_puthex(Raum_Thema);
									//							delay_ms(800);
									//lcd_cls();	
									/*
									 lcd_clr_line(0);
									 lcd_gotoxy(0,0);
									 //	RaumTable: Namen der Räume
									 strcpy_P(titelbuffer, (PGM_P)pgm_read_word(&(RaumTable[Raum_Thema>>4])));//Bit 7 - 4
									 //	Raum anzeigen:
									 lcd_puts(titelbuffer);
									 delay_ms(1800);
									 */
									displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);			//Anzeige aktualisieren
									
								}
									break;
									
								case 0x10:								// erste Unterebene, Thema
								{
									if ((Raum_Thema & 0x0F)<0x07)	//	(Raum_Thema & 0x0F): Bit 3-0, Pos in Themen-Liste
									{
										
										Raum_Thema += 0x01;			//naechstes Thema
									}
									else
									{
										lcd_gotoxy(13,0);
										lcd_puts(">>\0");				// Ende der Liste
										delay_ms(800);
										lcd_gotoxy(14,0);
										lcd_puts("  \0");
									}
									
									//lcd_gotoxy(9,0);
									//lcd_puthex(Raum_Thema);
									//lcd_gotoxy(12,0);
									//lcd_puthex(Menu_Ebene);
									
									displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde),Menu_Ebene);			//Anzeige aktualisieren
									
								}
									break;
								case 	0x20:								// zweite Unterebene
								{
									
								}break;
							}//switch Menu_Ebene
							
						}
							break;
							
						case 3:	//
						{
						
						}break;
							
						case 4:	// Vortag
						{
							
							//err_clr_line(0);
							//err_gotoxy(0,0);
							//err_puts("Taste 4\0");
							//delay_ms(50);
							//err_clr_line(0);
							
							
							switch (Menu_Ebene & 0xF0)//Bits 7-4	
							{
								case 0x00:	//oberste Ebene
								{
									//err_clr_line(0);
									//err_puts("E0\0");
									
								}break;
									
								case 0x10: // erste Ebene
								{
									
									
								}break;
									
								case 0x20: // zweite Ebene, Plan
								{
									err_clr_line(0);
									err_puts("T4 E2 \0");
									err_puthex(Raum_Thema);
									err_putc(' ');
									if ((Raum_Thema & 0x0F)==0x00)	//Plan
									{	
										//err_puts(" Plan\0");
										if (AnzeigeWochentag & 0x0F)	//	Nicht Montag
										{
											AnzeigeWochentag -= 0x01;	//VorTag
										}
										else
										{
											lcd_gotoxy(3,0);
											lcd_puts("Es ist schon MO!\0");
											delay_ms(400);
											
										}
										
										//err_clr_line(1);
										
										//err_puts("Tag:\0");
										//err_putint(AnzeigeWochentag & 0x0F);
										displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);	//	Anzeige aktualisieren
										
										//uint8_t tagblock[buffer_size];
										//uint8_t taglesenerfolg=TagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock, 0, (AnzeigeWochentag));
										//TagZeigen(tagblock,(Menu_Ebene & 0x0F));
										//delay_ms(800);
									}	// if Plan
									else
									{
										
									}
									err_putint1(AnzeigeWochentag);
									delay_ms(800);
								}break;	//	case 0x20
									
									
							}//switch Menu_Ebene & 0xF0
							
						} break; // case Vortag
							
							
						case 5:								// Ebene tiefer
						{
							//Taste=99;
							lcd_clr_line(1);
							//lcd_gotoxy(0,1);
							//lcd_puts("Taste 5\0");
							//delay_ms(200);
							//lcd_clr_line(1);
							//err_gotoxy(3,0);
							//err_puthex(DCF77daten[5]);
							Raum_Thema &= 0xF0;							//	Bits 7 - 4 behalten, Bits 3-0 loeschen
							Menu_Ebene &= 0xF0;							//	Bits 7 - 4 behalten, Bits 3-0 loeschen
							if ((Menu_Ebene & 0xF0)<0x20)
							{
								switch (Menu_Ebene & 0xF0)
								{
									case 0x00: // erste Ebene, Thema
									{
										Menu_Ebene = 0x10;
										AnzeigeWochentag=(Zeit.wochentag) & 0x0F; 
									}break;
										
									case 0x10:
									{
										Menu_Ebene = 0x20;
										lcd_CGRAMInit_A();							//	Zeichen fuer Wochenplan setzen
										//Objekt_Wochentag =0x00;						//	Objekt 0 seetzen
										//Objekt_Wochentag = ((DCF77daten[5]) & 0x0F);		//	Wochentag setzen								//Objekt_Wochentag = 0x06;
										//err_gotoxy(5,0);
										//delay_ms(200);
										Objekt_Wochentag = ((Zeit.wochentag) & 0x0F);		//	Wochentag setzen								//Objekt_Wochentag = 0x06;
										lcd_cls();
										
									}break;
										
										
										
										
								}//switch Menu_Ebene
								
								
								
								
								//Raum_Thema &= 0xF0;							//	Bits 7 - 4 behalten, Bits 3-0 loeschen
								//Menu_Ebene &= 0xF0;							//	Bits 7 - 4 behalten, Bits 3-0 loeschen
								
								/*
								 lcd_clr_line(0);
								 lcd_clr_line(1);
								 lcd_clr_line(2);
								 lcd_clr_line(3);
								 
								 lcd_gotoxy(0,0);
								 strcpy_P(titelbuffer, (PGM_P)pgm_read_word(&(RaumTable[(Raum_Thema>>4)])));
								 lcd_puts(titelbuffer);
								 
								 lcd_gotoxy(12,0);
								 lcd_put_wochentag(DCF77buffer[5]);
								 lcd_gotoxy(15,0);
								 lcd_put_zeit(DCF77buffer[0],DCF77buffer[1]);
								 
								 
								 //lcd_gotoxy(9,0);
								 //lcd_puthex(Raum_Thema);
								 
								 //lcd_gotoxy(12,0);
								 //lcd_puthex(Menu_Ebene);
								 
								 //uint16_t aadr=(uint16_t)MenuTable[Raum_Thema>>4];// Bit 7-4
								 //lcd_gotoxy(0,1);
								 //lcd_puthex(Raum_Thema>>4);
								 //lcd_gotoxy(3,1);
								 //lcd_putint(adr);
								 */
								
								displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);	//	Anzeige aktualisieren
								
								/*
								 uint8_t adr=8*(Raum_Thema>>4)+(Raum_Thema & 0x0F); // Page: Bits 7-4 Zeile: Bits 3-0
								 strcpy_P(menubuffer, (PGM_P)pgm_read_word(&(P_MenuTable[adr])));//Bit 3 - 0	Untermenu im PROGMEM
								 //lcd_clr_line(1);
								 lcd_gotoxy(0,1);
								 //lcd_puthex((uint8_t)&adr);
								 lcd_puts(menubuffer);
								 */	
								
							}
						}				break;
							
						case 6: // Folgetag
						{
							/*
							 err_clr_line(1);
							 err_gotoxy(0,1);
							 err_puts("Taste 6\0");
							 delay_ms(50);
							 err_clr_line(1);
							 */
							
							switch (Menu_Ebene & 0xF0)//Bits 7-4	
							{
								case 0x00:	//oberste Ebene
									
									err_clr_line(0);
									err_puts("E0\0");
									
									break;
									
								case 0x10: // erste Ebene
								{
									
									
								}break;
									
								case 0x20: // zweite Ebene, Plan
								{
									
									err_clr_line(0);
									err_puts("T6 E2 \0");
									err_puthex(Raum_Thema);
									err_putc(' ');
									if ((Raum_Thema & 0x0F)==0x00)	//Plan
									{
										//err_puts(" Plan\0");
										if ((AnzeigeWochentag & 0x0F)< 6)	//	Nicht Sonntag
										{
											AnzeigeWochentag += 0x01;	//naechster Tag
										}
										else
										{
											lcd_gotoxy(3,0);
											lcd_puts("Es ist schon SO!\0");
											delay_ms(400);
											
										}
										//err_putint1(AnzeigeWochentag);
										//delay_ms(800);
										
										err_clr_line(1);
										err_puts("Tag:\0");
										err_putint(AnzeigeWochentag & 0x0F);
										displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);	//	Anzeige aktualisieren
										
										//uint8_t tagblock[buffer_size];
										//uint8_t taglesenerfolg=TagLesen(EEPROM_WOCHENPLAN_ADRESSE, tagblock, 0, (Menu_Ebene & 0x0F));
										//TagZeigen(tagblock,(Menu_Ebene & 0x0F));
									}	// if Plan
									else
									{
										
									}
									
								}break;	// case 0x20
									
							} // Menu_Ebene & 0xF0
							
						} break; // case Folgetag
							
							//case 7:
							
							//	break;
							
							
						case 8:												//Menu rueckwaertsschalten
						{
							//err_gotoxy(0,1);
							//err_puts("M:\0");
							//err_putint2(Menu_Ebene & 0xF0);
							//err_gotoxy(10,1);
							//err_puts("T:\0");
							//err_putint2(Taste);
							//delay_ms(1000);
							
							switch (Menu_Ebene & 0xF0)//Bits 7-4				oberste Ebene, Raeume
							{
								case 0x00:
								{
									
									//					lcd_gotoxy(12,0);
									//					lcd_puthex(Raum_Thema);
									//delay_ms(800);
									//					pos=Raum_Thema;
									//					pos>>4;
									
									if ((Raum_Thema & 0xF0)>0)
									{
										
										Raum_Thema -= 0x10;				//vorheriges Thema
									}
									else
									{
										lcd_gotoxy(14,0);
										lcd_puts("<<\0");					//Ende der Liste
										delay_ms(800);
										lcd_gotoxy(14,0);
										lcd_puts("  \0");
										
									}
									//lcd_gotoxy(9,0);
									//lcd_puthex(Raum_Thema);
									
									//lcd_gotoxy(12,0);
									//lcd_puthex(Menu_Ebene);
									//							lcd_cls();	
									/*
									 lcd_gotoxy(0,0);
									 lcd_clr_line(0);
									 //	RaumTable: Namen der Räume
									 strcpy_P(titelbuffer, (PGM_P)pgm_read_word(&(RaumTable[Raum_Thema>>4])));//Bit 7 - 4
									 //	Raum anzeigen:
									 lcd_puts(titelbuffer);
									 */						
									displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);			//Anzeige aktualisieren
								}
									break;
									
								case 0x10:									// erste Unterebene, Thema
								{
									if ((Raum_Thema & 0x0F)>0x00)
									{								
										Raum_Thema -= 0x01;				//vorhergehendes Thema
									}
									else
									{
										lcd_gotoxy(14,0);
										lcd_puts(">>\0");					// Ende der Liste
										delay_ms(800);
										lcd_gotoxy(14,0);
										lcd_puts("  \0");
									}
									
									displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);				//Anzeige aktualisieren
									
								}
									break;
									
							}// switch Menu_Ebene
							
						}
							break;
							
							//case 9:
							
							
							//	break;
							
							
						case 12:// Ebene hoeher
						{
							//Taste=99;
							
							//lcd_clr_line(1);
							//lcd_gotoxy(0,1);
							//lcd_puts("Taste 12\0");
							//delay_ms(100);
							//lcd_clr_line(1);
							switch (Menu_Ebene & 0xF0)
							{
								case 0x00:
								{
									
								}break;
									
								case 0x10:
								{
									Menu_Ebene = 0x00;							//Ebene 0
									//Menu_Ebene += (DCF77daten[5] & 0x0F);		//	Wochentag setzen
									Raum_Thema &=0xF0;
									AnzeigeWochentag=(Zeit.wochentag) & 0x0F;
									lcd_CGRAMInit_Titel();	//	Zeichen fuer Titel setzen
									lcd_cls();
									displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde), Menu_Ebene);	//	Anzeige aktualisieren
									
									
								}break;
								case 0x20:
								{
									Menu_Ebene = 0x10;							//	Ebene 1
									Menu_Ebene &= 0xF0;							//	Bits 7 - 4 behalten, Bits 3-0 loeschen
									//Menu_Ebene += (DCF77daten[5] & 0x0F);		//	Wochentag setzen
									Raum_Thema &=0xF0;
									AnzeigeWochentag=(Zeit.wochentag) & 0x0F;
									lcd_CGRAMInit_Titel();	//	Zeichen fuer Titel setzen
									lcd_cls();
									displayRaum(Raum_Thema, AnzeigeWochentag, (Zeit.stunde>>4), Menu_Ebene);	//	Anzeige aktualisieren
									
									
								}break;
									
									
							}//switch MenuEbene
							
							
						}break;
							
							
					}//switch Taste
					
				}
				//TastaturCount=0x00;
			}
			
		}								// end kein WEB-Request
		
		else							// start WEB-Request:  Bit 0 ist LOW
		{
			//err_gotoxy(4,1);
			//err_puts("--- ");
			
			//err_gotoxy(18,0);
			//delay_ms(1);
			//err_puts("I\0");	
			//delay_ms(1);
			
		}								// end mit IOW-Request
		
		//		sei();
		//		uint8_t buffer=DCF77daten[5];
		
	}//while
	return 1;
	
	
}





