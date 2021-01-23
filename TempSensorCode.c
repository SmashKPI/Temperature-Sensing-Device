// &&& Temperature Sensor Circuit &&&&&&&&&&&&&&&&&&&&&&&
/********************************************************
File name:		TempSensor.c
Author:			DTsebrii
Date:			23/APR/2020
Modified:		None
Description:	The code for a temperature sensor circuit. Every second
				new sample is taken, after that the program waits for 
				collecting first 60 samples and then is calculating 
				the average sample. Every minute the collected data 
				sends by ESP8266.
********************************************************/
// *** Libraries ****************************************
#include <stdlib.h>
#include <stdio.h>
#include <p18f45k22.h>
#include <usart.h>

// *** Constants ***************************************
#define TRUE	1
#define FALSE	0
#define TMR0FLAG INTCONbits.TMR0IF
#define HIGHBIT	0x0B
#define LOWBIT	0xDC
#define RC1BYTE RCREG1
#define RC1FLAG	PIR1bits.RC1IF
#define BUFFSIZE 30
#define MINUTE 60
// String parameters
#define ADDYTO 1
#define MYADDY 1

// ESP response
#define RESPONCES 5
#define OK	0
#define ERROR	1
#define FAIL	2
#define WIFICONNECTED 3
#define WIFIGOTIP	4
// ADC constants
#define ADCMASK	0x83
#define TENBITS	1024
#define MAXVOLT	5
#define MINTEMP 100
#define MAXTEMP 100
#define TSLOPE	0.025
#define TOFFSET	2.5
#define ADCRES	0.0048   // 5/1024
// Object constant
#define TEMPCHAN	0x01
#define SAMPSIZE	20
// WDT Constants
#define WDTPSV		0x0C  // PSV 1 to 4096 

// Interrupt Routine
#define TWOBITSON	0xC0

void ISR (); //calling the prototype for an interrupt function
#pragma code int_vector = 0x008
//*** int_vector*********************************************
/*Author:	CTalbot
Date: 		14/MAR/2020
Description: Calling the ISR function
Input:		None
Output:		None
*************************************************************/
void int_vector (void)
{
	_asm
		GOTO ISR
	_endasm
}//eo int_vector::
#pragma code
// *** Global variables *********************************
// Sensor Object
typedef struct temperatureSensor
{
	int averageSamp;  // To store the current average value
	char counter;	// To move through the array
	int samples[ SAMPSIZE ];  // Array of samples
} tempSen_t;
tempSen_t tmprSys;
// AT Commands
rom char *AT = "AT\r\n";  // Check the status of ESP module. OK is a respond
rom char *RST = "AT+RST\r\n";  // Check the status of ESP module. OK is a respond
rom char *wifiMode = "AT+CWMODE=1\r\n";  // Setup the wifi mode to station mode 
rom char *connectWiFi = "AT+CWJAP=\"SSID\",\"PSSWD\"\r\n";  // Replace SSID & PSSWD by Wi-Fi name and password
rom char *connectTCP = "AT+CIPSTART=\"TCP\",\"HOST\",PORT\r\n";  // Replace HOST & PORT by host IPv4 & port number
rom char *closeTCP = "AT+CIPCLOSE\r\n";   // Close the server connection

char timeFlag = FALSE;  // To count every second
char averageFlag = FALSE;  // Flag to signalaze that sampling array is full
char sendFlag = FALSE;
char timer = 0;  // To calculate one minute to send the data
char rcBuffer[ BUFFSIZE ];
// *** Functions ****************************************
/*** interruptSetup *******************************************************************************
Author:		DTsebrii
Date:		14/MAR/2020
Modified:	23/APR/2020
Description:Function to set the INTCON register parameters
Input:		None
Output:		None
*********************************************************************************************/
void interruptSetup (void)
{
	//Config TMR0 interrupt
	INTCON |= 0x20;
	INTCON2 &= 0xFB;
	//Global Interrupts
	INTCON |= TWOBITSON;  // 0xC0 
}// eo configInterrupt::

/*** oscSetup *******************************************************************************
Author:		DTsebrii
Date:		12/MAR/2020
Description:Function to set the oscilograph parameters
Input:		None
Output:		None
*********************************************************************************************/
void oscSetup (void)
{
	OSCCON = 0x52;  // Fosc = 4MHz
	OSCCON2 = 0x04;
	OSCTUNE = 0x80;
	while( OSCCONbits.HFIOFS!=1 );
}// eo oscSetup::

/*** timerSetup *******************************************************************************
Author:		DTsebrii
Date:		12/MAR/2020
Description:Function to set the TMR0 parameters
Input:		char lowBit and highBit - TMRH and TMRL bits value
Output:		None
*********************************************************************************************/
void timerSetup ( char lowBit, char highBit )
{
	T0CON = 	0x93; //t0 enabled, 16 bit, clkout, 16 psv
	TMR0H = 	highBit;
	TMR0L = 	lowBit;
	TMR0FLAG = 	FALSE; //Turn back to the zero after event happens
}// eo timerSetup::

/*** serialSetup *******************************************************************************
Author:		DTsebrii
Date:		12/MAR/2020
Description:Function to set the serial communication parameters
				to communicate with ESP8266
Input:		None
Output:		None
*********************************************************************************************/
void serialSetup()
{
	SPBRG= 		8;	// Sets it to 4MHZ and 115k Baudrate
	TXSTA1= 	0x26;  // BRGH = 1
	RCSTA1= 	0x90;
	BAUDCON1= 	0x48; // BRG16 = 1
}// eo serialSetup::

/*** adcSetup *******************************************************************************
Author:		DTsebrii
Date:		12/MAR/2020
Description:Function to set the ADC parameters
Input:		None
Output:		None
*********************************************************************************************/
void adcSetup (void)
{
	ADCON0|= 0x01;// ADC enable 
	ADCON1 = 0x00;
	ADCON2 = 0xA9;// 12 TAD right just F/8
}// eo adcSetup::

/*** portSetup *******************************************************************************
Author:			DTsebrii
Date:			12/MAR/2020
Modified:		DTsebrii, 30/MAR/2020
Description:	Function to set the ports parameters
Input:			None
Output:			None
*********************************************************************************************/
void portSetup()
{
	//PORTA
	ANSELA =	0x02;  // RA1 is temperature sensor
	LATA =		0x00;  // No input voltage
	TRISA = 	0xFF;  //All input
	//PORTB
	ANSELB = 	0x00;
	LATB = 		0x00;
	TRISB = 	0xFF;
	//PORTC
	ANSELC = 	0x00;
	LATC = 		0x00;
	TRISC = 	0xFF;
	//PORTD
	ANSELD = 	0x00;
	LATD = 		0x00;
	TRISD = 	0xFF;
	//PORTE
	ANSELE = 	0x00;  
	LATE = 		0x00;
	TRISE = 	0xFF;
}// eo portSetup::

/*** initObj *******************************************************************************
Author:			DTsebrii
Date:			23/APR/2020
Modified:		None
Description:	Function to initialize the data structure object
Input:			sptr pointer to data structre object
Output:			None
*********************************************************************************************/
void initObj( tempSen_t *sptr )
{
	/*
	object:
	samples;
	averagesamp;
	counter;
	*/
	char cnt;
	// Non iterational part
	sptr->averageSamp = FALSE;
	sptr->counter = FALSE;
	// Iterational part
	for( cnt = 0; cnt<SAMPSIZE; cnt++ )
		sptr->samples[ cnt ] = FALSE;
}  // eo initObj::


/*****************************************************************
Name:			resetWDT
Author:			DTsebrii
Date:			11/MAY/2020
Description: 	Turn OFF and ON the WDT 
Input:			None
Output:			None
*****************************************************************/
void resetWDT()
{
	WDTCON = FALSE;
	WDTCON = TRUE;
	
}  // eo resetWDT:: 

/*****************************************************************
Name:			systemSetup
Author:			DTsebrii
Date:			23/APR/2019
Description: 	Call all initialize/ config functions
Input:			None
Output:			None
*****************************************************************/
void systemSetup()
{
	// Hardwere part
	oscSetup();  // 4MHZ
	portSetup();
	serialSetup();
	adcSetup();  // 12TAD Fosc/8
	timerSetup( LOWBIT, HIGHBIT );  // 1 sec 16PSV
	interruptSetup();
	
	// Software part
	initObj( &tmprSys );
}  // systemSetup::

//---------espGetC-------------------------------------
/*----------------------------------------------
Author:		DTsebrii
Date:		03/02/2020
Modified:	DTsebrii, 27/MAR/2020
Description:Collecting a character sent back
Input:		None
Output:		char RCREG1 byte
--------------------------------------------------------------*/
char espGetC()
{
	// Prevent the overrun
	if(RCSTA1bits.OERR)
	{
		RCSTA1bits.CREN = FALSE;
		RCSTA1bits.CREN = TRUE;
	}
	resetWDT();  // To check the connection stability
	while( !RC1FLAG );
	WDTCON = FALSE;  // Stop watchdog timer
	return RC1BYTE;
} // eo espGetC::



//---------isWiFiReady-------------------------------------
/*----------------------------------------------
Author:		DTsebrii
Date:		03/02/2020
Modified:	DTsebrii, 10/MAR/2020
Description:Waiting till ESP8266 return "OK"
Input:		None
Output:		TRUE
--------------------------------------------------------------*/
char isWiFiReady()
{
	while( espGetC() != 'O');
	while( espGetC() != 'K' );
	return TRUE;
} //eo isWiFiReady::

/*** espSetConnection ****************************************************************************
Author:			DTsebrii
Date:			31/MAY/2020
Description:	Prompt the commands required to set the WiFi connection
Input:			None
Output:			None
*************************************************************************************************/
void espSetConnection()
{
	printf((const rom far char *)"%S", AT);
	if( isWiFiReady() )
	{
		printf((const rom far char *)"%S", wifiMode);//set  station wifi mode
		isWiFiReady();
		printf((const rom far char *)"%S",connectWiFi);//connect to wifi: AT+CWJAP="myWiFi","myPassword"
		isWiFiReady();
	}
}  // eo espSetConnection::

/*** sendToServer ****************************************************************************
Author:			DTsebrii
Date:			31/MAY/2020
Description:	Prompt the commands required to set the TCP connection and send the 
				message to the server
Input:			None
Output:			None
*************************************************************************************************/
void sendToServer()
{
	printf((const rom far char *)"%S",connectTCP);//connect to TCP server as a client: AT+CIPSTART="TCP","10.0.0.75",1337
	if( !isWiFiReady() )
		printf((const rom far char *)"%S", RST);
		
}  // eo sendToServer::

/*** createChecksum *******************************************************************************
Author:     	DTsebrii
Date:       	18/MAR/2020
Midified:   	DTsebrii,25/MAR/2020
Description:	Going through the input array and compare the each character with an unique checksum
					verification variable
Input:      	char ptr pointer t ocollected sentence
Output:     	char cs, checkSum
*********************************************************************************************/
char createChecksum( char *ptr )
{
    char cs = 0;
    while( *ptr != '\0' )
    {
        cs ^= *ptr;
		ptr++;
    }//eo while
    return cs;
}//eo createChecksum::

/*** createSentence *******************************************************************************
Author:		DTsebrii
Date:		09/APR/2020
Description:Create a new average sentence
Input:		None
Output:		None
*********************************************************************************************/
void createSentence()
{
	char cs = 0;
	sprintf( rcBuffer,"$TMPUPD,%i,%i,%i", ADDYTO, MYADDY, tmprSys.averageSamp );
	cs = createChecksum( rcBuffer );
	sprintf( rcBuffer, "%s,%i#\0", rcBuffer, cs );
} // eo createSentence()


/*****************************************************************
Name:			makeString
Author:			DTsebrii
Date:			24/APR/2019
Modified:		DTsebrii, 31/MAY/2020
Description: 	Combine the gathered value to a suitable string 
Input:			none
Output:			none
*****************************************************************/
void makeString(tempSen_t *sptr)  // SHOULD BE ADJUSTED IN THE FUTURE
{
	char lenght = 0;
	createSentence();
	lenght = strlen( rcBuffer );
	sendToServer();
	printf( (const rom far char *)"AT+CIPSEND=%d\r\n",lenght );
	isWiFiReady();
	printf( (const rom far char *)"%s", rcBuffer );
	//printf( (const rom far char *)"%s", buffer );
}  // eo makeString::
/*****************************************************************
Name:			sampADC
Author:			DTsebrii
Date:			23/APR/2019
Description: 	Read the ADC value
Input:			chID, char that represents ADC channel
Output:			ADRES - ADC result
*****************************************************************/
int sampADC( char chID )
{
	ADCON0&= ADCMASK;
	ADCON0|=  chID<<2;
	ADCON0bits.GO = TRUE;
	while( ADCON0bits.GO );
	return ADRES;
	
}  // eo sampADC::



/*****************************************************************
Name:			sampling
Author:			DTsebrii
Date:			23/APR/2019
Description: 	Convert the ADC into an actual temperature Value
Input:			none
Output:			none
*****************************************************************/
void sampling( tempSen_t *sptr )
{
	float result = 0;
	char cnt = 0;
	int sum = 0;
	result = sampADC(TEMPCHAN);
	result *= ADCRES;  // Convert to volts
	result -= TOFFSET;  
	result /= TSLOPE;
	sptr->samples[ sptr->counter ] = result;
	if( sptr->counter >= SAMPSIZE )
	{
		sptr->counter = FALSE;
		averageFlag = TRUE;
	}
	else
		sptr->counter++;
	if( averageFlag )  // There is not reseting of averageFlag
	{
		for( cnt = 0; cnt < SAMPSIZE; cnt++ )
			sum += sptr->samples[ cnt ];
		sptr->averageSamp = sum/SAMPSIZE;
	}	
}  // eo sampling::


/*****************************************************************
Name:			displayInfo
Author:			DTsebrii
Date:			11/MAR/2020
Description: 	Displaying the whole information  to debug
Input:			tempSen_t *sptr -> pointer to a temperature sensor
Output:			none
*****************************************************************/
void displayInfo( tempSen_t *sptr )
{
	printf("\033[0;0H\033[2JThe Temperature Sensor...\n");
	printf("\033[2;0HObject is working...\n");
	printf("\033[4;0HThe Current Sensed Value is: %i\n", sptr->samples[ sptr->counter-1 ]);
	createSentence();
	printf("\033[5;0HThe Command string is: %s", rcBuffer );

}  // eo displayInfo::

#pragma interrupt ISR
/*** ISR *******************************************************************************
Author:		CTalbot
Date:		14/MAR/2020
Modifier:	DTsebrii, 23/APR/2020
Description:Reseting TMR0
Input:		None
Output:		None
*********************************************************************************************/
void ISR()
{
	if( TMR0FLAG )
	{
		TMR0FLAG = FALSE;
		timer++;
		timeFlag = TRUE;
	}
	INTCON |= TWOBITSON;
}  // eo ISR::

// &&& MAIN &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void main()
{
	initializeSys();
	espSetConnection();
	while( TRUE )
	{
		if( timeFlag )
		{
			timeFlag = FALSE;
			sampling( &tmprSys );
			//displayInfo( &tmprSys );  // To watch output via Serial Communication
			
			if( timer>=MINUTE )
			{
				makeString( &tmprSys );
				timer = FALSE;
			}
			
		}
	} // eo while
}  // eo main
