/*
 * main.c - Example project for UT.6.02x Embedded Systems - Shape the World
 * Jonathan Valvano and Ramesh Yerraballi
 * October 27, 2015
 * Hardware requirements 
     TM4C123 LaunchPad, optional Nokia5110
     CC3100 wifi booster and 
     an internet access point with OPEN, WPA, or WEP security
 
 * derived from TI's getweather example
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/*
 * Application Name     -   Get weather
 * Application Overview -   This is a sample application demonstrating how to
                            connect to openweathermap.org server and request for
              weather details of a city.
 * Application Details  -   http://processors.wiki.ti.com/index.php/CC31xx_SLS_Get_Weather_Application
 *                          doc\examples\sls_get_weather.pdf
 */
 /* CC3100 booster pack connections (unused pins can be used by user application)
Pin  Signal        Direction      Pin   Signal     Direction
P1.1  3.3 VCC         IN          P2.1  Gnd   GND      IN
P1.2  PB5 UNUSED      NA          P2.2  PB2   IRQ      OUT
P1.3  PB0 UART1_TX    OUT         P2.3  PE0   SSI2_CS  IN
P1.4  PB1 UART1_RX    IN          P2.4  PF0   UNUSED   NA
P1.5  PE4 nHIB        IN          P2.5  Reset nRESET   IN
P1.6  PE5 UNUSED      NA          P2.6  PB7  SSI2_MOSI IN
P1.7  PB4 SSI2_CLK    IN          P2.7  PB6  SSI2_MISO OUT
P1.8  PA5 UNUSED      NA          P2.8  PA4   UNUSED   NA
P1.9  PA6 UNUSED      NA          P2.9  PA3   UNUSED   NA
P1.10 PA7 UNUSED      NA          P2.10 PA2   UNUSED   NA

Pin  Signal        Direction      Pin   Signal      Direction
P3.1  +5  +5 V       IN           P4.1  PF2 UNUSED      OUT
P3.2  Gnd GND        IN           P4.2  PF3 UNUSED      OUT
P3.3  PD0 UNUSED     NA           P4.3  PB3 UNUSED      NA
P3.4  PD1 UNUSED     NA           P4.4  PC4 UART1_CTS   IN
P3.5  PD2 UNUSED     NA           P4.5  PC5 UART1_RTS   OUT
P3.6  PD3 UNUSED     NA           P4.6  PC6 UNUSED      NA
P3.7  PE1 UNUSED     NA           P4.7  PC7 NWP_LOG_TX  OUT
P3.8  PE2 UNUSED     NA           P4.8  PD6 WLAN_LOG_TX OUT
P3.9  PE3 UNUSED     NA           P4.9  PD7 UNUSED      IN (see R74)
P3.10 PF1 UNUSED     NA           P4.10 PF4 UNUSED      OUT(see R75)

UART0 (PA1, PA0) sends data to the PC via the USB debug cable, 115200 baud rate
Port A, SSI0 (PA2, PA3, PA5, PA6, PA7) sends data to Nokia5110 LCD

*/
#include "..\cc3100\simplelink\include\simplelink.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/cmdline.h"
#include "application_commands.h"
#include "LED.h"
#include <string.h>
#include "ST7735.h"
#include "../inc/tm4c123gh6pm.h"
//#include "ff.h"
#include "bmps.h"

// To Do: replace the following tree lines with your access point information
#define SSID_NAME  "Project4WeatherQuest" /* Access point name to connect to */
#define SEC_TYPE   SL_SEC_TYPE_WPA
#define PASSKEY    "12345678"  /* Password in case of secure AP */ 
#define MAXLEN 100

//------------UART_Init------------
// Initialize the UART for 115,200 baud rate (assuming 50 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART_Init(void){
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0_CTL_R = 0;                      // disable UART
  UART0_IBRD_R = 27;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART0_FBRD_R = 8;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= UART_CTL_RXE|UART_CTL_TXE|UART_CTL_UARTEN;// enable Tx, RX and UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
                                        // configure PA1-0 as UART
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
}

#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F
uint8_t UART_InChar(void)
{
  while((UART0_FR_R&UART_FR_RXFE) != 0); // wait until the receiving FIFO is not empty
  return((uint8_t)(UART0_DR_R&0xFF));
}

void UART_OutChar(uint8_t data)
{
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}

void UART_InString(uint8_t *bufPt, uint16_t max) 
{
  int length=0;
  char character;
  character = UART_InChar();
  while(character != CR)
    {
  UART_OutChar(character);
    if(character != DEL)
    { // back space
			bufPt[length] = character;
			length++;
    }
		else{
			if(length){
				bufPt[length] = 0;
				length--;
			}
		}
    character = UART_InChar();
  }
    for(;length < max; length++){
        bufPt[length] = 0;
        
    }
}
void OutCRLF0(void)
{
  UART_OutChar(CR);
  UART_OutChar(LF);
}

#undef CR
#undef LF
#undef BS
#undef ESC
#undef SP
#undef DEL
//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt){
  while(*pt){
		while((UART0_FR_R&UART_FR_TXFF) != 0);
		UART0_DR_R = *pt;
    pt++;
  }
}



#define MAX_RECV_BUFF_SIZE  1024
#define MAX_SEND_BUFF_SIZE  512
#define MAX_HOSTNAME_SIZE   40
#define MAX_PASSKEY_SIZE    32
#define MAX_SSID_SIZE       32


#define SUCCESS             0

#define CONNECTION_STATUS_BIT   0
#define IP_AQUIRED_STATUS_BIT   1

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,/* Choosing this number to avoid overlap w/ host-driver's error codes */

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


/* Status bits - These are used to set/reset the corresponding bits in 'g_Status' */
typedef enum{
    STATUS_BIT_CONNECTION =  0, /* If this bit is:
                                 *      1 in 'g_Status', the device is connected to the AP
                                 *      0 in 'g_Status', the device is not connected to the AP
                                 */

    STATUS_BIT_IP_AQUIRED,       /* If this bit is:
                                 *      1 in 'g_Status', the device has acquired an IP
                                 *      0 in 'g_Status', the device has not acquired an IP
                                 */

}e_StatusBits;


#define SET_STATUS_BIT(status_variable, bit)    status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)    status_variable &= ~(1<<(bit))
#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & (1<<(bit))))
#define IS_CONNECTED(status_variable)           GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTION)
#define IS_IP_AQUIRED(status_variable)          GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_IP_AQUIRED)

typedef struct{
    UINT8 SSID[MAX_SSID_SIZE];
    INT32 encryption;
    UINT8 password[MAX_PASSKEY_SIZE];
}UserInfo;

/*
 * GLOBAL VARIABLES -- Start
 */

char Recvbuff[MAX_RECV_BUFF_SIZE];
char SendBuff[MAX_SEND_BUFF_SIZE];
char HostName[MAX_HOSTNAME_SIZE];
unsigned long DestinationIP;
int SockID;


typedef enum{
    CONNECTED = 0x01,
    IP_AQUIRED = 0x02,
    IP_LEASED = 0x04,
    PING_DONE = 0x08

}e_Status;
UINT32  g_Status = 0;
/*
 * GLOBAL VARIABLES -- End
 */


 /*
 * STATIC FUNCTION DEFINITIONS  -- Start
 */

static int32_t configureSimpleLinkToDefaultState(char *);


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */


void Crash(uint32_t time){
  while(1){
    for(int i=time;i;i--){};
    LED_RedToggle();
  }
}
/*
 * Application's entry point
 */
// 1) change Austin Texas to your city
// 2) metric(for celsius), imperial(for fahrenheit)
// api.openweathermap.org/data/2.5/weather?q={city name},{state code}&appid={API key}
//#define REQUEST "GET /data/2.5/weather?q=Murrieta&APPID=69fe3a9e2aadad473ececcece5ed3577&units=metric HTTP/1.1\r\nUser-Agent: Keil\r\nHost:api.openweathermap.org\r\nAccept: */*\r\n\r\n"
// 1) go to http://openweathermap.org/appid#use 
// 2) Register on the Sign up page
// 3) get an API key (APPID) replace the 7907b2abac2053aed180a74b9310b119 with your APPID
const char* req_pt2 = "&APPID=69fe3a9e2aadad473ececcece5ed3577&units=imperial HTTP/1.1\r\nUser-Agent: Keil\r\nHost:api.openweathermap.org\r\nAccept: */*\r\n\r\n";
char REQUEST_DATA[197] = "GET /data/2.5/weather?"; // req_pt1 + data_string (32) + req_pt2 + &lon= + 0
char data_string[32];
char menu_select[5];

void clear_data_string(){
	char i = 0;
	for(i = 0; i < 30; i++){
		data_string[i] = 0;
	}
}
void clear_menu_select(){
	char i = 0;
	for(i = 0; i < 5; i++){
		menu_select[i] = 0;
	}
}

void get_city_name(){
	UART_OutString("Please enter the city name:\n\r");
	char i = 0;
	data_string[i] = 'q';
	i++;
	data_string[i] = '=';
	i++;
	UART_InString((uint8_t*)&data_string[i], 30);
}

void get_city_id(){
	UART_OutString("Please enter the city id:\n\r");
	char i = 0;
	data_string[i] = 'i';
	i++;
	data_string[i] = 'd';
	i++;
	data_string[i] = '=';
	i++;
	UART_InString((uint8_t*)&data_string[i], 29);
}

void get_geographic_coordinates(){
	UART_OutString("enter the data in the following format:{lat} {lon}\n\r");
	char i = 0;
	data_string[i] = 'l';
	i++;
	data_string[i] = 'a';
	i++;
	data_string[i] = 't';
	i++;
	data_string[i] = '=';
	i++;
	UART_InString((uint8_t*)&data_string[i], 28);
}

void get_zip_code(){
	UART_OutString("enter the data in the following format:{zip code} {country code}\n\r");
	char i = 0;
	data_string[i] = 'z';
	i++;
	data_string[i] = 'i';
	i++;
	data_string[i] = 'p';
	i++;
	data_string[i] = '=';
	i++;
	UART_InString((uint8_t*)&data_string[i], 28);
}

void combine_strings(){
	uint8_t i = 22;
	uint8_t j = 0;
	for(j = 0; j < 32; j++){
		if(data_string[j] == 0){
			break;
		}
		else if(data_string[j] == ' '){
			if(data_string[0] == 'l'){ // coordinates case
				REQUEST_DATA[i] = '&';
				i++;
				REQUEST_DATA[i] = 'l';
				i++;
				REQUEST_DATA[i] = 'o';
				i++;
				REQUEST_DATA[i] = 'n';
				i++;
				REQUEST_DATA[i] = '=';
				i++;
			}
			else if(data_string[0] == 'z'){ // zip code case
				REQUEST_DATA[i] = ',';
				i++;
			}
			else if(data_string[0] == 'q'){ //city name case
				REQUEST_DATA[i] = '+';
				i++;
			}
		}
		else{
			REQUEST_DATA[i] = data_string[j];
			i++;
		}
	}
	for(j = 0; j < 137; j++){
		REQUEST_DATA[i] = req_pt2[j];
		i++;
	}
	for(; i < 197; i++){
		REQUEST_DATA[i] = 0;
	}
	return;
}

void get_city_data(){
	clear_data_string();
	clear_menu_select();
	UART_OutString("Please choose your query criteria:\n\r");
	UART_OutString("1. City Name\n\r");
	UART_OutString("2. City ID\n\r");
	UART_OutString("3. Geographic Coordinates\n\r");
	UART_OutString("4. Zip Code\n\r");
	UART_InString((uint8_t*)menu_select, 5);
	OutCRLF0();
	switch(menu_select[0]){
		case '1':
			get_city_name();
			break;
		case '2':
			get_city_id();
			break;
		case '3':
			get_geographic_coordinates();
			break;
		case '4':
			get_zip_code();
			break;
		default:
			get_city_data(); // potentially problematic :)))
		return;
		break;
	}
	combine_strings();
}

void delay(uint32_t t){
	int i = 0;
	while(t--){
		for(i = 0; i < 2000000; i++);
	}
}

uint8_t city_mode = 0;

void select_city_mode(){
	uint8_t i = 0;
	UART_OutString("1. Once city\n\r");
	UART_OutString("2. Three cities\n\r");
	UART_InString((uint8_t*)menu_select, 5);
	switch(menu_select[0]){
		case '1':
			city_mode = 1;
		break;
		case '2':
			city_mode = 3;
		break;
		default:
			select_city_mode();
		break;
	}
}


//static FATFS g_sFatFs;
//uint16_t img1_buffer[676];
//uint16_t img2_buffer[676];
uint8_t toggle = 0;
uint8_t cityNumber = 0;
uint16_t counter = 0;
uint8_t condition[3];
//void ST7735_DrawCharS(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size);
void ST7735_OutStringSized(uint16_t x, uint16_t y, char* word, uint8_t size, uint16_t bg, uint16_t color){
	uint32_t count = 0;
	while(*word){
		ST7735_DrawCharS(x*6*size, y*10*size, *word, color, bg, size);
		x ++;
		word++;
		if(x > (128/(6*size))){
			y ++;
			x = 1;
		}
		count++;
  }
}

	char Humidity[3][MAXLEN];
	char City[3][MAXLEN];
	char Temperature[3][MAXLEN];
	char Temp_min[3][MAXLEN];
	char Temp_max[3][MAXLEN];
	char Weather[3][MAXLEN];

void SystickHandler(void){
	static int s_counter = 1;
	if(s_counter == 0){
		s_counter++;
			if(counter < 9){
				counter++;
			}else{
				counter = 0;
				if(city_mode == 3){
					cityNumber = (cityNumber + 1)% 3;
					ST7735_FillScreen(ST7735_BLACK);
					ST7735_FillRect(0,53, 128, 107, ST7735_WHITE);
					ST7735_OutStringSized(0, 0, City[cityNumber], 2, ST7735_BLACK, ST7735_WHITE);
					ST7735_DrawString(0, 7, "Current Temp: ", ST7735_WHITE);
					ST7735_OutStringSized(0, 3, Temperature[cityNumber], 3,  ST7735_WHITE, ST7735_BLACK);
					ST7735_DrawString(0, 13, "Max Temp: ", ST7735_RED);
					ST7735_DrawString(10, 13, Temp_max[cityNumber], ST7735_RED);
					ST7735_DrawString(0, 14, "Min Temp: ", ST7735_CYAN);
					ST7735_DrawString(10, 14, Temp_min[cityNumber], ST7735_CYAN);
					ST7735_DrawString(0, 15, "Humidity: ", ST7735_WHITE);
					ST7735_DrawString(10, 15, Humidity[cityNumber], ST7735_WHITE);
				}
			}
			//	ST7735_DrawBitmap(ST7735_TFTWIDTH/2, 40, sun1, 40, 40);
			//	ST7735_DrawBitmap(ST7735_TFTWIDTH/2, 80, cloud1, 40, 40);
			//	ST7735_DrawBitmap(ST7735_TFTWIDTH/2, 120, rain1, 40, 40);
			if(toggle){
				toggle = 0;
				if(condition[cityNumber] == 1){
					ST7735_DrawBitmap(ST7735_TFTWIDTH/2 - 13, 52, sun1, 26, 26, ST7735_BLACK);
				}
				else if(condition[cityNumber] == 2){
					ST7735_DrawBitmap(ST7735_TFTWIDTH/2 - 13, 52, cloud1, 26, 26, ST7735_BLACK);
				}
				else if(condition[cityNumber] == 3){
					ST7735_DrawBitmap(ST7735_TFTWIDTH/2 - 13, 52, rain1, 26, 26, ST7735_BLACK);
				}
				//ST7735_DrawBitmap(ST7735_TFTWIDTH/2, 40, img2_buffer, 26, 26, ST7735_BLACK);
			}
			else{
				toggle = 1;
				if(condition[cityNumber] == 1){
					ST7735_DrawBitmap(ST7735_TFTWIDTH/2 - 13, 52, sun2, 26, 26, ST7735_BLACK);
				}
				else if(condition[cityNumber] == 2){
					ST7735_DrawBitmap(ST7735_TFTWIDTH/2 - 13, 52, cloud2, 26, 26, ST7735_BLACK);
				}
				else if(condition[cityNumber] == 3){
					ST7735_DrawBitmap(ST7735_TFTWIDTH/2 - 13, 52, rain2, 26, 26, ST7735_BLACK);
				}
			//	ST7735_DrawBitmap(ST7735_TFTWIDTH/2, 40, img1_buffer, 26, 26, ST7735_BLACK);
			}
		}
	else{
		s_counter++;
		if(s_counter == 2){
			s_counter = 0;
		}
	}
}

int main(void){
//	UINT successfulreads, successfulwrites;
//	FIL Handle,Handle2;
//	FRESULT MountFresult;
//	FRESULT Fresult;
	int32_t retVal;  
	uint8_t c;
	SlSecParams_t secParams;
  char *pConfig = NULL; 
	INT32 ASize = 0; 
	SlSockAddrIn_t  Addr;
	char *pt = NULL;
	uint8_t i;

  initClk();        // PLL 50 MHz
  UART_Init();      // Send data to PC, 115200 bps
  LED_Init();       // initialize LaunchPad I/O 
	ST7735_InitR(INITR_REDTAB);
  UART_OutString("Weather App\n\r");
	ST7735_OutString("Weather App\n\r");
	ST7735_DrawFastVLine(44, 80, 40, ST7735_YELLOW);
	ST7735_DrawFastHLine(44, 120, 40, ST7735_YELLOW);
	ST7735_DrawLine(44, 80, 84, 120, ST7735_YELLOW);
	ST7735_DrawCircle(64, 80, ST7735_YELLOW);
	
  retVal = configureSimpleLinkToDefaultState(pConfig); // set policies
  if(retVal < 0)Crash(4000000);
  retVal = sl_Start(0, pConfig, 0);
  if((retVal < 0) || (ROLE_STA != retVal) ) Crash(8000000);
  secParams.Key = PASSKEY;
  secParams.KeyLen = strlen(PASSKEY);
  secParams.Type = SEC_TYPE; // OPEN, WPA, or WEP
  sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
  while((0 == (g_Status&CONNECTED)) || (0 == (g_Status&IP_AQUIRED))){
    _SlNonOsMainLoopTask();
  }
  UART_OutString("Connected\n\r");
  ST7735_OutString("Connected\n\r");
	
  while(1){
		select_city_mode();
		for(counter = 0; counter < city_mode; counter++){
    strcpy(HostName,"api.openweathermap.org");
    retVal = sl_NetAppDnsGetHostByName(HostName,
             strlen(HostName),&DestinationIP, SL_AF_INET);
    if(retVal == 0){
      Addr.sin_family = SL_AF_INET;
      Addr.sin_port = sl_Htons(80);
      Addr.sin_addr.s_addr = sl_Htonl(DestinationIP);// IP to big endian 
      ASize = sizeof(SlSockAddrIn_t);
      SockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
      if( SockID >= 0 ){
        retVal = sl_Connect(SockID, ( SlSockAddr_t *)&Addr, ASize);
      }
			get_city_data();
      if((SockID >= 0)&&(retVal >= 0)){
        strcpy(SendBuff,REQUEST_DATA); 
        sl_Send(SockID, SendBuff, strlen(SendBuff), 0);// Send the HTTP GET 
        sl_Recv(SockID, Recvbuff, MAX_RECV_BUFF_SIZE, 0);// Receive response 
        sl_Close(SockID);
        LED_GreenOn();
        UART_OutString("\r\n\r\n");
        UART_OutString(Recvbuff);  UART_OutString("\r\n");
				
				// process received weather information
				/* find ticker name in response*/
				pt = strstr(Recvbuff, "\"name\"");
				i = 0; 
				if( NULL != pt ){
					pt = pt + 8; // skip over "name":"
					while((i<MAXLEN)&&(*pt)&&(*pt!='\"')){
						City[counter][i] = *pt; // copy into City string
						pt++; i++;    
					}
				}
				City[counter][i] = 0;

				/* find Temperature Value in response */
				pt = strstr(Recvbuff, "\"temp\"");
				i = 0; 
				if( NULL != pt ){
					pt = pt + 7; // skip over "temp":
					while((i<MAXLEN)&&(*pt)&&(*pt!=',')){
						Temperature[counter][i] = *pt; // copy into Temperature string
						pt++; 
						i++;    
					}
				}
				Temperature[counter][i] = 0;
				
				pt = strstr(Recvbuff, "\"temp_min\"");
				i = 0; 
				if( NULL != pt ){
					pt = pt + 11; // skip over "temp":
					while((i<MAXLEN)&&(*pt)&&(*pt!=',')){
						Temp_min[counter][i] = *pt; // copy into Temperature string
						pt++; 
						i++;    
					}
				}
				Temp_min[counter][i] = 0;
				
				pt = strstr(Recvbuff, "\"temp_max\"");
				i = 0; 
				if( NULL != pt ){
					pt = pt + 11; // skip over "temp":
					while((i<MAXLEN)&&(*pt)&&(*pt!=',')){
						Temp_max[counter][i] = *pt; // copy into Temperature string
						pt++; 
						i++;    
					}
				}
				Temp_max[counter][i] = 0;
				

				/* find weather in response */
				pt = strstr(Recvbuff, "\"description\"");
				i = 0; 
				if( NULL != pt ){
					pt = pt + 15; // skip over "description":"
					while((i<MAXLEN)&&(*pt)&&(*pt!='\"')){
						Weather[counter][i] = *pt; // copy into weather string
						pt++; 
						i++;    
					}
				}
				Weather[counter][i] = 0;   
				
				pt = strstr(Recvbuff, "\"humidity\"");
				i = 0; 
				if( NULL != pt ){
					pt = pt + 11; // skip over "description":"
					while((i<MAXLEN)&&(*pt)&&(*pt!=',')){
						Humidity[counter][i] = *pt; // copy into weather string
						pt++; 
						i++;    
					}
				}
				Humidity[counter][i] = 0;   
				
				OutCRLF0();
				OutCRLF0();
				UART_OutString(City[counter]);
				OutCRLF0();
				UART_OutString("min temp: ");
				UART_OutString(Temp_min[counter]);
				OutCRLF0();
				UART_OutString("max temp: ");
				UART_OutString(Temp_max[counter]);
				OutCRLF0();
				UART_OutString("cur temp: ");
				UART_OutString(Temperature[counter]);
				OutCRLF0();
				UART_OutString("Humid: ");
				UART_OutString(Humidity[counter]);
				OutCRLF0();
				UART_OutString("Weather: ");
				UART_OutString(Weather[counter]);
				OutCRLF0();
				
				// update ST7735 LCD with weather information
				ST7735_OutString(City[counter]); ST7735_OutString("\n\r");
				ST7735_OutString(Temperature[counter]); ST7735_OutString(" C\n\r");
				ST7735_OutString(Weather[counter]);
		
      }
    }
//		MountFresult = f_mount(&g_sFatFs, "", 0);
//		if(MountFresult){
//				ST7735_DrawString(0, 0, "f_mount error", ST7735_Color565(0, 0, 255));
//				while(1){};
//		}
		if(strstr(Recvbuff, "Not Found")){
			condition[counter] = 0;
		}
		else if(strstr(Weather[counter], "clear")){ //load sun1 and sun 2
//			Fresult = f_open(&Handle, "sun1.bin", FA_READ);
//			if(Fresult == FR_OK){
//				// get a character in 'c' and the number of successful reads in 'successfulreads'
//				for(counter = 0; counter < 1352; counter++){
//					Fresult = f_read(&Handle, &c, 1, &successfulreads);
//					if((Fresult == FR_OK) && (successfulreads == 1)){
//						if(counter%2){ // if odd hex value
//							img1_buffer[counter/2] |= c << 8;
//						}
//						else{
//							img1_buffer[counter/2] = c;
//						}
//					}
//					else{
//						break;
//					}
//				}
//			}
//			else{
//				UART_OutString("ERROR OPENING SUN2.BIN\n\r");
//			}
//			Fresult = f_close(&Handle);
//			if(Fresult != FR_OK){
//				UART_OutString("ERROR CLOSING SUN2.BIN\n\r");
//			}
//			Fresult = f_open(&Handle, "sun1.bin", FA_READ);
//			if(Fresult == FR_OK){
//				// get a character in 'c' and the number of successful reads in 'successfulreads'
//				for(counter = 0; counter < 1352; counter++){
//					Fresult = f_read(&Handle, &c, 1, &successfulreads);
//					if((Fresult == FR_OK) && (successfulreads == 1)){
//						if(counter%2){ // if odd hex value
//							img2_buffer[counter/2] |= c << 8;
//						}
//						else{
//							img2_buffer[counter/2] = c;
//						}
//					}
//					else{
//						break;
//					}
//				}
//			}
//			else{
//				UART_OutString("ERROR OPENING SUN1.BIN\n\r");
//			}
//			Fresult = f_close(&Handle);
//			if(Fresult != FR_OK){
//				UART_OutString("ERROR CLOSING SUN1.BIN\n\r");
//			}
condition[counter] = 1;
		}
		else if(strstr(Weather[counter], "cloud")) { //load cloud1 and cloud2
			condition[counter] = 2;
//			Fresult = f_open(&Handle, "cloud2.bin", FA_READ);
//			if(Fresult == FR_OK){
//				// get a character in 'c' and the number of successful reads in 'successfulreads'
//				for(counter = 0; counter < 1352; counter++){
//					Fresult = f_read(&Handle, &c, 1, &successfulreads);
//					if((Fresult == FR_OK) && (successfulreads == 1)){
//						if(counter%2){ // if odd hex value
//							img1_buffer[counter/2] |= c << 8;
//						}
//						else{
//							img1_buffer[counter/2] = c;
//						}
//					}
//					else{
//						break;
//					}
//				}
//			}
//			else{
//				UART_OutString("ERROR OPENING CLOUD2.BIN\n\r");
//			}
//			Fresult = f_close(&Handle);
//			if(Fresult != FR_OK){
//				UART_OutString("ERROR CLOSING CLOUD2.BIN\n\r");
//			}
//			Fresult = f_open(&Handle, "cloud1.bin", FA_READ);
//			if(Fresult == FR_OK){
//				// get a character in 'c' and the number of successful reads in 'successfulreads'
//				for(counter = 0; counter < 1352; counter++){
//					Fresult = f_read(&Handle, &c, 1, &successfulreads);
//					if((Fresult == FR_OK) && (successfulreads == 1)){
//						if(counter%2){ // if odd hex value
//							img2_buffer[counter/2] |= c << 8;
//						}
//						else{
//							img2_buffer[counter/2] = c;
//						}
//					}
//					else{
//						break;
//					}
//				}
//			}
//			else{
//				UART_OutString("ERROR OPENING CLOUD1.BIN\n\r");
//			}
//			Fresult = f_close(&Handle);
//			if(Fresult != FR_OK){
//				UART_OutString("ERROR CLOSING CLOUD1.BIN\n\r");
//			}
//		}
//		else{ //load rain1 and rain2
//			Fresult = f_open(&Handle, "rain2.bin", FA_READ);
//			if(Fresult == FR_OK){
//				// get a character in 'c' and the number of successful reads in 'successfulreads'
//				for(counter = 0; counter < 1352; counter++){
//					Fresult = f_read(&Handle, &c, 1, &successfulreads);
//					if((Fresult == FR_OK) && (successfulreads == 1)){
//						if(counter%2){ // if odd hex value
//							img1_buffer[counter/2] |= c << 8;
//						}
//						else{
//							img1_buffer[counter/2] = c;
//						}
//					}
//					else{
//						break;
//					}
//				}
//			}
//			else{
//				UART_OutString("ERROR OPENING rain2.BIN\n\r");
//			}
//			Fresult = f_close(&Handle);
//			if(Fresult != FR_OK){
//				UART_OutString("ERROR CLOSING rain2.BIN\n\r");
//			}
//			Fresult = f_open(&Handle, "rain1.bin", FA_READ);
//			if(Fresult == FR_OK){
//				// get a character in 'c' and the number of successful reads in 'successfulreads'
//				for(counter = 0; counter < 1352; counter++){
//					Fresult = f_read(&Handle, &c, 1, &successfulreads);
//					if((Fresult == FR_OK) && (successfulreads == 1)){
//						if(counter%2){ // if odd hex value
//							img2_buffer[counter/2] |= c << 8;
//						}
//						else{
//							img2_buffer[counter/2] = c;
//						}
//					}
//					else{
//						break;
//					}
//				}
//			}
//			else{
//				UART_OutString("ERROR OPENING rain1.BIN\n\r");
//			}
//			Fresult = f_close(&Handle);
//			if(Fresult != FR_OK){
//				UART_OutString("ERROR CLOSING rain1.BIN\n\r");
//			}
		}
		else{
			condition[counter] = 3;
		}
	}
		cityNumber = 0;
		counter = 0;
		//INIT SYSTICK
	
	/*
		char Humidity[3][MAXLEN];
	char City[3][MAXLEN];
	char Temperature[3][MAXLEN];
	char Temp_min[3][MAXLEN];
	char Temp_max[3][MAXLEN];
	char Weather[3][MAXLEN];
	*/
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_CURRENT_R = 0;
	NVIC_ST_RELOAD_R = 0xFFFFFF;
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN;
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_FillRect(0,53, 128, 107, ST7735_WHITE);
	ST7735_OutStringSized(0, 0, City[cityNumber], 2, ST7735_BLACK, ST7735_WHITE);
	ST7735_DrawString(0, 7, "Current Temp: ", ST7735_WHITE);
	ST7735_OutStringSized(0, 3, Temperature[cityNumber], 3,  ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(0, 13, "Max Temp: ", ST7735_RED);
	ST7735_DrawString(10, 13, Temp_max[cityNumber], ST7735_RED);
	ST7735_DrawString(0, 14, "Min Temp: ", ST7735_CYAN);
	ST7735_DrawString(10, 14, Temp_min[cityNumber], ST7735_CYAN);
	ST7735_DrawString(0, 15, "Humidity: ", ST7735_WHITE);
	ST7735_DrawString(10, 15, Humidity[cityNumber], ST7735_WHITE);
    while(Board_Input()==0){
		}; // wait for touch
		NVIC_ST_CTRL_R = 0;
		delay(3);
    LED_GreenOff();
  }
}


/*!
    \brief This function puts the device in its default state. It:
           - Set the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregister mDNS services

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static int32_t configureSimpleLinkToDefaultState(char *pConfig){
  SlVersionFull   ver = {0};
  UINT8           val = 1;
  UINT8           configOpt = 0;
  UINT8           configLen = 0;
  UINT8           power = 0;

  INT32           retVal = -1;
  INT32           mode = -1;

  mode = sl_Start(0, pConfig, 0);


    /* If the device is not in station-mode, try putting it in station-mode */
  if (ROLE_STA != mode){
    if (ROLE_AP == mode){
            /* If the device is in AP mode, we need to wait for this event before doing anything */
      while(!IS_IP_AQUIRED(g_Status));
    }

        /* Switch to STA role and restart */
    retVal = sl_WlanSetMode(ROLE_STA);

    retVal = sl_Stop(0xFF);

    retVal = sl_Start(0, pConfig, 0);

        /* Check if the device is in station again */
    if (ROLE_STA != retVal){
            /* We don't want to proceed if the device is not coming up in station-mode */
      return DEVICE_NOT_IN_STATION_MODE;
    }
  }
    /* Get the device's version-information */
  configOpt = SL_DEVICE_GENERAL_VERSION;
  configLen = sizeof(ver);
  retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (unsigned char *)(&ver));

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
  retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);

    /* Remove all profiles */
  retVal = sl_WlanProfileDel(0xFF);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
  retVal = sl_WlanDisconnect();
  if(0 == retVal){
        /* Wait */
     while(IS_CONNECTED(g_Status));
  }

    /* Enable DHCP client*/
  retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);

    /* Disable scan */
  configOpt = SL_SCAN_POLICY(0);
  retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
  power = 0;
  retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&power);

    /* Set PM policy to normal */
  retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);

    /* TBD - Unregister mDNS services */
  retVal = sl_NetAppMDNSUnRegisterService(0, 0);


  retVal = sl_Stop(0xFF);


  g_Status = 0;
  memset(&Recvbuff,0,MAX_RECV_BUFF_SIZE);
  memset(&SendBuff,0,MAX_SEND_BUFF_SIZE);
  memset(&HostName,0,MAX_HOSTNAME_SIZE);
  DestinationIP = 0;;
  SockID = 0;


  return retVal; /* Success */
}




/*
 * * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent){
  switch(pWlanEvent->Event){
    case SL_WLAN_CONNECT_EVENT:
    {
      SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'sl_protocol_wlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * sl_protocol_wlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
    }
    break;

    case SL_WLAN_DISCONNECT_EVENT:
    {
      sl_protocol_wlanConnectAsyncResponse_t*  pEventData = NULL;

      CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
      CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_AQUIRED);

      pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
      if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code){
        UART_OutString(" Device disconnected from the AP on application's request \r\n");
      }
      else{
        UART_OutString(" Device disconnected from the AP on an ERROR..!! \r\n");
      }
    }
    break;

    default:
    {
      UART_OutString(" [WLAN EVENT] Unexpected event \r\n");
    }
    break;
  }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent){
  switch(pNetAppEvent->Event)
  {
    case SL_NETAPP_IPV4_ACQUIRED:
    {

      SET_STATUS_BIT(g_Status, STATUS_BIT_IP_AQUIRED);
        /*
             * Information about the connected AP's ip, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */

    }
    break;

    default:
    {
            UART_OutString(" [NETAPP EVENT] Unexpected event \r\n");
    }
    break;
  }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pServerEvent - Contains the relevant event information
    \param[in]      pServerResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse){
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
  UART_OutString(" [HTTP EVENT] Unexpected event \r\n");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent){
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
  UART_OutString(" [GENERAL EVENT] \r\n");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock){
  switch( pSock->Event )
  {
    case SL_NETAPP_SOCKET_TX_FAILED:
    {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
      switch( pSock->EventData.status )
      {
        case SL_ECLOSE:
          UART_OutString(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\r\n");
          break;


        default:
          UART_OutString(" [SOCK EVENT] Unexpected event \r\n");
          break;
      }
    }
    break;

    default:
      UART_OutString(" [SOCK EVENT] Unexpected event \r\n");
    break;
  }
}
/*
 * * ASYNCHRONOUS EVENT HANDLERS -- End
 */


