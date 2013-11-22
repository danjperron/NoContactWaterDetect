/* 
 * File:   WaterDetectModbus.c
 * Author: daniel
 *
 * Created on 11 novembre 2013, 20:49
 */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  WaterDetect using   serial modbus protocol
//
// Simple program to access Detect Water Level using cap sense sensor
// Serial communication will be use to communicate outside

//   Date: 11 Novembre 2013
//   programmer: Daniel Perron
//   Version: 1.0
//   Processor: PIC12F1840
//   software: Microchip MPLAB V8.9   with light version of Hitech C
//   OR
//   Software: Microchip MPLAB  X IDE v1.95(freeware version)
//
//
//  Date: 17 Novembre 2013
//  Version: 1.01
//
//  Time out of 2 character  added to find end of packet

#define SOFTWARE_ID     0x6531
#define RELEASE_VERSION 0x0101

///////////////////  How to program the IC.
//
//    1 - Use MPLAB with pickit 3  (need version 8.90 at least)
//  or
//    2 - Use standalone pickit2 v 2.61 (Select the I.C. and load the corresponding hex file).
//  or
//    3 - Use Raspberry Pi board  with burnVLP.py . Software available from https://github.com/danjperron/burnLVP
//
////////////////////////////////////  GPL LICENSE ///////////////////////////////////


/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



/*  COMMUNICATION PROTOCOL

     baud rate 9600 , 8 bits data , no parity
     just send frequency follow by a cariage return

      using TTL UART with fix baud at 9600, 8 bits data, no parity.


//////////////////////////  PORT DESCRIPTION


 PIN 7  RA0 OUT    	SERIAL OUT
 PIN 6  RA1	IN     	SERIAL IN
 PIN 5  RA2	SENSE 	CAPACITIVE SENSE CPS2
 PIN 4  RA3	IN	 	MCLR  Master Clear  (Reset cpu when is low)
 PIN 3  RA4	OUT   	Alert signal (high=no water lo=water)
 PIN 2  RA5	OUT   	RS-485 DIRECTION  (high=transmit LOW=receive)
*/

// led output
#define ALERT_OUTPUT RA4

//rs-485 data direction
#define TXM_ENABLE   RA5

#ifdef __XC__
#include <xc.h>
#include <stdio.h>
#else
#include <htc.h>
#include <stddef.h>
#include <conio.h>
#endif


#include <stdlib.h>

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 4MHz system frequency is assumed
 #define _XTAL_FREQ 8000000
#endif



#define iabs(A)  (A<0 ?  (-A) :  A)

#ifdef __XC__
// CONFIG1
#pragma config FOSC = INTOSC // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON // Low-Voltage Programming Enable (Low-voltage programming enabled)

#else

//__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
//__CONFIG(WRT_OFF & PLLEN_OFF & BORV_LO & LVP_ON);
//__IDLOC(0000);

__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_OFF & BORV_LO & LVP_ON);
__IDLOC(0000);
#endif


//Set default value
__EEPROM_DATA(0x48,0xd,0x60,0x9,127,0xff,0xff,0xff);  // Default AirFrequency=3400 (0xd48) Water=2400 (0x960)  Slave Address=127


typedef struct{
unsigned short AirFrequency;
unsigned short WaterFrequency;
unsigned char  SlaveAddress;
}SettingStruct;



SettingStruct Setting;

near volatile unsigned short Timerms;       //  Interrupt Timer counter in ms
near volatile unsigned char SenseFlag;

#pragma pack 1
typedef union {
  unsigned short USHORT;
  unsigned char BYTE[2];
}ByteShortUnion;

unsigned short ThresholdFrequency;

near ByteShortUnion  ReportFrequency;

near unsigned char CurrentTimer1H;
near unsigned char CurrentTimer1L;

// serial buffer
#define SERIAL_BUFFER_SIZE 16
near volatile unsigned char InFiFo, OutFiFo;   // these are the buffer pointers for interrupt serial communication; InFiFo ctrl by putch, OutFiFo ctrl by interrupt
near volatile unsigned char RcvInFiFo, RcvOutFiFo;
char SerialBuffer[SERIAL_BUFFER_SIZE];
char RcvSerialBuffer[SERIAL_BUFFER_SIZE];

unsigned char SerialSum;    // use for check sum
unsigned char RcvSerialSum;  // use for check sum verification

unsigned char ModbusTransmitFlag;  //0 receive  , 1 transmit
unsigned char RcvModbusFrameTimeOutFlag;  //0 still waiting, 1 got a full Frame to analyze
unsigned char RcvModbusFrameTooLong;      // 0 indicate Frame is ok, 1 it is too long at we should ignore it

// MODBUS

// CRC16 source code is in CRC16.c file
extern unsigned short CRC16(unsigned char * puchMsg, unsigned char usDataLen);



unsigned char ModbusFunction;
unsigned char ModbusSlave;
unsigned short ModbusAddress;
unsigned short ModbusData;
volatile unsigned short ModbusCRC;
unsigned char ModbusBufferPointer;
near volatile unsigned char TxmEnableDelay;

#define MODBUS_BUFFER_SIZE 16
unsigned char ModbusBuffer[MODBUS_BUFFER_SIZE];

//MODBUS EXCEPTION
#define ILLEGAL_FUNCTION     1
#define ILLEGAL_DATA_ADDRESS 2
#define ILLEGAL_DATA_VALUE   3
#define SLAVE_DEVICE_FAILURE 4
#define ACKNOWLEDGE          5
#define SLAVE_DEVICE_BUZY    6
#define NEGATIVE_AKNOWLEDGE  7
#define MEMORY_PARITY_ERROR  8

#define INIT_MODE            0
#define READ_FIRST_BYTE      1
#define READ_TIME_OUT        2
#define WRITE_FRAME          3
#define WRITE_UNTIL_END      4 
#define CLEAR_END            5

volatile near unsigned char CycleMode = 0;


// EEPROM LOAD AND SAVE SETTING

void LoadSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;


  for(idx=0; idx < sizeof(Setting);idx++)
     *(pointer++) = eeprom_read(idx);
}

void SaveSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;
 
  for(idx=0; idx < sizeof(Setting);idx++)
      eeprom_write(idx, *(pointer++));
}



/////////////////////////////////////////// Timer usage
//
// Timer 1 Is the Cap sensor counter
//
// Timer 2 is the 1 ms clock timer
//
// Timer 0 is the Time out for modbus Receive Frame
//
//  2 character length at 9600 baud , 1 start, 8 bit , 1 stop = 2.083 ms
//
//  at 8Mhz clock the count has to be
//   2.083 ms / (1/(8Mz/4)) = 4167 count
//
//   with a 1:32 prescaler we got  41676/32=  130,21 => 130
//   than we need to reload TMR0 with 256-130 = 126;
//
#define TIME_OUT_VALUE 126 


void InitTimer()
{

// Set Timer2
// 1 ms time period
// we want 1ms period so
// clock of 8Mhz/4 give  500 ns   1ms / 500ns = 2000
// 2000  prescaler of 16 = 125 start count



T2CON=0;
PR2 = 125;
TMR2=0;
T2CON= 0b00000110; // Timer on prescaller 16
 // Enable IRQ
TMR2IF=0;
Timerms=0;
SenseFlag=0;
TMR2IE=1;
PEIE = 1;
GIE=1;


}

void InitCapSense()
{
  // set cap sense on Timer1
  // CPSON=1
  // CPSRM=0;  // fixed voltage
  // CPSRNG=1;  // low current range
  // T0XCS=0;    // TMR0 control itself
  CPSCON0 = 0b10001000;

  CPSON=1;
  CPSCON1=2;

  // now set Timer1 without starting it
  // TMR1CS = 3;   // Timer1 Clock source is CAPOSC
  // T1CKPS = 0;   // Prescaler at 1
  // T1OSCEN= 0;   // dedicated oscillator of
  // T1SYNC=  0;   // SYNC CLOCK
  // TMR1ON=  0;   // TIMER OFF FOR now

  T1CON= 0b11000000;

  //TMR1GE = 0;   // Count regardless of gate
  //TMR1GPOL = 0;  // gate polarity (don't care)
  //T1GTM=0;        // Gate toggle is disable
  //T1GSPM=0;        // Gate Single pulse disable
  //T1GGO=0;         // Gate Go
  //T1GVAL=0;
  //T1GSS=00;

  // no timer gate
  T1GCON = 0;
  // now let's clear the Timer and variables related to cap sense

  TMR1H=0;
  TMR1L=0;


 ReportFrequency.USHORT=0;
 CurrentTimer1H=0;
 CurrentTimer1L=0;

  // and now disable Timer1 overflow interupt
  TMR1IF=0;
  TMR1IE=0;

 }




static void interrupt isr(void){
// Timer 1 ms
if(TMR2IF){
 TMR2IF=0;
 Timerms++;

 if(Timerms==100)
  {
   Timerms=0;
   
   // stop TIMER1
   TMR1ON = 0;

   // ok let's transfer
   // record timer1 data
   CurrentTimer1H=TMR1H;
   CurrentTimer1L=TMR1L;
   
   // now let's reset everything
   TMR1IF=0;
   TMR1H=0;
   TMR1L=0;

   // And restart TIMER1
   TMR1ON=1;

   // And tell we have data
   SenseFlag=1;


  }

      if(TxmEnableDelay>0)
     {
       TxmEnableDelay--;
     }

}
// check serial transmit Interrupt
if(TXIE)
 if(TXIF)
  {
     // do we have a new char to send
    if(InFiFo != OutFiFo)
      {
         TXM_ENABLE=1;

        TXREG= SerialBuffer[OutFiFo];
        OutFiFo++;
       if(OutFiFo >= SERIAL_BUFFER_SIZE)
         OutFiFo=0;
      }
     else
   if(OutFiFo == InFiFo)
     {
       // no more char to send
       // just disable interrupt
       // but one character left in buffer
       // just add a delay of 2 ms before switching to receive mode
       
       TXIE=0;
     }
  }

// check serial  receive
if(RCIE)
 if(RCIF)
   {
   
     RcvSerialBuffer[RcvInFiFo++]= RCREG;
     // OK we got character  we need to reset timer 0 for time out
     TMR0= TIME_OUT_VALUE;  // set the 3 character delay
     TMR0IF=0;              // clear timer 0 flag
     TMR0IE=1;              // enable interrupt 
  
     if(RcvInFiFo == SERIAL_BUFFER_SIZE) // circular buffer just reset it
        RcvInFiFo=0;
   }

// Timer0 Timeout
if(TMR0IE)
    if(TMR0IF)
     {
     // RCIE=0;                // disable new received
        // got A Timer0 Timeout
      RcvModbusFrameTimeOutFlag=1;  // Set The flag
      TMR0IE=0;            // disable new TIMER0 interrupt
      TMR0IF=0;     
 
    }
}




void putch(char char_out)
{
  unsigned char temp;
  SerialSum+= (unsigned char) char_out;
  // increment InFiFo and  loop resize if need it.
  temp = InFiFo + 1;
  if(temp >= SERIAL_BUFFER_SIZE)
     temp = 0;

  //  wait  if buffer full
  while(temp == OutFiFo);

  // ok write the buffer
  SerialBuffer[InFiFo]=char_out;
  // now tell the interrupt routine we have a new char
  InFiFo= temp;
  // and enable interrupt
  TXIE=1;
}






 void printHexNibble(unsigned  char value)
  {
    value = value & 0xf;
 
    if(value<10)
     putch('0'+value);
    else
     putch('A'-10+value);
  }


 void printHexByte(unsigned char value)
 {
     printHexNibble(value >> 4);
     printHexNibble(value);
 }


 void printHexUShort( unsigned short value)
 {
     printHexByte(value >> 8);
     printHexByte(value);
 }



 // if ascii caracter is a hex value return
 // the subtracted value to get the numeric value
 // if it is not return 0
 unsigned char IsHexValid(char value)
 {
     if(value < '0') return 0;
     if(value <= '9') return '0';
     if(value < 'A')  return 0;
     if(value < 'G')  return ('A' - 10);
     if(value < 'a')  return 0;
     if(value < 'g')  return ('a' -10);
     return 0;
 }



 // convert an hex ascii value to the
 // corresponding numeric value
 // if it is not hex return 0
 unsigned char GetHexValue( unsigned char value)
 {

     unsigned char temp;

     temp = IsHexValid(value);
     if(temp==0) return 0;

     return(value - temp);
 }

 void printUShort( unsigned short value)
{
   char buffer[16];
   utoa(buffer,value,10); cputs(buffer);
}




void SetThreshold(void)
{
    // we assume that sum won't be over 65535
    // if power of cap sense is increase, we need to deal with it
  ThresholdFrequency = (Setting.AirFrequency + Setting.WaterFrequency)/2;
}


unsigned char  RcvGetBufferLength(void)
{
   unsigned char temp;

   temp =  SERIAL_BUFFER_SIZE;
   temp += RcvInFiFo;
   temp -= RcvOutFiFo;
   
   return (temp % SERIAL_BUFFER_SIZE);
}

void RcvClear(void)
{
    unsigned char _key;

    GIE=0;
    RcvInFiFo=0;
    RcvOutFiFo=0;
    _key=RCREG;
    GIE=1;
}

unsigned char RcvIsDataIn(void)
 {
     return (RcvInFiFo == RcvOutFiFo ? 0 : 1);
 }

char RcvGetChar(void)
 {
    char temp;

   // wait until we received something
   while(!RcvIsDataIn());

  // get the character
    temp =  RcvSerialBuffer[RcvOutFiFo];
    RcvOutFiFo++;
    if(RcvOutFiFo >= SERIAL_BUFFER_SIZE)
      RcvOutFiFo=0;

    return temp;
}

void  SendFrameError(unsigned char Function , unsigned char ErrorCode)
{
   unsigned loop;
   unsigned char buffer[8];
   unsigned short  CRC;

    buffer[0]= Setting.SlaveAddress;
    buffer[1]= Function | 0x80;
    buffer[2]= ErrorCode;

    CRC = CRC16(buffer,3);
    buffer[3] = CRC >> 8;
    buffer[4] = CRC & 0xff;

    for(loop=0;loop<5;loop++)
        putch(buffer[loop]);

    ModbusTransmitFlag=1;
}


void SendReadFrame(unsigned  short value)
{

   unsigned loop;
   unsigned char buffer[8];
   unsigned short  CRC;

   buffer[0]= Setting.SlaveAddress;
   buffer[1]= ModbusFunction;
   buffer[2]= 2;  // byte count
   buffer[3]= value >> 8;
   buffer[4]= value & 0xff;


   CRC = CRC16(buffer,5);
   buffer[5] = CRC >> 8;
   buffer[6] = CRC & 0xff;

    for(loop=0;loop<7;loop++)
        putch(buffer[loop]);

   ModbusTransmitFlag=1;

}


void SendPresetFrame()
{
   unsigned loop;
   unsigned char buffer[10];
   unsigned short  CRC;

   buffer[0]= Setting.SlaveAddress;
   buffer[1]= ModbusFunction;
   buffer[2]= 2;  // byte count

   buffer[3]= ModbusAddress >> 8;
   buffer[4]= ModbusAddress & 0xff;

   buffer[5]= ModbusData >> 8;
   buffer[6]= ModbusData & 0xff;


   CRC = CRC16(buffer,7);
   buffer[7] = CRC >> 8;
   buffer[8] = CRC & 0xff;

    for(loop=0;loop<9;loop++)
        putch(buffer[loop]);

   ModbusTransmitFlag=1;

}


void WaitForSendComplete(void)
{
    // ok wait until TXIE is disable
    //no tramsit caracter in shift register
    
     while(InFiFo!=OutFiFo);  // ok no more data in fifo
     while(TXIE==1);          // ok transmit interrupt stopped
     //check TRMT STATUS
     while(!TRMT);            // and transmit shift register is empty
     TXM_ENABLE=0;            // now we go in receive mode

}

void Listen()
{
    // Disable TIMER0 time out
    // Enable serial IN
    TMR0IE=0;  // disable TIMER0
    TMR0IF=0;
    RcvClear();
    TXM_ENABLE=0;

}


unsigned char DecodeSerial(unsigned char *pt, unsigned char tsize)
{
   
    unsigned char rcode;
    unsigned short CalcCRC;
 
    
    if(tsize<8)
        rcode=3;
    else
    {
    
    ModbusSlave= *(pt++);
    ModbusFunction= *(pt++);


    #define ToUSHORT    ((((unsigned short)*pt) * 256) + ((unsigned short) pt[1]));pt+=2;

    ModbusAddress= ToUSHORT;
    ModbusData= ToUSHORT;
    ModbusCRC= ToUSHORT;

   
    CalcCRC = CRC16(ModbusBuffer,tsize-2);

   if(CalcCRC != ModbusCRC)      rcode=0;    
   else if(ModbusSlave==Setting.SlaveAddress) rcode=1;
   else rcode=2;
    }
   return rcode;
  
}


  // Function 3  Read Holding Register
  //
  // Address 0: Air Frequency Detection
  // Address 1: Water Frequency Detection
  // Address 2: SlaveAddress  (Node ID)
  // Address 3: Version Number
  // Address 4: Software ID NUMBER

void   ReadHoldingRegister()
{
    unsigned short temp;

    // only the first 4 address possible
    // and only 1 address at a time
   if((ModbusAddress>4) || (ModbusData !=1))
       SendFrameError(ModbusFunction | 0x80, ILLEGAL_DATA_ADDRESS);
   else
   {


       switch(ModbusAddress)
       {
           case 0: temp = Setting.AirFrequency;break;
           case 1: temp = Setting.WaterFrequency;break;
           case 2: temp = Setting.SlaveAddress;break;
           case 3: temp = RELEASE_VERSION;break;
           case 4: temp = SOFTWARE_ID;break;
       }
    SendReadFrame(temp);
   }
}


  // Function 4  Read Current Register
  //
  // Address 0:  Current Capacitive Frequency
  // Address 1:  CPU PORT A

void   ReadCurrentRegister()
{
    unsigned short temp;

    // and only 1 address at a time

   if((ModbusAddress>1) || (ModbusData != 1))
       SendFrameError(ModbusFunction | 0x80, ILLEGAL_DATA_ADDRESS);
   else
   {
       switch(ModbusAddress)
       {
           case 0: temp = ReportFrequency.USHORT;break;
           case 1: temp = PORTA;
       }
    SendReadFrame(temp);
   }
}

 // Function 6 Preset Single Register
  //
  // Address 0: Air Frequency Detection
  // Address 1: Water Frequency Detection
  // Address 2: SlaveAddress

void PresetSingleRegister()
{

    if(ModbusAddress > 2)
       SendFrameError(ModbusFunction | 0x80 , ILLEGAL_DATA_ADDRESS);
	else
	{
     switch(ModbusAddress)
     {
        case 0 :  Setting.AirFrequency=ModbusData;break;
        case 1 :  Setting.WaterFrequency=ModbusData;break;
        case 2 :  Setting.SlaveAddress=ModbusData;break;
     }
     SaveSetting();
     SetThreshold();
     SendPresetFrame();
    }
}


void ExecuteFunction(void)
{

  __delay_ms(2); 
 
  if(ModbusFunction == 3)
      ReadHoldingRegister();
  else if(ModbusFunction == 4)
      ReadCurrentRegister();
  else if(ModbusFunction == 6)
      PresetSingleRegister();
  else
     SendFrameError(ModbusFunction | 0x80, ILLEGAL_FUNCTION);

}



main(void){
 unsigned char loop;
 unsigned char rcode;
 unsigned char _key;
 
 OSCCON		= 0b01110011;	// 8MHz  internal clock
 OPTION_REG	= 0b00000100;	// pullups on, TMR0 @ (Fosc/4)/32

 ANSELA		= 0b00000;	// NO Analog
 PORTA   	= 0b00000001;

 WPUA		= 0b00111011;	// pull-up ON
 TRISA		= 0b00001110;	// ALL INPUT  except RA0, RA4, RA5 OUTPUT
 INTCON		= 0b00000000;	// no interrupt


// Set Timer0
// Use to detect Modbus Frame time out
// prescaler 32
 TMR0IE=0;

 //  A/D & FVR OFF
 ADCON0=0;
 FVRCON=0;

 Setting.AirFrequency=3400;
 Setting.SlaveAddress=1;
 Setting.WaterFrequency=2400;
 SetThreshold();


 TxmEnableDelay=0;
   
 // set serial com with 9600 baud
 APFCON=0;
 TXSTA = 0b10000010;
 RCSTA = 0;
 BRGH =1;
 BRG16 = 1;
 SYNC =0;
 SPBRGL = 207;  //9600  baud
 //SPBRGL = 16; //115200 baud
 SPBRGH =0;
 TXEN =1;   // enable transmitter
 SPEN = 1;  // enable serial port
 CREN = 1;  // enable receiver
 TXIE =0;   // disable transmit interrupt
 RCIF =0;   // clear received flag
 TXIF = 0;
 SCKP = 0;
 ABDEN = 0;

 
 // reset interrupt fifo buffer
 InFiFo=0;
 OutFiFo=0;
 RcvInFiFo=0;
 RcvOutFiFo=0;

 GIE = 1;
 PEIE =1;   // enable peripheral
 RCIE =1;   // Enable received interrupt;

 // load setting from flash eerom data
 LoadSetting();

 InitTimer() ;
 InitCapSense() ;

	// CycleMode  is the  status state of the cpu
 CycleMode = READ_FIRST_BYTE;
 
 while(1)
 {
  
     switch(CycleMode)
     {
   
      case INIT_MODE:          TMR0IE=0;            // disable new TIMER0 interrupt
      						   TMR0IF=0;     
                               RcvModbusFrameTimeOutFlag=0;
                               TXM_ENABLE=0;
                              __delay_ms(1);
                              Listen();
                              CycleMode=READ_FIRST_BYTE;

      case READ_FIRST_BYTE:  // read first caracter
                             // it is always a byte between 1.. 247
                             // this is a way to remove preliminary glitch  like 0x00 and 0xff

                              if(FERR)
                                 {
                                  _key=RCREG;
                                  break;
                                 }

                               if(OERR)
                                 {
                                   CREN=0;
                                   CREN=1;
                                 }
                                                          
                             if(!RcvIsDataIn()) break;  // nothing in ?
           
                             _key = RcvGetChar();
                             if((_key==0) || (_key>247))
                             {
							   // ok invalid character! Just forget it
                               // prevent time out flag to occurs
                               TMR0IE=0;            // disable new TIMER0 interrupt
      						   TMR0IF=0;     
                               RcvModbusFrameTimeOutFlag=0;
                               break;
                             }
                             // We have a valid Slave Address
                             ModbusBufferPointer=1;
                             ModbusBuffer[0]=_key;
                             RcvModbusFrameTooLong=0;
                             CycleMode=READ_TIME_OUT;
                             break;
      case READ_TIME_OUT:    // incremnent until we got time out
                             while(RcvIsDataIn())
                               {
                                 _key= RcvGetChar();
                                 if(RcvModbusFrameTooLong) break;
                                 ModbusBuffer[ModbusBufferPointer++]=_key;
                                 if(ModbusBufferPointer>=(MODBUS_BUFFER_SIZE-1))
                                 RcvModbusFrameTooLong=1;
                               }
                              if(RcvModbusFrameTimeOutFlag)
                               {
                                 // obviously if we have RcvModbusFrammeTooLong it is not for us
                                 if(RcvModbusFrameTooLong)
                                   {
                                     //ok READ_FIRST_BYTE  again
                                     CycleMode= READ_FIRST_BYTE;
                                     break;
								   }
 						       // let's analyze the Frame in buffer
                                 rcode = DecodeSerial(&ModbusBuffer[0],ModbusBufferPointer);
                          		// rcode = 
   								// 0 BAD CRC
   								// 1 Valid FRAME
  								// 2 Wrong module
  								// 3 Size to small
                                if(rcode == 1)
                                     CycleMode=WRITE_FRAME; //send answer
                                else if(rcode == 2)
                                  {
                                     CycleMode=INIT_MODE;
                                     break;
                                  } 
                                else if(rcode ==3)
								   {
                                      
								      CycleMode=INIT_MODE;
                                      break;
								   }
                                 else
                                    {
                   
                                      CycleMode=CLEAR_END;
                                    }
                               }   
                                break;
                             
      case WRITE_FRAME:       // ok send packet
                              ExecuteFunction();
                              CycleMode= CLEAR_END;
      case CLEAR_END:           // wait until everything has been send
  							  WaitForSendComplete();
                              TXM_ENABLE=0;     
                              CycleMode= INIT_MODE;
      default:
							  CycleMode=INIT_MODE;
                              break;
     }    

 if(SenseFlag)
  {
	  // let's report the frequency found
      ReportFrequency.BYTE[1]=CurrentTimer1H;
      ReportFrequency.BYTE[0]=CurrentTimer1L;
  
      SenseFlag=0;

      if(ReportFrequency.USHORT < ThresholdFrequency)
        ALERT_OUTPUT=1;
      else
        ALERT_OUTPUT=0;
   }



   }

 }







