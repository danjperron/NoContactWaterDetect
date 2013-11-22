/* 
 * File:   WaterDetect.c
 * Author: daniel
 *
 * Created on 7 novembre 2013, 20:49
 */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  WaterDetect
//
// Simple program to access Detect Water Level using cap sense sensor
// Serial communication will be use to communicate outside

//   Date: 6 Novembre 2013
//   programmer: Daniel Perron
//   Version: 1.01
//   Processor: PIC12F1840
//   software: Microchip MPLAB V8.9   with light version of Hitech C
//   OR
//   Software: Microchip MPLAB  X IDE v1.95(freeware version)
//
//   Revise version 10 Novembre 2013
//   Cap sense has been chnage to low power  to reduce power
//   Clock Frequency has been change to 8Mhz to reduce power
//   Overclock cap sense timer1Ovr removed since clock will never overclock
//   Serial In communication removed\ need pins for caliration buttons
//   Serial Out set at 9600 and only avaiable on ICSP connector for debug purpose only
//   Calibration for air and water are now set by momentary push buttons.


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
/*

 RA0    OUT      SERIAL OUT
 RA1	IN       SWITCH Air Calibration set
 RA2	SENSE    CAPACITIVE SENSE CPS2
 RA3	IN	 MCLR  Master Clear  (Reset cpu when is low)
 RA4	OUT      Alert signal (high=no water lo=water)
 RA5	IN	 SWITCH Water Calibration set
*/

#define ALERT_OUTPUT RA4
#define AIR_BUTTON   RA1
#define WATER_BUTTON RA5

// button debouncing

#define NO_BUTTON_PRESS    0
#define AIR_BUTTON_PRESS   1
#define WATER_BUTTON_PRESS 2

// base on 1/10 sec time  .3 sec debounce
#define SET_DEBOUNCE_TIME  3

unsigned char ButtonDebounce;

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

__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_OFF & BORV_LO & LVP_ON);
__IDLOC(0000);


//////////////   eerom storage
// Air Calib Data and Water Calib Data (both unsigned short)

__EEPROM_DATA(0x48,0xd,0x60,0x9,0,0,0,0);  // Default AirFrequency=2400 (0x960) Water=3400 (0xd48)

// set the __EEPROM_DATA  according to the following structure
#endif


typedef struct {
unsigned short AirFrequency;
unsigned short WaterFrequency;
}SettingStruct;

#ifdef __XC__
__eeprom SettingStruct eSetting= { 3400,2400};
#endif

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
#define SERIAL_BUFFER_SIZE 32
near volatile unsigned char InFiFo, OutFiFo;   // these are the buffer pointers for interrupt serial communication; InFiFo ctrl by putch, OutFiFo ctrl by interrupt
char SerialBuffer[SERIAL_BUFFER_SIZE];


/* Timer utilisation

Timer1  Capsense counter
Timer2  1ms interrupt timer
*/



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



unsigned char CheckButtons(void)
{
    unsigned char temp;


    temp = NO_BUTTON_PRESS;


    if((AIR_BUTTON==1) && (WATER_BUTTON==1))
        return NO_BUTTON_PRESS;

    if(ButtonDebounce>0)
    {
      ButtonDebounce=SET_DEBOUNCE_TIME;
    }
    else
    {

      if((AIR_BUTTON==0) && (WATER_BUTTON==0))
      {
        // very little chance to go here but just in case
      }
      else if(AIR_BUTTON == 0)
        temp= AIR_BUTTON_PRESS;
      else if(WATER_BUTTON == 0)
        temp= WATER_BUTTON_PRESS;

      ButtonDebounce = SET_DEBOUNCE_TIME;
    
    }

    return temp;
}



void Init1msTimer()
{

// we want 1mss period so
// clock of 8Mhz give  125 ns   1ms / 125ns = 2000
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
  CPSCON0 = 0b10000100;

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
// check serial transmit Interrupt
if(TXIE)
 if(TXIF)
  {
     // do we have a new char to send
    if(InFiFo != OutFiFo)
      {
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
       TXIE=0;
     }
  }




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

   if(ButtonDebounce>0)
       ButtonDebounce--;
  }
}

}



void putch(char char_out)
{
   unsigned char temp;

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





 main(void){

 char UserKey;  //UserKey. just reset the serial in buffer

 OSCCON		= 0b01110011;	// 8MHz  internal clock
 OPTION_REG	= 0b00000011;	// pullups on, TMR0 @ Fosc/4/16 ( need to interrupt on every 80 clock 16*5)
 ANSELA		= 0b00000;	// NO Analog
 PORTA   	= 0b00100000;
 WPUA		= 0b00111011;	// pull-up ON
 TRISA		= 0b00101110;	// ALL INPUT  except RA0, RA4 OUTPUT
 INTCON		= 0b00000000;	// no interrupt


 //  A/D & FVR OFF
 ADCON0=0;
 FVRCON=0;

 LoadSetting();
 SetThreshold();
   
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
    RCIE =0;   // disable received interrup;
    TXIE =0;   // disable transmit interrupt
    RCIF =0;   // clear received flag
    TXIF = 0;
    SCKP = 0;
    ABDEN = 0;
// reset interrupt fifo buffer
    InFiFo=0;
    OutFiFo=0;
    GIE = 1;
    PEIE =1;  // enable peripheral



    Init1msTimer() ;
    InitCapSense() ;



 // wait for  serial uart  i/o pin toggle ( at least one full caracter length)
  __delay_ms(100);
  cputs("Water Detect V1.01\n\r");

  UserKey= RCREG; // clear receive buffer;
  ButtonDebounce = 5; //Force .5 sec before button are valid

 while(1)
{

 // if serial in just kill it
 if(RCIF)
   {
     UserKey = RCREG;  // get user command
   }


 switch(CheckButtons())
 {
     case AIR_BUTTON_PRESS:
          Setting.AirFrequency = ReportFrequency.USHORT;
          SaveSetting();
          SetThreshold();
          cputs("Store Air Frequency\n\r");
          break;

     case WATER_BUTTON_PRESS:
          Setting.WaterFrequency=ReportFrequency.USHORT;
          SaveSetting();
          SetThreshold();
          cputs("Store Water Frequency\n\r");
 }


  
 if(SenseFlag)
  {
	  // let's report the frequency found
      ReportFrequency.BYTE[1]=CurrentTimer1H;
      ReportFrequency.BYTE[0]=CurrentTimer1L;
      printUShort(ReportFrequency.USHORT);
      putch('\r');
      putch('\n');
  
      SenseFlag=0;

      if(ReportFrequency.USHORT < ThresholdFrequency)
        ALERT_OUTPUT=1;
      else
        ALERT_OUTPUT=0;
   }

 }

}



