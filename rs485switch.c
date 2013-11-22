/* 
 * File:   newmain.c
 * Author: Daniel
 *
 * Created on November 14, 2013, 12:14 PM
 */

             /*
 * File:   WaterDetect.c
 * Author: daniel
 *
 * Created on 7 novembre 2013, 20:49
 */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  RS-485 SwitcherWaterDetect
//
// Simple program to switch transmit/receive direction
// on MAX-485
//
//   Date: 14 Novembre 2013
//   programmer: Daniel Perron
//   Version: 1.0
//   Processor: PIC12F1840
//   software: Microchip MPLAB V8.9   with light version of Hitech C
//   OR
//   Software: Microchip MPLAB  X IDE v1.95(freeware version)
//
//   When  RPI_TXM pin go low , the direction is switch to transmit
//   When no transmission is detected after a 2ms delay (base on 2 character at 9600 BAUD)
//   The system is flipped to received mode.
//   
//   Use Timer0 for Timeout Delay to detect end of transmission
//
//   9600 baud => 2 * 10(1start,8bits,1stop) / 9600 = 2,083 ms
//
//   A delay of 0,5ms is added to remove glitch when transmit is switch back to receive mode
//
//   RA0  PIN 7   INPUT    DIP1      DIP SWITCH BAUD RATE SETTINGS
//   RA1  PIN 6   INPUT    DIP2
//
//      BAUD   DIP1    DIP2
//      9600   OFF     OFF
//     19200   OFF     ON
//     38400   ON      OFF
//     57600   ON      ON
//
//   RA2  PIN 5   INPUT    RPI_TXM      Transmit signal from RPI Any change on the pi will enable transmit for 2ms
//   RA3  PIN 4   INPUT    MCLR         Reset cpu when low (We will using LVP than RA3 can't be changed).
//   RA4  PIN 3   OUTPUT   OUT_ENABLE   Turn high and stay until no more input activity for 2 ms
//   RA5  PIN 2   OUTPUT   IN_ENABLE    Turn high when OUT_ENABLE is high and have a delay of 0.5 ms when OUT_ENABLE go low

#define RPI_TXM			RA2
#define OUT_ENABLE		RA4
#define IN_ENABLE		RA5




// 
//   TIMER 0 is used for temporisation delay
//
//   -- 9600 BAUD
//   2.083ms at 32Mhz CLock  =>  (32Mhz/4) * 2.083ms = 16666 count  / 128 (PRESCALER) = 130 count  (256 -130) = 126
//   0.5  ms                      (8Mhz * 0.5 ms)/128 =  31 count   (256 - 31) = 225
//
//   -- 57600 BAUD
//   347.222us  => 2778 count / 32 (prescaler) = 87  (256-87)= 169
//

#define T0_9600_PRESCALER 6
#define T0_9600_TIME_OUT  126
#define T0_9600_RCV_DELAY 225

#define T0_19200_PRESCALER 5
#define T0_19200_TIME_OUT  126
#define T0_19200_RCV_DELAY 225

#define T0_38400_PRESCALER 4
#define T0_38400_TIME_OUT  126
#define T0_38400_RCV_DELAY 225

#define T0_57600_PRESCALER 4
#define T0_57600_TIME_OUT  169
#define T0_57600_RCV_DELAY 234

                                     // 57600,38400,19200,9600
const unsigned char BaudPrescaler[4]= { T0_57600_PRESCALER,\
                                        T0_38400_PRESCALER,\
                                        T0_19200_PRESCALER,\
                                        T0_9600_PRESCALER};

const unsigned char BaudTimeOut[4]= {   T0_57600_TIME_OUT,\
                                        T0_38400_TIME_OUT,\
                                        T0_19200_TIME_OUT,\
                                        T0_9600_TIME_OUT};


const unsigned char BaudRcvDelay[4]= {  T0_57600_RCV_DELAY,\
                                        T0_38400_RCV_DELAY,\
                                        T0_19200_RCV_DELAY,\
                                        T0_9600_RCV_DELAY};


near volatile unsigned char T0_TimeOut;
near volatile unsigned char T0_RcvDelay;
near volatile unsigned char T0_Prescaler;

////////////////////////////////////  GPL LICENSE ////////////////
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


#ifdef __XC__
#include <xc.h>

#else
#include <htc.h>
#endif



#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 4MHz system frequency is assumed
 #define _XTAL_FREQ 32000000
#endif


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
#pragma config PLLEN = ON // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON // Low-Voltage Programming Enable (Low-voltage programming enabled)

#else


#ifndef BORV_LO
#define BORV_LO BORV_19
#endif


__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_ON & BORV_LO & LVP_ON);
__IDLOC(0000);

#endif






static void interrupt isr(void){
// check serial transmit Interrupt
if(IOCIE)
    if(IOCAFbits.IOCAF2)
    {
      // OK The RPI is transmitting something
      // We are in transmit mode
      IOCAFbits.IOCAF2=0;
      IN_ENABLE=1;          // disable reception
      OUT_ENABLE=1;         // enable transmission
      TMR0= T0_TimeOut;     // Reset the time out timer with correct delay
      TMR0IF=0;        	    // clear the timer flag
      TMR0IE=1;             // enable interrupt for time out
    }

	
// do we have time out?
if(TMR0IE)
 if(TMR0IF)
  {
     // ok we got time out
	 if(OUT_ENABLE)
	    {
	     // ok transmit time out
	     OUT_ENABLE=0;       // disable transmission
	     TMR0= T0_RcvDelay;  // Now set timer for 0.5ms delay
	     TMR0IF=0;
	     TMR0IE=1;
	   }
	   else
	   {
	    // ok 0.5 ms out for receive enable
	    IN_ENABLE=0;	// Enable Reception
	    TMR0IE=0;       // Disable Timer0 interrupt
	    TMR0IF=0;
	  }
  }

}



void ReadDipSwitch(void)
{
    unsigned char idx;
    unsigned char temp;


    idx = PORTA & 0x3;  // read dip switch (RA0 & RA1)



    // ok change prescaler
    temp = BaudPrescaler[idx];

    if(temp!=T0_Prescaler)
    {
     T0_Prescaler= temp;
     OPTION_REG= T0_Prescaler;  //handy since we are just using Prescaler
                                // in OPTION_REG
    }

    // ok Time out
    temp = BaudTimeOut[idx];
    if(temp!= T0_TimeOut)
        T0_TimeOut= temp;

    // rcv delay
    temp = BaudRcvDelay[idx];
    if(temp!=T0_RcvDelay)
        T0_RcvDelay= temp;
}


void main(void) {

// set default settings
OSCCON		= 0b11110000;	// 32MHz
OPTION_REG	= T0_9600_PRESCALER;	// pullups on, TMR0 @ Fosc/4, prescaler at 128
ANSELA		= 0b00000000;	// no analog pins
TRISA   	= 0b11001111;   // RA4 and RA5 output
PORTA		= 0b00000000;	// output low
WPUA		= 0b00111111;	// pull-up

T0_Prescaler = T0_9600_PRESCALER;
T0_TimeOut= T0_9600_TIME_OUT;
T0_RcvDelay = T0_9600_RCV_DELAY;

//  A/D & FVR OFF
ADCON0=0;
FVRCON=0;


// ok interrupt. Enable RA2 interrup on change. (up and down).
// if IN_TXM toggle ,we are in transmit mode

IOCAP = 0b00000100;   // enable positive edge on RA2
IOCAN = 0b00000100;   // enable negative edge on RA2
IOCAF = 0;            // clear flag



// enable interrupt
IOCIE = 1;  // enable interrupt on change
GIE=1;      // enable general interrupt

//TIMER 0 interrupt is enabled when a change occurs
// Check isr function.

// system in receive mode
OUT_ENABLE=0;
IN_ENABLE=0;

while(1) //loop all the times
{
    // everything is done in the interrupt routine
    // First! we need a change on RA2 pin to start interrupt
		
    // ***** Sleep mode to slow to return to normal
    // ***** Too much delay to enable transmit
    // ***** Sleep mode disable for now
    /*
      if(IN_ENABLE==0)
      {
       // OK RS_485 in receive mode
       // since we are waiting for change in transmit
       // we will put the cpu on sleep mode
       // and wait until we have to transmit
       asm("sleep");
	   asm("nop");
	  }
   */
    ReadDipSwitch();
}
}

