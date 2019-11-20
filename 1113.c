/*
 * File:        Final Pro
 * 
 * Author:      Bruce Land modified by Ximeng Zhang
 * For use with Sean Carroll's Big Board
 * http://people.ece.cornell.edu/land/courses/ece4760/PIC32/target_board.html
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_3_2.h"
// threading library
#include "pt_cornell_1_3_2.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// fixed point types
#include <stdfix.h>
// need for sine function and for rounding
#include <math.h>
////////////////////////////////////
#define	SYS_FREQ 40000000
// lock out timer interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR
#define start_spi2_critical_section INTEnable(INT_T2, 0);
#define end_spi2_critical_section INTEnable(INT_T2, 1);
// string buffer
char buffer[60];
//initialize control signals
int adc_val1 = 0;
int adc_val2 = 0;
int adc_val3 = 0;


// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_control;
int sys_time_seconds ;
// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt)){
    PT_BEGIN(pt);
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++;
//        tft_setCursor(3, 3);
//            tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
//            tft_fillRoundRect(3,3, 180, 24, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            sprintf(buffer,"Time:%d \n",sys_time_seconds);
//            tft_writeString(buffer);

        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Control Thread =================================================
// read 3 ADC values
static PT_THREAD (protothread_control(struct pt *pt))
{
    PT_BEGIN(pt);
    
      while(1) {
          adc_val1 = ReadADC10(0);
          adc_val2 = ReadADC10(1);
          adc_val3 = ReadADC10(2);
          tft_fillScreen(ILI9340_BLACK);
          tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
          tft_setCursor(40, 40);
          sprintf(buffer,"ADC1Reading: %d\n   ADC2Reading: %d\n   ADC3Reading: %d\n",adc_val1,adc_val2,adc_val3);
          tft_writeString(buffer);
          PT_YIELD_TIME_msec(500);
      }
    PT_END(pt);
}

// === Main  ======================================================

void main() {
  
   //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
  // the ADC ///////////////////////////////////////
        // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
    
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
        // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
        // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
        // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
        #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //want fast adc read, so auto sampling is on

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_3 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
        //
	// Define setup parameters for OpenADC10
        // use peripherial bus clock | set sample time | set ADC clock divider
        // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
        // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
        #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// define setup parameters for OpenADC10
	// set AN11 and  as analog inputs
	#define PARAM4	 ENABLE_AN9_ANA | ENABLE_AN5_ANA | ENABLE_AN11_ANA

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_AN0 |SKIP_SCAN_AN1 |SKIP_SCAN_AN2 |SKIP_SCAN_AN3 |SKIP_SCAN_AN4 |SKIP_SCAN_AN10 |SKIP_SCAN_AN6 |SKIP_SCAN_AN7 |SKIP_SCAN_AN8 

	// use ground as neg ref for A | use AN11 for input A     
	// configure to sample AN11 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF ); // configure to sample AN11 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
    PT_INIT(&pt_timer);
    PT_INIT(&pt_control);
     // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_control(&pt_control));
  }
      
  
} // main

// === end  ======================================================
