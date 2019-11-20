/*
 * File:        Final Project 
 *  		Capacitive sensing robot arm
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
#include <plib.h>
// need for sine function and for rounding
#include <math.h>
///////////////////////////////////
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
// some precise, fixed, short delays
// to use for cap charging time
#define NOP asm("nop");
// 1/2 microsec
#define wait20 NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// one microsec
#define wait40 wait20;wait20;

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_control,pt_ctmu;
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
          //adc_val1 = ReadADC10(0); 
          // adc_val1 is now CTMU
          adc_val2 = ReadADC10(1);
          adc_val3 = ReadADC10(2);
          tft_fillRoundRect(0,140, 240, 30, 1, ILI9340_BLACK);// x,y,w,h,radius,color
          tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
          tft_setCursor(0, 140);
          sprintf(buffer,"ADC2Reading: %d\n   ADC3Reading: %d\n",adc_val2,adc_val3);
          tft_writeString(buffer);
          PT_YIELD_TIME_msec(100);
      }
    PT_END(pt);
}

// === CTMU Thread =============================================
// set up capacitance measurement

static PT_THREAD (protothread_ctmu(struct pt *pt))
#define Vdd 3.3
#define ADC_max 1023.0
{
    PT_BEGIN(pt);
      static int raw_adc;
      static float I[4]={550e-6, 0.55e-6, 5.5e-6, 55e-6} ; // current settings in amps
      static float I_set;
      static float C;
      
      // CTMU Setup
      // Current Range IRNG 0x3 => 100x, approx 55 microamps;
      // IRNG 0x2 => 10x, approx 5.5 microamps
      // IRNG 0x1 => 1x, approx 0.55 microamps
      // IRNG 0x00 => 1000x, approx 550 microamps
      
      tft_setCursor(0, 80);
      tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
      tft_writeString("ADC     C\n");

      CTMUCONbits.ON = 1; // Turn on CTMU
      
      while(1) {
        PT_YIELD_TIME_msec(200);
        // choose a current level
        // using 55e-6 current
        I_set = I[3];
        CTMUCONbits.IRNG = 3;
        // dischrge the cap
        AcquireADC10(0); // start ADC sampling -- connects ADC sample cap to circuit
        // and discharge
        CTMUCONbits.IDISSEN = 1; // start drain of circuit
        PT_YIELD_TIME_msec(1); // wait for discharge
        CTMUCONbits.IDISSEN = 0; // End drain of circuit
        // start charging and wait 2 microsecs
        CTMUCONbits.EDG1STAT = 1;
        wait40;wait40;
        // end charging
        CTMUCONbits.EDG1STAT = 0;
        // stop samping and start conversion
        // note that in:
        //#define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_MANUAL | ADC_AUTO_SAMPLING_OFF
        // clock is manual and auto sampling is off
        ConvertADC10(0); // end sampling & start conversion       
        // wait for complete
        while (!AD1CON1bits.DONE){}; // Wait for ADC conversion       
        // read the result of channel from the idle buffer
        raw_adc =  ReadADC10(0) ;        
        // convert raw to resistance ADC reads 11 at zero resistance
        // Vref = Vdd = 3.3 ; 2 microsec charge pulse
        C = (I_set * 2e-6) / ((float)(raw_adc)/ADC_max * Vdd)  ; // c = q/v
        // draw capacitance results
        // erase
        tft_fillRoundRect(0,100, 240, 30, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        // update
        tft_setCursor(0, 100);
        tft_setTextColor(ILI9340_WHITE); tft_setTextSize(2);
        sprintf(buffer,"%d %6.2e", raw_adc, C);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // ctmu thread

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
    PT_INIT(&pt_ctmu);
     // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_control(&pt_control));
      PT_SCHEDULE(protothread_control(&pt_ctmu));
  }
      
  
} // main

// === end  ======================================================
