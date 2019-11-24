// this is the PWM Test Code
#include "config_1_3_2.h"
// threading library
//#include "pt_cornell_1_2_3.h"
#include "pt_cornell_1_3_2.h"
// yup, the expander
#include "port_expander_brl4.h"

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// need for sin function
#include <math.h>
////////////////////////////////////

// lock out timer 2 interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)

// string buffer
char buffer[60];

 
 // PWM used variables
#define clockspeed 40000000 //40MHz clock
#define PWM_Pulse_Width_Ms 20
#define PWM_Pulse_Width_Cycles 800000/256 // 20ms clock timer
#define PWM_Min  (PWM_Pulse_Width_Cycles / 40)
#define PWM_Max (PWM_Pulse_Width_Cycles / 8)
#define ADC_2_PWM (adc) adc/1023 * (PWM_Max - PWM_Min) // input adc value and output cycles to output to PWM
// end PWM used varibles


//for the game button, some useful macros
#define EnablePullupB(bits) CNPDBCLR = bits; CNPUBSET = bits;
#define DisablePullupB(bits) CNPDBCLR = bits;

//== Timer 2 interrupt handler ===========================================
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 4 ; // 10 MHz max speed for port expander!!

int dutycycle1;
int dutycycle2;


#define dmaChn 0

int dutycycle;
int i,j; //loop vars if needed  




void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    //int junk;
    mT2ClearIntFlag();
    //1KHz interrupt
    //PID logic and PWM control
    //read beam angle with the ADC
    //adcVal = ReadADC10(0);
    
    //SetDCOC3PWM(dutycycle);
    ///zz = (int)dutycyle_display
    //interpret motor control signal
    //output the motor control signal over DACB
    /*
    SPI_Mode16();
    mPORTBClearBits(BIT_4);

    mPORTBSetBits(BIT_4);
     */
    
}

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_screen, pt_serial, pt_button;
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output ;

// system 1 second interval tick
int sys_time_seconds ;


// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     // set up LED to blink
     mPORTASetBits(BIT_0 );	//Clear bits to ensure light is off.
     mPORTASetPinsDigitalOut(BIT_0 );    //Set port as output
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        // toggle the LED on the big board
        mPORTAToggleBits(BIT_0);
        // draw sys_time
        sprintf(buffer,"Time=%d", sys_time_seconds);   
               tft_writeString(buffer);

        // NEVER exit while
        
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Screen Update Thread =============================================
static PT_THREAD (protothread_screen(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
        // yield time 100 millisecond
          
        PT_YIELD_TIME_msec(200);
        //update the screen at 5 times a second
      } // END WHILE(1)
  PT_END(pt);
} // screen update thread

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  



    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // NOTE!! IF you are using the port expander THEN
    // -- clk divider must be set to 4 for 10 MHz
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 4);
  // end DAC setup
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_screen);

  // init the display
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  //srand(1);


  //--------output compare unit setup-----------
  // RPB 8 and RPB 9 are used
    // set up timer2 to generate the wave period -- SET this to servo specs
    // set timer2 to 32bit
    // T2CON = 0x0018;
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, PWM_Pulse_Width_Cycles);
    // Need ISR to compute PID controller
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    // set up compare3 for PWM mode
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0,0); //
    
    // OC1 is PPS group 1, map to RPB7
    PPSOutput(1, RPB7, OC1);
    SetDCOC1PWM(1000); //for testing

    // OC2 is in group 2  
    // allowed to output to : RPA1 RPB5 RPB1 RPB11 
    //                        RPB8 RPA9 RPC8 RPA9

    OpenOC2(OC_ON|OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,0,0);
    // GROUP 2,   output TO RPB 8
    PPSOutput(2, RPB8, OC2);

    // set both compares to min
    SetDCOC2PWM(1000);
    mT2ClearIntFlag();







  //----------------------------------------------
    
    
    //--------------------------------
    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
    tft_fillScreen(ILI9340_BLACK);
    
    


  // round-robin scheduler for threads    
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_screen(&pt_screen));
      }
  } // main

// === end  ======================================================

