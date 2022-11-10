#include "lpc824.h"
#include "serial.h"

#define F_CPU 60000000L
#define SYSTEM_CLK 30000000L
#define DEFAULT_F 100000L // For a 10us servo resolution

/// -----------------------
#define MOT1_POS GPIO_B2
#define MOT1_NEG GPIO_B3
#define MOT2_POS GPIO_B10
#define MOT2_NEG GPIO_B11


//----------------------

volatile int ISR_pwm1=150, ISR_pwm2=150, ISR_cnt=0;

//-----------------------------------------------

// sets initial position of servos
volatile unsigned int servo_cnt = 0;
volatile unsigned int servo_upper = 130; // 210 means a pulse of 2.1ms
volatile unsigned int servo_lower = 190; // 230 means a pulse of 2.3ms



///---------------------------------------------

void InitTimer(void)
{
	SCTIMER_CTRL |= BIT2; // halt SCTimer

    // Assign a pin to the timer. So we can check the frequency.  (It should be 100kHz)
    // Assign GPIO_14 to SCT_OUT0_O
	SWM_PINASSIGN7 &= 0x00ffffff;
	SWM_PINASSIGN7 |= (14 << 24); 
	
	SYSCON_SYSAHBCLKCTRL |= BIT8; // Turn on SCTimer 
	SYSCON_PRESETCTRL |=  BIT8; // Clear the reset SCT control
	
	SCTIMER_CONFIG |= BIT0; // Unified 32 bit counter
	SCTIMER_MATCH0 = SYSTEM_CLK/DEFAULT_F; // Set delay period 
	SCTIMER_MATCHREL0 = SYSTEM_CLK/DEFAULT_F;
	SCTIMER_EV0_STATE = BIT0;  // Event 0 pushes us into state 0
	// Event 0 configuration:
	// Event occurs on match of MATCH0, new state is 1	
	SCTIMER_EV0_CTRL =  BIT12 + BIT14 + BIT15;
	// State 1 configuration
	SCTIMER_EV1_STATE = BIT1;  // Event 1 pushes us into state 1
	// Event 1 configuration
	// Event occurs on MATCH0, new state is 0
	SCTIMER_EV1_CTRL =  BIT12 + BIT14;
	// OUT0 is set by event 0
	SCTIMER_OUT0_SET = BIT0;
	// OUT1 is cleared by event 1
	SCTIMER_OUT0_CLR = BIT1;
	// Processing events 0 and 1
	SCTIMER_LIMIT_L = BIT0 + BIT1;
	// Remove halt on SCTimer
	SCTIMER_CTRL &= ~BIT2;		
		
	SCTIMER_EVEN = 0x01; //Interrupt on event 0
	NVIC_ISER0|=BIT9; // Enable SCT interrupts in NVIC
}

void Reload_SCTIMER (unsigned long Dly)
{
	SCTIMER_CTRL |= BIT2; // halt SCTimer
	SCTIMER_MATCH0 = Dly; // Set delay period 
	SCTIMER_MATCHREL0 = Dly;
	SCTIMER_COUNT = 0;
	SCTIMER_CTRL &= ~BIT2;	// Remove halt on SCTimer	
}

void STC_IRQ_Handler(void)
{
		SCTIMER_EVFLAG = 0x01; // Clear interrupt flag
	servo_cnt++;
	if(servo_cnt==2000) servo_cnt=0; // Period of servo signal is 20ms
	if(servo_cnt<=servo_upper)
	{
		GPIO_B9 = 1;
	}
	else
	{
		GPIO_B9 = 0;
	}
	
	if(servo_cnt<=servo_lower)
	{
		GPIO_B1 = 1;
	}
	else
	{
		GPIO_B1 = 0;
	}
}

void wait_1ms(void)
{
	// For SysTick info check the LPC824 manual page 317 in chapter 20.
	SYST_RVR = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SYST_CVR = 0; // load the SysTick counter
	SYST_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while((SYST_CSR & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SYST_CSR = 0x00; // Disable Systick counter
}

void wait_us(unsigned int us)

{
	// For SysTick info check the LPC824 manual page 317 in chapter 20.
	//SYST_RVR = ((F_CPU*us)/1000000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SYST_RVR = (60*us) - 1;  // set reload register, counter rolls over from zero, hence -1
	SYST_CVR = 0; // load the SysTick counter
 
	SYST_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while((SYST_CSR & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SYST_CSR = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD (GPIO_B13) // Read period from PIO0_13 (pin 3)

// GetPeriod() seems to work fine for frequencies between 400Hz and 400kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	SYST_RVR = 0xffffff;  // 24-bit counter set to check for signal present
	SYST_CVR = 0xffffff; // load the SysTick counter
	SYST_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(SYST_CSR & BIT16) return 0;
	}
	SYST_CSR = 0x00; // Disable Systick counter

	SYST_RVR = 0xffffff;  // 24-bit counter set to check for signal present
	SYST_CVR = 0xffffff; // load the SysTick counter
	SYST_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(SYST_CSR & BIT16) return 0;
	}
	SYST_CSR = 0x00; // Disable Systick counter
	
	SYST_RVR = 0xffffff;  // 24-bit counter reset
	SYST_CVR = 0xffffff; // load the SysTick counter to initial value
	SYST_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(SYST_CSR & BIT16) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(SYST_CSR & BIT16) return 0;
		}
	}
	SYST_CSR = 0x00; // Disable Systick counter

	return 0xffffff-SYST_CVR;
}

void ConfigPins(void)
{
	GPIO_DIR0 &= ~(BIT13);  // Configure PIO0_13 as input (pin 2).
	GPIO_DIR0 |= BIT14;    // check ISR rate Configure PIO0_14 as output (pin 20).
	
	
	SWM_PINENABLE0 |= BIT4; // Disable SWCLK on pin PIO0_3
	GPIO_DIR0 |= BIT3;    // Configure PIO0_3  as output (pin 7).
	
	SWM_PINENABLE0 |= BIT5; // Disable SWIO on pin PIO0_2
	GPIO_DIR0 |= BIT2;    // Configure PIO0_2  as output (pin 8).
	
	// Warning: PIO0_11 and PIO0_10 need external pull-up resistors to 3.3V (1k seems to work)
	// Check page 90 of the user manual, section: 8.4.3 Pin mode
	GPIO_DIR0 |= BIT11;   // Configure PIO0_11 as output (pin 9).
	GPIO_DIR0 |= BIT10;   // Configure PIO0_10 as output (pin 10).
	GPIO_DIR0 |= BIT15;   // Configure PIO0_15 as output (pin 11).
	GPIO_DIR0 |= BIT1;    // lower servo Configure PIO0_1  as output (pin 12).
	GPIO_DIR0 |= BIT9;    // upper servo Configure PIO0_9  as output (pin 13).
}

/* Start ADC calibration */
void ADC_Calibration(void)
{
	unsigned int saved_ADC_CTRL;

	// Follow the instructions from the user manual (21.3.4 Hardware self-calibration)
	
	//To calibrate the ADC follow these steps:
	
	//1. Save the current contents of the ADC CTRL register if different from default.	
	saved_ADC_CTRL=ADC_CTRL;
	// 2. In a single write to the ADC CTRL register, do the following to start the
	//    calibration:
	//    – Set the calibration mode bit CALMODE.
	//    – Write a divider value to the CLKDIV bit field that divides the system
	//      clock to yield an ADC clock of about 500 kHz.
	//    – Clear the LPWR bit.
	ADC_CTRL = BIT30 | ((300/5)-1); // BIT30=CALMODE, BIT10=LPWRMODE, BIT7:0=CLKDIV
	// 3. Poll the CALMODE bit until it is cleared.
	while(ADC_CTRL&BIT30);
	// Before launching a new A/D conversion, restore the contents of the CTRL
	// register or use the default values.
	ADC_CTRL=saved_ADC_CTRL;
}


void InitADC(void)
{
	// Will use pins 1 and 2 of TSSOP-20 package (PIO_23 and PIO_17) for ADC.
	// These correspond to ADC Channel 3 and 9.  Also connect the
	// VREFN pin (pin 17 of TSSOP-20) to GND, and VREFP the
	// pin (pin 17 of TSSOP-20) to VDD (3.3V).
	
	SYSCON_PDRUNCFG &= ~BIT4; // Power up the ADC
	SYSCON_SYSAHBCLKCTRL |= BIT24;// Start the ADC Clocks
	ADC_Calibration();
	ADC_SEQA_CTRL &= ~BIT31; // Ensure SEQA_ENA is disabled before making changes	
	
	ADC_CTRL =1;// Set the ADC Clock divisor
	SWM_PINENABLE0 &= ~BIT16; // Enable the ADC function on PIO_23 (ADC_3, pin 1 of TSSOP20)	
	SWM_PINENABLE0 &= ~BIT22; // Enable the ADC function on PIO_17 (ADC_9, pin 2 of TSSOP20)	
}

// WARNING: in order to use the ADC with other pins, the pins need to be configured in
// the function above.
int ReadADC(int channel)
{
	ADC_SEQA_CTRL &= ~BIT31; // Ensure SEQA_ENA is disabled before making changes
	ADC_SEQA_CTRL &= 0xfffff000; // Deselect all previously selected channels	
	ADC_SEQA_CTRL |= (1<<channel); // Select Channel	
	ADC_SEQA_CTRL |= BIT31 + BIT18; // Set SEQA and Trigger polarity bits
	ADC_SEQA_CTRL |= BIT26; // Start a conversion:
	while( (ADC_SEQA_GDAT & BIT31)==0); // Wait for data valid
	return ( (ADC_SEQA_GDAT >> 4) & 0xfff);
}


//-------------------------------------------------
//motor functions:
void stop(void) //stops motors when called
{
    MOT1_POS=0;
    MOT1_NEG=0;
    MOT2_POS=0;
    MOT2_NEG=0;
}

void forward(void) //drives forward when called
{
    MOT1_POS=1;
    MOT1_NEG=0;
    MOT2_POS=1;
    MOT2_NEG=0;
}

void backward(void) //drives backward when called
{
    MOT1_POS=0;
    MOT1_NEG=1;
    MOT2_POS=0;
    MOT2_NEG=1;
}

void clockwise(void) //turns clockwise when called
{
    MOT1_POS=0;
    MOT1_NEG=1;
    MOT2_POS=1;
    MOT2_NEG=0;
}

void counter_clockwise(void) //turns counter clockwise when called
{
    MOT1_POS=1;
    MOT1_NEG=0;
    MOT2_POS=0;
    MOT2_NEG=1;
}

void move_arm(void)
{
    #define SIZE 3
    int upper_sequence[SIZE] = {210, 150, 210}; // movement sequence for upper servo
    int lower_sequence[SIZE] = {230, 230, 80}; // movement sequence for lower servo
    int i;
    initUART(115200);
    enable_interrupts();
    

   	
    for(i = 0; i < SIZE; i++) {
	servo_upper = upper_sequence[i];
   	servo_lower = lower_sequence[i];
	waitms(500);
     }	 		
}

//------------------------------------------------

// In order to keep this as nimble as possible, avoid
// using floating point or printf on any of its forms!
void main(void)
{


//-------------------------------
	#define SIZE 3
	int upper_sequence[SIZE] = {190, 260, 170}; // movement sequence for upper servo
	int lower_sequence[SIZE] = {130, 130, 90}; // movement sequence for lower servo
	int i = 0;
	
//-------------------------------
	int j, v1, v2;
	int coins=0;
	int period;
	long int count, f;
	unsigned char LED_toggle=0;
	
	
	ConfigPins();	
	initUART(115200);
	InitADC();
	InitTimer();
	enable_interrupts();
	
	waitms(500); // Give PuTTY time to start	
	eputs("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	eputs("\r\nLPC824 multi I/O example.\r\n");
	eputs("Measures the voltage at channels 3 and 9 (pins 1 and 2 of TSSOP20 package)\r\n");
	eputs("Measures period on PIO0_13 (pin 3 of TSSOP20 package)\r\n");
	eputs("Toggles PIO0_3, PIO0_2, PIO0_11, PIO0_10, PIO0_15, PIO0_1, PIO0_9 (pins 7, 8, 9, 10, 11 of TSSOP20 package)\r\n");
	eputs("Generates servo PWMs on PIO0_1, PIO0_9 (pins 12, 13 of TSSOP20 package)\r\n");
	eputs("WARNING: PIO0_11 (pin 9) and PIO0_10 (pin 10) need external pull-up resistors to 3.3V (1k seems to work)\r\n");
	
	while(coins <20)	
	{
		forward();
		waitms(100);
		period = GetPeriod(35);
		PrintNumber(period, 10, 7);
		eputs("\r\n");
		/*
		if(period < 36900)
		{
			backward();
			waitms(300);
			stop();
			// normal resting position
			servo_upper = upper_sequence[0];
			waitms(500);
			servo_lower = lower_sequence[0];
				
			waitms(500);
			waitms(500);
				
			servo_upper = upper_sequence[1];
			waitms(500);
			servo_lower = lower_sequence[1];
				
			GPIO_B15 = 1; // pin 11, turns on electromagnet
			eputs("working\r\n");
				
			waitms(500);
			counter_clockwise();
			waitms(400);
			clockwise();
			waitms(400);
			stop();
			waitms(1000);
				
			servo_upper = upper_sequence[2];
			waitms(500);
			servo_lower = lower_sequence[2];
				
			waitms(500);
			waitms(500);
			waitms(500);
				
			coins++;
			eputs("coin!\r\n");
			GPIO_B15 = 0;
			// function for moving to normal position
				
			waitms(500);
			waitms(500);
			waitms(500);
				
			// normal resting position
			servo_upper = upper_sequence[0];
			servo_lower = lower_sequence[0];
			waitms(2000);
		}
		*/
		waitms(100);
		/*
		forward();
		//waitms(400);
		*/
		j=ReadADC(3);
		v1=(j*33000)/0xfff;
		eputs("ADC[3]=0x");
		PrintNumber(j, 16, 4);
		eputs(", ");
		PrintNumber(v1/10000, 10, 1);
		eputc('.');
		PrintNumber(v1%10000, 10, 4);
		eputs("V ");;
		
		j=ReadADC(9);
		v2=(j*33000)/0xfff;
		eputs("ADC[9]=0x");
		PrintNumber(j, 16, 4);
		eputs(", ");
		PrintNumber(v2/10000, 10, 1);
		eputc('.');
		PrintNumber(v2%10000, 10, 4);
		eputs("V ");;
		/*
		if(v2 > 7000)
		{
			eputs("turn around\r\n");
			backward();
			waitms(1000);
			counter_clockwise();
			waitms(500);
		}
		*/
	//eputs("Coins");
		//waitms(500);
		
	///function go forwards
	/*
	if (v2/10000 > 1.1) {
		eputs("Test");
		backward();
    	waitms(500);
    	clockwise();
    	waitms(2000);
    	stop();
	}
	
	if (v1/10000 > 1.1) {
	eputs("Test");
		backward();
    	waitms(500);
    	clockwise();
    	waitms(2000);
    	stop();
	}
	
	for(int counter=0;counter<100;counter++){
		GPIO_B14 = 1;	
		count=GetPeriod(35);
		GPIO_B14 = 0;
	
		
		if(count>0)
		{
		eputs("Coins");
			f=(F_CPU*35L)/count;
			eputs("f=");
			PrintNumber(f, 10, 7);
			eputs("Hz, count=");
			PrintNumber(count, 10, 6);
			eputs("          \r");
			//eputs("TESTTEST");
			//waitms (500);
	 		//	GPIO_B3 = 1 ;
			//eputs("GPIO_B3=1");
			//waitms (500);
		//	waitms (500);
			GPIO_B3 =0 ; // PIN 7
			//waitms (500);
			//waitms (500);
				
			// function for picking up a coin
				
			// resting position
		}
			//stop();
			if (f > 23050){
				eputs("Pickup");
				stop();
				waitms(100);
				backward();
		      	waitms(200);
		      	 stop();
			
				// normal resting position
				servo_upper = upper_sequence[0];
				waitms(500);
				servo_lower = lower_sequence[0];
				
				waitms(500);
				waitms(500);
				
				servo_upper = upper_sequence[1];
				waitms(500);
				servo_lower = lower_sequence[1];
				
				GPIO_B15 = 1; // pin 11, turns on electromagnet
				eputs("working");
				
				waitms(500);
				waitms(500);
				waitms(500);
				
				servo_upper = upper_sequence[2];
				waitms(500);
				servo_lower = lower_sequence[2];
				
				waitms(500);
				waitms(500);
				waitms(500);
				
				coins++;
				GPIO_B15 = 0;
				// function for moving to normal position
				
				waitms(500);
				waitms(500);
				waitms(500);
				
				// normal resting position
				servo_upper = upper_sequence[0];
				servo_lower = lower_sequence[0];
				
				}
			
			//waitms(300);
			
		}
		*/
	/*else
	{
		eputs("NO SIGNAL                     \r");
	}*/
	/*
	forward();
	
	// Now toggle the pins on/off to see if they are working.
	// First turn all off:
	//	GPIO_B3=0;
		GPIO_B2=0;
		GPIO_B11=0;
		GPIO_B10=0;
		GPIO_B15=0;
		GPIO_B1=0;
		GPIO_B9=0;
		// Now turn on one of outputs per cycle to check
		switch (LED_toggle++)
		{
			case 0:
				//GPIO_B3=1;
				break;
			case 1:
		//		GPIO_B2=1;
				break;
			case 2:
		//		GPIO_B11=1;
				break;
			case 3:
		//		GPIO_B10=1;
				break;
			case 4:
		//		GPIO_B15=1;
				break;
			default:
				break;
		}
	//if(LED_toggle>4) 
		//LED_toggle=0;
		
		waitms(200);	
	}
	forward();
	*/
	}
}

