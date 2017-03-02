// Soldering station "JAY-936"
// Author - Chernyakov Sergey S. - www.embed.com.ua

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define DIG1 PB0          /*Cathode 1-st digit indicator */
#define DIG2 PB6          /*Cathode 2-nd digit indicator*/
#define DIG3 PB7          /*Cathode 3-rd digit indicator */
#define DIG4 PD7          /*Cathode 4-th digit indicator */
#define PORT_DIG1 PORTB   /*Port cathode 1st digit indicator */
#define PORT_DIG2 PORTB   /*Port cathode 2st digit indicator*/
#define PORT_DIG3 PORTB   /*Port cathode 3st digit indicator */
#define PORT_DIG4 PORTD   /*Port cathode 4st digit indicator */
#define BUZZER PB2        /*Output buzzer*/
#define PORT_BUZZER PORTB /*Port buzzer*/
#define DDR_BUZZER DDRB   /*Register Direction buzzer*/
#define LED  PC4          /*Output LED*/ 
#define PORT_LED PORTC    /*Port LEDs*/
#define DDR_LED DDRC      /*Register LED direction*/
#define BUTTON PC3        /*Login button*/
#define PORT_BUTTON PORTC /*Port button*/
#define DDR_BUTTON DDRC   /*Register direction buttons*/
#define PIN_BUTTON PINC   /*Contact button*/
#define ENC_0 PC0         /*0-its entrance ýnkodera*/
#define ENC_1 PC1         /*1st encoder input*/
#define PORT_ENC PORTC    /*Port encoder*/
#define DDR_ENC DDRC      /*Register direction encoder inputs */
#define PIN_ENC PINC      /*Contact encoder inputs*/
#define Vdd 4930          /*Supply voltage - reference for the ADC*/
#define PI_PIP TCCR2 |= (1 << CS21) | (1 << CS20) /*Macro for buzzer beeping*/

//const unsigned char cifra [] = {0x77,0x44,0x5B,0x5E,0x6C,0x3E,0x3F,0x54,0x7F,0x7E};// An array of codes to form figures
//		7 6 5 4 3 2 1 0
//        b f a g c d e
//		0 0 0 0 1 0 0 0		0	0x08
//		0 0 1 1 1 0 1 1		1	0x3B
//		0 0 1 0 0 1 0 0		2	0x24
//		0 0 1 0 0 0 0 1		3	0x21
//                               0   1     2    3    4    5   6     7   8    9
const unsigned char cifra [] = {0x08, 0x3B, 0x24, 0x21, 0x13, 0x41, 0x40, 0x2B, 0x0, 0x1};// An array of codes to form figures
unsigned char  sot, des, edi, EncState;
unsigned int *a;//A pointer to the display value (setting encoder / actual temperature)
volatile unsigned int Temperature, time, EncData, pwm, status;
enum {work, sleep};

void podsvet (void) //Display values Function
{
    static unsigned char q;//Indoor counter Display category
	
	q++;
	switch(q) //Bust discharge Display Now
		{
    	    case 1:PORT_DIG4 &= ~_BV(DIG4);//Gasim Previous rank
			       PORTD &= 0x80; PORTD |= cifra[sot];//Set the corresponding number //edi
				   //PORTD &= 0x80; PORTD |= 0b0000011;
	               PORT_DIG1 |= _BV(DIG1);//Highlights by following discharge
			break;
			
			case 2:PORT_DIG1 &= ~_BV(DIG1);
			       PORTD &= 0x80; PORTD |= cifra[des]; //des
				   //PORTD &= 0x80; PORTD |= 0x21; //appearing half
				   //PORTD &= 0x80; PORTD |= 0b00111011;
	               PORT_DIG2 |= _BV(DIG2);
		    break;
			
			case 3:PORT_DIG2 &= ~_BV(DIG2);
			       PORTD &= 0x80; PORTD |= cifra[edi]; // sot
				   //PORTD &= 0x80; PORTD |= 0b00100100;
	               PORT_DIG3 |= _BV(DIG3);
			break;
			
			case 4:PORT_DIG3 &= ~_BV(DIG3);
			       //PORTD &= 0x80; PORTD |= 0b01010100;//In the fourth permanent illumination of the discharge letter t // cifra [tis];
				   PORTD &= 0x80; PORTD |= 0b01000111;
				   //PORTD &= 0x7F; PORTD |= 0b00111011;
	               PORT_DIG4 |= _BV(DIG4);
				   q=0;//As highlighted in the last 4-th bit counter reset
			break;
			
		    default:
		    break;
		}
}


void EncoderScan(void)//encoder processing function
{
    unsigned char New;//Partly new value encoder
 
    New = PIN_ENC & (_BV(ENC_0) | _BV(ENC_1));// Read the current position of the encoder
 
    if(New != EncState)//If the value has changed compared to the previous
    {
        switch(EncState) //Enumerating the last value encoder
	    {
	    case 2:if(New == 3) EncData++;//Depending on the increase
		       if(New == 0) EncData--;//Or reduce  
		       break;
	    case 0:if(New == 2) EncData++;
		       if(New == 1) EncData--; 
		       break;
	    case 1:if(New == 0) EncData++;
		       if(New == 3) EncData--; 
		       break;
	    case 3:if(New == 1) EncData++;
		       if(New == 2) EncData--; 
		       break;
        default:break;
	    }
		if(EncData > 360) EncData = 360;//Be careful not to go beyond the top
		if(EncData < 90) EncData = 90;//and lower limit
        EncState = New;	// We write the new value of the previous state
		a = &EncData;//Pointer to assign an address encoder current value (set temperature)
		
		TCCR0 = TCNT0 = time = 0;  //Timer 0 is stopped and reset
		TCCR0 |= (1 << CS02) | (1 << CS00); //And restart for reference 3 seconds and 15 minutes
					// clk io / 1024 prescaler
		PI_PIP;//Pipiknut for confirmation hearing
	}
}

void A_D_C (void)//ADC service function
{
    static unsigned long  Usum;
	static unsigned int i;

    ADCSRA |=(1<<ADSC);            // Run ADC
    while(!(ADCSRA & (1<<ADIF))); // Wait for the end of conversion
    ADCSRA |=(1<<ADIF);            // Reset completion flag conversion
	
	Usum += ADCW;                  //Summarize the ADC values to calculate the average (eliminating the "twitch" bits)

	if(i++ >= 300)                 //If a 300 times already measured
	{	
	    Temperature = (((Usum/300)*Vdd)/1023)/10;//Calculate the value of the temperature
		Usum=i=0;                                  //voltage and reset the counter
	}	
}

ISR (TIMER0_OVF_vect) //Timer 0 - for the reference time interval (overflow ~ 32ms)
{
    static unsigned int p,l;//Counters for "baking" and flashing LED in sleep mode
	
	if (status == work)//If the status of the work
	{
	    time++;//Increase the counter overflows
	    if(time < 91)//If less than 3 seconds after the installation of the encoder temperature
	    {
	        if(bit_is_clear(PIN_BUTTON, BUTTON)) 
		    {
		        eeprom_update_word(0x10,EncData);//when you press the button to record the value in the EEPROM
			    PI_PIP;//and pipiknut
		    }
	    }
	
	    if(time >= 91)           //If it has been 3 seconds = 32 ms x 91
	    a = &Temperature;    //Pointer assign the address of the current tip temperature
	
	    if(time >= 28125) //If it's been 15 minutes
	    {
	        TCNT0 = 0;  //Timer 0 reset
		    TCCR1B = TCCR1A = 0;//PWM stop
		    time = 0;//Reset the counter overflows
	        status = sleep;//Go to "sleep"
	    }    
	}
	
	else if (status == sleep)//If the state sleep
	{
	    if(p++ >= 500) p = 0;//Increment counter for Tweeters not go beyond
		if(p == 4) PI_PIP;//Pipikaem four times every ~ 15 seconds
	    if(p == 8) PI_PIP;
        if(p == 12) PI_PIP;  
		if(p == 16) PI_PIP;

		if(l++ >= 60) l = 0;//Increase the count for the LED does not go beyond
		if(l == 4) PORT_LED |= _BV(LED);//Morgan twice every 2 sec
		if(l == 8) PORT_LED &= ~_BV(LED);
		if(l == 12) PORT_LED |= _BV(LED);
		if(l == 16) PORT_LED &= ~_BV(LED);
	}
}
		

ISR (TIMER1_OVF_vect) //Timer 1 - for maintenance Shimano (overflow ~ 65 ms)
{
    if (Temperature < EncData)       //If the actual temperature is less than the encoder
	{
	    if(pwm++ > 1023) pwm = 1023;//Increase the heat, do not go beyond the boundaries of the PWM
	}
    if (Temperature > EncData)     //If the actual temperature exceeds the set encoder
	{
	    if(pwm-- <= 0) pwm = 0;     //Reduce the heat, do not go beyond the boundaries of the PWM
	}
	if (Temperature == EncData)     //If the actual temperature is set encoder
	{
	    pwm = OCR1A;     //We leave everything as it was
	}
	OCR1A = pwm;                    //Write a heating value in the comparison register
}

ISR (TIMER2_OVF_vect) //Timer 2 - for Tweeters
{
   static unsigned char w;//Variable defining the duration of the "pi-beeping"
	
	if(w++ < 100) PORT_BUZZER ^= _BV(BUZZER);//If the duration is not over - change the state of the output to the opposite (square wave)
	else //If the duration is over
	{
	    w=0;//Counter reset duration
		TCCR2 = 0;//Timer stop
	}
}

int main (void)
{
	DDRD = 0xFF;//outputs for display
	DDRB = 0xFF;//
	DDR_BUZZER |= _BV(BUZZER);//Output for Tweeters
	PORT_BUZZER &= ~_BV(BUZZER);//with zero
	DDR_LED |= _BV(LED);//Output for LEDs
	PORT_LED |= _BV(LED);//Highlighting to indicate power-on
	DDR_BUTTON &= ~_BV(BUTTON);//Login button
	PORT_BUTTON |= _BV(BUTTON);//a pull-up resistor
    DDR_ENC &= ~_BV(ENC_0);DDR_ENC &= ~_BV(ENC_1);	//Inputs for encoder
	PORT_ENC |= _BV(ENC_0) | _BV(ENC_1);//a pull-up resistor
	
	TIMSK |= (1 << TOIE0);//Interrupt Enable timer 0 overflow
	
	TCCR1A |= _BV(WGM10) | _BV(WGM11) | _BV(COM1A1);//10-bit Phase correct PWM, the output is connected OC1A (neinventirovanny PWM)
	TIMSK |= (1 << TOIE1);//Interrupt Enable Timer overflow 1
	TCCR1B |= _BV(CS11);// | _BV(CS10);//Start Timer 1 prescaler 64
	
	TIMSK |= (1 << TOIE2);//Interrupt Enable Timer overflow 2
	
	ADMUX |= _BV(REFS0) | _BV(MUX2) | _BV(MUX0);//Reference voltage - supply voltage, 5 th ADC channel
	ADCSRA |= _BV(ADEN) | _BV(ADSC)  | _BV(ADPS2) | _BV(ADPS1);
	EncData = eeprom_read_word(0x10);
	status = work;

	sei();
	
	for (;;)
	    {
		    if(status == work) EncoderScan();
            A_D_C ();
			sot = ((*a)%1000)/100; 
	        des = (((*a)%1000)%100)/10;
	        edi = (((*a)%1000)%100)%10;
            podsvet();
		}
}