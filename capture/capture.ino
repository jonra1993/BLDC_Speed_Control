#include <Arduino.h>

volatile unsigned int t_1=0;
volatile unsigned int t_2=0;
float period=0;
float frequency =0;

//capture Flag
volatile uint8_t flag;

void setup() {
  // initialize timer
  timer1_init();
  Serial.begin(9600);
}

void loop() {
  if (flag==2)
  {
    flag =0;
    TIFR1=(1<<ICF1);
    period= (t_2-t_1);
    
    frequency= (float) (16000000/(float) (256*period));
    Serial.print("Frecuency: ");
    Serial.print(frequency);
    Serial.println(" [Hz]");
		//timer1_start();   
  }
}

void timer1_stop()
{
  TCCR1B= 0x00;  /* Stop the timer */
  cli();
}

void timer1_start()
{
  TIFR1=(1<<ICF1);
  // timer 1 setup with a pre-scalar of 256
  TCCR1B |=(1<<CS12);
  //Input capture on rising edge
  TCCR1B|=(1<<ICES1);
  
  TCNT1=0;
  sei();
}
void timer1_init()
{
	
	// Starting timer 1 in normal mode
	TCCR1B= 0x00;
	TCCR1A= 0x00;
	// setting interrupt flag register to 0.
	TIFR1=0x00;
	// timer 1 setup with a pre-scalar of 256
	TCCR1B |=(1<<CS12);
	//Input capture on rising edge
	TCCR1B|=(1<<ICES1);
	
	// setting the timer/counter i/o locations to 0.
	TCNT1=0;
	// enabling input capture
	TIMSK1=(1<<ICIE1);
	// enabling global interrupt
	sei();
}

ISR (TIMER1_CAPT_vect)
{
	if (flag==0)
	{
		t_1=ICR1;
		TIFR1=(1<<ICF1);

	}
	if (flag==1)
	{
		t_2= ICR1;
		TIFR1=(1<<ICF1);
    //timer1_stop();		
	}
	flag ++;
}
