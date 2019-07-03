
volatile uint16_t pulses=0;    //Main revolution counter
volatile uint16_t rps=0;   //Revolution per second

volatile long count2=10000;    //Main revolution counter
volatile float period=0;
volatile float avg_rpm =0;  
volatile float rpm[2] ={0,0};    
volatile boolean mem=false;

void setup() {
 
  pinMode(13,OUTPUT);
  Serial.begin(9600);  
  
//set timer1 interrupt at 1Hz>>
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR2A = 99;// = (16*10^6) / (1*8*10000) - 1 (must be <65536)
  //99 0.1ms
  //9 0.01ms
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS12 for a 32 bits prescaler
  TCCR2B |= (1 << CS21);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), interruptCount, RISING); 
  sei();//allow interrupts
}

void loop() {
  Serial.println((long)avg_rpm);
  //delay(200);
}


void interruptCount()
{
  delayMicroseconds(25);
	int aux=digitalRead(2);
	if (aux==1)
	{
		cli();
    period=(float)count2*0.0001f;
    avg_rpm=(120.0f/period);
    count2=0;
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);
    sei();
	}
}


ISR(TIMER2_COMPA_vect)
{
  count2++;
}
