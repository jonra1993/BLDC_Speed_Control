
volatile uint16_t pulses=0;    //Main revolution counter
volatile uint16_t rpm=0;   //Revolution per minute
volatile uint16_t rps=0;   //Revolution per second

volatile uint16_t count2=0;    //Main revolution counter
volatile uint16_t countms=0;    //Main revolution counter

void setup() {
 
  pinMode(13,OUTPUT);
  Serial.begin(9600);  
  
//set timer1 interrupt at 1Hz>>
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR2A = 249;// = (16*10^6) / (1*32*1000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS12 for a 32 bits prescaler
  TCCR2B |= (1 << CS22)|(1 << CS20);  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), interruptCount, RISING); 
  sei();//allow interrupts
}

void loop() {
  
  //delay(200);
}


void interruptCount()
{
  pulses++;
}


ISR(TIMER2_COMPA_vect)
{
  count2++;
  if(count2==500){
    //CPU Jumps here every 1 sec exactly!
    rps=pulses;
    rpm=rps*60;
    pulses=0;
    count2=0;
    Serial.println(rpm);
  }
}
