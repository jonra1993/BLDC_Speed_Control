
volatile uint16_t count=0;    //Main revolution counter
volatile uint16_t rpm=0;   //Revolution per minute
volatile uint16_t rps=0;   //Revolution per second

void setup() {
 
  Serial.begin(9600);  
  
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), interruptCount, RISING); 
  sei();//allow interrupts
}

void loop() {
  Serial.println(rpm);
  delay(200);
}


void interruptCount()
{
  count++;
}


ISR(TIMER1_COMPA_vect)
{
   //CPU Jumps here every 1 sec exactly!
   rps=count;
   rpm=rps*60;
   count=0;
}
