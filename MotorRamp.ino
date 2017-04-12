//Operating under half-step (MS1: H, MS2: L, MS3: L)
//half-step: 0.45deg/pulse, 800 pulses/sec
//full-step: 0.90deg/pulse, 400 pulses/sec
//3:1 gear ratio
//1 Hz: 3750 us between pulses


//ratchet motor variables
int ratchetStep = 5;
int motorDirection = 7;
int MS1 = 1;
int MS2 = 2;
int MS3 = 3;

//state-switch motor variables
int stateSwitchStep = 8;
int stateSwitchDirection = 9;
int limitSwitchCollect = 10;
int limitSwitchRatchet = 11;

//ramp rates (change between target and initial freq over total time to ramp)
float ratchetRamp = 5;
float stateSwitchRamp = 200;

//initialize target frequencies
float ratchetFreq = 0;  // Adjust this value
float stateSwitchFreq = 5;

//ramp variables
unsigned long time = micros();
unsigned long refTime = 0;
unsigned long currentTime = time - refTime;
unsigned long prevTime = 0;
float targetFreq;
float initialFreq = 1;
float initialPulse = 3/(400*initialFreq);
float initialPulseMicros = initialPulse*1000000;
float prevPulse = initialPulse;
unsigned long dt = 0;
float rampRate;
float rdt = rampRate*dt;
float currentPulse = initialPulse/(1+(initialPulse*rdt));
float currentPulseMicros = currentPulse*1000000;
float currentFreq = 1/currentPulse;

//chatter variables
unsigned long chatterTime;
unsigned long timeDelay = 100;
int chatterRate;
bool setTime = true;

//GUI variables
int mydata;
int input;


class motor {
  int motorNum, rampRate;
  public:
    motor(int, int, int);
    void ramp();
    void resetVar();
    void changeFreq();
};

motor::motor(int a, int b, int c)  {
  motorNum = a;
  rampRate = b;
  targetFreq = c;
}

//motor-type variables
motor ratchet (ratchetStep, ratchetRamp, ratchetFreq);
motor state (stateSwitchStep, stateSwitchRamp, stateSwitchFreq);

void setup() {                
  // initialize the digital pin as an output.
  pinMode(motorDirection,OUTPUT);  //motor ratchet direction
  pinMode(ratchetStep,OUTPUT);  //motor ratchet step
  
  pinMode(stateSwitchDirection,OUTPUT); //switch state direction
  pinMode(stateSwitchStep,OUTPUT);  //switch state step
  
  pinMode(limitSwitchCollect,INPUT); //limit switch control
  pinMode(limitSwitchRatchet,INPUT);

  
  pinMode(MS1,OUTPUT);  //microsteppers 1,2,3
  pinMode(MS2,OUTPUT);
  pinMode(MS3,OUTPUT);
  
  digitalWrite(MS1,HIGH); //operating at half-steps (H,L,L)
  digitalWrite(MS2,LOW);
  digitalWrite(MS3,LOW);
  
  digitalWrite(motorDirection,LOW);
  
  Serial.begin(9600);
  Serial.setTimeout(100);
}


// the loop routine runs over and over again forever:
void loop() {
  
  if (Serial.available()>0)  {  // when data is received through the Serial port, read data
    mydata = Serial.parseInt();  // reads integers until a non-integer, non-negative sign
    if(mydata != 0)  {
      input = mydata;
      Serial.println(input);
    }
  }
  
  if (input == 101)  {
    //if mydata receives 101, motor will ramp
    if (currentFreq == 0) {
      Serial.println("Please pick a mode and frequency!");
    }
    else {
      ratchet.ramp();
    }
  }
  
  if (input == 100)  {
    //if mydata receives 100, motor will reset variables to initial conditions and stop running
    ratchet.resetVar();
    state.resetVar();
    setTime = true;
  }
  
  else if(input == 200)  {
    //if mydata receives a '200', motor moves to ratchet/separation state and rotates clockwise
    moveToRatchetState();
    digitalWrite(motorDirection,LOW);
    ratchet.resetVar();
  }
  
  else if(input == 201)  {
    //if mydata receives a '201', motor moves to concentration state rotates counter-clockwise
    moveToCollectState();
    digitalWrite(motorDirection,HIGH);
    ratchet.resetVar();
  }
  
  else if(input == 202)  {
    //if mydata receives a '202', motor moves to ratchet/separation state rotates counter-clockwise
    moveToRatchetState();
    digitalWrite(motorDirection,HIGH);
    ratchet.resetVar();
  }
  
  else if(input == 203)  {
    //if mydata receives a '200', motor moves to concentration state rotates clockwise
    moveToCollectState();
    digitalWrite(motorDirection,LOW);
    ratchet.resetVar();
  }
  
  else if(input >= 1 && input <= 15)  {
    //if mydata receives a number between 1-15, adjust target freq accordingly
    ratchet.changeFreq();
    ratchet.resetVar();
  }
  
  else if(input == 300) {
    chatter();
  }
}

//motor control functions
void motor::ramp()  {  // motor ramp function
    time = micros();
    currentTime = time - refTime;
    if (currentTime >= prevTime + currentPulseMicros)  {
      digitalWrite(motorNum,HIGH);
      delayMicroseconds(50);
      digitalWrite(motorNum,LOW);
      prevTime = currentTime;
    }
    if ((initialFreq+rdt) >= targetFreq)  {  // If current frequency greater than or equal to target frequency, just set to target frequency
      //potential change: abs(targetFreq - currentFreq) < 1
      currentFreq = targetFreq;
      currentPulse = 3/(400*currentFreq);
      currentPulseMicros = currentPulse*1000000;
      }
    else {
      dt = currentTime/10000;
      rdt = rampRate*dt/100;
      currentFreq = initialFreq + rdt;
      currentPulse = 3/(400*currentFreq);
      currentPulseMicros = currentPulse*1000000;
     }
}

void motor::resetVar()  {  // reset variables for ratchet
  time = micros();
  refTime = time;
  currentTime = time - refTime;
  prevTime = 0;
  initialFreq = 1;
  initialPulse = 3/(800*initialFreq);
  initialPulseMicros = initialPulse*1000000;
  prevPulse=initialPulse;
  dt = 0;
  rdt = rampRate*dt;
  currentPulse = initialPulse/(1+(initialPulse*rdt));
  currentPulseMicros = currentPulse*1000000;
  currentFreq = 1/currentPulse;
}

void motor::changeFreq()  {  // adjust frequency of ratcheting
  targetFreq = input;
}


//state-switch functions
void moveToCollectState()  {  // move to collect position
  digitalWrite(stateSwitchDirection,LOW);
  if(digitalRead(limitSwitchCollect)==HIGH)  {
    state.ramp();
  }
}

void moveToRatchetState()  {  // move to ratchet position
  digitalWrite(stateSwitchDirection,HIGH);
  if(digitalRead(limitSwitchRatchet)==HIGH)  {
    state.ramp();
  }
}

void chatter()  { //time-based chatter (need encoder to get angle-based)
  time = millis();
  if (setTime == true)  {
    chatterTime = time + timeDelay;
    if (digitalRead(stateSwitchDirection)==HIGH)  {
      digitalWrite(stateSwitchDirection, LOW);
    }
    else if (digitalRead(stateSwitchDirection)==LOW) {
      digitalWrite(stateSwitchDirection, HIGH);
    }
    setTime = false;
  }
  if (time < chatterTime)  {
    state.ramp();
  }
  else if (time >= chatterTime) {
    state.resetVar();
    setTime = true;
  }
}

