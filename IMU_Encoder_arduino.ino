/* Set the delay between fresh samples */

unsigned long myTime; //time that keeps updating with millis function
unsigned long sampleTimer=0; //stores last timer
unsigned long sampleInterval=20; //select the sample period in ms, should be able to handle 5ms if using only one sensor

unsigned long syncTime_millis=0;
unsigned long syncTime_micros=0;

const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pins 2 and 3

volatile long motorPosition = 0;        // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0;         // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).

int gyroX = A5;
int accelX = A3;
int accelY = A2;

void setup(void)
{
  Serial.begin(115200);
  digitalWrite(PIN_NR_ENCODER_A, HIGH);  
  digitalWrite(PIN_NR_ENCODER_B, HIGH);
  attachInterrupt(0, updateMotorPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  pinMode(accelX, INPUT) ;
  pinMode(accelY, INPUT) ;
  pinMode(gyroX, INPUT) ;
  
  byte incoming = 't';
  while(incoming != 's') //wait for 's' character to start
  {
    if(Serial.available())
    {
      incoming = Serial.read();      
    }
    //Serial.println(incoming);
  }
  syncTime_millis=millis();
  syncTime_micros=micros();
}


void loop(void)
{
  myTime=millis();
  if(myTime-sampleTimer>=sampleInterval) //enforce 20ms sampling
  {
    float Vax = analogRead(accelX)*3.3/675.18; //range is 5V. We're using 3.3V
    float Vay = analogRead(accelY)*3.3/675.18;
    float Vgx = analogRead(gyroX)*3.3/675.18;

    //float ax = (Vax - 1.5)/0.0306;
    //float ay = (Vay - 1.5)/0.0306;
    //float gx = (Vgx - 1.5)/143.27;

    float Sa = 0.033; //Change this to the accelerometer sensitivity. Is this the same value as the datasheet?
    float Sg = 0.14; //Change this to the gyroscope sensitivity. Is this the same value as the datasheet?
    //make sure you're working in m/s2 and rad/s
    float Vbias_ax = 1.65;
    float Vbias_ay = 1.65;
    float Vbias_g = 1.5;
    
    float ax = (Vax - Vbias_ax)/Sa; //m/s2
    float ay = (Vay - Vbias_ay)/Sa; //m/s2
    float gx = (Vgx - Vbias_g)/Sg; //rad/s
    
    Serial.print(millis()-syncTime_millis);

    Serial.print(F(","));
    Serial.print(ax);
    Serial.print(F(","));
    Serial.print(ay);
    Serial.print(F(","));
    Serial.print(gx);
    Serial.print(F(","));
    Serial.print(motorPosition*0.18);
    Serial.println("");
    sampleTimer=myTime;
  }
}

void updateMotorPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++;         // increase the encoder count by one
  } 
  else if (encoderStatus == 1 || encoderStatus == 7 || encoderStatus == 8 || encoderStatus == 14) {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--;         // decrease the encoder count by one
  }
}
