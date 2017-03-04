/* Program designed for an Uno style board to operate a servo and ESC.
 *  It is operating as a slave using I2C communication.
 *  It has on RGB LED to indicate the states of steering and throttle
 */
#include <Servo.h>
#include <Wire.h>

Servo Throttle;        
Servo FrontSteering; 

#define THROTTLE_PIN 8        // ESC output pin
#define FRONT_STEERING_PIN 9  // Steering servo control pin  

int instruction = 0;     // the information from the Master Arduino is stored here

// STEERING SERVO SETTINGS Using writeMicroseconds()
//   \\ MAX LEFT 1950 ...  // MAX RIGHT 974 
int FrontCenter = 1474;  
int SoftRight =  1224;   
int SoftLeft =   1712;   
int HardRight = 974;
int HardLeft =  1950;

// SPEED CONTROL SERVO SETTINGS Using writeMicroseconds()
// 1446 is neutral  forward is 1966... reverse is 956
int ThrottleCenter = 1446; // center = +238 OR -250
int ThrottleSpeed = 1750;
int SlowForward = 1600;
int FastForward = 1966;
int SlowReverse = 1300;
int FastReverse = 956;

/////////////////////////////////UPDATE TIMING/////////////////
int32_t lastCommandUpdate;                // the last time a command was received
int32_t LastPrintTime;                // the last time a command was received
bool newUpdate;    //we have a new update
unsigned long interval = 300;
unsigned long previousMillis = 0;
////////////////////////////LED CONTROL/////////////////
int Red_LED   = 13;
int Green_LED = 12;
int Blue_LED  = 11;
bool Red_State = LOW;
bool Green_State = LOW;
bool Blue_State = LOW;

void setup()
{
  Serial.begin(115200);
  Serial.println("It is alive");
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Throttle.attach(THROTTLE_PIN);
  FrontSteering.attach(FRONT_STEERING_PIN);
  Throttle.writeMicroseconds(ThrottleCenter);             // sets the  Throttle servo position
  FrontSteering.writeMicroseconds(FrontCenter);           // sets the  Front Steering servo position
  // set the digital pins as output:
  pinMode(Red_LED, OUTPUT);
  pinMode(Green_LED, OUTPUT);
  pinMode(Blue_LED, OUTPUT);
  // set the digital pin states
  digitalWrite(Red_LED, Red_State);
  digitalWrite(Green_LED, Green_State);
  digitalWrite(Blue_LED, Blue_State);
  Serial.println("we are here");
}

void loop()
{
  unsigned long currentMillis = millis();

  if ((newUpdate == 1) && ((lastCommandUpdate + 1200)) < millis()){
    lastCommandUpdate = millis();
    newUpdate = 0;  //reset the update flag
  }

  else {
    lastCommandUpdate = millis();
    // instruction = 0;  // stop the truck we do not have a command
  }
  // 0 = Stop, 11 = Hard Left, 12 = Hard Right, 20 = Fast Straight, 21 = Left, 22 = Right,  30 = Reverse
  switch (instruction) {
  case 0:    //stop with no fix
    //Serial.println("stop");
    Throttle.writeMicroseconds(ThrottleCenter);                  // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(FrontCenter);                  // sets the  Front Steering servo position
    digitalWrite(Red_LED, Red_State);
    digitalWrite(Green_LED, LOW);
    digitalWrite(Blue_LED, LOW);
    break;
  case 11:   //hard left
    //Serial.println("Hard Left turn");
    Throttle.writeMicroseconds(SlowForward);                    // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(HardLeft);                  // sets the  Front Steering servo position
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, LOW);
    digitalWrite(Blue_LED, Blue_State);
    break;
  case 12:   //hard right
    //Serial.println("hard right turn");
    Throttle.writeMicroseconds(SlowForward);                     // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(HardRight);                  // sets the  Front Steering servo position
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, LOW);
    digitalWrite(Blue_LED, Blue_State);
    break;
  case 20:   //fast straight
    //Serial.println("forward");
    Throttle.writeMicroseconds(ThrottleSpeed);                  // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(FrontCenter);               // sets the  Front Steering servo position
    digitalWrite(Red_LED, Red_State);
    digitalWrite(Green_LED, LOW);
    digitalWrite(Blue_LED, Red_State);
    break;
  case 21:   //fast left
    //Serial.println("Left turn");
    Throttle.writeMicroseconds(ThrottleSpeed);                 // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(SoftLeft);                  // sets the  Front Steering servo position
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, Green_State);
    digitalWrite(Blue_LED, LOW);
    break;
  case 22:   //fast right
    // Serial.println("right turn");
    Throttle.writeMicroseconds(ThrottleSpeed);                 // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(SoftRight);                  // sets the  Front Steering servo position
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, Green_State);
    digitalWrite(Blue_LED, LOW);
    break;

  case 30:    // your hand is close to the sensor
    // Serial.println("Reverse");
    Throttle.writeMicroseconds(SlowReverse);                  // sets the  Throttle servo position
    FrontSteering.writeMicroseconds(SoftRight);                  // sets the  Front Steering servo position
    break;
  }
  if ((LastPrintTime + 1000) < millis()){
    LastPrintTime = millis();
    //Serial.println(instruction);         // print the integer
  }
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (Red_State == LOW)
      Red_State = HIGH;
    else
      Red_State = LOW;

    if (Green_State == LOW)
      Green_State = HIGH;
    else
      Green_State = LOW;

    if (Blue_State == LOW)
      Blue_State = HIGH;
    else
      Blue_State = LOW;
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while(0 < Wire.available()) // loop through all but the last
  {
    instruction = Wire.read();    // receive byte as an integer
    // Serial.println(instruction);
    newUpdate = 1; //we have a new update
  }
}




