// ----------------------------------------------------------LED FLASHER CODE-------------------------------------- //
#include <avr/io.h>
#include <avr/interrupt.h>

class Flasher
{
	// Class Member Variables
	// These are initialized at startup
	int ledPin;      // the number of the LED pin
	long OnTime;     // milliseconds of on-time
	long OffTime;    // milliseconds of off-time
 
	// These maintain the current state
	int ledState;             		// ledState used to set the LED
	unsigned long previousMillis;  	// will store last time LED was updated
 
  // Constructor - creates a Flasher 
  // and initializes the member variables and state
  public:
  Flasher(int pin, long on, long off)
  {
	ledPin = pin;
	pinMode(ledPin, OUTPUT);     
	  
	OnTime = on;
	OffTime = off;
	
	ledState = LOW; 
	previousMillis = 0;
  }
 
  void Update(unsigned long currentMillis)
  {
    if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
    {
    	ledState = LOW;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
    {
      ledState = HIGH;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      digitalWrite(ledPin, ledState);	  // Update the actual LED
    }
  }
};
//LED FLASHER
Flasher led(9, 1000, 1000);

// --------------------------------------------------INSERT CODE HERE---------------------------------------- //
#include <SPI.h>;
#include <Servo.h>;

//Use Inverse logic when operating relays
#define ON LOW
#define OFF HIGH


//Servo controllers, controlled through PWM
int ArmServoPin = 13;
int ClawServoPin = 12;
int BaseServoPin = 4;
int PayloadServoPin = 5;

Servo ArmServo;
Servo ClawServo;
//Servo BaseServo;
Servo PayloadServo;

//Motor controls, controlled through relays
int WinchMotor = 6;
int WireFeedMotor = 7;

int WinchStopInput = 11; //Sensor to detect when winch arm is at proper position
int PauseInput = 8; //Push button to pause all robotic functions

//Function prototypes
void Pause();
//void TurnBase();
//void TurnBaseBack();
void FeedWire();
void LiftRail();
void PayloadDoor();
void ClawOpen();
void ClawClose();
void ArmLift();
void ArmLower();
void ArmRelease();

unsigned long tstart;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //LED FLASHER
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  ArmServo.attach(ArmServoPin);  
  ClawServo.attach(ClawServoPin);
  //BaseServo.attach(BaseServoPin);
  PayloadServo.attach(PayloadServoPin);
  
  
  // PIN MODE DEFINITION
  pinMode(WinchMotor, OUTPUT);
  pinMode(WireFeedMotor, OUTPUT);
  pinMode(WinchStopInput, INPUT_PULLUP);
  pinMode(PauseInput, INPUT);
  
  //Initial Startup
  delay(200);
  Serial.println("Initial Startup");
  digitalWrite(WinchMotor, OFF);
  digitalWrite(WireFeedMotor, OFF);
  //digitalWrite(ArmServoUpPin, OFF);
  //digitalWrite(ArmServoDownPin, OFF);
  Serial.println("Initial Loop Entered");


}

void loop() {
  // put your main code here, to run repeatedly:
  
  Pause();
  
  //Flight Day Script
  ClawClose();
  ArmLift();
  //TurnBase();
  ClawOpen();
  //TurnBaseBack();
  ArmLower();
  PayloadDoor();
  LiftRail();
  FeedWire();
  Serial.println("Sequence Complete! Rocket is ready to launch.");

}

void Pause() { //Stops the function of all robotic functions and holds the logic in a loop until started again.
  Serial.println("Entering pause loop");
  delay(1000);
  
  while(digitalRead(PauseInput) == HIGH) {
    delay(200);
  }
  
  while (digitalRead(PauseInput) == LOW) {
    delay(200);
  }
  
  Serial.println("Leaving pause loop");
  delay(2000);
}
/*
void TurnBase() {
  Serial.println("Turning base");
  delay(200);
  
  int StartPosition = 120;
  int EndPosition = 30;
  int TurningSpeed = 100; //milliseconds per degree
  int dir = -1; //+1 for clockwise, -1 for counterclockwise
  
  int pos = StartPosition;
  int PrevElapsedTime;
  tstart = millis();
  while (pos != EndPosition) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      Pause();
      tstart = (millis() - PrevElapsedTime);
    }
    pos = StartPosition + (dir * ((millis() - tstart) / TurningSpeed));
    BaseServo.write(pos);
    Serial.println(pos);
  }
  Serial.println("Finished Turning Base");
  
}

void TurnBaseBack() {
  Serial.println("Turning back base");
  delay(200);
  
  int StartPosition = 30;
  int EndPosition = 120;
  int TurningSpeed = 100; //milliseconds per degree
  int dir = 1; //+1 for clockwise, -1 for counterclockwise
  
  int pos = StartPosition;
  int PrevElapsedTime;
  tstart = millis();
  while (pos != EndPosition) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      Pause();
      tstart = (millis() - PrevElapsedTime);
    }
    pos = StartPosition + (dir * ((millis() - tstart) / TurningSpeed));
    BaseServo.write(pos);
    Serial.println(pos);
  }
  Serial.println("Finished Turning Base");
}
*/
void FeedWire() {
  unsigned long feedtime = 3800; //length of time to feed wire in milliseconds
  Serial.println("Beginning to feed wire");
  delay(200);
  
  tstart = millis();
  unsigned long prevElapsedTime = 0;
  digitalWrite(WireFeedMotor, ON);
  while ((prevElapsedTime + (millis() - tstart)) < feedtime) {
    
    delay(20);
    
    if (digitalRead(PauseInput) == HIGH) {
      prevElapsedTime += (millis() - tstart);
      digitalWrite(WireFeedMotor, OFF);
      Pause();
      tstart = millis();
      digitalWrite(WireFeedMotor, ON);
    }
  }
  digitalWrite(WireFeedMotor, OFF);
  Serial.println("Finished feeding wire");
  
}

void LiftRail() {
  Serial.println("Beginning winch motor");
  delay(200);
  
  tstart = millis();
  unsigned long prevElapsedTime = 0;
  unsigned long lifttime = 20000;
  
  digitalWrite(WinchMotor, ON);
  while ((digitalRead(WinchStopInput) == OFF) && ((prevElapsedTime + (millis() - tstart)) < lifttime)) {
    delay(20);
    if (digitalRead(PauseInput) == HIGH) {
      digitalWrite(WinchMotor, OFF);
      prevElapsedTime = (millis() - tstart);
      Pause();
      digitalWrite(WinchMotor, ON);
    }
  }
  digitalWrite(WinchMotor, OFF);
  Serial.println("Lift Complete");
  
}

void PayloadDoor() {
  Serial.println("Closing Payload Door");
  delay(200);
  int StartPosition = 120;
  int EndPosition = 60;
  int TurningSpeed = 100; //milliseconds per degree
  int dir = -1;
  int pos = StartPosition;
  unsigned long PrevElapsedTime; //This is used in a slightly different way than it is in the wire feed function
  tstart = millis();
  while (pos != EndPosition) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      Pause();
      tstart = millis() - PrevElapsedTime;
    }
    pos = StartPosition + (dir * ((millis() - tstart) / TurningSpeed));
    PayloadServo.write(pos);
    //Serial.println(pos); //debug statement
  }
  Serial.println("Payload Door Closed");
  Serial.println("Returning Arm to Normal Position");
  
  tstart = millis();
  while (pos != StartPosition) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      Pause();
      tstart = millis() - PrevElapsedTime;
    }
    pos = EndPosition + ((-1)*dir*((millis() - tstart) / TurningSpeed));
    PayloadServo.write(pos);
    //Serial.println(pos); //debug statement
  }
  Serial.println("Arm in Normal Position");
}

void ClawOpen() {
  Serial.println("Opening Claw");
  delay(200);
  int OpeningSpeed = 80; //Speed is scaled to degrees. Smaller number = higher speed
  int StopSpeed = 90; //Don't change this. This is the stop command for the controller.
  unsigned long MotionTime = 3000; //Time for movement in milliseconds
  unsigned long PrevElapsedTime = 0;
  tstart = millis();
  while (MotionTime > (PrevElapsedTime + (millis() - tstart))) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      Pause();
      tstart = millis() - PrevElapsedTime;
    }
    ClawServo.write(OpeningSpeed);
  }
  ClawServo.write(StopSpeed);
  Serial.println("Claw Open");
  
}

void ClawClose() {
  Serial.println("Closing Claw");
  delay(200);
  int ClosingSpeed = 103; //Speed is scaled to degrees. Bigger number = higher speed
  int StopSpeed = 90; //Don't change this. This is the stop command for the controller.
  unsigned long MotionTime = 3000; //Time for movement in milliseconds
  unsigned long PrevElapsedTime = 0;
  tstart = millis();
  while (MotionTime > (PrevElapsedTime + (millis() - tstart))) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      Pause();
      tstart = millis() - PrevElapsedTime;
    }
    ClawServo.write(ClosingSpeed);
  }
  ClawServo.write(StopSpeed);
  Serial.println("Claw Closed");
  
}

void ArmLift() {
  unsigned long MotionTime = 3300; //Arm will remain powered at end of function. This value just keeps the robot from doing anything else while are is moving
  unsigned long PrevElapsedTime = 0;
  tstart = millis();
  Serial.println("Lifting Arm");
  //digitalWrite(ArmServoDownPin, OFF);
  //digitalWrite(ArmServoUpPin, ON);
  ArmServo.write(60);
  //Serial.println(millis() - tstart); //debug statement

  
  while (MotionTime > (PrevElapsedTime + (millis() - tstart))) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      //digitalWrite(ArmServoUpPin, OFF);
      ArmServo.write(90);
      Pause();
      tstart = millis() - PrevElapsedTime;
      //digitalWrite(ArmServoUpPin, ON);
      ArmServo.write(60);
    }
  }
  ArmServo.write(90);  
  Serial.println("Arm up");
}

void ArmLower() {
  unsigned long MotionTime = 3300; //This is slightly different from MotionTime. In this case, this is meant to start the motion before gravity takes over
  unsigned long PrevElapsedTime = 0;
  Serial.println("Lowering Arm");
  //digitalWrite(ArmServoUpPin, OFF);
  //digitalWrite(ArmServoDownPin, ON);
  ArmServo.write(120);
  tstart = millis();
  //Serial.println(tstart); //debug statement
  //Serial.println(millis() - tstart); //debug statement
  //Serial.println(millis()); //debug statement
  
  while (MotionTime > (PrevElapsedTime + (millis() - tstart))) {
    if (digitalRead(PauseInput) == HIGH) {
      PrevElapsedTime = (millis() - tstart);
      //digitalWrite(ArmServoDownPin, OFF);
      ArmServo.write(90);
      Pause();
      tstart = millis() - PrevElapsedTime;
      //digitalWrite(ArmServoDownPin, ON);
      ArmServo.write(120);
    }
  }
  ArmServo.write(90);
  //delay(750); //This is to allow gravity to lower the arm.
  Serial.println("Arm lowered"); 
}

//LED FLASHER COMPARE INTERRUPT METHOD
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  
  led.Update(currentMillis);
} 
