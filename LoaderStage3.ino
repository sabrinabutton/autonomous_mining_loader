/********************************************************
  ProjectLoaderStage3

  Original by S. Button, 22/11/2022

  This code drives the MiniBot autonomously
  around the arena in the sequence defined
  in the project instructions.
*********************************************************/
#include <Servo.h>
Servo leftWheel;
Servo rightWheel;
Servo liftServo;  // Makes a servo object to control lifting servo
Servo curlServo;  // Makes a servo object to control curling servo 

// Pin Assignments
int RED = 10;           //red LED Pin
int GRN = 9;           //green LED Pin
int YLW = 5;           //yellow LED Pin
int BUTTON = 7;        //pushbutton Pin

const int LSENSOR = A1; // Left Sensor on Analog Pin 1
const int RSENSOR = A2; // Right Sensor on Analog Pin 2

int SHARP = A3;         // Sharp Sensor on Analog Pin 3

int MOTOR_R = 4;        // Right motor signal pin
int MOTOR_L = 3;        // Left motor signal pin

int sharp_value = 0; // value read by the sharp sensor

// Stop pulse
const int stopPulse = 146;    // stop speed for motors

int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value
float ldelta = 0; // left sensor delta
float rdelta = 0; // right sensor delta
float offset = 0.67; // offset so motors run the same speed relative to eachother //-1.7 for me

const int THRESHOLD = 1468; // sensor threshold in mV for black (> threshhold) vs white detection
const int STOP_DIST = 1800; // the reading on the sharp sensor which calls for stopping

// Delta values for motors
const float TURN_PRIME = 11; // delta for driving motor if turning
const float TURN_SECDY = 5; // delta for the slow motor when turning
const float STRAIGHT = 12; // delta when moving straight

// these variables are used as cooldowns and buffers to prevent erronous turns
// more explanation is provided in the main loop
int intersect_count = 0;
int obstacle_count = 0;
int just_turned = 0;
bool just_loaded = true;

// globals for pickup/dropoff
int servoPinLift = 11;     // Bucket servomotor #1 pin
int servoPinCurl = 12;

int liftAngleUp = 90;     // highest angle (lift), puts almost straight, set to 110
int liftAngleLow = 165;    // initial angle, bucket lifts off ground if too high

int curlAngleUp = 90;
int curlAngleDump = 130;

int posLift = liftAngleLow;   // if set to 180, bucket lifts robot off of ground
int posCurl = curlAngleUp;

// routine_segment tracks what part of the cycle the minibot is at.
// 0 = initial line following (or, we have dropped off)
// 1 = we have picked up
int routine_segment = 0;

// Set-up Routine
void setup() {

  // Initialize led pins as outputs.
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);

  // Initialize button and sharp sensor pins as inputs
  pinMode(BUTTON, INPUT);
  pinMode(SHARP, INPUT);

  // Initialize line following sensor pins as inputs
  pinMode(LSENSOR, INPUT);
  pinMode(RSENSOR, INPUT);

  // Set motor control pins as servo pins
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

  // Set-up servo motors
  liftServo.write(posLift);         // Servo A starting position
  liftServo.attach(servoPinLift);   // Attaches the servo to the servo object

  curlServo.write(posCurl);         // Servo A starting position
  curlServo.attach(servoPinCurl);   // Attaches the servo to the servo object

  // Initialize serial and monitor
  Serial.begin(9600);

  delay(2000);
  Serial.println("Press button to take begin...");
  do {
    digitalWrite(GRN, HIGH);
    delay(125);
    digitalWrite(GRN, LOW);
    delay(125);
    runMotors(0, 0);
  } while (digitalRead(BUTTON) == LOW);

  //bring the arm up to commence
  up();
}

// Main Routine
void loop() {

  read_sharp();
  read_location();

  if(obstacle_count >=2 && just_loaded == false){
      runMotors(0, 0);
      delay(2000);

      // if we havent picked anything up yet:
      if(routine_segment == 0){ // do pickup routine

        turn_180();

          // back away before lowering arm so that the arm gets under the parts
          runMotors(STRAIGHT, STRAIGHT);
          delay(500);
          runMotors(0,0);
          delay(1000);
        
          //lower arm
          lower();
        
          //drive into parts 
          runMotors(-STRAIGHT, -STRAIGHT - offset);
          delay(800);
          runMotors(0,0);
          delay(1000);
        
          // back away so that the arm doesnt get caught on the wall while lifting parts
          runMotors(STRAIGHT, STRAIGHT);
          delay(100);
          runMotors(0,0);
          delay(1000);
        
          // pickup
          pick_up();

          routine_segment = 1;
      } else if (routine_segment == 1){ // if we have picked something up
  
          turn_180();

        //drive into site
      
        runMotors(-STRAIGHT, -STRAIGHT);
        delay(250);
        runMotors(0,0);
        delay(1000);
      
        // drop parts
        drop();
      
      
        // back away before restarting the cycle so that the arm doesnt hit the drop bucket 
        runMotors(STRAIGHT, STRAIGHT);
        delay(500);
        runMotors(0,0);
        delay(1000);

        up();

          
        // we want to reset our routine_segment to zero
        routine_segment = 0;
      }

      just_turned = 3;

      // reset the counter for if we've encountered an obstacle
      obstacle_count = 0;
      
  } else if (intersect_count >=2 && just_turned <= 0 && just_loaded == false) {
      runMotors(0, 0);
      delay(2000); // Pause for 2 seconds
      
      turn_right();

      // reset the counter for if we've encountered an obstacle
      intersect_count = 0;
      
    } else{ //LINE FOLLOWING
      if (sharp_value >= STOP_DIST) {
        // increment the counter for encountering an obstacle
        obstacle_count+=1;
      }
      else if (intersection()) {
        // increment the counter for encountering an obstacle
        intersect_count+=1;
      }
      else {
        line_follow();

        // decrease the just turned counter "cooldown"
        just_turned -=1;
        just_loaded = false;
        
      }
    }
}

//********** Functions for Navigation Sensing *****************

void read_sharp(){
   // read the value of the sharp sensor
  sharp_value = analogRead(SHARP);
  sharp_value = map(sharp_value, 0, 1023, 0, 3300); //convert AtoD count to millivolts
}

void read_location(){
  //read the line sensor value
  lvalue = analogRead(LSENSOR);
  rvalue = analogRead(RSENSOR);

  //map the values into millivolts (assuming 3000 mV reference voltage)
  lvalue = map(lvalue, 0, 1023, 0, 3000);
  rvalue = map(rvalue, 0, 1023, 0, 3000);

}

// returns true if the minibot is off the left side of the line
bool off_left() {
  if (lvalue >= THRESHOLD && rvalue < THRESHOLD) {
    turnOnLED(RED);
    return true;
  }
  return false;
}

// returns true if the minibot is off the left side of the line
bool off_right() {
  if (rvalue >= THRESHOLD && lvalue < THRESHOLD) {
    turnOnLED(YLW);
    return true;
  }
  return false;
}

// returns true if the minibot is straddling the line (reading only white)
bool straddle() {
  if (rvalue < THRESHOLD && lvalue < THRESHOLD) {
    digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
    return true;
  }
  return false;
}

// returns true if the minibot is at an intersection (reading only black)
bool intersection() {
  if (lvalue >= THRESHOLD && rvalue >= THRESHOLD) {
    turnOnLED(GRN);
    return true;
  }
  return false;
}


//********** Functions for Arm Lifting ****************

void pick_up(){  
    
    for (posLift = liftAngleLow; posLift >= liftAngleUp; posLift--) { // Lift action
      liftServo.write(posLift);
      delay(20);
    }

    delay(1000);
    
}

void drop(){
  
    for (posCurl = curlAngleUp; posCurl <= curlAngleDump; posCurl ++) { // Dump action
      curlServo.write(posCurl);
      delay(20);
    }

    delay(1000);

    for (posCurl = curlAngleDump; posCurl  >= curlAngleUp; posCurl --) { // Curl action
      curlServo.write(posCurl);
      delay(20);
    }
    
    delay(1000);
    
}

void lower(){
      for (posLift = liftAngleUp; posLift <= liftAngleLow; posLift++) {  // Lower action
      liftServo.write(posLift);
      delay(20);
     }
     
     delay(1000);
}

void up(){
  for (posLift = liftAngleLow; posLift >= liftAngleUp; posLift--) {  // Lower action
      liftServo.write(posLift);
      delay(20);
     }
     
     delay(1000);
}


//******** Functions for movement *************

void line_follow(){

   //basic line following algorithm to eliminate repetition in main loop
    read_location();
        if (straddle()) {
          ldelta = STRAIGHT;
          rdelta = STRAIGHT;
        } else if (off_left()) {
          rdelta = TURN_PRIME;
          ldelta = TURN_SECDY;
        } else if (off_right()) {
          ldelta = TURN_PRIME;
          rdelta = TURN_SECDY;
        } 

        runMotors(ldelta, rdelta);
}

void turn_180() {
  // first back up
  ldelta = -10;
  rdelta = -10;
  runMotors(ldelta, rdelta);
  delay(600); 
  runMotors(0, 0);
  
  //spin :)
  runMotors(-10,10);

  //go for a little bit then slowly adjust to get to the line
  delay(1500);

  adjust();

  // stop
  runMotors(0, 0);
}


void turn_right() {
//   first back up
  ldelta = -7;
  rdelta = -7;
  runMotors(ldelta, rdelta);
  delay(600); 
  runMotors(0, 0);

  // stop
  delay(1000);

  runMotors(10,0);
  delay(1250);

  
  while(!straddle()){
    read_location(); 
    
    if(off_left()){
      runMotors(-8, 8);
    }
    else if(off_right()){
      runMotors(8, -8);
    } 
  }
  delay(40);

  runMotors(0, 0);
}

void adjust(){
   
  while(straddle() ){
    read_location();
    runMotors(-10,10);
  } 
  delay(40);

  
  
  // keep turning until the sensor sees the line


  runMotors(0,0);


   //straighten out based on how the minibot is oriented
   while(!straddle()){
    read_location(); 
    
    if(off_left()){
      runMotors(-8, 8);
    }
    else if(off_right()){
      runMotors(8, -8);
    } else{
      //on an intersection. SPIN!
      runMotors(-8,8);
    }
   }
   delay(40);
   runMotors(0,0);

} 


// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}

// Drive wheels
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL) * 10;  // length of pulse in microseconds
  int pulseR = (stopPulse + deltaR - offset) * 10;
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}
