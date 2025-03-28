#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

/*
   BNO055 Gyroscope Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground


   Arm Servo Connections
   ===========
   Grey to ~9
   White to 5V
   Black to GND


   Rover Servo Connections
   ===========
   Brown to GND
   Orange to ~10/~11


*/

// BNO055
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

// Arm Servo
Servo armServo;


// Rover Servos
#define ForwardFast 120
#define ForwardSlow 110
#define Stop 90
#define ReverseSlow 70
#define ReverseFast 60
// Time in ms for the Rover to turn. Must be fine tuned
#define TurnTimer 750
Servo roverS1;
Servo roverS2;



void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


  /* BNO055 initalize */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


  /* Arm servo initalize */
  armServo.attach(9); 
  armServo.write(90);
  delay(2000);

  /* Rover servo initalize*/
  roverS1.attach(10);
  roverS2.attach(11);

  // wait for button press 
  


}

void loop() {
  // put your main code here, to run repeatedly:
  //dumpArm();
  //delay(2000);

  if(Serial.available() > 0) {
    String msg = Serial.readString(); // reads in from python
    // perform actions
    if (msg == "forward") {
      roverForward();
    }
    else if (msg == "stop") {
      roverHalt();
    }
    else if (msg == "dump") {
      dumpArm();

    }
    
  }
  //delay(2000);
  //roverForward();
  //delay(2000);
  //roverHalt();
  //delay(100000);

}




void dumpArm() {

  int pos;  
  for (pos = 90; pos <= 40; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    armServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  delay(500);

  for (pos = 40; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    armServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void roverTurnRight() {
  // Stop, and Turn
  roverS1.write(ForwardFast);
  roverS2.write(ReverseFast);
  delay(TurnTimer);
}
void roverTurnLeft() {
  // Stop, and Turn
  roverS1.write(ForwardFast);
  roverS2.write(ReverseFast);
  delay(TurnTimer);
}

void roverForward() {
  roverS1.write(ForwardFast);
  roverS2.write(ForwardFast);
}

void roverHalt () {
  roverS1.write(Stop);
  roverS2.write(Stop);
}
