//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the 
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//
//   

#include <SPI.h>  
#include <Pixy.h>
#include<Servo.h>

// This is the main Pixy object 
Pixy pixy;
Servo BaseServo;
Servo Servo1;
Servo Servo2;
Servo Gripper;

int BaseServo_pin = 8; // pin to which the base servo attaches to Arduino
int base_angle=0; // to store base servo angle
int obj_x; // to save x coordinate of the object


// variables from inverse kinematics code
//int i = 5; // just for debugging; not used in the main code
float L1=0.18;
float L2=0.19;
float x;
float height;
float y=0.03;
int initialservo1=90;
int initialservo2=90;
float costheta2;
float theta2;
float theta1;
float Qc;
float Qs;


void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...\n");
  Serial.println("hi in setup");

  pixy.init();
  BaseServo.attach(BaseServo_pin); // declare the pin for base servo
  Servo2.attach(9);
  Servo1.attach(10);
  Gripper.attach(7);
  
  BaseServo.write(0); // command the servo to start position
  delay(2000); // give a 2 seond delay for the base to stabilize
  Servo1.write(initialservo1);
  delay(500);
  Servo2.write(initialservo2);
  delay(500);
}

void loop()
{ 
  Serial.println("hi in void loop");
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
   
  // grab blocks!
  while(base_angle<180)
  {
    Serial.println("hi in while"); // for debugging
    BaseServo.write(base_angle); // write the corresponding angle to the Servo
    delay(100);
    base_angle = base_angle + 1;

    blocks = pixy.getBlocks(); 
    
  // If there are detect blocks, print them!
  if (blocks)
  {
    Serial.println("hi in blocks found"); // for debugging

    Serial.println(base_angle); // for debugging
    
    i++;
   
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      
      for (j=0; j<blocks; j++)
      {
        Serial.println(pixy.blocks[j].x);
       obj_x = pixy.blocks[j].x; // store x-cooridnate of the object
       height=pixy.blocks[j].height; // height of the object
//       Serial.println(obj_x);
       }

       // -----------------  range is being tested--------------
      if(obj_x<170 && obj_x>160){
        Serial.println("Hi, breaking now!");
        base_angle = 180; // can be removed maybe, do not take rist at this point
        break; // break out of the while loop
       
      }
    }  

 }

x=608.37/height/100+0.1; // convert the height to x-distance; formula from excel
x = x-0.01yu ; // decrease 1 cm  
Serial.println(x);
costheta2=(sq(x)+sq(y)-sq(L1)-sq(L2))/(2*L1*L2);
theta2=-acos(costheta2)*180/3.14;
Qc=(L1+L2*costheta2)*x+L2*sin(theta2*3.14/180)*y;
Qs=-L2*sin(theta2*3.14/180)*x+(L1+L2*costheta2)*y;
theta1=atan(Qs/Qc)*180/3.14;
  if (theta1<0)
  { theta1=180+theta1;
  }
//Serial.println(theta1);
Gripper.write(180);
delay(1000);

Servo1.write(30+theta1-20); 
Servo2.write(10-theta2+20);
delay(1000);
Gripper.write(180);
delay(1000);
Gripper.write(0);
delay(1000);
Servo1.write(90);
Servo2.write(90);
delay(1000);
BaseServo.write(180);
delay(1000);
Gripper.write(180);
delay(500);
Gripper.write(0);
delay(500);

// cut power from all the servos
BaseServo.detach();
Servo1.detach();
Servo2.detach();
Gripper.detach();

while(1){} // once done, stop all the activity

}

