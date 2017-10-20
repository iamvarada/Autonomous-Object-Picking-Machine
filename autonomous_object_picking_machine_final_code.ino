/*
 * ME 545: Mechatronics Final Project - Spring 2017
 * Project Title: Design of An Autonomous Object Picking Robot
 * Group Members: Yii Shan and Krishna Varadarajan
 * Course Instructor: Dr. Bo Cheng
 * Department of Mechanical and Nuclear Engineering, The Pennsylvania State University, University Park, PA
 */

#include <SPI.h>  // include SPI library
#include <Pixy.h> // include Pixy library
#include<Servo.h> // include servo library

// This is the main Pixy object 
Pixy pixy; // Pixy object
Servo BaseServo; // Servo object for base servo
Servo Servo1; // Servo object for link 1
Servo Servo2; // Servo object for link 2
Servo Gripper; // Servo for objec tfor the gripper

int BaseServo_pin = 8; // pin to which the base servo attaches to Arduino
int base_angle=0; // to store base servo angle
int obj_x; // to save x coordinate of the object


// variables from inverse kinematics code
//int i = 5; // just for debugging; not used in the main code
float L1=0.18; // length  of link 1
float L2=0.19; // length  of link 2
float x;
float height; // to store height of the object
float y=0.03; // desired height of the gripper after it reaches the object
int initialservo1=90; // initial servo1 angle
int initialservo2=90; // initial servo2 angle

// variables for storing inverse kinematic variables
float costheta2; 
float theta2;
float theta1;
float Qc;
float Qs;


void setup()
{
  Serial.begin(9600); // start serial communication
  Serial.println("Starting...\n");

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
  
   
  while(base_angle<180) // keep rotating until angle reaches 180 deg
  {
//    Serial.println("hi in while"); // for debugging
    BaseServo.write(base_angle); // write the corresponding angle to the Servo
    delay(100);
    base_angle = base_angle + 1; // increment angle by 1 deg.

    blocks = pixy.getBlocks();  // store the ibjects detected

  // what to be done if object is detected!
  if (blocks)
  {
//    Serial.println("hi in blocks found"); // for debugging

//    Serial.println(base_angle); // for debugging
    
    
      sprintf(buf, "Detected %d:\n", blocks); // print the block number
      Serial.print(buf);

      // for all the blocks of object detected, store the x-coordinate of the centre and the height
      for (j=0; j<blocks; j++)
      {
        Serial.println(pixy.blocks[j].x);
       obj_x = pixy.blocks[j].x; // store x-cooridnate of the object
       height=pixy.blocks[j].height; // height of the object
       }

    // wait till the object is centered w.r.t the camera; if centred then break out of the while loop
    if(obj_x<170 && obj_x>160){
        base_angle = 180; 
        break; // break out of the while loop
       
      }
    }  

 }

x=608.37/height/100+0.1; // convert the height to x-distance; formula from excel
x = x-0.01 ; // decrease 1 cm  ; for accuracy; calculated from experiments
Serial.println(x);// for debugging

// inverse kinematics calculations; refer report for details on what these varaibles stand for
costheta2=(sq(x)+sq(y)-sq(L1)-sq(L2))/(2*L1*L2);
theta2=-acos(costheta2)*180/3.14;
Qc=(L1+L2*costheta2)*x+L2*sin(theta2*3.14/180)*y;
Qs=-L2*sin(theta2*3.14/180)*x+(L1+L2*costheta2)*y;
theta1=atan(Qs/Qc)*180/3.14;
 
  if (theta1<0)
  { theta1=180+theta1;
  }

// open gripper before servos move to desired position  
Gripper.write(180); 
delay(1000);

// move the servos to the angles calculated using inverse kinematics steps above
Servo1.write(30+theta1-20); 
Servo2.write(10-theta2+20);
delay(1000);

// close the griper now
Gripper.write(0);
delay(1000);

// stop the servos attached to links 1 and 2
Servo1.write(90);
Servo2.write(90);
delay(1000);

// rotate the base to the final position - 180 deg. here
BaseServo.write(180);
delay(1000);

// open the gripper
Gripper.write(180);
delay(500);
// object is dropped at this point; close the gripper
Gripper.write(0);
delay(500);

// cut power from all the servos
BaseServo.detach();
Servo1.detach();
Servo2.detach();
Gripper.detach();

while(1){} // once done, stop all the activity

}

