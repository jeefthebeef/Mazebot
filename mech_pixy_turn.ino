//mechatronics lab 4 code - Jeffrey Xing
#include <Pixy2I2C.h>
Pixy2I2C pixy; //instantiate I2C pixy (5V, GND, SDL, SCA)
//motor pin defs
#define dir_A   12 //motor shield channel A direction
#define spd_A   3  //channel A speed 
#define brk_A   9  //channel A brake 
#define dir_B   13 //motor shield channel B direction 
#define spd_B   11 //channel B speed  
#define brk_B   8  //channel B brake 
//reserved motor pins
#define curr_A  A0 //channel A current sensing
#define curr_B  A1 //channel B current sensing
//sensor pin defs
#define IR_L    A2 //left IR sensor
#define IR_R    A3 //right IR sensor
#define ping    7  //front ping sensor
//predef delays
long leftTurnDelay = 1200;
long rightTurnDelay = 1200;
long revDelay = 400;
long adjDelay = 80;
float wall_v = 2.8; //threshold IR voltage for wall adjustment
int movespd = 160;   //general movement speed of motor

void setup() {
    Serial.begin(115200);     //pixy baud rate
    pixy.init();              //setup pixy
    pinMode(spd_A, OUTPUT);   //motor control outputs
    pinMode(brk_A, OUTPUT);
    pinMode(dir_A, OUTPUT);
    pinMode(spd_B, OUTPUT);
    pinMode(brk_B, OUTPUT);
    pinMode(dir_B, OUTPUT);
}

void pause() {
  //pause for 0.2s
  analogWrite(spd_A, 0);
  analogWrite(spd_B, 0);
  delay(200);
}

//movement mode setting
void setMovement(bool dirL, bool enL, int spdL, bool dirR, bool enR, int spdR) {
  //channel A motor on right side
  digitalWrite(dir_A, dirR);
  digitalWrite(brk_A, enR);
  analogWrite(spd_A, spdR);
  //channel B motor on left side
  digitalWrite(dir_B, dirL);
  digitalWrite(brk_B, enL);
  analogWrite(spd_B, spdL);
}

//directional motor setup
void forward() {
  //move forward 
  setMovement(HIGH, LOW, movespd, HIGH, LOW, movespd);
}
void reverse() {
  //move backward 
  setMovement(LOW, LOW, movespd, LOW, LOW, movespd);
}
void goRight() {
  //turn right 
  setMovement(LOW, LOW, movespd, HIGH, LOW, movespd);
}
void goLeft() {
  //turn left 
  setMovement(HIGH, LOW, movespd, LOW, LOW, movespd);
}
void brake() {
  //stop moving
  digitalWrite(brk_A, HIGH);
  digitalWrite(brk_B, HIGH);
  delay(100);
}

//display motion data to serial monitor
void showSerial(String motion, float distance) {
  //displays motion and distance
  Serial.print(motion);
  Serial.print(" , ");
  Serial.print(distance);
  Serial.println(" cm");
}

//measure front distance using us ping sensor
float frontDist() {
  unsigned long pulsedur = 0;
  pinMode(ping, OUTPUT);    //set to output for emitting pulse
  digitalWrite(ping, LOW);
  delayMicroseconds(2);
  digitalWrite(ping, HIGH); //send 5us pulse
  delayMicroseconds(5);
  digitalWrite(ping, LOW);
  pinMode(ping, INPUT);     //set to input for reading bouneback pulse
  pulsedur = pulseIn(ping, HIGH);
  return pulsedur/29.15/2;  //speed of sound gives 29.15us/cm, div/2 for bounce
}

//adjust to veer left
void adjLeft(){             
  setMovement(HIGH, LOW, movespd-25, HIGH, LOW, movespd+25); //right motor speeds up
  delay(adjDelay);
  //readjust towards forward position by veering right
  //setMovement(HIGH, LOW, movespd+25, HIGH, LOW, movespd);
  //delay(adjDelay); // delay to prevent zig zag motion
}

//adjust to veer right
void adjRight(){            
  setMovement(HIGH, LOW, movespd+25, HIGH, LOW, movespd+25); //left motor speeds up
  delay(adjDelay);
  //readjust towards forward position by veering left
  //setMovement(HIGH, LOW, movespd, HIGH, LOW, movespd+25);
  //delay(adjDelay); // delay to prevent zig zag motion
}

void loop() {
  pixy.ccc.getBlocks(); //return pixy view data
  //float dist = 1368.3/pixy.ccc.blocks[0].m_width; //conversion from px to cm
  if (frontDist() < 7) { //read pattern signatures only when closer than 7cm
    if (pixy.ccc.blocks[0].m_signature == 1) {
      //turn right when pixy sees yellow, half throttle
      pause();
      goRight();
      showSerial("Right", frontDist());
      delay(rightTurnDelay);
    } else if (pixy.ccc.blocks[0].m_signature == 2) {
      //turn left when pixy sees orange
      pause();
      goLeft();
      showSerial("Left", frontDist());
      delay(leftTurnDelay);
      brake();
    } else if (pixy.ccc.blocks[0].m_signature == 3) {
      //u turn when pixy sees pink, turns left but with twice turn delay
      pause();
      goLeft();
      showSerial("U-turn", frontDist());
      delay(3*leftTurnDelay);
      brake();
    } else {
      //move back when pixy is within 10cm and sees nothing
      pause();
      reverse();
      showSerial("Reverse", frontDist());
      //delay(revDelay);
      //brake();
    }
  } else { //move forward if distance more than 10cm
    int x = pixy.ccc.blocks[0].m_x;
    if (x < 153) {
      adjLeft();
    }
    else if (x > 163) {
      adjRight();
    }
    else {
      float left_v = analogRead(IR_L)*0.0048828125;  //left IR voltage
      float right_v = analogRead(IR_R)*0.0048828125; //right IR voltage
      forward();
      showSerial("Forward", frontDist());
      if(left_v > wall_v) {  //veer right if left wall too close
        adjRight();
        Serial.println("Left wall too close");
      }
      if(right_v > wall_v) { //veer left if right wall too close
        adjLeft();
        Serial.println("Right wall too close");
      }
    }
  }
  delay(100); //sample data at 10Hz
}
