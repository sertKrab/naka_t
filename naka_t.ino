

// Include iBus Library
#include <IBusBM.h>

// Create iBus Object
IBusBM ibus;


// LED Connection
#define carLED 13

// Motor A Control Connections
#define pwmRight 8
#define in1Right 9
#define in2Right 10

// Motor B Control Connections
#define pwmLeft 4
#define in1Left 5
#define in2Left 6

#define rLinearMo 24
#define lLinearMo 25

// TB6612FNG Standby Pin
#define stby 6

#define INTERVAL_DIR_SPEED_CONTROL 50 //5วินาที
#define INTERVAL_MODE_CONTROL 5 //7วินาที
#define INTERVAL_MESSAGE3 11000 //11วินาที
#define INTERVAL_MESSAGE4 13000 //13วินาที

unsigned long time_dir_speed = 0;
unsigned long time_mode_control = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;


// Channel Values

int rcLeft_Right = 0; // Left - Right
int rcFor_Rev = 0; // Forward - Reverse
int rcCH3 = 0; // Acceleration
int rcCH4 = 0;
int rcCH5 = 0; // Spin Control
bool rcCH6 = 0; // Mode Control

// Motor Speed Values - Start at zero
int MotorLeft = 0;
int MotorRight = 0;

// Motor Direction Values - 0 = backward, 1 = forward
int MotorDirA = 1;
int MotorDirB = 1;

int controllerMode = 0;


// Control Motor A
void mControlRightMoter(int mspeed) {
  /*
    if (dir > 100 ) {
      // Motor backward
      Serial.print("R STOP ");
      digitalWrite(in2Right, HIGH);
      digitalWrite(in1Right, HIGH);
      analogWrite(pwmRight, 255);
      return;
    }
    // Determine direction
    if (mspeed >= 0) {
      // Motor forward
      Serial.print("R forward ");
      digitalWrite(in2Right, LOW);
      digitalWrite(in1Right, HIGH);
    } else {
      // Motor backward
      Serial.print("R backward ");
      digitalWrite(in2Right, HIGH);
      digitalWrite(in1Right, LOW);
    }
    int speedcontrol =  abs(mspeed);
    if (speedcontrol < 20) {
      speedcontrol = 0;
    }
    // Control motor
    analogWrite(pwmRight, speedcontrol);
    Serial.print(" mspeed ");
    Serial.print(speedcontrol);
  */
  // Determine direction
  if (mspeed >= 0) {
    // Motor forward
    Serial.print("R forward ");
    digitalWrite(in2Right, LOW);
    digitalWrite(in1Right, HIGH);
  } else {
    // Motor backward
    Serial.print("R backward ");
    digitalWrite(in2Right, HIGH);
    digitalWrite(in1Right, LOW);
  }
  int speedcontrol =  abs(mspeed);
  if (speedcontrol < 20) {
    speedcontrol = 0;
  }
  // Control motor
  analogWrite(pwmRight, speedcontrol);
  Serial.print(" mspeed ");
  Serial.print(speedcontrol);
}

// Control Motor Left
void mControlLeftMotor(int mspeed) {
  /* if (dir < -100 ) {
     // Motor backward
     Serial.print("L Stop ");

     digitalWrite(in1Left, HIGH);
     digitalWrite(in2Left, HIGH);
     analogWrite(pwmRight, 255);
     return;
    }

    // Determine direction
    if (mspeed >= -50) {
     // Motor forward
     digitalWrite(in1Left, LOW);
     digitalWrite(in2Left, HIGH);
     Serial.print("L forward ");
     // Serial.print(rcCH2);
    } else {
     // Motor backward
     Serial.print("L backward ");
     digitalWrite(in1Left, HIGH);
     digitalWrite(in2Left, LOW);
    }

    int speedcontrol =  abs(mspeed);
    if (speedcontrol < 20) {
     speedcontrol = 0;
    }
    // Control motor
    analogWrite(pwmLeft, speedcontrol);
    Serial.print(speedcontrol);
  */
  // Determine direction
  if (mspeed >= 0) {
    // Motor forward
    digitalWrite(in1Left, LOW);
    digitalWrite(in2Left, HIGH);
    Serial.print("L forward ");
    // Serial.print(rcCH2);
  } else {
    // Motor backward
    Serial.print("L backward ");
    digitalWrite(in1Left, HIGH);
    digitalWrite(in2Left, LOW);
  }

  int speedcontrol =  abs(mspeed);
  if (speedcontrol < 20) {
    speedcontrol = 0;
  }
  // Control motor
  analogWrite(pwmLeft, speedcontrol);
  Serial.print(speedcontrol);

}







int getControllerMode(int chMode) {
  if (chMode < 70 && chMode > -70) {
    /// Contrcoller LinearMotor
    return 1;
  } else if (chMode < -70) {
    return 2;
  }

  return 0;
}


int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

int controllerMotorBrake(int chMode, int rlDirection) {
  if (chMode != 2) {
    return 0;
  }

  if (rlDirection < 20) {
    return 0;
  }

  Serial.print("controllerMotorBrake motor  speed ");
  digitalWrite(in2Right, HIGH);
  digitalWrite(in1Right, HIGH);
  digitalWrite(in1Left, HIGH);
  digitalWrite(in2Left, HIGH);
  analogWrite(pwmLeft, rlDirection);
  analogWrite(pwmRight, rlDirection);
  Serial.print(rlDirection);
  return 1;


}

void controllerLinerMotorMode(int chMode, int rlDirection) {
  // mode 1
  if (chMode != 1) {
    digitalWrite(rLinearMo, LOW);
    digitalWrite(lLinearMo, LOW);
    return;
  }

  Serial.print("linear motor");
  Serial.print(rlDirection);

  if (rlDirection < 100 && rlDirection > -100) {
    digitalWrite(rLinearMo, LOW);
    digitalWrite(lLinearMo, LOW);
    Serial.print("stanby linear motor");
    return;

  }

  if (rlDirection > 100) {
    digitalWrite(rLinearMo, HIGH);
    digitalWrite(lLinearMo, LOW);
    Serial.print("up linear motor");
    return;

  }
  if (rlDirection < -100) {
    digitalWrite(rLinearMo, LOW);
    digitalWrite(lLinearMo, HIGH);
    Serial.print("down linear motor");
    return;

  }


}




void setup()
{

  // Start serial monitor for debugging
  Serial.begin(9600);

  // Attach iBus object to serial port
  ibus.begin(Serial1);

  // Set all the motor control pins to outputs

  pinMode(pwmRight, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(in1Right, OUTPUT);
  pinMode(in2Right, OUTPUT);
  pinMode(in1Left, OUTPUT);
  pinMode(in2Left, OUTPUT);
  pinMode(stby, OUTPUT);

  pinMode(rLinearMo, OUTPUT);
  pinMode(lLinearMo, OUTPUT);


  // Set LED pin as output
  pinMode(carLED, OUTPUT);

  // Keep motors on standby for two seconds & flash LED
  digitalWrite(stby, LOW);
  digitalWrite(carLED, HIGH);
  delay (100);
  digitalWrite(carLED, LOW);
  delay (100);
  digitalWrite(stby, HIGH);

}


void loop() {

  if (millis() - time_mode_control > INTERVAL_MODE_CONTROL) {
    time_mode_control = millis() ;
    rcCH3 = readChannel(2, -200, 200, 0);
    rcCH4 = readChannel(3, -200, 200, 0);


    int controlMode = getControllerMode(rcCH3);
    controllerLinerMotorMode(controlMode, rcCH4);
    int statusUpdate = controllerMotorBrake(controlMode, rcCH4);

    if (statusUpdate == 1)
    {
      return;
    }


  }

  if (millis() - time_dir_speed > INTERVAL_DIR_SPEED_CONTROL) {
    time_dir_speed = millis();
    rcLeft_Right = readChannel(0, -210, 210, 0);
    rcFor_Rev = readChannel(1, -210, 210, 0);
    MotorLeft = rcFor_Rev + rcLeft_Right ;
    MotorRight = rcFor_Rev - rcLeft_Right;
    mControlLeftMotor(MotorLeft);
    mControlRightMoter(MotorRight);

  }
  Serial.println(" end  ");
  // Get RC channel values  Read left right value


  //  rcCH6 = readSwitch(5, false);




}
