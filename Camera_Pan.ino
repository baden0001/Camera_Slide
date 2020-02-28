/*
 * Original file pulled from https://forum.arduino.cc/index.php?topic=277692.0
 * User: Robin2
   // testing a stepper motor with a Pololu A4988 driver board or equivalent

   // this version uses millis() to manage timing rather than delay()
   // and the movement is determined by a pair of momentary push switches
   // press one and it turns CW, press the other and it turns CCW
 * 
 * Added analog read and two stepper control
 * Stepper controller to allow for a camera
 * to pan across the scene.
 */

 /*Amount of time between steps:
  * These are calculations, but the real world is
  *  showing that it takes longer.
  * 1 hr timing
  * 3600 seconds / 65535 = .054 932 seconds/pulse
  *   1st speed (all lights off) = 55000
  *   analog input 0-205
  * 45 minute timing
  * 2700 seconds / 65535 = .041 199 seconds/pulse
  * 30 minute timing
  * 1800 seconds / 65535 = .027 466 seconds/pulse
  *   2nd speed (first light on) = 27500
  *   analog input 206-410
  * 15 minute timing
  * 900 seconds / 65535 = .013 733 seconds/pulse
  *   3rd speed (second light on) = 13700
  *   analog input 411-615
  * 10 minute timing
  * 600 seconds / 65535 = .009 155 seconds/pulse
  * 5 minute timing
  * 300 seconds / 65535 = .004 577 seconds/pulse
  *   4th speed (third light on) = 4600
  *   analog input 616-820
  * 1 minute timing
  * 60 seconds / 65535 = .000 915 seconds/pulse
  * 30 seconds / 65535 = .000 457 seconds/pulse
  *   5th speed (all lights on) = 350
  *   analog input 821-1023
  * 10 seconds / 65535 = .000 152 seconds/pulse
  *  5 seconds / 65535 = .000 076 seconds/pulse
  * Actual test:
  *   setting of 350 useconds lasted 45 seconds
  * calculates out to:
  *   .000 350 * 65535 / 45 = 
  */

//Sense potentiometer location
// ranges 0-1023
int sensorPinPan = A0;
//int sensorPinRotate = A0;

byte directionPinPan = 9;
byte stepPinPan = 8;

byte directionPinRotate = 11;
byte stepPinRotate = 10;

byte led1PinPan = 2;
byte led2PinPan = 3;
byte led3PinPan = 4;
byte stagePan = 1;

byte led1PinRotate = 5;
byte led2PinRotate = 6;
byte led3PinRotate = 7;
byte stageRotate = 1;

byte debug = 0;

//Total steps in rail approx. 65535
unsigned long totalCountPan = 0;
unsigned long totalCountRotate = 0;

//keep track of direction to allow for 
// automatic reversal at a given location
bool dirPan = true;
bool dirRotate = true;

unsigned long maxCountPan = 65535;
/* 15 teeth on motor
 72 teeth on camera mount
 1.8 degrees/step
  200 full steps/revolution of motor
  200 * 72 / 15 = 960 full steps per camera revolution
  960 * 32 microsteps/step = 30720 steps/camera revolution
*/
unsigned long maxCountRotate = 30720;

unsigned long curMicros;
unsigned long prevStepMicrosPan = 0;
unsigned long prevStepMicrosRotate = 0;

unsigned long minStepSpeedPan =  350;
unsigned long minStepSpeedRotate = 350;

unsigned long maxStepSpeedPan = 55000;
unsigned long maxStepSpeedRotate = 55000;

unsigned long scaledSpeedPan = 0;
unsigned long scaledSpeedRotate = 0;

void setup() {

     pinMode(directionPinPan, OUTPUT);
     pinMode(stepPinPan, OUTPUT);

     pinMode(directionPinRotate, OUTPUT);
     pinMode(stepPinRotate, OUTPUT);
     
     pinMode(led1PinPan, OUTPUT);
     pinMode(led2PinPan, OUTPUT);
     pinMode(led3PinPan, OUTPUT);
     
     pinMode(led1PinRotate, OUTPUT);
     pinMode(led2PinRotate, OUTPUT);
     pinMode(led3PinRotate, OUTPUT); 

     Serial.begin(9600);
     
}

void loop() {
     
    curMicros = micros();
    readInputs();
    actOnInputs();
   
}

void readInputs() {   
    stagePan = stageSelect(led1PinPan, led2PinPan, led3PinPan, sensorPinPan);
    setPanStageSpeed();

    //Caculate the time between rotational axis pulses to allow for the rotational axis max count to
    // be met when the linear axis max count is reached.
    scaledSpeedRotate = (maxCountPan * scaledSpeedPan) / maxCountRotate ;

}

void actOnInputs() {

    checkDirectionPan();
    checkDirectionRotate();
    singleStepPan();
    singleStepRotate();
}

void setPanStageSpeed() {
    if (stagePan == 1) {
        scaledSpeedPan = 55000;
    }
    if (stagePan == 2) {
        scaledSpeedPan = 27500;
    }
    if (stagePan == 3) {
        scaledSpeedPan = 13700;
    }
    if (stagePan == 4) {
        scaledSpeedPan = 4600;
    }
    if (stagePan == 5) {
        scaledSpeedPan = 350;
    }
}

void checkDirectionPan() {
    if (dirPan && (totalCountPan >= maxCountPan)) {
        dirPan = false;
    }

    if (!dirPan && (totalCountPan <= 0)) {
        dirPan = true;
    }
    digitalWrite(directionPinPan, dirPan);
}

void checkDirectionRotate() {
    if (dirRotate && (totalCountRotate >= maxCountRotate)) {
        dirRotate = false;
    }

    if (!dirRotate && (totalCountRotate <= 0)) {
        dirRotate = true;
    }
    digitalWrite(directionPinRotate, dirRotate);

}

void singleStepPan() {
    if (curMicros - prevStepMicrosPan >= scaledSpeedPan) {
        prevStepMicrosPan = curMicros;
        digitalWrite(stepPinPan, HIGH);
        digitalWrite(stepPinPan, LOW);
        if (!dirPan && (totalCountPan >= 0)) {
            totalCountPan -= 1;
        }
        if (dirPan) {
            totalCountPan += 1;
        }
    }
}

void singleStepRotate() {
    if (curMicros - prevStepMicrosRotate >= scaledSpeedRotate) {
        prevStepMicrosRotate = curMicros;
        digitalWrite(stepPinRotate, HIGH);
        digitalWrite(stepPinRotate, LOW);
        if (!dirRotate && (totalCountRotate >= 0)) {
            totalCountRotate -= 1;
        }
        if (dirRotate) {
            totalCountRotate += 1;
        }
    }
}

int stageSelect(byte led1, byte led2, byte led3, byte analogPin) {
    //Detect the stage of the analog pot
    int analogValue = analogRead(analogPin);
    int x = 0;
    if ((analogValue <= 205)) {
        //Stage 1
        digitalWrite(led1, LOW);
        digitalWrite(led2, LOW);
        digitalWrite(led3, LOW);
        x = 1;
        
    } else if ((analogValue > 205) && (analogValue <= 410)) {
        //Stage 2
        digitalWrite(led1, HIGH);
        digitalWrite(led2, LOW);
        digitalWrite(led3, LOW);
        x = 2;
        
    } else if ((analogValue > 410) && (analogValue <= 615)) {
        //Stage 3  
        digitalWrite(led1, LOW);
        digitalWrite(led2, HIGH);
        digitalWrite(led3, LOW); 
        x = 3;
           
    } else if ((analogValue > 615) && (analogValue <= 820)) {
        //Stage 4
        digitalWrite(led1, LOW);
        digitalWrite(led2, LOW);
        digitalWrite(led3, HIGH);
        x = 4;
        
    } else if (analogValue > 820) {
        //Stage 5
        digitalWrite(led1, HIGH);
        digitalWrite(led2, HIGH);
        digitalWrite(led3, HIGH);
        x = 5;
        
    }
    return x;
}
