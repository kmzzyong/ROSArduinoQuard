/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
const int ALEFT_IN1 =27;
const int ALEFT_IN2 =29;
const int ALEFT_PWM =10;

const int ARIGHT_IN1 =31;
const int ARIGHT_IN2 =33;
const int ARIGHT_PWM =9;

const int BLEFT_IN1 =23;
const int BLEFT_IN2 =25;
const int BLEFT_PWM =11;

const int BRIGHT_IN1 =35;
const int BRIGHT_IN2 =37;
const int BRIGHT_PWM =8;

boolean direcAL=false;
boolean direcAR=false;
boolean direcBL=false;
boolean direcBR=false;

boolean directionWheel(int wheel)
{
  if(wheel == ALEFT)
    return direcAL;
  else if(wheel == ARIGHT)
    return direcAR;
  else if(wheel == BLEFT)
    return direcBL;
  else
    return direcBR;  
}
 
/* Wrap the motor driver initialization */
void initMotorController() {
  pinMode(ALEFT_IN1,OUTPUT);
  pinMode(ALEFT_IN2,OUTPUT);
  pinMode(ALEFT_PWM,OUTPUT);
  
  pinMode(ARIGHT_IN1,OUTPUT);
  pinMode(ARIGHT_IN2,OUTPUT);
  pinMode(ARIGHT_PWM,OUTPUT);

  pinMode(BLEFT_IN1,OUTPUT);
  pinMode(BLEFT_IN2,OUTPUT);
  pinMode(BLEFT_PWM,OUTPUT);

  pinMode(BRIGHT_IN1,OUTPUT);
  pinMode(BRIGHT_IN2,OUTPUT);
  pinMode(BRIGHT_PWM,OUTPUT);
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int wheel, int spd) {
   
  if (spd > MAX_PWM)
     spd=MAX_PWM;
     
  if (spd < -MAX_PWM)
     spd=-1*MAX_PWM;
     
  if (wheel == ALEFT){
    if (spd > 0){
      direcAL=FORWARDS;
//      Serial.println(direcAL);
      digitalWrite(ALEFT_IN1,HIGH);
      digitalWrite(ALEFT_IN2,LOW);
      analogWrite(ALEFT_PWM,spd);
    }
    else if(spd < 0){
      direcAL=BACKWARDS;
//      Serial.println(direcAL);
      digitalWrite(ALEFT_IN1,LOW);
      digitalWrite(ALEFT_IN2,HIGH);
      analogWrite(ALEFT_PWM,-spd);
    }
    else analogWrite(ALEFT_PWM,0);
  }
  else if (wheel == ARIGHT){
    if (spd > 0 ){
      direcAR=FORWARDS;
      digitalWrite(ARIGHT_IN1,LOW);
      digitalWrite(ARIGHT_IN2,HIGH);
      analogWrite(ARIGHT_PWM,spd);
    }
    else if(spd < 0){
      direcAR=BACKWARDS;
      digitalWrite(ARIGHT_IN1,HIGH);
      digitalWrite(ARIGHT_IN2,LOW);
      analogWrite(ARIGHT_PWM,-spd);
    }
    else analogWrite(ARIGHT_PWM,0);
  }
  else if (wheel == BLEFT){
    if (spd > 0){
      direcBL=FORWARDS;
      digitalWrite(BLEFT_IN1,HIGH);
      digitalWrite(BLEFT_IN2,LOW);
      analogWrite(BLEFT_PWM,spd);
    }
    else if(spd < 0){
      direcBL=BACKWARDS;
      digitalWrite(BLEFT_IN1,LOW);
      digitalWrite(BLEFT_IN2,HIGH);
      analogWrite(BLEFT_PWM,-spd);
   }
   else analogWrite(BLEFT_PWM,0);
  }
  else {
    if (spd > 0){
      direcBR=FORWARDS;
      digitalWrite(BRIGHT_IN1,LOW);
      digitalWrite(BRIGHT_IN2,HIGH);
      analogWrite(BRIGHT_PWM,spd);
    }
    else if(spd < 0){
      direcBR=BACKWARDS;
      digitalWrite(BRIGHT_IN1,HIGH);
      digitalWrite(BRIGHT_IN2,LOW);
      analogWrite(BRIGHT_PWM,-spd);
    }
    else analogWrite(BRIGHT_PWM,0);   
  }
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int aleftSpeed, int arightSpeed,int bleftSpeed,int brightSpeed) {
   setMotorSpeed(ALEFT, aleftSpeed);
   setMotorSpeed(ARIGHT, arightSpeed);
   setMotorSpeed(BLEFT, bleftSpeed);
   setMotorSpeed(BRIGHT, brightSpeed);
}
