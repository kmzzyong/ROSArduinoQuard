/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */

volatile long aleft_enc_pos = 0L;
volatile long aright_enc_pos = 0L;
volatile long bleft_enc_pos = 0L;
volatile long bright_enc_pos = 0L;

void initEncoders(){
  
  pinMode(ALEFT_ENC_PIN_A,INPUT);  
  pinMode(ALEFT_ENC_PIN_B,INPUT);  
  attachInterrupt(5, encoderAL_ISR, CHANGE);

  pinMode(ARIGHT_ENC_PIN_A,INPUT);  
  pinMode(ARIGHT_ENC_PIN_B,INPUT);  
  attachInterrupt(0, encoderAR_ISR, CHANGE);
  
  pinMode(BLEFT_ENC_PIN_A,INPUT);  
  pinMode(BLEFT_ENC_PIN_B,INPUT);  
  attachInterrupt(4, encoderBL_ISR, CHANGE);
  
  pinMode(BRIGHT_ENC_PIN_A,INPUT);  
  pinMode(BRIGHT_ENC_PIN_B,INPUT);  
  attachInterrupt(1, encoderBR_ISR, CHANGE);
}

void encoderAL_ISR(){
  if( directionWheel(ALEFT) == BACKWARDS ){
    aleft_enc_pos--;
//    Serial.println(aleft_enc_pos);
  }else{
    aleft_enc_pos++;
//    Serial.println(aleft_enc_pos);
  }
}

void encoderAR_ISR(){
  if( directionWheel(ARIGHT) == BACKWARDS )
    aright_enc_pos--;
  else
    aright_enc_pos++;
}

void encoderBL_ISR(){
  if( directionWheel(BLEFT) == BACKWARDS )
    bleft_enc_pos--;
  else
    bleft_enc_pos++;
}

void encoderBR_ISR(){
  if( directionWheel(BRIGHT) == BACKWARDS )
    bright_enc_pos--;
  else
    bright_enc_pos++;
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
    if (i == ALEFT)
      return aleft_enc_pos;
    else if (i == ARIGHT)
      return aright_enc_pos;
    else if ( i == BLEFT)
      return bleft_enc_pos;
    else
      return bright_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
    if (i == ALEFT){
      aleft_enc_pos=0L;
      return;
    } 
    else if(i == ARIGHT){ 
      aright_enc_pos=0L;
      return;
    }
    else if(i == BLEFT){ 
      bleft_enc_pos=0L;
      return;
    }
    else{
      bright_enc_pos=0L;
      return;
    }
}


/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(ALEFT);
  resetEncoder(ARIGHT);
  resetEncoder(BLEFT);
  resetEncoder(BRIGHT);
}
