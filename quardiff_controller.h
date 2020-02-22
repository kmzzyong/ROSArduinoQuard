/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo AleftPID, ArightPID,BleftPID, BrightPID;

/* PID Parameters */
int Aleft_Kp = 20;
int Aleft_Kd = 12;
int Aleft_Ki = 0;
int Aleft_Ko = 50;

int Aright_Kp = 20;
int Aright_Kd = 12;
int Aright_Ki = 0;
int Aright_Ko = 50;

int Bleft_Kp = 20;
int Bleft_Kd = 12;
int Bleft_Ki = 0;
int Bleft_Ko = 50;

int Bright_Kp = 20;
int Bright_Kd = 12;
int Bright_Ki = 0;
int Bright_Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   AleftPID.TargetTicksPerFrame = 0.0;
   AleftPID.Encoder = readEncoder(ALEFT);
   AleftPID.PrevEnc = AleftPID.Encoder;
   AleftPID.output = 0;
   AleftPID.PrevInput = 0;
   AleftPID.ITerm = 0;

   ArightPID.TargetTicksPerFrame = 0.0;
   ArightPID.Encoder = readEncoder(ARIGHT);
   ArightPID.PrevEnc = ArightPID.Encoder;
   ArightPID.output = 0;
   ArightPID.PrevInput = 0;
   ArightPID.ITerm = 0;

   BleftPID.TargetTicksPerFrame = 0.0;
   BleftPID.Encoder = readEncoder(BLEFT);
   BleftPID.PrevEnc = BleftPID.Encoder;
   BleftPID.output = 0;
   BleftPID.PrevInput = 0;
   BleftPID.ITerm = 0;

   BrightPID.TargetTicksPerFrame = 0.0;
   BrightPID.Encoder = readEncoder(BRIGHT);
   BrightPID.PrevEnc = BrightPID.Encoder;
   BrightPID.output = 0;
   BrightPID.PrevInput = 0;
   BrightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doAleftPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Aleft_Kp * Perror - Aleft_Kd * (input - p->PrevInput) + p->ITerm) / Aleft_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Aleft_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

void doArightPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Aright_Kp * Perror - Aright_Kd * (input - p->PrevInput) + p->ITerm) / Aright_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Aright_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

void doBleftPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Bleft_Kp * Perror - Bleft_Kd * (input - p->PrevInput) + p->ITerm) / Bleft_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Bleft_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

void doBrightPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Bright_Kp * Perror - Bright_Kd * (input - p->PrevInput) + p->ITerm) / Bright_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Bright_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}
/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  AleftPID.Encoder = readEncoder(ALEFT);
  ArightPID.Encoder = readEncoder(ARIGHT);
  BleftPID.Encoder = readEncoder(BLEFT);
  BrightPID.Encoder = readEncoder(BRIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (AleftPID.PrevInput != 0 || ArightPID.PrevInput != 0 || BleftPID.PrevInput != 0 || BrightPID.PrevInput != 0){
      resetPID();
//      Serial.println(moving); 
    }
    return;
  }

  /* Compute PID update for each motor */

  doAleftPID(&AleftPID);
  doArightPID(&ArightPID);
  doBleftPID(&BleftPID);
  doBrightPID(&BrightPID);
  
  /* Set the motor speeds accordingly */
  setMotorSpeeds(AleftPID.output, ArightPID.output, BleftPID.output, BrightPID.output);
}

long readPidIn(int wheel){

  if(wheel==ALEFT){
    return AleftPID.PrevInput;
  }
  else if(wheel==ARIGHT){
    return ArightPID.PrevInput;
  }
  else if(wheel==BLEFT){
    return BleftPID.PrevInput;
  }
  else
    return BrightPID.PrevInput;
  
}

long readPidOut(int wheel){

  if(wheel==ALEFT){
    return AleftPID.output;
  }
  else if(wheel==ARIGHT){
    return ArightPID.output;
  }
  else if(wheel==BLEFT){
    return BleftPID.output;
  }
  else
    return BrightPID.output;
 }
