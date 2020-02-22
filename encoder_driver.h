/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
#define ALEFT_ENC_PIN_A 18  
#define ALEFT_ENC_PIN_B 24  
  
#define ARIGHT_ENC_PIN_A 2  
#define ARIGHT_ENC_PIN_B 26   

#define BLEFT_ENC_PIN_A 19  
#define BLEFT_ENC_PIN_B 22  
 
#define BRIGHT_ENC_PIN_A 3  
#define BRIGHT_ENC_PIN_B 28  
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void initEncoders();
