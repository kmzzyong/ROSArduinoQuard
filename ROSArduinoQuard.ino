/*********************************************************************
 *  ROSArduinoBridge
 * 
 * A set of simple serial commands to control a differential drive
 * robot and receive back sensor and odometry data. Default 
 * configuration assumes use of an Arduino Mega + Pololu motor
 * controller shield + Robogaia Mega Encoder shield.  Edit the
 * readEncoder() and setMotorSpeed() wrapper functions if using 
 * different motor controller or encoder method.
 * 
 * Created for the Pi Robot Project: http://www.pirobot.org
 * and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
 * 
 * Authors: Patrick Goebel, James Nugen
 * 
 *********************************************************************/

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "quardiff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 200
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization */

// A pair of varibles to help parse serial commands 
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[64];
char argv2[64];
char argv3[64];
char argv4[64];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[16];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
  case READ_ENCODERS:
    Serial.print(readEncoder(ALEFT));
    Serial.print(" ");
    Serial.print(readEncoder(ARIGHT));
    Serial.print(" ");
    Serial.print(readEncoder(BLEFT));
    Serial.print(" ");
    Serial.println(readEncoder(BRIGHT));
    break;
  case READ_PIDIN:
    Serial.print(readPidIn(ALEFT));
    Serial.print(" ");
    Serial.print(readPidIn(ARIGHT));
    Serial.print(" ");
    Serial.print(readPidIn(BLEFT));
    Serial.print(" ");
    Serial.println(readPidIn(BRIGHT));
    break;
  case READ_PIDOUT:
    Serial.print(readPidOut(ALEFT));
    Serial.print(" ");
    Serial.print(readPidOut(ARIGHT));
    Serial.print(" ");
    Serial.print(readPidOut(BLEFT));
    Serial.print(" ");
    Serial.println(readPidOut(BRIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    //nextPID=millis();
    if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
      setMotorSpeeds(0, 0, 0, 0);
      resetPID();
      moving = 0;
    }
    else 
      moving = 1;
    AleftPID.TargetTicksPerFrame = arg1;
    ArightPID.TargetTicksPerFrame = arg2;
    BleftPID.TargetTicksPerFrame = arg3;
    BrightPID.TargetTicksPerFrame = arg4;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
      pid_args[i] = atoi(str);
      i++;
    }
    Aleft_Kp = pid_args[0];
    Aleft_Kd = pid_args[1];
    Aleft_Ki = pid_args[2];
    Aleft_Ko = pid_args[3];

    Aright_Kp = pid_args[4];
    Aright_Kd = pid_args[5];
    Aright_Ki = pid_args[6];
    Aright_Ko = pid_args[7];

    Bleft_Kp = pid_args[8];
    Bleft_Kd = pid_args[9];
    Bleft_Ki = pid_args[10];
    Bleft_Ko = pid_args[11];

    Bright_Kp = pid_args[12];
    Bright_Kd = pid_args[13];
    Bright_Ki = pid_args[14];
    Bright_Ko = pid_args[15];

//    Serial.print(Aleft_Kp);
//    Serial.print(" ");
//    Serial.print(Aleft_Kd);
//    Serial.print(" ");
//    Serial.print(Aleft_Ki);
//    Serial.print(" ");
//    Serial.print(Aleft_Ko);
//    Serial.print(" ");   
//    Serial.print(Aright_Kp);
//    Serial.print(" ");
//    Serial.print(Aright_Kd);
//    Serial.print(" ");
//    Serial.print(Aright_Ki);
//    Serial.print(" ");
//    Serial.print(Aright_Ko);
//    Serial.print(" ");   
//    Serial.print(Bleft_Kp);
//    Serial.print(" ");
//    Serial.print(Bleft_Kd);
//    Serial.print(" ");
//    Serial.print(Bleft_Ki);
//    Serial.print(" ");
//    Serial.print(Bleft_Ko);
//    Serial.print(" ");   
//    Serial.print(Bright_Kp);
//    Serial.print(" ");
//    Serial.print(Bright_Kd);
//    Serial.print(" ");
//    Serial.print(Bright_Ki);
//    Serial.print(" ");
//    Serial.print(Bright_Ko);
//    Serial.println(" ");    
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  initEncoders();

  initMotorController();
  resetPID();

}

/* Enter the main loop.  Read and parse input from the serial port
 and run any valid commands. Run a PID calculation at the target
 interval and check for auto-stop conditions.
 */
void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1)
        argv1[index] = NULL;
      else if (arg == 2)
        argv2[index] = NULL;
      else if (arg == 3)
        argv3[index] = NULL;
      else if (arg == 4)
        argv4[index] = NULL;

      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0)
        arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      else if (arg == 2)  {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
      else if (arg == 3)  {
        argv3[index] = NULL;
        arg = 4;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
      else if (arg == 4) {
        argv4[index] = chr;
        index++;
      }
    }
  }

  // If we are using base control, run a PID calculation at the appropriate intervals

  if (millis() > nextPID){
//    Serial.print(readPidIn(ALEFT));
//    Serial.print(" ");
//    Serial.println(readPidOut(ALEFT));
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
//    Serial.println(lastMotorCommand);
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
    resetPID();
//    Serial.println(readEncoder(ALEFT));
  }

}
