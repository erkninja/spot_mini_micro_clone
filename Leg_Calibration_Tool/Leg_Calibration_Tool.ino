// If you enable the TINKERCAD define, you can use wokwi.com to test
// your code before running it on your Arduino/Teensy. Some things 
// obviously won't work like the servo controller or saving the EEPROM
// between sessions, so this flag lets you hack around it and fake
// some stuff. 
//#define TINKERCAD

#include <EEPROM.h>
#ifndef TINKERCAD
#include <Adafruit_PWMServoDriver.h>
#endif

#include "NovaServos.h"

int incomingByte = 0; // for incoming serial data
int menu_option = 0;  // keeps track of which menu option is selected
int leg_select = 0; // keeps track of the leg that is being calibrated 
int joint_select = 0; // keeps track of the joint/servo to calibrate

// menu switch
// TODO: Make me one giant enum. We don't care about values on these,
// just names so collisions aren't an issue. 
#define NONE 0
enum motor_config_state{HOME=1, MIN, MAX, DONE};
enum menus{LEG_CAL=1, SWEEP, CAL_OUTPUT, EXCEL_OPTION};
enum cal_menus{TEST_AGAIN=1, SAVE_VAL, EXIT_NO_SAVE};

// other global variables
#define SPD 5
#ifndef TINKERCAD
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif
int servoCalFlags = 0;

// function prototypes
int  motor_cal(int menu_option, int prev_motor_pwm);
int  getNume(void);
void saveToEEPROM(int address, int offset, float value);
bool getServoCal(int leg, int joint);
void setServoCal(int leg, int joint, bool calibrated);
int  getServo(int leg, int joint);
void setServo(int leg, int joint, int pos);
void sweepServo(int leg, int joint, int pos);

// EEPROM Address
// Pick an EEPROM address high enough that it probably won't be in use. 
// You can change this freely, but make sure you have enough room to fit
// both arrays. 
// servoHome can store 16 servo values maximum
// servoLimit can store 16x2 servo values maximum
// The flag variable is used to limit writes to the EEPROM and to give us 
// hints about what servo values have already been calibrated. 
// Sneak the flag in right below our arrays. 
#define eeAddress_home 100
#define eeAddress_limit eeAddress_home + 16*sizeof(float)
// TODO: Make sure we use our calibration flag
// TODO: flag might need to be a byte[3] array so we can pick out home/min/max
#define eeAddress_flag eeAddress_home - 16


void
setup(void)
{
  Serial.begin(19200); // opens serial port, sets data rate to a few bps

  // Fetch both our arrays and our flags out of EEPROM. 
  EEPROM.get(eeAddress_flag, servoCalFlags);
  EEPROM.get(eeAddress_home, servoHome);
  EEPROM.get(eeAddress_limit, servoLimit);
#ifdef TINKERCAD
// Set some test data in our servo arrays variables. 
// Wokwi.com doesn't save EEPROM between test runs. 
float myServoHome[TOTAL_SERVOS] = {
  368, 301, 435,
  355, 422, 355,
  348, 344, 396,
  364, 341, 306,
};
memcpy(servoHome, myServoHome, sizeof(servoHome));

float myServoLimit[TOTAL_SERVOS][2] = {
  {330, 450}, {226, 466}, {322, 548},
  {0, 0}, {0, 0}, {0, 0},
  {0, 0}, {0, 0}, {0, 0},
  {0, 0}, {0, 0}, {0, 0},
};
memcpy(servoLimit, myServoLimit, sizeof(servoLimit));
#endif

  // Woof woof beep boop. 
  Serial.println("*********************************");
  Serial.println("* Nova SM3 Leg Calibration Tool *");
  Serial.println("*       ^..^      /             *");
  Serial.println("*       /_/\\_____/              *");
  Serial.println("*          /\\   /\\              *");
  Serial.println("*         /  \\ /  \\             *");
  Serial.println("*********************************");
  Serial.println("\n");

#ifndef TINKERCAD
  // We don't have simulated servo controllers. 
  pwm.begin();
  pwm.setOscillatorFrequency(25000000); // 25_000_000
  pwm.setPWMFreq(60);
#endif
}

void
loop(void)
{
  byte my_joint = 0; // joint iteration/selection temp variable

  //Top menu
  MENU_START:
  Serial.println("-------------------------------------");
  Serial.println("1. Single Motor Calibration");
  Serial.println("2. Leg Sweep");
  Serial.println("3. Print Calibration Values");
  Serial.println("4. Automatic Calibration Method");
  Serial.println("-------------------------------------");
  Serial.println("Selection Option: ");

  menu_option = getMenuNum();
  Serial.print("\n");
  Serial.print("\n");


  switch (menu_option) {
  case LEG_CAL:
      //Selecting leg
      LEG_PICK:
      Serial.println("-------------------------------------");
      Serial.println("Left       Right");
      Serial.println(" [2]--Head--[1]");
      Serial.println("    |     |   ");
      Serial.println("    |     |   ");
      Serial.println(" [4]--Butt--[3]");
      Serial.println("-------------------------------------");
      Serial.println("Select Leg (1-4): ");
      leg_select = getMenuNum();
      Serial.print("\n");
      Serial.print("\n");

      if(leg_select < 0 || leg_select > TOTAL_LEGS){
        Serial.println("Invalid input");
        Serial.print("\n");
        goto LEG_PICK;
      }

      //Selecting motor
      JOINT_PICK:
      Serial.println("-------------------------------------");
      Serial.println("     Femur  Coax");
      Serial.println("      [2]---[1]");
      Serial.println("       |");
      Serial.println("       |");
      Serial.println("Tibia [3]");
      Serial.println("       |");
      Serial.println("       U");
      Serial.println("-------------------------------------");
      Serial.println("Select Motor (1-3): ");
      joint_select = getMenuNum();
      Serial.print("\n");
      Serial.print("\n");

      if(joint_select < 0 || joint_select > 3){
        Serial.println("Invalid input");
        Serial.print("\n");
        goto JOINT_PICK;
      }

      //Selecting calibration variable
      CAL_TYPE:
      Serial.println("-------------------------------------");
      Serial.print("Leg: ");
      Serial.print(leg_select, DEC);
      Serial.print(" \t ");
      Serial.print("Joint: ");
      Serial.print(joint_select, DEC);
      Serial.print("\n");
      Serial.println("-------------------------------------");

      Serial.print("\n");
      Serial.println("1. Home");
      Serial.println("2. Minimum Movement");
      Serial.println("3. Maximum Movement");
      Serial.println("4. Done");
      menu_option = getMenuNum();
      Serial.print("\n");
      Serial.print("\n");

      // The Home array is 1D so we can index directly in to the EEPROM location
      // with just my_joint. The Limits array is 2D however so we need to 
      // multiply by 2 and then add 0 or 1 to get to the right min/max
      // position offset in memory. 
      my_joint = servoLeg[leg_select-1][joint_select-1];
      switch (menu_option){
      case HOME:
        servoHome[my_joint] = motor_cal(menu_option, servoHome[my_joint]);
        saveToEEPROM(eeAddress_home, my_joint, servoHome[my_joint]);
        setServoCal(leg_select, joint_select, true);
        break;
      case MIN:
        servoLimit[my_joint][0] = motor_cal(menu_option, servoLimit[my_joint][0]);
        saveToEEPROM(eeAddress_limit, my_joint*2+0, servoLimit[my_joint][0]);
        setServoCal(leg_select, joint_select, true);
        break;
      case MAX:
        servoLimit[my_joint][1] = motor_cal(menu_option, servoLimit[my_joint][1]);
        saveToEEPROM(eeAddress_limit, my_joint*2+1, servoLimit[my_joint][1]);
        setServoCal(leg_select, joint_select, true);
        break;
      case DONE:
        return;
        break;
      default:
        Serial.println("Invalid Option");
        Serial.print("\n");
        goto CAL_TYPE;
        break;
      }
      break;  //Break for case 1

    //Sweep Menu
    case SWEEP:
      // Sweep each joint on a leg one at a time. 
      // A sweep is a home->min->max->home. 
      Serial.println("Sweep Menu");
      // TODO: Draw a leg select menu
      leg_select = getMenuNum();

      // Check that all motors are calibrated before we start. 
      for(int j=1; j<4; j++) {
        if(getServoCal(leg_select, j)) {
          Serial.print("Leg #");Serial.print(leg_select);
          Serial.print(" Joint #");Serial.print(j);
          Serial.print(" is not calibrated. Bailing out!\n\n");
          return;
        }
      }

#ifdef TINKERCAD
      for(int j=1; j<4; j++) {
        my_joint = servoLeg[i_leg][i_joint];
        sweepServo(leg_select, j, servoHome[my_joint]);
        sweepServo(leg_select, j, servoLimit[my_joint][0]);
        sweepServo(leg_select, j, servoLimit[my_joint][1]);
        sweepServo(leg_select, j, servoHome[my_joint]);
      }
#endif
      break;  //break for case 2

    //Calibration Print Out
    case CAL_OUTPUT:
      Serial.println("Calibration Values For Code");
#ifdef TINKERCAD
      // Fetch our arrays from EEPROM to ensure it works. 
      EEPROM.get(eeAddress_home, servoHome);
      EEPROM.get(eeAddress_limit, servoLimit);
#endif

      Serial.println("float servoHome[TOTAL_SERVOS] = {");
      for(int i_leg=0; i_leg<TOTAL_LEGS; i_leg++) {
        Serial.print("  ");
        for(int i_joint=0; i_joint<3; i_joint++) {
          my_joint = servoLeg[i_leg][i_joint];
          Serial.print(servoHome[my_joint], 0);Serial.print(", ");
        }
        Serial.print("\n");
      }
      Serial.println("};");

      Serial.println("float servoLimit[TOTAL_SERVOS] = {");
      for(byte i_leg=0; i_leg<TOTAL_LEGS; i_leg++) {
        Serial.print("  ");
        for(byte i_joint=0; i_joint<3; i_joint++) {
          Serial.print("{");
          my_joint = servoLeg[i_leg][i_joint];
          Serial.print(servoLimit[my_joint][0], 0);Serial.print(", ");
          Serial.print(servoLimit[my_joint][1], 0);Serial.print("}, ");
        }
        Serial.print("\n");
      }
      Serial.println("};");
      Serial.print("\n");
      break;  //break for case 3

    case EXCEL_OPTION:
      int calibrated_leg;

      Serial.println("Automatic calibration tool:");
      Serial.println("**************************************************************");
      Serial.println("WARNING: Automatic calibration requires all motors to have:");
      Serial.println(" - a HOME position");
      Serial.println(" - One leg to be fully configured (HOME, MIN, MAX values)");
      Serial.print("\n");
      Serial.println("Failure to provide requirement will result in incorrect");
      Serial.println("calibration and may damage servos.");
      Serial.println("Afterwards you should sweep each leg to ensure the positions.");
      Serial.println("**************************************************************");
      Serial.println("-------------------------------------");
      Serial.println("1. No");
      Serial.println("2. Yes");
      Serial.println("-------------------------------------");
      Serial.println("Select Option: ");
      menu_option = getMenuNum();
      if(menu_option != 2) { goto MENU_START; }

      Serial.println("Which leg is configured?: ");
      calibrated_leg = getMenuNum();

      // Calculating the variables for each servo of the calibrated leg
      int my_side, other_side;
      int travel[3];
      int center[3];
      int travel_min[3][2];
      int travel_max[3][2];
      int cent_min[3][2];
      int cent_max[3][2];

      for(int i=0; i<3; i++){
        my_joint = servoLeg[calibrated_leg-1][i];
        // Percent sign is the modulo operator
        // Check the remainder of the division converted in to an int
        // This bounces between 0 and 1 for left and right
        my_side = calibrated_leg % 2;
        other_side = (calibrated_leg + 1) % 2;

        travel[i] = servoLimit[my_joint][1] - servoLimit[my_joint][0];
        center[i] = servoLimit[my_joint][1] - (travel[i]/2);

        travel_min[i][my_side] = servoHome[my_joint] - servoLimit[my_joint][0];
        travel_max[i][my_side] = servoHome[my_joint] - servoLimit[my_joint][1];
        cent_min[i][my_side] = servoLimit[my_joint][1] - center[i];
        cent_max[i][my_side] = center[i] - servoLimit[my_joint][0];

        travel_min[i][other_side] = -travel_min[i][my_side];
        travel_max[i][other_side] = -travel_max[i][my_side];
        cent_min[i][other_side] = -cent_min[i][my_side];
        cent_max[i][other_side] = -cent_max[i][my_side];
      }

      for(int i_leg=0; i_leg<TOTAL_LEGS; i_leg++) {
        // We loop over all legs, so let's skip the one we know is calibrated
        if(i_leg == calibrated_leg-1) { continue; }

        for(int i_joint=0; i_joint<3; i_joint++) {
          my_joint = servoLeg[i_leg][i_joint];

          servoLimit[my_joint][0] = servoHome[my_joint] 
            - travel_min[i_joint][(i_leg+1) % 2];
          servoLimit[my_joint][1] = servoHome[my_joint] 
            - travel_max[i_joint][(i_leg+1) % 2];
        }
      }

      Serial.print("\n\n");
      Serial.println("-------------------------------------");
      Serial.println("Automatic calibration complete.");
      Serial.println("Please check new values.");
      Serial.println("-------------------------------------");
      Serial.print("\n\n");

      break;

    default:
      Serial.println("Invalid Option");
      Serial.print("\n");
      goto MENU_START;
      break;
    } //end of switch
} //end main loop function



// TODO: We probably don't need to pass in a prev_motor_pwm anymore. 
// We can just restore from flash if we don't like our setting. 
// Check if we're calibrated first before restoring. 
int motor_cal(int menu_option, int prev_motor_pwm) {
    int servoPosition = 0;

    PWM_START:
    Serial.println("-------------------------------------");
    Serial.print("Leg: ");
    Serial.print(leg_select, DEC);
    Serial.print(" \t ");
    Serial.print("Joint: ");
    Serial.print(joint_select, DEC);
    Serial.print("\n");
    switch(menu_option){
    case HOME:
      Serial.print("Home");
      break;
    case MIN:
      Serial.print("Min");
      break;
    case MAX:
      Serial.print("Max");
      break;
    }
    Serial.println("-------------------------------------");

    Serial.println(" PWM value: ");
    servoPosition = getMenuNum();
    Serial.print("\n");
    Serial.print("\n");

    //DO MOTOR STUFF
    // Sweep each motor from it's current PWM position to it's new one
#ifndef TINKERCAD
    sweepServo(leg_select, joint_select, servoPosition);
#endif

    PWM_OOPS:
    Serial.println("-------------------------------------");
    Serial.println("1. Test Another PWM Value");
    Serial.println("2. Save Value");
    Serial.println("3. Exit without saving");
    Serial.println("-------------------------------------");
    Serial.println("Select Option: ");

    menu_option = getMenuNum();

    switch (menu_option){
    case TEST_AGAIN:
      goto PWM_START;
      break;

    case SAVE_VAL:
      return servoPosition;
      break;

    case EXIT_NO_SAVE:
      return prev_motor_pwm;
      break;

   default:
      Serial.println("Invalid Input...");
      Serial.print("\n");
      goto PWM_OOPS;
      break;
  }
}


// 
// Helper functions

// Spin on zero until the user enters a valid integer via serial. 
// You can not fetch zero as a menu option as timeouts return zero. 
// Both positive and negative integers are available. 
int
getMenuNum(void)
{
  int menuNum;
  while( 0 == (menuNum = Serial.parseInt()) ){}
  return menuNum;
}

// Wrap EEPROM.put to shove a float-sized value in to an address plus offset. 
// Offset is also counted as a float size. 
void
saveToEEPROM(int address, int offset, float value)
{
        EEPROM.put(address + offset*sizeof(float), value);
}

// Use bitwise math to check if a single bit of our servo flags has been set. 
// This takes 1-based leg and joint values. 
bool
getServoCal(int leg, int joint)
{
  int my_joint = servoLeg[leg-1][joint-1];
  return servoCalFlags & (1 << my_joint);
}

// Use bitwise math to set a single bit in our servo flags. 
// This takes 1-based leg and joint values. 
void
setServoCal(int leg, int joint, bool calibrated)
{
  int my_joint = servoLeg[leg-1][joint-1];
  servoCalFlags |= (1 << my_joint);
  EEPROM.put(eeAddress_flag, servoCalFlags);
}

// Servo helper functions. These take in the 1-based leg and joint values. 
// servoSetup is the array holding the pin names to control this servo. 
int
getServo(int leg, int joint)
{
  return pwm.getPWM(servoSetup[leg-1][joint-1]);
}

// setServo() should not be used directly as this will snap instantly to 
// the value and likely damage your leg. You should modify sweepServo 
// or create your own sweep function to prevent torque-related damage. 
void
setServo(int leg, int joint, int pos)
{
  pwm.setPWM(servoSetup[leg-1][joint-1], 0, pos);
}

// Sweep the servo from where we are to where we want to be. 
// Compare our current location with our desired location and 
// determine if we need to increase or decrease to reach it. 
// Delaying by SPD makes this very slow for long distances. 
// SPD defaults to 5ms which is about half a second for 100 values. 
// This is good for calibration but slow for normal movement. 
void
sweepServo(int leg, int joint, int pos)
{
  int curPos = getServo(leg, joint);
  for(int i = curPos; i != pos; (curPos<pos) ? i++ : i--) {
    setServo(leg, joint, pos);
    delay(SPD);
  }
}
