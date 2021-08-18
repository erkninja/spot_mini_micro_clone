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
float headerServoHome[TOTAL_SERVOS];
float headerServoLimit[TOTAL_SERVOS][2];

// menu switch
// Each enum starts at 1 instead of 0 to align with the menu options
#define NONE 0
enum menus{LEG_CAL=1, SWEEP, CAL_OUTPUT, AUTO_CAL, RESET_CAL, LOAD_CAL, SERVO_DISABLE};
enum motor_config_state{HOME=1, MIN, MAX, DONE};
enum cal_menus{TEST_AGAIN=1, SAVE_VAL, EXIT_NO_SAVE};

// other global variables
#define SPD 5
#ifndef TINKERCAD
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif
int servoCalFlags[3];

// function prototypes
int  motor_cal(int menu_option);
int  getMenuNum(void);
void saveToEEPROM(int address, int offset, float value);
bool getServoCal(int leg, int joint, int calPos);
void setServoCal(int leg, int joint, int calPos, bool calibrated);
int  getServo(int leg, int joint, int calPos);
void setServo(int leg, int joint, int pos);
void sweepServo(int leg, int joint, int pos);
int  legSweep(int leg);
int  areYouSure(void);

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
#define eeAddress_limit eeAddress_home + sizeof(servoHome)
#define eeAddress_flag eeAddress_home - sizeof(servoCalFlags)


void
setup(void)
{
  Serial.begin(19200); // opens serial port, sets data rate to a few bps
  while (!Serial) {
    // Wait for Arduino Serial Monitor to be ready
  }
  // Grabs the values from the header before they are overwritten by
  // EEPROM values. These will be used if you want to manually add calibration
  // values in the header file 
  memcpy(headerServoHome, servoHome, sizeof(servoHome));
  memcpy(headerServoLimit, servoLimit, sizeof(servoLimit));

  
  
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
  Serial.println("*************************************");
  Serial.println("* Leg Calibration Tool For Nova SM3 *");
  Serial.println("*           ^..^      /             *");
  Serial.println("*          /_/\\_____/               *");
  Serial.println("*             /\\   /\\               *");
  Serial.println("*            /  \\ /  \\              *");
  Serial.println("*************************************");
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
  Serial.println("5. Reset Calibration Values");
  Serial.println("6. Load Calibration Values from file");
  Serial.println("7. Disable all Servos");
  Serial.println("-------------------------------------");
  Serial.print("Selection Option: ");

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
      Serial.print("Select Leg (1-4): ");
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
      Serial.print("Select Motor (1-3): ");
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
      Serial.println("-------------------------------------");
      Serial.print("Selection Option: ");

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
        servoHome[my_joint] = motor_cal(menu_option);
        saveToEEPROM(eeAddress_home, my_joint, servoHome[my_joint]);
        setServoCal(leg_select, joint_select, HOME, true);
        break;
      case MIN:
        servoLimit[my_joint][0] = motor_cal(menu_option);
        saveToEEPROM(eeAddress_limit, my_joint*2+0, servoLimit[my_joint][0]);
        setServoCal(leg_select, joint_select, MIN, true);
        break;
      case MAX:
        servoLimit[my_joint][1] = motor_cal(menu_option);
        saveToEEPROM(eeAddress_limit, my_joint*2+1, servoLimit[my_joint][1]);
        setServoCal(leg_select, joint_select, MAX, true);
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
      break;  //Break for HOME (case 1)

    //Sweep Menu
    case SWEEP:
      // Sweep each joint on a leg one at a time. 
      // A sweep is a home->min->max->home. 
      SWEEP:
        Serial.println("Sweep Menu");
        Serial.println("-------------------------------------");
        Serial.println("Left       Right");
        Serial.println(" [2]--Head--[1]");
        Serial.println("    |     |   ");
        Serial.println("    |     |   ");
        Serial.println(" [4]--Butt--[3]");
        Serial.print("\n");
        Serial.println("5.  All legs");
        Serial.println("-------------------------------------");
        Serial.print("Select Leg (1-4) or All legs (5): ");

        leg_select = getMenuNum();
        // TOTAL_LEGS + 1 allows for the menu option of sweeping all legs, which would be the next number after TOTAL_LEGS
        if(leg_select < 0 || leg_select > TOTAL_LEGS + 1){
          Serial.println("**************");
          Serial.println("Invalid input");
          Serial.println("**************");
          Serial.print("\n");
          goto SWEEP;
        }
        // Sweep all 3 motors on 1 leg
        if (leg_select < 5 && leg_select > 0){
          legSweep(leg_select);
        }

        // Sweep all 3 motors on all legs
        if (leg_select == 5){
          for(int leg = 1; leg < 5; leg++){
            legSweep(leg);
          }
        }

      break;  //break for SWEEP (case 2)

    //Calibration Print Out
    case CAL_OUTPUT:
      //Serial.println("Calibration Values For Code");
#ifdef TINKERCAD
      // Fetch our arrays from EEPROM to ensure it works. 
      EEPROM.get(eeAddress_home, servoHome);
      EEPROM.get(eeAddress_limit, servoLimit);
#endif
      Serial.println("-------------------------------------");
      Serial.println("Calibration Values for Header File");
      Serial.println("-------------------------------------");
      Serial.print("\n");
      Serial.println("Copy these calibration values into the header file");
      Serial.print("\n\n");


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
      
      Serial.print("\n\n");


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
      
      Serial.print("\n\n");

      // Outputs the calibration flags in groups of 3
      // Starts with motor 1 and prints out a new line for each motor
      // The columns are HOME, MIN, MAX
      Serial.println("-------------------------------------");
      Serial.println("       Calibration Flags ");
      Serial.println("1 = Calibrated, 0 = Not Calibrated");
      Serial.print("--------------------------------------");
      for (int i_leg = 1; i_leg<TOTAL_LEGS+1; i_leg++) {
        Serial.print("\n\nLeg ");
        Serial.print(i_leg);
        Serial.print("\n");
        Serial.println("HOME  MIN   MAX");
        for(int j_joint = 1; j_joint<4; j_joint++){
          Serial.print("\n");
          for(int pos = 1; pos < 4; pos++){
            if (getServoCal(i_leg, j_joint, pos) == true){
              Serial.print(" 1    ");
            }
              else {
                Serial.print(" 0    ");
                }
            
          }
        }
        
        
      }

      Serial.print("\n\n");

      
      break;  //break for CAL_OUTPUT (case 3)

    case AUTO_CAL:
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
      Serial.print("Select Option: ");
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

      break;  // Break for AUTO_CAL

    case RESET_CAL:
        Serial.println("Are you sure you want to reset all cal values?");
        Serial.println("-------------------------------------");
        Serial.println("1. No");
        Serial.println("2. Yes");
        Serial.println("-------------------------------------");
        Serial.print("Select Option: ");
      
        menu_option = getMenuNum();
      
        switch(menu_option){
        
        // No
        case 1:
          goto MENU_START;
          break;
        
        // Yes
        case 2:
          break;

        default:
          Serial.println("Invalid Option");
          Serial.print("\n");
          goto MENU_START;
          break;
        }
        
      // Setting the arrays in RAM to 0
      memset(servoHome,0,sizeof(servoHome));
      memset(servoLimit,0,sizeof(servoLimit));
      
      // Setting the bits in the EEPROM of Calibration Flags to 0
      for(int i=1; i<5; i++){
        for(int j=1; j<4; j++){
          for(int pos=1; pos<4; pos++){
            setServoCal(i, j, pos, false);
          }
          
        } 
       }
      
      break;  // Break for RESET_CAL

    case LOAD_CAL:
      // TODO: Figure out what to do with cal flags...are they calibrated?
      // What if they only bring in a few values and not all?
      // Take the vaues from the header file (saved in setup function)
      // and put the values into EEPROM
      Serial.println("**********************************************************************");
      Serial.println("WARNING: All cal flags will be set to Yes if calibration is");
      Serial.println("imported from the header file. There will be no safety checks");
      Serial.println("for running an uncalibrated servo after loading from the header.");
      Serial.println("Ensure all HOME, MIN, and MAX values are filled in the header file.");
      Serial.println("**********************************************************************");
      Serial.print("\n");
      Serial.println("Are you sure you want to load header file calibration values?");
      Serial.println("-------------------------------------");
      Serial.println("1. No");
      Serial.println("2. Yes");
      Serial.println("-------------------------------------");
      Serial.print("Select Option: ");
    
      menu_option = getMenuNum();
    
      switch(menu_option){
      
      // No
      case 1:
        goto MENU_START;
        break;
      
      // Yes
      case 2:
        break;

      default:
        Serial.println("Invalid Option");
        Serial.print("\n");
        goto MENU_START;
        break;
      }
        
      EEPROM.put(eeAddress_home, headerServoHome);
      EEPROM.put(eeAddress_limit, headerServoLimit);
      // Put new EEPROM data into the arrays
      EEPROM.get(eeAddress_home, servoHome);
      EEPROM.get(eeAddress_limit, servoLimit);
      break;  // Break for LOAD_CAL



    // TODO: If possible, implement a way to disable the servos
    case SERVO_DISABLE:
      for(int i=0; i<16; i++){
        pwm.setPin(i,0);    
      }
      Serial.println("All Servos Disabled!");
      Serial.print("\n");
      break;  // Break for SERVO_DISABLE
      
    default:
      Serial.println("Invalid Option");
      Serial.print("\n");
      goto MENU_START;
      break;
    } //end of switch

    
} //end main loop function



int
motor_cal(int menu_option)
{
    int servoPosition = 0;

    PWM_START:
    Serial.println("-------------------------------------");
    Serial.print("Leg: ");
    Serial.print(leg_select, DEC);
    Serial.print(" \t ");
    Serial.print("Joint: ");
    Serial.print(joint_select, DEC);
    Serial.print("\n");
    Serial.println("-------------------------------------");
    
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
    
    Serial.print(" PWM value: ");
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
    Serial.print("Select Option: ");

    menu_option = getMenuNum();

    switch (menu_option){
    case TEST_AGAIN:
      Serial.println("------------------------------------");
      Serial.print("Previous PWM Value: ");
      Serial.println(servoPosition);
      goto PWM_START;
      break;

    case SAVE_VAL:
      return servoPosition;
      break;

    case EXIT_NO_SAVE:
      int my_joint = servoLeg[leg_select-1][joint_select-1];
      if (menu_option == HOME){
        return getFromEEPROM(eeAddress_home, my_joint);
      } else {
        // my_joint*2 travels to the correct joint number inside servoLimit
        // menu_option-MIN uses the enum to create a 0/1 value to offset
        // to the correct MIN/MAX position inside servoLimit
        return getFromEEPROM(eeAddress_limit, my_joint*2+(menu_option-MIN));
      }

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
  // Arduino IDE Serial Montior does not display the values that
  // you type. If you're using a different Serial Monitor that
  // displays the inputs, then remove the serial prints below.
  Serial.print(menuNum);
  Serial.print("\n");
  return menuNum;
}

// Wrap EEPROM.put to shove a float-sized value in to an address plus offset. 
// Offset is also counted as a float size. 
void
saveToEEPROM(int address, int offset, float value)
{
  EEPROM.put(address + offset*sizeof(float), value);
}

// Wrap EEPROM.get to not deal with all the sizeof math everywhere. 
float
getFromEEPROM(int address, int offset)
{
  float value;
  EEPROM.get(address + offset*sizeof(float), value);
  return value;
}

// Use bitwise math to check if a single bit of our servo flags has been set. 
// This takes 1-based leg and joint values. 
bool
getServoCal(int leg, int joint, int calPos)
{
  int my_joint = servoLeg[leg-1][joint-1];
  return servoCalFlags[calPos-1] & (1 << my_joint);
}

// Use bitwise math to set a single bit in our servo flags. 
// This takes 1-based leg and joint values.
// calPos is HOME, MIN, or MAX, which have values of 1, 2, 3 respectively
// calibrated is being passed telling setServoCal whether or not the flag should be set
// true = calibrated, false = uncalibrated
void
setServoCal(int leg, int joint, int calPos, bool calibrated)
{
  int my_joint = servoLeg[leg-1][joint-1];
  if (calibrated == true){
    servoCalFlags[calPos-1] |= (1 << my_joint);

  }
  else {
    servoCalFlags[calPos-1] &= (0 << my_joint);
  }
  EEPROM.put(eeAddress_flag + (calPos-1)*sizeof(int), servoCalFlags[calPos-1]);
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
  pwm.setPWM(servoSetup[(leg*joint)-1][1], 0, pos);
}

// Sweep the servo from where we are to where we want to be. 
// Compare our current location with our desired location and 
// determine if we need to increase or decrease to reach it. 
// Delaying by SPD makes this very slow for long distances. 
// SPD defaults to 5ms which is about half a second for 100 values. 
// This is good for calibration but slow for normal movement. 
//TODO: Coax and Femur move on sweep, but tibia does not
void
sweepServo(int leg, int joint, int pos)
{
  int curPos = getServo(leg, joint);
  for(int i = curPos; i != pos; (curPos<pos) ? i++ : i--) {
    setServo(leg, joint, pos);
    delay(SPD);
  }
}

// Check that all servos on one leg are calibrated before we start. 
// Check every position (HOME/MIN/MAX) for each joint
int
legSweep(int leg)
{
  for(int j=1; j<4; j++) {
    for(int p=1; p<4; p++) {
      if(!getServoCal(leg, j, p)) {
        Serial.print("Leg #");Serial.print(leg);
        Serial.print(" Joint #");Serial.print(j);
        Serial.print(" is not calibrated. Bailing out!\n\n");
        return 0;
      }
    }
  }

#ifndef TINKERCAD
  // Sweep leg from Home > Min > Max > Home
  for(int j=1; j<4; j++) {
    //my_joint = servoLeg[leg][j];
    sweepServo(leg, j, servoHome[j]);
    sweepServo(leg, j, servoLimit[j][0]);
    sweepServo(leg, j, servoLimit[j][1]);
    sweepServo(leg, j, servoHome[j]);
  }
#endif
}
