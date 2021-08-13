//#define TINKERCAD

#include <EEPROM.h>
#ifndef TINKERCAD
#include <Adafruit_PWMServoDriver.h>
#endif

#include "NovaServos.h"

int incomingByte = 0; // for incoming serial data
int menu_option = 0;  //keeps track of which menu option is selected
int leg_select = 0; //keeps track of the leg that is being calibrated 
int joint_select = 0; //keeps track of the joint/motor to calibrate

//menu switch
#define NONE = 0
enum motor_config_state{HOME=1, MIN, MAX, DONE};
enum menus{LEG_CAL=1, SWEEP, CAL_OUTPUT, EXCEL_OPTION};
enum cal_menus{TEST_AGAIN=1, SAVE_VAL, EXIT_NO_SAVE};

//other global variables
#define SPD = 5;
#ifndef TINKERCAD
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

//function prototypes
int motor_cal(int menu_option, int prev_motor_pwm);

void setup() {
#ifdef TINKERCAD
// Set some test data in our servo arrays variables
float myServoHome[TOTAL_SERVOS] = {         //home pos
  368, 301, 435,                          //RFx
  355, 422, 355,                          //LFx
  348, 344, 396,                          //RRx
  364, 341, 306,                          //LRx
};
memcpy(servoHome, myServoHome, sizeof(servoHome));

float myServoLimit[TOTAL_SERVOS][2] = {     //min, max
  {330, 450}, {226, 466}, {322, 548},     //RFx•
  {0, 0}, {0, 0}, {0, 0},     //LFx
  {0, 0}, {0, 0}, {0, 0},     //RRx
  {0, 0}, {0, 0}, {0, 0},     //LRx
};
memcpy(servoLimit, myServoLimit, sizeof(servoLimit));
#endif

  Serial.begin(19200); // opens serial port, sets data rate to 9600 bps
  Serial.println("*********************************");
  Serial.println("* Nova SM3 Leg Calibration Tool *");
  Serial.println("*       ^..^      /             *");
  Serial.println("*       /_/\\_____/              *");
  Serial.println("*          /\\   /\\              *");
  Serial.println("*         /  \\ /  \\             *");
  Serial.println("*********************************");
  Serial.println("\n");

#ifndef TINKERCAD
  pwm.begin();
  pwm.setOscillatorFrequency(25000000); // 25_000_000
  pwm.setPWMFreq(60);
#endif
}

void loop() {
  char my_joint = 0; // joint iteration/selection temp variable

  //Top menu
  MENU_START:
  Serial.println("-------------------------------------");
  Serial.println("1. Single Motor Calibration");
  //TODO Need to implement Leg Sweep
  Serial.println("2. Leg Sweep");
  Serial.println("3. Print Calibration Values");
  Serial.println("4. Automatic Calibration Method");
  Serial.println("-------------------------------------");
  Serial.println("Selection Option: ");

  while( 0 == (menu_option = Serial.parseInt()) ){}
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
      while( 0 == (leg_select = Serial.parseInt()) ){}
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
      while( 0 == (joint_select = Serial.parseInt()) ){}
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
      while( 0 == (menu_option = Serial.parseInt()) ){}
      Serial.print("\n");
      Serial.print("\n");

      switch (menu_option){
      case HOME:
        my_joint = servoLeg[leg_select-1][joint_select-1];
        servoHome[my_joint] = motor_cal(menu_option, servoHome[my_joint]);
        break;
      case MIN:
        my_joint = servoLeg[leg_select-1][joint_select-1];
        servoLimit[my_joint][0] = motor_cal(menu_option, servoLimit[my_joint][0]);
        break;
      case MAX:
        my_joint = servoLeg[leg_select-1][joint_select-1];
        servoLimit[my_joint][1] = motor_cal(menu_option, servoLimit[my_joint][1]);
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
      Serial.println("Sweep Menu");
      // TODO
      break;  //break for case 2

    //Calibration Print Out
    case CAL_OUTPUT:
      Serial.println("Calibration Values For Code");

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
      for(char i_leg=0; i_leg<TOTAL_LEGS; i_leg++) {
        Serial.print("  ");
        for(char i_joint=0; i_joint<3; i_joint++) {
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
      while( 0 == (menu_option = Serial.parseInt()) ){}
      if(menu_option != 2) { goto MENU_START; }

      Serial.println("Which leg is configured?: ");
      while( 0 == (calibrated_leg = Serial.parseInt()) ){}

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



int motor_cal(int menu_option, int prev_motor_pwm) {
    int motor_pwm = 0;
    int motor_location = 0;

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
    while( 0 == (motor_pwm = Serial.parseInt()) ){}   //Wait for user input for motor pwm
    Serial.print("\n");
    Serial.print("\n");

    //DO MOTOR STUFF
    // Sweep each motor from it's current PWM position to it's new one
    // Compare our current location with our desired location and 
    // determine if we need to increase or decrease to reach it. 
#ifndef TINKERCAD
    motor_location = pwm.getPWM(servoSetup[leg_select-1][joint_select-1]);
    for(int i = motor_location; 
        i != motor_pwm; 
        (motor_location<motor_pwm) ? i++ : i--) {
      pwm.setPWM(servoSetup[leg_select-1][joint_select-1], 0, motor_pwm);
      delay(SPD);
    }
#endif

    PWM_OOPS:
    Serial.println("-------------------------------------");
    Serial.println("1. Test Another PWM Value");
    Serial.println("2. Save Value");
    Serial.println("3. Exit without saving");
    Serial.println("-------------------------------------");
    Serial.println("Select Option: ");

    while( 0 == (menu_option = Serial.parseInt()) ){}

    switch (menu_option){
    case TEST_AGAIN:
      goto PWM_START;
      break;

    case SAVE_VAL:
      return motor_pwm;
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
