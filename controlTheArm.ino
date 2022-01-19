#include <dynamixel2.h>
#include <communication.h>
#include <math.h>

using namespace DynamixelProtocol2;

// Motor Constants
const double MX28C1a = 211.7;
const double MX28C1b = 427.4;
const double MX28C1c = 642.0;
const double MX28C2 = 115.3;

const double MX64C1a = 80.9;
const double MX64C1b = 152.7;
const double MX64C1c = 224.5;
const double MX64C2 = 105.3;

const double MX106C1a = 40.4;
const double MX106C1b = 83.9;
const double MX106C1c = 127.5;
const double MX106C2 = 160.6;

// PD controller gains
const double Kd = 0.5;
const double Kp = 5;

double KpArr[3] = {1, 1, 8};
double KdArr[3] = {2, 2, 0.1};
double grav_comp[3];

// Crust Crawler link lenghts
double L1 = 0.07 , L2 = 0.23 , L3 = 0.27;

int presentPos1;

double theta_2Rad;
double theta_3Rad;
double beta;

int currentposition4 = 0;
int currentposition5 = 0;
double alpha;
double Lx;
double x0; // End-effector position on X axis
double z0; // End-effector position on Z axis

double motor1Radians, motor2Radians, motor3Radians;

// Initialize motor objects
Dynamixel motor1(1, 57600);
Dynamixel motor2(2, 57600);
Dynamixel motor3(3, 57600);
Dynamixel motor4(4, 57600);
Dynamixel motor5(5, 57600);

// Initialize the motor list
Dynamixel motorList[5] = {motor1, motor2, motor3, motor4, motor5};

// Myoband variables
int j1 = 0;
int check = 1, rollref = 0, mode = 1, mode2 = 2;
int counter = 0;
bool canEnter = false;

int onint;

double totalTime = 0.0;

int myox;
int myoy;
int myoz;

// EE desired velocity
float JX = 0.2;
float JY = 0.2;
// EE desired acceleration
float dJX = 10;
float dJY = 10;

double velocity[3]; // joint desired velocities
double acceleration[3]; // joint desired velocities

double torqueNm[3];
int PWM[3];
double thetas[3];
double ang_vel_err[3], pos_err[3];

void setup() {
  Serial.begin(57600);
  Serial3.begin(57600);

  for (int i = 0; i < 5; i++) {
    motorList[i].Write<Parameters::TorqueEnable>(1);
  }

  ///////////////////////////// Forward Kinematics //////////////////////////////////////
  motor1Radians = (((double)motor1.Read<PresentPosition, uint32_t>() * (360.0 / 4096.0))) * (3.14159265359 / 180.0); 
  motor2Radians = (((double)motor2.Read<PresentPosition, uint32_t>() * (360.0 / 4096.0))) * (3.14159265359 / 180.0);
  motor3Radians = (((double)motor3.Read<PresentPosition, uint32_t>() * (360.0 / 4096.0))) * (3.14159265359 / 180.0);

  x0 = (cos(motor1Radians) * (L3 * (cos(motor3Radians) * (cos(motor1Radians) * cos(motor2Radians) - cos(M_PI / 2) * sin(motor1Radians) * sin(motor2Radians)) - sin(motor3Radians) * (cos(motor1Radians) * sin(motor2Radians) + cos(M_PI / 2) * cos(motor2Radians) * sin(motor1Radians))) + L2 * (cos(motor1Radians) * cos(motor2Radians) - cos(M_PI / 2) * sin(motor1Radians) * sin(motor2Radians)))) / (pow(cos(motor1Radians), 2) + pow(sin(motor1Radians), 2)) + (sin(motor1Radians) * (L3 * (cos(motor3Radians) * (cos(motor2Radians) * sin(motor1Radians) + cos(M_PI / 2) * cos(motor1Radians) * sin(motor2Radians)) - sin(motor3Radians) * (sin(motor1Radians) * sin(motor2Radians) - cos(M_PI / 2) * cos(motor1Radians) * cos(motor2Radians))) + L2 * (cos(motor2Radians) * sin(motor1Radians) + cos(M_PI / 2) * cos(motor1Radians) * sin(motor2Radians)))) / (pow(cos(motor1Radians), 2) + pow(sin(motor1Radians), 2));
  z0 = (L3 * (sin(M_PI / 2) * cos(motor2Radians) * sin(motor3Radians) + sin(M_PI / 2) * cos(motor3Radians) * sin(motor2Radians)) + L2 * sin(M_PI / 2) * sin(motor2Radians));
  ///////////////////////////// Forward Kinematics End //////////////////////////////////
}

void loop() {
  ///////////////////// Myoband start /////////////////////////////
  //**************************************************************
  if (Serial.available() > 0)
  {
    String info;
    info = Serial.readStringUntil('\n');

    int str_len = info.length();
    char char_array[str_len];
    info.toCharArray(char_array, str_len);

    String a = String(info[0]);
    int a1 = a.toInt();
    String b = String(info[1]);
    int b1 = b.toInt();
    String c = String(info[2]);
    int c1 = c.toInt();

    String d = String(info[3]);
    int d1 = d.toInt();
    String e = String(info[4]);
    int e1 = e.toInt();
    String f = String(info[5]);
    int f1 = f.toInt();

    String g = String(info[6]);
    int g1 = g.toInt();
    String h = String(info[7]);
    int h1 = h.toInt();
    String i = String(info[8]);
    int i1 = i.toInt();

    String j = String(info[9]);
    j1 = j.toInt();

    String on = String(info[10]);
    int onint = on.toInt();
    //**********************************************************

    int myox = (a1 * 100) + (b1 * 10) + c1;
    int myoy = (d1 * 100) + (e1 * 10) + f1;
    int myoz = (g1 * 100) + (h1 * 10) + i1;

    // Used for monitoring the changes made while the crust crawler moves
    Serial.print("     z0:  "); Serial.print(z0); Serial.print("         x0:  ");   Serial.print(x0);  Serial.print("       Mode:  "); Serial.print(mode); Serial.print("       sqrt:  "); Serial.print(sqrt(pow(x0, 2) + pow(z0, 2))); Serial.print("     roll ref: "); Serial.print(rollref); Serial.print("   myox:  "); Serial.print(myox); Serial.print("   check:  "); Serial.print(info);
    
    ///////////////////// Creating Myo modes///////////////////////////////
    if (onint == 1 && check == 1)
    {
      check = 2;
      rollref = myox;
    }
    if (onint != 1)
    {
      check = 1;
    }
    //-----------------------------------------

    // checks doubletap once
    if (j1 != 6) {
      canEnter = true;
    }
    if (j1 == 6 && canEnter == true) {
      canEnter = false;
      counter++;
    }
    //-----------------------------------------

    // shifts between x and y axis
    if ((counter % 2) == 0)
    {
      mode2 = 2;
    }
    else
    {
      mode2 = 3;
    }
    //-----------------------------------------
    // shifts between xy and z axis
    if (rollref < 20)
    {
      int lim = 360 + rollref - 20;
      if (myox > lim || myox < rollref + 20)
      {
        mode = 1;
      }
      else
      {
        mode = mode2;
      }
    }
    else
    {
      if (myox > rollref - 20 && myox < rollref + 20)
      {
        mode = 1;
      }

      else
      {
        mode = mode2;
      }
    }
    /////////////////////////// Creating Myo modes end /////////////////////

    /////////////////////////// Creating reference limits /////////////////////
    if (sqrt(pow(z0, 2) + pow(x0, 2)) < 0.46 && z0 < 0)
    {
      if (mode == 2 && j1 == 5)
      {
        z0 -= 0.01;
      }
      if (mode == 3 && j1 == 4)
      {
        x0 -= 0.01;
      }
    }
    else if (sqrt(pow(z0, 2) + pow(x0, 2)) < 0.46)
    {
      if (mode == 2 && j1 == 4)
      {
        z0 += 0.01;
      }
      if (mode == 3 && j1 == 4)
      {
        x0 -= 0.01;
      }
    }
    if (sqrt(pow(z0, 2) + pow(x0, 2)) > 0.33 && z0 < 0)
    {
      if (mode == 2 && j1 == 4)
      {
        z0 += 0.01;
      }
      if (mode == 3 && j1 == 5)
      {
        x0 += 0.01;
      }
    }
    else if (sqrt(pow(z0, 2) + pow(x0, 2)) > 0.33)
    {
      if (mode == 2 && j1 == 5)
      {
        z0 -= 0.01;
      }
      if (mode == 3 && j1 == 5)
      {
        x0 += 0.01;
      }
    }

    /////////////////////////// Creating reference limits end /////////////////////

    //////////////////// Move only motor one //////////////////////////////////////
    if (mode == 1) {
      if (j1 == 5) {
        motorList[0].Write<GoalPWM>(100);
      }

    }
    if (j1 == 4) {
      motorList[0].Write<GoalPWM>(-100);
    }
  }

  if (j1 != 5 && j1 != 4)
  {
    motorList[0].Write<GoalPWM>(0);
  }
  if (mode != 1)
  {
    motorList[0].Write<GoalPWM>(0);
  }


  //////////////////// Move only motor one end ///////////////////////////////////

  ////////////////////// Move gripper /////////////////////////////////////////////
  currentposition4 = motor4.Read<PresentPosition, uint32_t>();
  currentposition5 = motor5.Read<PresentPosition, uint32_t>();
  if (j1 == 2)
  {
    motorList[3].Write<GoalPWM>(-100);
    motorList[4].Write<GoalPWM>(100);


  }
  if (j1 == 3)
  {
    motorList[3].Write<GoalPWM>(100);
    motorList[4].Write<GoalPWM>(-100);

  }
  if ((motorList[3].Read<PresentPosition, int32_t>() < 2729 && j1 != 3)) {
    motorList[3].Write<GoalPWM>(-300);
    motorList[4].Write<GoalPWM>(300);
  }
  else if (j1 != 2 && j1 != 3)
  {
    motorList[3].Write<GoalPWM>(0);
    motorList[4].Write<GoalPWM>(0);
  }
  ////////////////////// Move gripper end //////////////////////////////////////////


  ///////////////////////// Myoband end /////////////////////////////

  //////////////////////// Inverse Kinematics Calculations ////////////////
  Lx = sqrt(pow(z0, 2) + pow(x0, 2));

  beta = acos((pow(L2, 2) + pow(Lx, 2) - pow(L3, 2)) / (2 * L2 * Lx));

  alpha = -atan2((z0), (x0));

  if (z0 < 0) {
    alpha = -3.1416 - (3.1416 + atan2((z0), (x0)));
  }

  // The thetas
  theta_2Rad = -(beta + alpha);
  theta_3Rad = acos(-(pow(L3, 2) + pow(L2, 2) - pow(Lx, 2)) / (2 * L3 * L2));

  double theta_2 = theta_2Rad * 180 / M_PI;
  double theta_3 = theta_3Rad * 180 / M_PI;

  //////////////////////// Angle Limits /////////////////////////
  //  if (theta_2Rad > 3.542)
  //  {
  //    theta_2Rad = 3.542;
  //  }
  //  else if (theta_2Rad < 0.174532925)
  //  {
  //    theta_2Rad = 0.174532925 ; //1.5708;
  //  }
  if (theta_3Rad < 0.0169)
  {
    theta_3Rad = 0.0169;
  }
  //////////////////////// Angle Limits End /////////////////////////

  thetas[1] = theta_2Rad;
  thetas[2] = theta_3Rad;
  //////////////////////// Inverse Kinematics End ////////////////

  //////////////////////// Calculate the desired velocity and acceleration ////////////////////////////
  velocity[0] = 0.0;//(cos(theta_2 + theta_3) * JX) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3)) + (sin(theta_2 + theta_3) * JY) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3));
  velocity[1] = 0.0;//-(JX * (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3)) - (JY * (0.23 * sin(theta_2) + 0.27 * sin(theta_2 + theta_3))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3));
  velocity[2] = 0.0;
  acceleration[0] = 0.0;//(cos(theta_2 + theta_3) * (dJX + (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2)) * ((cos(theta_2 + theta_3) * JX) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3)) + (sin(theta_2 + theta_3) * JY) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3))) - 0.27 * cos(theta_2 + theta_3) * ((JX * (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3)) + (JY * (0.23 * sin(theta_2) + 0.27 * sin(theta_2 + theta_3))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3))))) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3)) + (sin(theta_2 + theta_3) * (dJY + (0.27 * sin(theta_2 + theta_3) + 0.23 * cos(theta_2)) * ((cos(theta_2 + theta_3) * JX) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3)) + (sin(theta_2 + theta_3) * JY) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3))) - 0.27 * sin(theta_2 + theta_3) * ((JX * (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3)) + (JY * (0.23 * sin(theta_2) + 0.27 * sin(theta_2 + theta_3))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3))))) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3));
  acceleration[1] = 0.0;//-((0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2)) * (dJX + (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2)) * ((cos(theta_2 + theta_3) * JX) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3)) + (sin(theta_2 + theta_3) * JY) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3))) - 0.27 * cos(theta_2 + theta_3) * ((JX * (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3)) + (JY * (0.23 * sin(theta_2) + 0.27 * sin(theta_2 + theta_3))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3))))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3)) - ((0.23 * sin(theta_2) + 0.27 * sin(theta_2 + theta_3)) * (dJY + (0.27 * sin(theta_2 + theta_3) + 0.23 * cos(theta_2)) * ((cos(theta_2 + theta_3) * JX) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3)) + (sin(theta_2 + theta_3) * JY) / (0.23 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * sin(theta_2) * cos(theta_2 + theta_3))) - 0.27 * sin(theta_2 + theta_3) * ((JX * (0.27 * cos(theta_2 + theta_3) + 0.23 * cos(theta_2))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3)) + (JY * (0.23 * sin(theta_2) + 0.27 * sin(theta_2 + theta_3))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3))))) / (0.23 * 0.27 * cos(theta_2) * sin(theta_2 + theta_3) - 0.23 * 0.27 * sin(theta_2) * cos(theta_2 + theta_3));
  acceleration[2] = 0.0;
  //////////////////////// Calculate the desired velocity and acceleration end ////////////////////////

  //////////////////////// Calculate gravity compensation ////////////////////////
  grav_comp[0] = 0;
  grav_comp[1] = (-0.384552 * cos((double)(motorList[2].Read<PresentPosition, int32_t>() * (360.0 / 4096.0)) * 0.0174533) - 1.023183) * cos((double)(motorList[1].Read<PresentPosition, int32_t>() * (360.0 / 4096.0)) * 0.0174533) + 0.384552 * sin((double)(motorList[1].Read<PresentPosition, int32_t>() * (360.0 / 4096.0)) * 0.0174533) * sin((double)(motorList[2].Read<PresentPosition, int32_t>() * (360.0 / 4096.0)) * 0.0174533);
  grav_comp[2] = -0.384552 * cos((double)(motorList[1].Read<PresentPosition, int32_t>() * (360.0 / 4096.0))) * cos((double)(motorList[2].Read<PresentPosition, int32_t>() * (360.0 / 4096.0))) + 0.384552 * sin((double)(motorList[1].Read<PresentPosition, int32_t>() * (360.0 / 4096.0))) * sin((double)(motorList[2].Read<PresentPosition, int32_t>() * (360.0 / 4096.0)));

  //////////////////////// Calculate gravity compensation end ////////////////////////


  //////////////////////////////// Controller /////////////////////////////////////////////////////////
  for (int i = 0; i < 3; i++) {
    ang_vel_err[i] = 0.0 - (double)motorList[i].Read<PresentVelocity, int32_t>() * 0.299 * 0.10472; // unit to rpm to rad/sec
    pos_err[i] = thetas[i] - (double)(motorList[i].Read<PresentPosition, int32_t>() * (360.0 / 4096.0)) * 0.0174533; // unit to deg to rad

    torqueNm[1] = -grav_comp[1] + Kd * ang_vel_err[1] + Kp * pos_err[1]; //acceleration[i] + Kd * ang_vel_err[i] + Kp * pos_err[i];
    torqueNm[2] = -grav_comp[2] + 0.1 * ang_vel_err[i] + 4 * pos_err[2];
  }

  ////////////////////////////// Converting torque to PWM ////////////////////////////////////////////////
  if (torqueNm[0] > 0) {
    // Motor 1
    if (velocity[0] == 0) {
      PWM[0] = torqueNm[0] * MX64C1b + 0 * MX64C2;
    }
    else if (velocity[0] > 0) {
      PWM[0] = torqueNm[0] * MX64C1c + 0 * MX64C2;
    }
    else {
      PWM[0] = torqueNm[0] * MX64C1a + 0 * MX64C2;
    }
    // Motor 2
    if (velocity[1] == 0) {
      PWM[1] = torqueNm[1] * MX106C1b +  0 * MX106C2;
    }
    else if (velocity[1] > 0) {
      PWM[1] = torqueNm[1] * MX106C1c + 0  * MX106C2;
    }
    else {
      PWM[1] = torqueNm[1] * MX106C1a + 0 * MX106C2;
    }
    // Motor 3
    if (velocity[2] == 0) {
      PWM[2] = torqueNm[2] * MX64C1b + 0 * MX64C2;
    }
    else if (velocity[1] > 0) {
      PWM[2] = torqueNm[2] * MX64C1c + 0 * MX64C2;
    }
    else {
      PWM[2] = torqueNm[2] * MX64C1a + 0 * MX64C2;
    }
  }
  else {
    // Motor 1
    if (velocity[0] == 0) {
      PWM[0] = torqueNm[0] * MX64C1b + 0 * MX64C2;
    }
    else if (velocity[0] > 0) {
      PWM[0] = torqueNm[0] * MX64C1a + 0 * MX64C2;
    }
    else {
      PWM[0] = torqueNm[0] * MX64C1c + 0 * MX64C2;
    }
    // Motor 2
    if (velocity[1] == 0) {
      PWM[1] = torqueNm[1] * MX106C1b + 0 * MX106C2;
    }
    else if (velocity[1] > 0) {
      PWM[1] = torqueNm[1] * MX106C1a + 0 * MX106C2;
    }
    else {
      PWM[1] = torqueNm[1] * MX106C1c + 0 * MX106C2;
    }
    // Motor 3
    if (velocity[2] == 0) {
      PWM[2] = torqueNm[2] * MX64C1b + 0 * MX64C2;
    }
    else if (velocity[2] > 0) {
      PWM[2] = torqueNm[2] * MX64C1a + 0 * MX64C2;
    }
    else {
      PWM[2] = torqueNm[2] * MX64C1c + 0 * MX64C2;
    }
  }
  ////////////////////////////// Converting torque to PWM end ////////////////////////////////////////////////
  //////////////////////////////// Controller end //////////////////////////////////

  /////////////////////////// PWM limits ////////////////////////////////////
  for (int i = 0; i < 3; i++) {
    if (PWM[i] > 800) {
      PWM[i] = 800;
      PWM[2] = 800;
    }
    else if (PWM[i] < -800) {
      PWM[i] = -800;
      PWM[2] = -800;
    }
  }
  /////////////////////////// PWM limits end ////////////////////////////////////
  
  // Move the motors with the calculated PWM values
  motorList[1].Write<GoalPWM>(PWM[1]); // move motor 2
  motorList[2].Write<GoalPWM>(PWM[2]); // move motor 3
}
