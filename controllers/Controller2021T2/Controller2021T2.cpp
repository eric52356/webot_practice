// File:             z5167306_MTRN4110_PhaseA.cpp
// Date:             01/06/2021
// Description:      Controller of e-Puck for phase A - driving and perception
// Author:           Hsu Mu-Kuan
// Platform:         MacOS


// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

#define WHEELRADIUS 0.02
#define MAZEDISTANCE 0.165
#define FORWARDPOS MAZEDISTANCE/WHEELRADIUS
#define AXLE 0.0566
#define ROTATEPOS AXLE*3.1415/(WHEELRADIUS*4)
// All the webots classes are defined in the "webots" namespace
using namespace webots;

std::string readfile(std::string file);
void RunCommand(char command, Motor* left_motor, Motor* right_motor);
void Forward(Motor* left_motor, Motor* right_motor);
void Left(Motor* left_motor, Motor* right_motor);
void Right(Motor* left_motor, Motor* right_motor);


int main(int argc, char **argv) {
  // Instanse initialisation
  Robot *robot = new Robot();
  Motor *left_motor = robot->getMotor("left wheel motor");
  Motor *right_motor = robot->getMotor("right wheel motor");  
  
  // Variable initialization
  int timeStep = (int)robot->getBasicTimeStep();
  int excecutingflag = 0; // used to check whether the car is excecuting command
  int commandcount = 3;
  double runtime = 0;
  char command;
  double timestamp;
  std::string plan;

  // read in file 
  plan = readfile("MotionPlan.txt");
  while (robot->step(timeStep) != -1) {
    //if non ececuting, read in the next command
    if(excecutingflag == 0){
      command = plan[commandcount];
      if (command == '\0') break;
      if (command == 'F') {
        runtime = 2;
      } else{
        runtime = 3.5;
      }
      std::cout << command << "\n";
      RunCommand(command,left_motor, right_motor);
      commandcount++;
      timestamp = robot->getTime();
      excecutingflag = 1;
    }else{
      if((robot->getTime() - timestamp)>runtime){
        excecutingflag = 0;
        robot->step(timeStep);
      }
    }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

std::string readfile(std::string file){
  std::ifstream inFile;
  inFile.open(file); //open the input file
  
  std::stringstream strStream;
  strStream << inFile.rdbuf(); //read the file
  std::string str = strStream.str(); //str holds the content of the file
  
  std::cout << str << "\n";
  return str;
}

void RunCommand(char command,Motor* left_motor,Motor* right_motor){
  switch (command){
    case 'F':
      Forward(left_motor, right_motor); 
      break;
    case 'L':
      Left(left_motor, right_motor);
      break;
    case 'R':
      Right(left_motor, right_motor);
      break;
  }
}

void Forward(Motor* left_motor, Motor* right_motor){
  double posleft, posright;
  posleft = left_motor->getTargetPosition() + FORWARDPOS;
  posright = right_motor->getTargetPosition() + FORWARDPOS;
  left_motor->setPosition(posleft);
  right_motor->setPosition(posright); 
  left_motor->setControlPID(10, 0.045, 0.001);
  right_motor->setControlPID(10, 0.045, 0.001);
  left_motor->setVelocity(6.28);
  right_motor->setVelocity(6.28);
}

void Left(Motor* left_motor, Motor* right_motor){
  double posleft, posright;
  posleft = left_motor->getTargetPosition() - ROTATEPOS;
  posright = right_motor->getTargetPosition() + ROTATEPOS;
  left_motor->setPosition(posleft);
  right_motor->setPosition(posright); 
  left_motor->setControlPID(3, 0.005, 0.9);
  right_motor->setControlPID(3, 0.005, 0.9);
  left_motor->setVelocity(6.28);
  right_motor->setVelocity(6.28);
}

void Right(Motor* left_motor, Motor* right_motor){
  double posleft, posright;
  posleft = left_motor->getTargetPosition() + ROTATEPOS;
  posright = right_motor->getTargetPosition() - ROTATEPOS;
  left_motor->setPosition(posleft);
  right_motor->setPosition(posright); 
  left_motor->setControlPID(3, 0.005, 0.9);
  right_motor->setControlPID(3, 0.005, 0.9);
  left_motor->setVelocity(6.28);
  right_motor->setVelocity(6.28);
}