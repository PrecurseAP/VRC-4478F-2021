//contains code that moves the robot during autonomous
#include "vex.h"
#include "auton-movement.h"
#include <vector>
#include "math.h"
#include <string>
#include <iostream>

int graphBottomBorder =  225;
int graphTopBorder = 25;
int graphLeftBorder = 75;
int graphRightBorder = 475;

class Graph {
  public:
    float* dataVar;
    int least = 0;
    int greatest = 0;
    float prevX = 0;
    float prevY = 0;
    float px = 0;
    float py = 0;

    std::vector<float> graphData;

    Graph(float* Var) {
      dataVar = Var;
    }

    void updateData(float dataPoint) {
      graphData.push_back(dataPoint);

      if (dataPoint > greatest) {
        greatest = dataPoint;
      }
      if (dataPoint < least) {
        least = dataPoint;
      }

      if (greatest == least) {
        least+=.01;
      }     
    }
    void drawGraph() {
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.drawLine(graphLeftBorder, graphTopBorder, graphLeftBorder, graphBottomBorder);
      Brain.Screen.drawLine(graphLeftBorder, graphBottomBorder, graphRightBorder, graphBottomBorder);
      Brain.Screen.setPenColor(color::green);

      prevX = graphLeftBorder;
      prevY = graphTopBorder + ((graphData[0] - least) * ((graphBottomBorder-graphTopBorder)/(greatest-least)));

      for (int i = 0; i < graphData.size(); i++) {
        px = i * ((graphRightBorder - graphLeftBorder) / graphData.size()) + graphLeftBorder;
        py = 272 - (graphTopBorder*2 + ((graphData[i] - least) * ((graphBottomBorder - graphTopBorder)/(greatest - least))));
        Brain.Screen.drawLine(prevX, prevY, px, py);    
        prevX = px;
        prevY = py;
      }

      Brain.Screen.printAt(graphLeftBorder-75, graphBottomBorder, "%f", least);
      Brain.Screen.printAt(graphLeftBorder-75, graphTopBorder, "%f", greatest);
      Brain.Screen.printAt(graphLeftBorder-75, py, "$f", graphData[graphData.size()-1]);
      Brain.Screen.setPenColor(color::red);
      Brain.Screen.drawLine(graphLeftBorder, py, graphRightBorder, py);
    }
};

void raiseLift(bool wait = true) {
  mArm.setVelocity(100, percent);
  mArm.spinToPosition(40, degrees, wait);
}
void raiseLiftFully(bool wait = true) {
  mArm.setVelocity(100, percent);
  mArm.spinToPosition(665, degrees, wait);
}

void lowerLift(bool wait = true) {
  mArm.spinToPosition(-40, degrees, wait);
}

void spinConveyor() {
  mConveyor.spin(forward, 169, rpm);
}

void lowerTilter(bool wait = true) {
  mLTray.setVelocity(100, percent);
  mRTray.setVelocity(100, percent);
  mLTray.spinToPosition(-540, degrees, false);
  mRTray.spinToPosition(-540, degrees, wait);
}
void lowerTilterSlow(bool wait = true) {
  mLTray.setVelocity(40, percent);
  mRTray.setVelocity(40, percent);
  mLTray.spinToPosition(-540, degrees, false);
  mRTray.spinToPosition(-540, degrees, wait);
}
void lowerTilterWithValue(bool wait = true, int val = -540) {
  mLTray.setVelocity(100, percent);
  mRTray.setVelocity(100, percent);
  mLTray.spinToPosition(val, degrees, false);
  mRTray.spinToPosition(val, degrees, wait);
}

void raiseTilterWithGoal(bool wait = true) {
  mLTray.setVelocity(100, percent);
  mRTray.setVelocity(100, percent);
  mLTray.spinToPosition(-270, degrees, false);
  mRTray.spinToPosition(-270, degrees, wait);
}

void raiseTilter(bool wait = true) {
  mLTray.setVelocity(100, percent);
  mRTray.setVelocity(100, percent);
  mLTray.spinToPosition(0, degrees, false);
  mRTray.spinToPosition(0, degrees, wait);
}

template <typename T>
T angleWrap(T _theta) {
  return fmod(_theta, 360);
}

void moveRightSide(int speed) {
  mRUpper.spin(forward, speed, percent);
  mRLower.spin(forward, speed, percent);
}
void moveLeftSide(int speed) {
  mLUpper.spin(forward, speed, percent);
  mLLower.spin(forward, speed, percent);
}
void stopAllDrive(brakeType bt) {
  mRUpper.stop(bt);
  mRLower.stop(bt);
  mLUpper.stop(bt);
  mLLower.stop(bt);
}

//use this and the move function
void spotTurn(float theta, int maxSpeed, int vecCount, int timeLimit) {
  int startTime = Brain.timer(msec);
  bool done = false;
  float integral = 0;

  float kP = 0.77;
  float kI = 0.0014;
  float kD = .19;

  std::vector<float> prevValues;
  repeat(vecCount) {
    prevValues.push_back(999);
  }

  int prevTime = 0;
  float prevError = 0;

  while (!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;

    prevTime = currentTime;

    float currentAngle = GYRO.heading(degrees);
    
    float angleError = theta - currentAngle;
    //std::cout << currentAngle << std::endl;
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }

    float derivative = (angleError - prevError) / delta_time;

    prevError = angleError;

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 15) {
      integral += angleError * delta_time;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    float driveSpeed = kP*angleError + kI*integral + kD*derivative;

    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);
    //std::cout << .007*integral << std::endl;
    
    
    if ((greatest < 5) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);

  }
}

void spotTurnWithNoAngleWrap(float theta, int maxSpeed, int vecCount, int timeLimit) {
  int startTime = Brain.timer(msec);
  bool done = false;
  float integral = 0;

  float kP = -0.71;
  float kI = -0.0006;
  float kD = -.65;

  std::vector<float> prevValues;
  repeat(vecCount) {
    prevValues.push_back(999);
  }

  int prevTime = 0;
  float prevError = 0;

  while (!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;

    prevTime = currentTime;

    float currentAngle = GYRO.heading(degrees);
    
    float angleError = currentAngle-theta;

    std::cout << angleError << std::endl;
    /*if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }*/

    float derivative = (angleError - prevError) / delta_time;

    prevError = angleError;

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 15) {
      integral += angleError * delta_time;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    float driveSpeed = kP*angleError + kI*integral + kD*derivative;

    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);
    //std::cout << .007*integral << std::endl;
    
    
    if ((greatest < 4) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);

  }
}


void spotTurnWithTilterGoal(float theta, int maxSpeed, int vecCount, int timeLimit) {
  int startTime = Brain.timer(msec);
  bool done = false;
  float integral = 0;

  float kP = 0.71;
  float kI = 0.0006;
  float kD = .65;

  std::vector<float> prevValues;
  repeat(vecCount) {
    prevValues.push_back(999);
  }

  int prevTime = 0;
  float prevError = 0;

  while (!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;

    prevTime = currentTime;

    float currentAngle = GYRO.heading(degrees);
    
    float angleError = theta - currentAngle;
    std::cout << currentAngle << std::endl;
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }

    float derivative = (angleError - prevError) / delta_time;

    prevError = angleError;

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 15) {
      integral += angleError * delta_time;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    float driveSpeed = kP*angleError + kI*integral + kD*derivative;

    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);
    //std::cout << .007*integral << std::endl;
    
    
    if ((greatest < 5) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);

  }
}

void spotTurnWithClawGoal(float theta, int maxSpeed, int vecCount, int timeLimit) {
  int startTime = Brain.timer(msec);
  bool done = false;
  float integral = 0;

  float kP = 0.71;
  float kI = 0.0008;
  float kD = .55;

  std::vector<float> prevValues;
  repeat(vecCount) {
    prevValues.push_back(999);
  }

  int prevTime = 0;
  float prevError = 0;

  while (!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;

    prevTime = currentTime;

    float currentAngle = GYRO.heading(degrees);
    
    float angleError = theta - currentAngle;
    //std::cout << currentAngle << std::endl;
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }

    float derivative = (angleError - prevError) / delta_time;

    prevError = angleError;

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 15) {
      integral += angleError * delta_time;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    float driveSpeed = kP*angleError + kI*integral + kD*derivative;

    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);
    //std::cout << .007*integral << std::endl;
    
    
    if ((greatest < 5) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);

  }
}

void spotTurnWith2Goals(float theta, int maxSpeed, int vecCount, int timeLimit) {
  int startTime = Brain.timer(msec);
  bool done = false;
  float integral = 0;

  float kP = 0.651;
  float kI = 0.00101;
  float kD = .41;

  std::vector<float> prevValues;
  repeat(vecCount) {
    prevValues.push_back(999);
  }

  int prevTime = 0;
  float prevError = 0;

  while (!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;

    prevTime = currentTime;

    float currentAngle = GYRO.heading(degrees);
    
    float angleError = theta - currentAngle;
    //std::cout << currentAngle << std::endl;
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }

    float derivative = (angleError - prevError) / delta_time;

    prevError = angleError;

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 15) {
      integral += angleError * delta_time;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    float driveSpeed = kP*angleError + kI*integral + kD*derivative;

    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);
    //std::cout << .007*integral << std::endl;
    
    
    if ((greatest < 5) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);

  }
}

//no worky
void moveForward(float d, int maxSpeed) {
  bool done = false;
  float integral = 0;
  float startAngle = GYRO.heading(degrees);
  std::vector<float> prevValues;
  repeat(50) {
    prevValues.push_back(999);
  }

  float startX = GPS.xPosition(inches);
  float startY = GPS.yPosition(inches);

    std::cout << startX << std::endl;
    std::cout << startY << std::endl;

  while(!done) {
    float currentX = GPS.xPosition(inches);
    float currentY = GPS.yPosition(inches);
    float currentAngle = GYRO.heading(degrees);

    float angleError = startAngle - currentAngle;

    float disError = d - hypot(currentX - startX, currentY - startY);
    std::cout << disError << std::endl;
    prevValues.erase(prevValues.begin());
    prevValues.push_back(disError);

    if (fabs(disError) < 15) {
      integral += disError;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    if ((fabs(disError)*2) > maxSpeed) {
      disError = maxSpeed * ((disError < 1) ? -1 : 1);
    }

    mLUpper.spin(forward, -(2*disError + .000001*integral + angleError), percent);
    mLLower.spin(forward, -(2*disError + .000001*integral + angleError), percent);
    mRUpper.spin(forward, -(2*disError + .000001*integral + angleError), percent);
    mRLower.spin(forward, -(2*disError + .000001*integral + angleError), percent);

    if ((greatest+angleError) < 2) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);
  }
}

void move(float d, int maxSpeed, int vecCount, int timeLimit) {
  int start_time = Brain.timer(msec);

  float kP = 4.60;
  float kI = .001;
  float kD = 2.4;

  bool done = false;

  float leftIntegral = 0;
  float rightIntegral = 0;

  std::vector<float> prevLeftValues;
  std::vector<float> prevRightValues;
  repeat(vecCount) {
    prevLeftValues.push_back(999);
    prevRightValues.push_back(999);
  }

  mLLower.setPosition(0, degrees);
  mRLower.setPosition(0, degrees);

  int prevTime = 0;
  float prevLeftError = 0;
  float prevRightError = 0;

  while(!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;
    prevTime = currentTime;

    float currentLeft = (mLLower.position(degrees)/360.0) * 4.0 * M_PI;
    float currentRight = (mRLower.position(degrees)/360.0) * 4.0 * M_PI;

    float leftError = d - currentLeft;
    float rightError = d - currentRight;

    float leftDerivative = (leftError - prevLeftError) / delta_time;
    float rightDerivative = (rightError - prevRightError) / delta_time;

    prevLeftError = leftError;
    prevRightError = rightError;

    prevLeftValues.erase(prevLeftValues.begin());
    prevRightValues.erase(prevRightValues.begin());
    prevLeftValues.push_back(leftError);
    prevRightValues.push_back(leftError);

    if (fabs(leftError) < 13) {
      leftIntegral += leftError * delta_time;
    } else {
      leftIntegral = 0;
    }

    if (fabs(rightError) < 13) {
      rightIntegral += rightError * delta_time;
    } else {
      rightIntegral = 0;
    }
    
    float greatestLeft = 0;
    float greatestRight = 0;

    for (float i = 0; i < prevLeftValues.size(); i++) {
      if (fabs(prevLeftValues[i]) > greatestLeft) {
        greatestLeft = fabs(prevLeftValues[i]); 
      }
    }
    for (float i = 0; i < prevRightValues.size(); i++) {
      if (fabs(prevRightValues[i]) > greatestRight) {
        greatestRight = fabs(prevRightValues[i]); 
      }
    }

    float leftSpeed = kP*leftError + kI*leftIntegral + kD*leftDerivative;
    float rightSpeed = kP*rightError + kI*rightIntegral + kD*rightDerivative;

    if ((fabs(leftSpeed)) > maxSpeed) {
      leftSpeed = maxSpeed * ((leftSpeed < 1) ? -1 : 1);
    }
    if ((fabs(rightError)) > maxSpeed) {
      rightSpeed = maxSpeed * ((rightSpeed < 1) ? -1 : 1);
    }

    mLUpper.spin(forward, leftSpeed, percent);
    mLLower.spin(forward, leftSpeed, percent);
    mRUpper.spin(forward, rightSpeed, percent);
    mRLower.spin(forward, rightSpeed, percent);
    
    if (((greatestLeft < 2) && (greatestRight < 2)) || (Brain.timer(msec)-start_time > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }
    wait(20, msec);
  }
}

void moveSlow(float d, int maxSpeed, int vecCount, int timeLimit) {
  int start_time = Brain.timer(msec);

  float kP = 5.2;
  float kI = 0.00001;
  float kD = 3.2;

  bool done = false;

  float leftIntegral = 0;
  float rightIntegral = 0;

  std::vector<float> prevLeftValues;
  std::vector<float> prevRightValues;
  repeat(vecCount) {
    prevLeftValues.push_back(999);
    prevRightValues.push_back(999);
  }

  mLLower.setPosition(0, degrees);
  mRLower.setPosition(0, degrees);

  int prevTime = 0;
  float prevLeftError = 0;
  float prevRightError = 0;

  while(!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;
    prevTime = currentTime;

    float currentLeft = (mLLower.position(degrees)/360.0) * 4.0 * M_PI;
    float currentRight = (mRLower.position(degrees)/360.0) * 4.0 * M_PI;

    float leftError = d - currentLeft;
    float rightError = d - currentRight;

    float leftDerivative = (leftError - prevLeftError) / delta_time;
    float rightDerivative = (rightError - prevRightError) / delta_time;

    prevLeftError = leftError;
    prevRightError = rightError;

    prevLeftValues.erase(prevLeftValues.begin());
    prevRightValues.erase(prevRightValues.begin());
    prevLeftValues.push_back(leftError);
    prevRightValues.push_back(leftError);

    if (fabs(leftError) < 6) {
      leftIntegral += leftError * delta_time;
    } else {
      leftIntegral = 0;
    }

    if (fabs(rightError) < 6) {
      rightIntegral += rightError * delta_time;
    } else {
      rightIntegral = 0;
    }
    
    float greatestLeft = 0;
    float greatestRight = 0;

    for (float i = 0; i < prevLeftValues.size(); i++) {
      if (fabs(prevLeftValues[i]) > greatestLeft) {
        greatestLeft = fabs(prevLeftValues[i]); 
      }
    }
    for (float i = 0; i < prevRightValues.size(); i++) {
      if (fabs(prevRightValues[i]) > greatestRight) {
        greatestRight = fabs(prevRightValues[i]); 
      }
    }

    float leftSpeed = kP*leftError + kI*leftIntegral + kD*leftDerivative;
    float rightSpeed = kP*rightError + kI*rightIntegral + kD*rightDerivative;

    if ((fabs(leftSpeed)) > maxSpeed) {
      leftSpeed = maxSpeed * ((leftSpeed < 1) ? -1 : 1);
    }
    if ((fabs(rightError)) > maxSpeed) {
      rightSpeed = maxSpeed * ((rightSpeed < 1) ? -1 : 1);
    }

    mLUpper.spin(forward, leftSpeed, percent);
    mLLower.spin(forward, leftSpeed, percent);
    mRUpper.spin(forward, rightSpeed, percent);
    mRLower.spin(forward, rightSpeed, percent);
    
    if (((greatestLeft < 2) && (greatestRight < 2)) || (Brain.timer(msec)-start_time > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }
    wait(20, msec);
  }
}

void turnToPoint(float x, float y, int vecCount, int timeLimit) {
  int startTime = Brain.timer(msec);
  bool done = false;
  float integral = 0;

  float kP = .5;
  float kI = 0.0012;
  float kD = 0.16;

  std::vector<float> prevValues;
  repeat(vecCount) {
    prevValues.push_back(999);
  }

  int prevTime = 0;
  float prevError = 0;

  while (!done) {
    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;

    float currY = GPS.yPosition(inches);
    float currX = GPS.xPosition(inches);
    /*std::cout << currY << std::endl;
    std::cout << currX << std::endl << std::endl;*/

    prevTime = currentTime;

    float angleError = round((180/M_PI)*atan2(y-currY, x-currX) - GPS.heading(degrees));
    //std::cout << currentAngle << std::endl;
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }
    std::cout << angleError << std::endl;
    std::cout << GPS.heading(degrees) << std::endl << std::endl;

    float derivative = (angleError - prevError) / delta_time;

    prevError = angleError;

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 5) {
      integral += angleError * delta_time;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    float driveSpeed = kP*angleError + kI*integral + kD*derivative;

    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);
    //std::cout << .007*integral << std::endl;
    
    
    if ((greatest < 3) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);

  }
}