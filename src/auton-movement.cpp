
//contains code that moves the robot during autonomous
#include "vex.h"
#include "odometry.h"
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
  mArm.spinToPosition(1150, degrees, wait);
}

void lowerLift(bool wait = true) {
  mArm.spinToPosition(0, degrees, wait);
}

void spinConveyor() {
  mConveyor.spin(forward, 180, rpm);
}

void lowerTilter(bool wait = true) {
  mLTray.setVelocity(100, percent);
  mRTray.setVelocity(100, percent);
  mLTray.spinToPosition(-580, degrees, false);
  mRTray.spinToPosition(-580, degrees, wait);
}

void raiseTilterWithGoal(bool wait = true) {
  mLTray.setVelocity(100, percent);
  mRTray.setVelocity(100, percent);
  mLTray.spinToPosition(-320, degrees, false);
  mRTray.spinToPosition(-320, degrees, wait);
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
void spotTurn(float theta, int maxSpeed) {
  bool done = false;
  float integral = 0;
  std::vector<float> prevValues;
  repeat(35) {
    prevValues.push_back(999);
  }

  while (!done) {
    float currentAngle = GYRO.heading(degrees);
    
    float angleError = theta - currentAngle;
    //std::cout << currentAngle << std::endl;
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }

    prevValues.erase(prevValues.begin());
    prevValues.push_back(angleError);

    if (fabs(angleError) < 15) {
      integral += angleError;
    } else {
      integral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

    mLUpper.spin(forward, angleError + .00005*integral, percent);
    mLLower.spin(forward, angleError + .00005*integral, percent);
    mRUpper.spin(forward, -angleError - .00005*integral, percent);
    mRLower.spin(forward, -angleError - .00005*integral, percent);
    std::cout << greatest << std::endl;
    
    
    if (greatest < 3) {
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

//doesnt work
void turnMoveToPoint(float gx, float gy, int maxSpeed) {
  bool done = false;
  float integral = 0;

  //initialize the vector containing last 30 error values
  std::vector<float> prevValues;
  repeat(30) {
    prevValues.push_back(999);
  }
  float tDiff;
  //Graph g = Graph(&tDiff);
  //begin turning loop
  while(!done/* && !Controller1.ButtonY.pressing()*/) {
    float cx = GPS.xPosition(inches);
    float cy = GPS.yPosition(inches);
    float ct = GYRO.heading(degrees) + 180;

    tDiff = angleWrap((180/M_PI)*atan2( gy-cy, gx-cx ) - ct);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0,0);
    Brain.Screen.print(tDiff);
    std::cout << tDiff << std::endl;
    if (tDiff > 180) {
      tDiff = tDiff - 360;
    } else if (tDiff < -180) {
      tDiff = 360 + tDiff;
    }
    //g.updateData(tDiff);
    //erase oldest error value and insert the newest one
    prevValues.erase(prevValues.begin());
    prevValues.push_back(tDiff);

    if (fabs(tDiff) < 15) {
      integral += tDiff;
    } else {
      integral = 0;
    }
    float total = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      total += prevValues[i];
    }

    //spin motors at PI value
    moveRightSide(-tDiff*5/* - .001*integral*/);
    moveLeftSide(tDiff*.5 /*+ .001*integral*/);

    //check if the average error over 600 ms is below 1 degree, if yes then stop loop
    if (fabs((total/prevValues.size())) < 1) {
      done = true;
      stopAllDrive(hold);
      break;
    }

    wait(20, msec);
  }

  prevValues.clear();
  /*
  repeat(30) {
    prevValues.push_back(999);
  }

  integral = 0;
  stopAllDrive(coast);
  g.drawGraph();
  //done = false;
  while(!done) {

    float cx = GPS.xPosition(inches);
    float cy = GPS.yPosition(inches);
    float ct = (M_PI/180) * angleWrap(450 - GYRO.heading(degrees));

    float distanceToGoal = hypot(gx-cx, gy-cy);//sqrt( (gx-cx)*(gx-cx) + (gy-cy)*(gy-cy) );

    prevValues.erase(prevValues.begin());
    prevValues.push_back(distanceToGoal);

    float angleToGoal = (180/M_PI)*atan2( gy-cy, gx-cx ) - (180/M_PI)*ct;

    if (fabs(distanceToGoal) < 15) {
      integral += distanceToGoal;
    } else {
      integral = 0;
    }

    float total = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      total += prevValues[i];
    }

    if (fabs((total/prevValues.size())) < 1) {
      done = true;
      break;
    }

    moveLeftSide(distanceToGoal*1.1 + integral*0.0001 + 0*angleToGoal);
    moveRightSide(distanceToGoal*1.1 + integral*0.0001 - 0*angleToGoal);
    wait(20, msec);
  }*/
}

void move(float d, int maxSpeed) {
  bool done = false;
  float leftIntegral = 0;
  float rightIntegral = 0;
  float startAngle = GYRO.heading(degrees);
  std::vector<float> prevValues;
  repeat(50) {
    prevValues.push_back(999);
  }
  mLLower.setPosition(0, degrees);
  mRLower.setPosition(0, degrees);
  while(!done) {
    float currentLeft = (mLLower.position(degrees)/360) * 4 * M_PI;
    float currentRight = (mRLower.position(degrees)/360) * 4 * M_PI;
    
    float angleError = startAngle - GYRO.heading(degrees);
    float leftError = d - currentLeft;
    float rightError = d - currentRight;
    std::cout << "leftError: " << leftError << std::endl;
    std::cout << "rightError: " << rightError << std::endl;

    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }
    std::cout << "angleError: " << angleError << std::endl << std::endl;
    prevValues.erase(prevValues.begin());
    prevValues.push_back(leftError);

    if (fabs(leftError) < 12) {
      leftIntegral += leftError;
    } else {
      leftIntegral = 0;
    }

    if (fabs(rightError) < 12) {
      rightIntegral += rightError;
    } else {
      rightIntegral = 0;
    }
    
    float greatest = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      if (fabs(prevValues[i]) > greatest) {
        greatest = fabs(prevValues[i]); 
      }
    }

/*
    if ((fabs(leftError)*2) > maxSpeed) {
      leftError = maxSpeed * ((leftError < 1) ? -1 : 1);
    }
    if ((fabs(rightError)*2) > maxSpeed) {
      rightError = maxSpeed * ((rightError < 1) ? -1 : 1);
    }*/

    mLUpper.spin(forward, 3.5*leftError + .0001*leftIntegral + 1.5*angleError, percent);
    mLLower.spin(forward, 3.5*leftError + .0001*leftIntegral + 1.5*angleError, percent);
    mRUpper.spin(forward, 3.5*rightError + .0001*rightIntegral - 1.5*angleError, percent);
    mRLower.spin(forward, 3.5*rightError + .0001*rightIntegral - 1.5*angleError, percent);
    
    if (greatest+angleError+rightError < 5) {
      done = true;
      stopAllDrive(hold);
      break;
    }
    wait(20, msec);
  }
}
