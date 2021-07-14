#include "graphing.h"
#include "vector"
#include "cmath"

int startGraph(float* input, int period) {
  
  using namespace std;

  int graphBottomBorder = 240;
  int graphLeftBorder = 15;
  int graphTopBorder = 5;
  int graphRightBorder = 475;

  bool canceled = false;
  vector<int> graphData;

  Brain.Screen.clearScreen();

  float least = *input;
  float greatest = *input;

  while(!canceled) {

    graphData.push_back(*input);

    if (*input < least) { least = *input; }
    if (*input > greatest) { greatest = *input; }

    Brain.Screen.setPenColor("#DDDDDD");
    Brain.Screen.drawLine(graphLeftBorder, graphTopBorder, graphLeftBorder, graphBottomBorder); //draw y axis of graph
    Brain.Screen.drawLine(graphLeftBorder, graphBottomBorder, graphRightBorder, graphBottomBorder); //draw x axis of graph

    Brain.Screen.setPenColor("11CC11");
    for(int i = 1, prevX = 0, prevY = 0; i < graphData.size(); i++) {
      
      int pointX = i * ( round(graphRightBorder-graphLeftBorder) / graphData.size() );
      int pointY = (graphData.at(i) * ((graphBottomBorder-graphTopBorder) / (greatest-least))) - least;

      Brain.Screen.drawLine(prevX, prevY, pointX, pointY);

      prevX = pointX;
      prevY = pointY;

    }

    wait(period, msec);

    Brain.Screen.clearScreen();
  
  }

  return 69420;
}
