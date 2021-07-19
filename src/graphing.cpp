#include "graphing.h"
#include "vector"
#include "cmath"

float graphVariable = 0;

int makeGraph() {
    
  using namespace std;

  int graphBottomBorder = 225;
  int graphLeftBorder = 15;
  int graphTopBorder = 25;
  int graphRightBorder = 475;

  bool canceled = false;
  vector<int> graphData;

  Brain.Screen.clearScreen();

  float least = graphVariable;
  float greatest = graphVariable;

  while(!canceled) {

    graphData.push_back(graphVariable);

    if (graphVariable < least) { least = graphVariable; }
    if (graphVariable > greatest) { greatest = graphVariable; }

    Brain.Screen.setPenColor("#DDDDDD");
    Brain.Screen.drawLine(graphLeftBorder, graphTopBorder, graphLeftBorder, graphBottomBorder); //draw y axis of graph
    Brain.Screen.drawLine(graphLeftBorder, graphBottomBorder, graphRightBorder, graphBottomBorder); //draw x axis of graph
    Brain.Screen.setPenColor("#11CC11");
    
    for(int i = 1, prevX = graphLeftBorder, prevY = graphBottomBorder; i < graphData.size(); i++) {
      
      int pointX = i * round((graphRightBorder-graphLeftBorder) / graphData.size()) + graphLeftBorder;
      int pointY = 272 - ((graphData.at(i) - least) * ((graphBottomBorder-graphTopBorder) / (greatest-least)));
      //pointY = 50;
      Brain.Screen.drawLine(prevX, prevY, pointX, pointY);

      prevX = pointX;
      prevY = pointY;

    }

    wait(200, msec);

    Brain.Screen.clearScreen();
  
  }
  return 1;
}
