#include "pathing.h"
//holy shit writing this code is like traversing the ninth circle of hell

Event::Event(eventType eT, int a, int b, bool seq) {
  type = eT;
  v1 = a;
  v2 = b;
  sequential = seq;
}
Event::Event(eventType eT, int a, bool seq) {
  type = eT;
  v1 = a;
  v2 = 0;
  sequential = seq;
}
Event::Event(eventType eT, bool seq) {
  type = eT;
  v1 = 0;
  v2 = 0;
  sequential = seq;
}
Event::Event(void) {
  type = empty;
  v1 = 0;
  v2 = 0;
  sequential = false;
}

Point::Point(float X, float Y, int IND, bool END) {
  x = X;
  y = Y;
  ind = IND;
  end = END;
  e = Event(empty, 0, 0, false);
}
Point::Point(float X, float Y, int IND) {
  x = X;
  y = Y;
  ind = IND;
  end = false;
  e = Event(empty, 0, 0, false);
}
Point::Point(float X, float Y, bool END) {
  //initialize the Point struct's constructor
  x = X;
  y = Y;
  ind = 0;
  end = END;
  e = Event(empty, 0, 0, false);
}
Point::Point(float X, float Y) {
  x = X;
  y = Y;
  ind = 0;
  end = false;
  e = Event(empty, 0, 0, false);
  //4478F: To Pay Respects
}
Point::Point(void) {
  x = 0;
  y = 0;
  ind = 0;
  end = false;
  e = Event(empty, 0, 0, false);
}

Path::Path(Point a, Point b, Point c, Point d, int res) {
  /**
  * Constructs a path object, containing an array of points along the path.
  * Then it generates points to fill the array.
  */
  this->p[res] = {0, 0, false};
  this->resolution = res;
  generateCubicBezier(this, a, b, c, d, res);
}

std::tuple<Point, float> Path::nearestPointAndDistance(Point inputP) {
  /**
  * Finds the nearest point on the path to the robot's position as well as the distance to it
  */
  Point closest = this->p[0];

  float dLeast = sqrt(sq(closest.x - inputP.x) + sq(closest.y - inputP.y));
 
  for(int t = resolution; t >= 0; t--) {
    Point tempPoint = this->p[t];

    float d = sqrt(sq(tempPoint.x - inputP.x) + sq(tempPoint.y - inputP.y));

    if (d < dLeast) {
      dLeast = d;
      closest = tempPoint;
    }
  }

  return std::make_tuple(closest, dLeast);
}

void Path::setPointAtIndex(int index, Point val) {
  /** allows access to private p[] variable to add points*/
  this->p[index] = val;
}

void Path::setEventAtIndex(int index, Event val) {
  /** allows access to private p[] variable to add points*/
  this->p[index].e = val;
}

Point Path::getPointAtIndex(int i) {
  /** returns the value in array p at index i */
  return this->p[i];
}

void generateCubicBezier(Path* path, Point a, Point b, Point c, Point d, int res) {
  /**
  * this generates res amount of points along a cubic bezier curve defined by
  * points a, b, c, d, then inserts the points into the given path
  */
  for(int i = 0, t = 0; i < res; i++, t += (1/res)) {
    int p_x = ( cb(1-t) * a.x)
              + ( (3*t) * (sq(1-t) * b.x))
              + ( (3*(t*t)) * (1-t) * c.x)
              + ( cb(t) * d.x);

    int p_y = ( cb(1-t) * a.y)
              + ( (3*t) * (sq(1-t) * b.y))
              + ( (3*(t*t)) * (1-t) * c.y)
              + ( cb(t) * d.y);

    path->setPointAtIndex(i, Point(p_x, p_y, i, t == 1));
    path->setEventAtIndex(i, Event(empty, false));
  } 
}

int eventListener(Path*) {
  //run in thread, needs to be async
  bool done = false;
  while(!done) {
    //code
  }
  return 0;
}