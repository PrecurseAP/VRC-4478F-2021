#include "pathing.h"
//holy shit writing this code is like traversing the ninth circle of hell
using namespace pathing;

pathing::point::point(float X, float Y, bool END) {
  x = X;
  y = Y;
  end = END;
}

class pathing::Path {
  private:
    point p[];
  public:
    Path( point a, point b, point c, point d, int res ) {
      /**
      * Constructs a path object, containing an array of points along the path.
      * By default, the path generated is a cubic bezier curve.
      */
      this->p[res] = {0,0,false};
      pathing::generateCubicBezier(this, a, b, c, d, res);
    }
    std::tuple<point, float> nearestPointAndDistance(point) {
      point nP = {0,0,false};
      return std::make_tuple(nP, 0.0);
    }

    void setPointAtIndex(int index, point val) {
      this->p[index] = val;
    }
};

void pathing::generateCubicBezier(Path* path, point a, point b, point c, point d, int res) {
  /*
  * i am a fucking psychopath LOOK AT THIS
  * this generates res amount of points along a cubic bezier curve defined by
  * points a, b, c, d
  */
  for(int i = 0, t = 0; i < res; i++, t += (1/res)) {
    int p_x = ( pow((1-t) , 3) * a.x)
              + ( (3*t) * (pow((1-t) , 2) * b.x)
              + ( (3*(t*t)) * (1-t) * c.x)
              + ( (t*t*t) * d.x);
    int p_y = ( pow((1-t) , 3) * a.y)
              + ( (3*t) * (pow((1-t) , 2) * b.y)
              + ( (3*(t*t)) * (1-t) * c.y)
              + ( (t*t*t) * d.y);

    path->setPointAtIndex(i, point(p_x, p_y, t == 1));
  } 
}