#ifndef POINT_H
#define POINT_H


struct Point {
  float x;
  float y;
  float theta;

  bool operator<(const Point& rhs) const
  {
    if (this->x == rhs.x) return this->y < rhs.y;
    return this->x < rhs.x;
   }
};

#endif // POINT_H
