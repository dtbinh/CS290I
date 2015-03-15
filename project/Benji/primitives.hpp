#ifndef _PRIMITIVES_H
#define _PRIMITIVES_H

#include <GL/glut.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

//matrix class
class Matrix3X3 {
  public:
    GLdouble a, b, c, d, e, f, g, h, i;
    Matrix3X3(GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble);
};

//vector class
class Vec3f {
  public:
    GLdouble x, y, z;
    Vec3f(GLdouble x=0.0, GLdouble y=0.0, GLdouble z=0.0);
    
    Vec3f& operator=(const Vec3f& v)
      { x = v.x; y = v.y; z = v.z; return *this; }
    
    Vec3f operator+(const Vec3f&) const;
    Vec3f operator-(const Vec3f&) const;
    Vec3f operator/(const GLdouble&) const;
    GLdouble mag () { return sqrt(pow(x,2) + pow(y,2) + pow(z,2)); }
    Vec3f operator*(const Matrix3X3&) const;
};

//primitives class
class PRIMITIVE
{
  public:
    virtual ~PRIMITIVE(){}
    Vec3f COLOR;
    virtual void draw_p() = 0;
};

class PLANE:public PRIMITIVE{
  private:
    vector<Vec3f> POINTS;
    Vec3f COLOR;
    int WALL;
    int CEILING;
    int FLOOR;
  public:
    PLANE(vector<Vec3f> points, Vec3f color, int wall, int ceiling, int floor);
    void draw_p();
    virtual ~PLANE() {}
};

class SPHERE:public PRIMITIVE{
  private:
    Vec3f CENTER;
    float RADIUS;
  public:
    SPHERE(Vec3f center, float radius, Vec3f color);
    void draw_p();
    virtual ~SPHERE() {}
};
/*
class CYLINDER:public PRIMITIVE{
  private:
    Vec3f CENTER;
    float RADIUS;
    float HEIGHT;
    GLdouble TRANSFORMATION[16];
    GLUquadric* qobj;
  public:
//    CYLINDER(Vec3f color, float radius, float height);
    CYLINDER(Vec3f color, float radius, float height, GLdouble transformation[16]);
    void draw_p();
    virtual ~CYLINDER() {}
};
*/
#endif
