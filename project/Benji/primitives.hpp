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

//matrix class
class Matrix4X4 {
  public:
    GLdouble a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
    Matrix4X4(GLdouble a=0.0, GLdouble b=0.0, GLdouble c=0.0, GLdouble d=0.0, GLdouble e=0.0, GLdouble f=0.0, GLdouble g=0.0, GLdouble h=0.0, GLdouble i=0.0, GLdouble j=0.0, GLdouble k=0.0, GLdouble l=0.0, GLdouble m=0.0, GLdouble n=0.0, GLdouble o=0.0, GLdouble p=0.0);
};

//vector class
class Vec3f {
  public:
    GLdouble x, y, z;
    Vec3f(GLdouble x=0.0, GLdouble y=0.0, GLdouble z=0.0);
    Vec3f operator=(const Vec3f&) const;
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
    Matrix4X4 TRANSFORMATION;
    virtual void draw_p() = 0;
};

class PLANE:public PRIMITIVE{
  private:
    vector<Vec3f> POINTS;
    Vec3f CENTROID;
    Vec3f COLOR;
    int WALL;
    int CEILING;
    int FLOOR;
  public:
    PLANE(vector<Vec3f> points, Vec3f centroid, Vec3f color, int wall, int ceiling, int floor);
    void draw_p();
    virtual ~PLANE() {}
};

class SPHERE:public PRIMITIVE{
  private:
    Vec3f CENTER;
    float RADIUS;
  public:
    SPHERE(Vec3f center, Vec3f color, float radius, Matrix4X4 transformation);
    void draw_p();
    virtual ~SPHERE() {}
};

class CYLINDER:public PRIMITIVE{
  private:
    Vec3f CENTER;
    float RADIUS;
    float HEIGHT;
    GLUquadric* qobj;
  public:
    CYLINDER(Vec3f center, Vec3f color, float radius, float height, Matrix4X4 transformation);
    void draw_p();
    virtual ~CYLINDER() {}
};

#endif
