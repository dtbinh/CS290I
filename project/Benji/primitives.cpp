#include "primitives.hpp"

//setting matrix
Matrix3X3::Matrix3X3 (GLdouble j, GLdouble k, GLdouble l, GLdouble m, GLdouble n, GLdouble o, GLdouble p, GLdouble q, GLdouble r) {
  a = j;
  b = k;
  c = l;
  d = m; 
  e = n;
  f = o;
  g = p;
  h = q;
  i = r;
}

//set vector
Vec3f::Vec3f (GLdouble a, GLdouble b, GLdouble c) {
  x = a;
  y = b;
  z = c;
}

///define overload minus
Vec3f Vec3f::operator- (const Vec3f& v) const
{
  Vec3f result((this->x - v.x),(this->y - v.y),(this->z - v.z));
  return result;
}

//define overload plus
Vec3f Vec3f::operator+ (const Vec3f& v) const
{
  Vec3f result((this->x + v.x),(this->y + v.y),(this->z + v.z));
  return result;
}

//define / overload by double
Vec3f Vec3f::operator/ (const GLdouble& m) const
{
  Vec3f result((this->x/m),(this->y/m),(this->z/m));
  return result;
}
//define matrix vector multiply
Vec3f Vec3f::operator* (const Matrix3X3& m) const
{
  GLdouble x = this->x;
  GLdouble y = this->y;
  GLdouble z = this->z;
  Vec3f result(m.a*x+m.b*y+m.c*z,m.d*x+m.e*y+m.f*z,m.g*x+m.h*y+m.i*z);
  return result;  
}

PLANE::PLANE(vector<Vec3f> points, Vec3f color, int wall, int ceiling, int floor){
  POINTS = points;
  COLOR = color;
  WALL = wall;
  CEILING = ceiling;
  FLOOR = floor;
}

void PLANE::draw_p(){
      glPushMatrix();
        glBegin(GL_TRIANGLE_FAN);
          glColor4f(COLOR.x,COLOR.y,COLOR.z,0);
 	  for (int i = 0; i < POINTS.size(); i++){
            glVertex3f(POINTS[i].x,POINTS[i].y,POINTS[i].z);
          }
        glEnd();
      glPopMatrix();
      return;
} 

SPHERE::SPHERE(Vec3f center, float radius, Vec3f color){
  CENTER = center;
  COLOR = color;
  RADIUS = radius;
}

void SPHERE::draw_p(){
      glPushMatrix();
        glColor4f(COLOR.x,COLOR.y,COLOR.z,0);
        glTranslatef(CENTER.x, CENTER.y, CENTER.z);
        glutSolidSphere(RADIUS,20.0,20.0);
      glPopMatrix();
      return;
} 
/*
//CYLINDER::CYLINDER(Vec3f center, Vec3f color, float radius, float height ){
CYLINDER::CYLINDER(Vec3f center, Vec3f color, float radius, float height, GLdouble transformation[16] ){
  CENTER = center;
  COLOR = color;
  RADIUS = radius;
  HEIGHT = height;
  for (int i = 0; i < 16; i++){
    TRANSFORMATION[i] = transformation[i]; }
  qobj = gluNewQuadric();
  gluQuadricNormals(qobj, GLU_SMOOTH);
}

void CYLINDER::draw_p(){
      glPushMatrix();
        glMultMatrixd(TRANSFORMATION);
        glColor4f(COLOR.x,COLOR.y,COLOR.z,0);
        gluCylinder(qobj,RADIUS,HEIGHT,20.0, 20.0,20.0);
      glPopMatrix();
      return;
} */
