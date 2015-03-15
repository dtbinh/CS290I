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

//rotation matricies for look left, look right, straife left and straife right
//Matrix3X3 mL (cos(.01), 0.0, sin(.01), 0.0, 1.0, 0.0,0.0-sin(.01),0.0,cos(.01));
//Matrix3X3 mL90 (cos(1.59155), 0.0, sin(1.59155), 0.0, 1.0, 0.0,0.0-sin(1.59155),0.0,cos(1.59155));
//Matrix3X3 mR (cos(.01), 0.0,0.0 -sin(.01), 0.0, 1.0, 0.0,sin(.01),0.0,cos(.01));

//setting matrix
Matrix4X4::Matrix4X4 (GLdouble aa, GLdouble bb, GLdouble cc, GLdouble dd, GLdouble ee, GLdouble ff, GLdouble gg, GLdouble hh, GLdouble ii, GLdouble jj, GLdouble kk, GLdouble ll, GLdouble mm, GLdouble nn, GLdouble oo, GLdouble pp) {
  a = aa;
  b = bb;
  c = cc;
  d = dd; 
  e = ee;
  f = ff;
  g = gg;
  h = hh;
  i = ii;
  j = jj;
  k = kk;
  l = ll;
  m = mm;
  n = nn;
  o = oo;
  p = pp;
}
//set vector
Vec3f::Vec3f (GLdouble a, GLdouble b, GLdouble c) {
  x = a;
  y = b;
  z = c;
}
//= overload
Vec3f Vec3f::operator= (const Vec3f& v) const
{
  Vec3f result(v.x, v.y, v.z);
  return result;
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

PLANE::PLANE(vector<Vec3f> points, Vec3f centroid, Vec3f color, int wall, int ceiling, int floor){
	POINTS = points;
  CENTROID = centroid;
  COLOR = color;
  WALL = wall;
  CEILING = ceiling;
  FLOOR = floor;
}

void PLANE::draw_p(){
	glPushMatrix();
  	glRotatef(90.0, 0.0, 1.0, 0.0);
    glTranslatef(-3.5,0.0,4.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(0,1,0,1);
    glVertex3f(CENTROID.x, CENTROID.y, CENTROID.z);
 	  for (int i = 0; i < POINTS.size(); i++){
    	glVertex3f(POINTS[i].x,POINTS[i].y,POINTS[i].z);
    }
    glEnd();
	glPopMatrix();
  return;
} 

SPHERE::SPHERE(Vec3f center, Vec3f color, float radius, Matrix4X4 transformation){
	CENTER = center;
  COLOR = color;
  RADIUS = radius;
  TRANSFORMATION = transformation;
}

void SPHERE::draw_p(){
	glPushMatrix();
  	glRotatef(90.0, 0.0, 1.0, 0.0);
    glTranslatef(-3.5,0.0,4.0);
    glutSolidSphere(1.0,10.0,10.0);
  glPopMatrix();
	return;
} 

CYLINDER::CYLINDER(Vec3f center, Vec3f color, float radius, float height, Matrix4X4 transformation){
	CENTER = center;
  COLOR = color;
  RADIUS = radius;
  HEIGHT = height;
  TRANSFORMATION = transformation;
  qobj = gluNewQuadric();
  gluQuadricNormals(qobj, GLU_SMOOTH);
}

void CYLINDER::draw_p(){
	glPushMatrix();
  	glRotatef(90.0, 0.0, 1.0, 0.0);
    glTranslatef(-3.5,0.0,4.0);
    gluCylinder(qobj,RADIUS,HEIGHT,10.0, 10.0,10.0);
  glPopMatrix();
	return;
} 
