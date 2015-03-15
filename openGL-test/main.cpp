//#include <OpenGL/gl.h>
//#include <OpenGL/glu.h>
#include <GL/glut.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
//global variables for camera
GLdouble eyeX;
GLdouble eyeY;
GLdouble eyeZ;
GLdouble centerX;
GLdouble centerY;
GLdouble centerZ;

//matrix class
class Matrix3X3 {
  public:
    GLdouble a, b, c, d, e, f, g, h, i;
    Matrix3X3(GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble);
};

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
Matrix3X3 mL (cos(.01), 0.0, sin(.01), 0.0, 1.0, 0.0,0.0-sin(.01),0.0,cos(.01));
Matrix3X3 mL90 (cos(1.59155), 0.0, sin(1.59155), 0.0, 1.0, 0.0,0.0-sin(1.59155),0.0,cos(1.59155));
Matrix3X3 mR (cos(.01), 0.0,0.0 -sin(.01), 0.0, 1.0, 0.0,sin(.01),0.0,cos(.01));

//matrix class
class Matrix4X4 {
  public:
    GLdouble a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
    Matrix4X4(GLdouble a=0.0, GLdouble b=0.0, GLdouble c=0.0, GLdouble d=0.0, GLdouble e=0.0, GLdouble f=0.0, GLdouble g=0.0, GLdouble h=0.0, GLdouble i=0.0, GLdouble j=0.0, GLdouble k=0.0, GLdouble l=0.0, GLdouble m=0.0, GLdouble n=0.0, GLdouble o=0.0, GLdouble p=0.0);
    Matrix4X4& operator=(const Matrix4X4& z){ a = z.a; b = z.b; c = z.c; d = z.d; e = z.e; f = z.f; g = z.g; h = z.h; i = z.i; j = z.j; k = z.k; l = z.l; m =z.m; n= z.n; o =z.o; p =z.p; return *this; }
    GLdouble array(){ GLdouble result[]={a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p}; return *result; }
};
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
    Vec3f CENTROID;
    Vec3f COLOR;
    int WALL;
    int CEILING;
    int FLOOR;
  public:
    PLANE(vector<Vec3f> points, Vec3f centroid, Vec3f color, int wall, int ceiling, int floor){
      POINTS = points;
      CENTROID = centroid;
      COLOR = color;
      WALL = wall;
      CEILING = ceiling;
      FLOOR = floor;
    }
    void draw_p(){
      glPushMatrix();
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
    virtual ~PLANE() {}
};

class SPHERE:public PRIMITIVE{
  private:
    Vec3f CENTER;
    float RADIUS;
  public:
    SPHERE(Vec3f center, float radius, Vec3f color){
      CENTER = center;
      COLOR = color;
      RADIUS = radius;
    }
    void draw_p(){
      glPushMatrix();
        glColor4f(COLOR.x,COLOR.y,COLOR.z,0);
        glTranslatef(CENTER.x, CENTER.y, CENTER.z);
        glutSolidSphere(RADIUS,20.0,20.0);
      glPopMatrix();
      return;
    } 
    virtual ~SPHERE() {}
};

class CYLINDER:public PRIMITIVE{
  private:
    Vec3f CENTER;
    float RADIUS;
    float HEIGHT;
    Matrix4X4 TRANSFORMATION;
    GLUquadric* qobj;
  public:
    CYLINDER(Vec3f center, Vec3f color, float radius, float height, Matrix4X4 transformation){
      CENTER = center;
      COLOR = color;
      RADIUS = radius;
      HEIGHT = height;
      TRANSFORMATION = transformation;
      qobj = gluNewQuadric();
      gluQuadricNormals(qobj, GLU_SMOOTH);
    }
    void draw_p(){
      glPushMatrix();
        const GLdouble p = TRANSFORMATION.array();
        glMultMatrixd(&p);
        gluCylinder(qobj,RADIUS,HEIGHT,20.0, 20.0,20.0);
      glPopMatrix();
      return;
    } 
    virtual ~CYLINDER() {}
};


//////////////////////////////////////////////////////////////////////////////////
// Draws to the OpenGL window
//////////////////////////////////////////////////////////////////////////////////
void display()
{
  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //set matrix to default
  glLoadIdentity();
  //set what we are looking at
  gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, 0.0, 1.0, 0.0);


  Vec3f center (-1, 0, 0);
  Vec3f color (1.0, 0, 0);
  SPHERE temp(center, .1, color);
  temp.draw_p();
  
  glutSwapBuffers();
}

//mod camera eye to go forward
void forward(Vec3f norm, Vec3f eye)
{
      Vec3f eyeFinal = eye - norm/(10*norm.mag());
      eyeX = eyeFinal.x;
      eyeY = eyeFinal.y;
      eyeZ = eyeFinal.z;
      
}

//mod camera eye to go back
void back(Vec3f norm, Vec3f eye)
{
      Vec3f eyeFinal = eye + norm/(10*norm.mag());
      eyeX = eyeFinal.x;
      eyeY = eyeFinal.y;
      eyeZ = eyeFinal.z;
     
}

//mod camera center to look left
void left( Vec3f norm, Vec3f eye)
{
      Vec3f temp = norm*mL;
      Vec3f centerFinal = eye+ temp;
      centerX = centerFinal.x;
      centerY = centerFinal.y;
      centerZ = centerFinal.z;
 
}

//mod camera center to look right
void right(Vec3f norm, Vec3f eye)
{
      Vec3f temp = norm*mR;
      Vec3f centerFinal = eye+ temp;
      centerX = centerFinal.x;
      centerY = centerFinal.y;
      centerZ = centerFinal.z;
}

//mod camera eye and center to straife left
void strafeLeft(Vec3f norm, Vec3f eye, Vec3f center)
{
  Vec3f leftTemp90 = norm*mL90;
  Vec3f left90mod = leftTemp90/(10*norm.mag());
  Vec3f left90eye = eye + left90mod;
  Vec3f left90center = center + left90mod;
  eyeX = left90eye.x;
  eyeY = left90eye.y;
  eyeZ = left90eye.z;
  centerX = left90center.x;
  centerY = left90center.y;
  centerZ = left90center.z;
 
}

//mod camera eye and center to straife right
void strafeRight(Vec3f norm, Vec3f eye, Vec3f center)
{
  Vec3f leftTemp90 = norm*mL90;
  Vec3f left90mod = leftTemp90/(10*norm.mag());
  Vec3f right90eye = eye - left90mod;
  Vec3f right90center = center - left90mod;
  eyeX = right90eye.x;
  eyeY = right90eye.y;
  eyeZ = right90eye.z;
  centerX = right90center.x;
  centerY = right90center.y;
  centerZ = right90center.z;
  

}




//////////////////////////////////////////////////////////////////////////////////
// Handles keyboard events
//////////////////////////////////////////////////////////////////////////////////
void keyboard(unsigned char k, int x, int y)
{
  //create center, eye and norm vectors
  Vec3f center (centerX, centerY, centerZ);
  Vec3f eye (eyeX, eyeY, eyeZ);
  Vec3f norm = (center - eye);

  switch (k)
  {
    // the escape key and 'q' quit the program
    case 27:
    case 'q':
      exit(0);
      break;
    case 'j':
     left( norm, eye);
     break;
    case 'l':
      right (norm, eye);
      break;
    case 'i':
      back(norm, eye);
     break;
    case 'k':
      forward(norm, eye);
     break;
    case 'J':
      strafeLeft(norm, eye, center);
     break;
    case 'L':
      strafeRight(norm, eye, center);
     break;
  }
  glutPostRedisplay();
}

//////////////////////////////////////////////////////////////////////////////////
// Called occasionally to see if anything's happening
//////////////////////////////////////////////////////////////////////////////////
void idle()
{
  glutPostRedisplay();
}

//////////////////////////////////////////////////////////////////////////////////
// Called if a mouse button is pressed
//////////////////////////////////////////////////////////////////////////////////
void mouseButton(int button, int state, int x, int y)
{
}

//////////////////////////////////////////////////////////////////////////////////
// Called if the mouse moves
//////////////////////////////////////////////////////////////////////////////////
void mouseMotion(int x, int y)
{
}

//reshape when window resizes
void reshape(int w, int h)
{
  //change viewport to new size
  glViewport(0,0, (GLsizei) w, (GLsizei)h);
  //set matrix mode to projection
  glMatrixMode(GL_PROJECTION);
  //clear matrix
  glLoadIdentity();
  //setup perspective
  gluPerspective (60.0, (GLfloat) w/(GLfloat)h, 0.01, 200.0);
  //set matrix mode to model view
  glMatrixMode(GL_MODELVIEW);
  //clear matrix
  glLoadIdentity();
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  
  //set camera eye and center to start in the maze
  eyeX = 0;
  eyeY = 0;
  eyeZ = 5;
  centerX = 0;
  centerY = 0;
  centerZ = 0;
  

  glutInit(&argc, argv);
  glutInitWindowSize(600,600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Final OpenGL");
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

  glEnable(GL_DEPTH_TEST);

  glutDisplayFunc(display);
  glutMotionFunc(mouseMotion);
  glutMouseFunc(mouseButton);
  glutMotionFunc(mouseMotion);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);
  glutReshapeFunc(reshape);
  glutMainLoop();

  return 0;
}
