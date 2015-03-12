#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
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
    Vec3f operator=(const Vec3f&) const;
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

class PRIMITIVE
{
  public:
    virtual ~PRIMITIVE(){}
    Vec3f COLOR;
    Matrix4X4 TRANSFORMATION;
    virtual void draw_p() = 0;
};

class CUBE:public PRIMITIVE{
  private:
    Vec3f P0;
    Vec3f P1;
    Vec3f P2;
    Vec3f P3;
    Vec3f P4;
    Vec3f P5;
    Vec3f P6;
    Vec3f P7;
  public:
    CUBE(Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3, Vec3f p4, Vec3f p5, Vec3f p6, Vec3f p7, Vec3f color, Matrix4X4 transformation){
      P0 = p0;
      P1 = p1;
      P2 = p2;
      P3 = p3;
      P4 = p4;
      P5 = p5;
      P6 = p6;
      P7 = p7;
      COLOR = color;
      TRANSFORMATION = transformation;
    }
    void draw_p(){
       glPushMatrix();
        glRotatef(90.0, 0.0, 1.0, 0.0);
        glTranslatef(-3.5,0.0,4.0);
         glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P0.x,P0.y,P0.z);
          glVertex3f(P1.x,P1.y,P1.z);
          glVertex3f(P2.x,P2.y,P2.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P0.x,P0.y,P0.z);
          glVertex3f(P2.x,P2.y,P2.z);
          glVertex3f(P3.x,P3.y,P3.z);
        glEnd();
        glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P3.x,P3.y,P3.z);
          glVertex3f(P2.x,P2.y,P2.z);
          glVertex3f(P4.x,P4.y,P4.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P3.x,P3.y,P3.z);
          glVertex3f(P4.x,P4.y,P4.z);
          glVertex3f(P5.x,P5.y,P5.z);
        glEnd();
         glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P5.x,P5.y,P5.z);
          glVertex3f(P4.x,P4.y,P4.z);
          glVertex3f(P6.x,P6.y,P6.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P5.x,P5.y,P5.z);
          glVertex3f(P6.x,P6.y,P6.z);
          glVertex3f(P7.x,P7.y,P7.z);
        glEnd();
         glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P7.x,P7.y,P7.z);
          glVertex3f(P6.x,P6.y,P6.z);
          glVertex3f(P1.x,P1.y,P1.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P7.x,P7.y,P7.z);
          glVertex3f(P1.x,P1.y,P1.z);
          glVertex3f(P0.x,P0.y,P0.z);
        glEnd();
         glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P0.x,P0.y,P0.z);
          glVertex3f(P3.x,P3.y,P3.z);
          glVertex3f(P5.x,P5.y,P5.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P0.x,P0.y,P0.z);
          glVertex3f(P5.x,P5.y,P5.z);
          glVertex3f(P7.x,P7.y,P7.z);
        glEnd();
         glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P1.x,P1.y,P1.z);
          glVertex3f(P2.x,P2.y,P2.z);
          glVertex3f(P4.x,P4.y,P4.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P1.x,P2.y,P1.z);
          glVertex3f(P4.x,P4.y,P4.z);
          glVertex3f(P6.x,P6.y,P6.z);
        glEnd();
      glPopMatrix();
      return;
    } 
    virtual ~CUBE() {}
};

class PLANE:public PRIMITIVE{
  private:
    Vec3f P0;
    Vec3f P1;
    Vec3f P2;
    Vec3f P3;
    int WALL;
    int CEILING;
    int FLOOR;
  public:
    PLANE(Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3, Vec3f color, int wall, int ceiling, int floor, Matrix4X4 transformation){
      P0 = p0;
      P1 = p1;
      P2 = p2;
      P3 = p3;
      COLOR = color;
      WALL = wall;
      CEILING = ceiling;
      FLOOR = floor;
      TRANSFORMATION = transformation;
    }
    void draw_p(){
      glPushMatrix();
        glRotatef(90.0, 0.0, 1.0, 0.0);
        glTranslatef(-3.5,0.0,4.0);
        glColor4f(0,1,0,1);
        //define first triangle
        glBegin(GL_TRIANGLES);
          glVertex3f(P0.x,P0.y,P0.z);
          glVertex3f(P1.x,P1.y,P1.z);
          glVertex3f(P2.x,P2.y,P2.z);
        glEnd();
        //define second triangle
        glColor4f(1,0,0,1);
        glBegin(GL_TRIANGLES);
          glVertex3f(P0.x,P0.y,P0.z);
          glVertex3f(P2.x,P2.y,P2.z);
          glVertex3f(P3.x,P3.y,P3.z);
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
    SPHERE(Vec3f center, Vec3f color, float radius, Matrix4X4 transformation){
      CENTER = center;
      COLOR = color;
      RADIUS = radius;
      TRANSFORMATION = transformation;
    }
    void draw_p(){
      glPushMatrix();
        glRotatef(90.0, 0.0, 1.0, 0.0);
        glTranslatef(-3.5,0.0,4.0);
        glutSolidSphere(1.0,10.0,10.0);
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
        glRotatef(90.0, 0.0, 1.0, 0.0);
        glTranslatef(-3.5,0.0,4.0);
        gluCylinder(qobj,RADIUS,HEIGHT,10.0, 10.0,10.0);
      glPopMatrix();
      return;
    } 
    virtual ~CYLINDER() {}
};


//draw square
void drawSquare(float x1, float x2)
{
  glColor4f(0,1,0,1);
  //define first triangle
  glBegin(GL_TRIANGLES);
    glVertex3f(x1,0.5f,0.0);
    glVertex3f(x1,-0.5f,0.0);
    glVertex3f(x2,-0.5f,0.0);
  glEnd();
  //define second triangle
  glColor4f(1,0,0,1);
  glBegin(GL_TRIANGLES);
    glVertex3f(x1,0.5f,0.0);
    glVertex3f(x2,0.5f,0.0);
    glVertex3f(x2,-0.5f,0.0);
  glEnd();
}


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

  glPushMatrix();
  glRotatef(90.0, 0.0, 1.0, 0.0);
  glTranslatef(-3.5,0.0,4.0);
//  glutSolidSphere(1.0,10.0,10.0);
  drawSquare(-0.5,0.5);
  glPopMatrix();
  
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

//////////////////////////////////////////////////////////////////////////////////
// Read in a raw PPM file of the "P6" style.
//
// Input: "filename" is the name of the file you want to read in
// Output: "pixels" will point to an array of pixel values
//         "width" will be the width of the image
//         "height" will be the height of the image
//
// The PPM file format is:
//
//   P6
//   <image width> <image height>
//   255
//   <raw, 8-bit binary stream of RGB values>
//
// Open one in a text editor to see for yourself.
//
//////////////////////////////////////////////////////////////////////////////////

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
  eyeX = 0.0;
  eyeY = 0.0;
  eyeZ = 3.5;
  centerX = 100.0;
  centerY = 0.0;
  centerZ = 0.0;

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
