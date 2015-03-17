//#include <OpenGL/gl.h>
//#include <OpenGL/glu.h>
//#include <GLUT/glut.h>
#include <GL/glut.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
//global variables for camera
GLdouble eyeX;
GLdouble eyeY;
GLdouble eyeZ;
GLdouble centerX;
GLdouble centerY;
GLdouble centerZ;

GLuint ceilingTex;
GLuint floorTex;
GLuint wallTex;

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
inline Vec3f cross(const Vec3f& u, const Vec3f& v){
  return Vec3f(u.y*v.z - v.y*u.z,
              -u.x*v.z + v.x*u.z,
               u.x*v.y - v.x*u.y);
}
class PRIMITIVE
{
  public:
    virtual ~PRIMITIVE(){}
    Vec3f COLOR;
    virtual void draw_p() = 0;
  //  void draw_p();
};

class PLANE:public PRIMITIVE{
  private:
    vector<Vec3f> POINTS;
    Vec3f NORMAL;
    Vec3f COLOR;
    GLuint TEXTURE;
  public:
    PLANE(vector<Vec3f> points, Vec3f color, Vec3f normal){
      POINTS = points;
      COLOR = color;
      NORMAL = normal;
    }
    void setTexture(GLuint wall, GLuint ceiling, GLuint floor){
      if (NORMAL.x*NORMAL.x <.1 && NORMAL.z*NORMAL.z <.1){
        if (NORMAL.y > 0){ TEXTURE = floor; }//floor
        else { TEXTURE = ceiling; } //ceiling
      } else { TEXTURE = wall; }
      return;
    } 
    void draw_p(){
      if (POINTS.size() < 3) { return; }
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_TEXTURE_2D);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
      glBindTexture(GL_TEXTURE_2D, TEXTURE);
      glPushMatrix();
        glBegin(GL_TRIANGLE_FAN);
          glColor4f(COLOR.x,COLOR.y,COLOR.z,0);
          Vec3f e1 = cross(NORMAL,((POINTS[1]-POINTS[0])/(POINTS[1]-POINTS[0]).mag()));
                  //cross product of normal and point-origin/mag(point-origin)
          Vec3f e2 = cross(NORMAL,e1);
          for (int i = 0; i < POINTS.size(); i++){
            Vec3f PO = POINTS[0] - POINTS[i];
            double t1 = e1.x*PO.x + e1.y*PO.y +e1.z*PO.z; //e1 dot p-o
            double t2 = e2.x*PO.x + e2.y*PO.y +e2.z*PO.z; //e2 dot p-o
            glTexCoord2d(t1,t2);
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
    float RADIUS;
    float HEIGHT;
    GLdouble TRANSFORMATION[16];
    GLUquadric* qobj;
  public:
    CYLINDER(Vec3f color, float radius, float height, GLdouble transformation[16]){
      COLOR = color;
      RADIUS = radius;
      HEIGHT = height;
      for (int i = 0; i < 16; i++){
        TRANSFORMATION[i] = transformation[i]; }
      qobj = gluNewQuadric();
      gluQuadricNormals(qobj, GLU_SMOOTH);
    }
    void draw_p(){
      glPushMatrix();
        glMultMatrixd(TRANSFORMATION);
        glColor4f(COLOR.x,COLOR.y,COLOR.z,0);
        gluCylinder(qobj,RADIUS,HEIGHT,20.0, 20.0,20.0);
      glPopMatrix();
      return;
    } 
    virtual ~CYLINDER() {}
};


vector<PRIMITIVE*> items;

void readPPM(const char* filename, unsigned char*& pixels, int& width, int& height)
{
  // try to open the file
  FILE* file;
  file = fopen(filename, "rb");
  if (file == NULL)
  {
    cout << " Couldn't open file " << filename << "! " << endl;
    exit(1);
  }

  // read in the image dimensions
  fscanf(file, "P6\n%d %d\n255\n", &width, &height);
  int totalPixels = width * height;

  // allocate three times as many pixels since there are R,G, and B channels
  pixels = new unsigned char[3 * totalPixels];
  fread(pixels, 1, 3 * totalPixels, file);
  fclose(file);
 
  // output some success information
  cout << " Successfully read in " << filename << " with dimensions: "
       << width << " " << height << endl;
}


GLuint loadTexture(const char* fileName)
{
  GLuint sheepTexture;
  GLubyte *texture;
  int textureHeight, textureWidth;
  unsigned char* pixels;
  readPPM(fileName, pixels, textureWidth, textureHeight);

  int size = textureWidth*textureHeight*3;
  texture=(GLubyte*)malloc(sizeof(GLubyte)*size);
  for (int i=0; i< textureHeight; i++)
  {
    for (int j=0; j<3*textureWidth; j++)
    {
      texture[i*3*textureWidth+j] = pixels[i*3*textureWidth+j];
    }
  }
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glGenTextures(1, &sheepTexture);
  glBindTexture(GL_TEXTURE_2D, sheepTexture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
               GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
               GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth,
            textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
            pixels);
  return sheepTexture;
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
  cout << "drawing" << endl;
  for (int i = 0; i < items.size(); i++){
    cout << "drawing 1" << endl;
    items[i]->draw_p();
  }
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

  floorTex = loadTexture("../textures/lava.ppm");
  ceilingTex = loadTexture("../textures/sky2.ppm");
  wallTex = loadTexture("../textures/stone.ppm");

  GLdouble m[16]= {1,0,0,0,
                   0,1,0,0,
                   0,0,1,0,
                   1,0,0,1};
  
  int capacity = 10;
  vector<Vec3f> points;
  points.reserve(capacity);
  for (int i = 0; i < capacity; i++){
    float bob = i*6.28/capacity;
    points.push_back(Vec3f(sin(bob),cos(bob),-1)); 
  }

    

  cout << "here" << endl;
  SPHERE temp( Vec3f(-1, 0, 0), .1,  Vec3f(1, 0, 0));
  cout << "here 1" << endl;
  CYLINDER temp2(Vec3f(0,0,1), .1, .1, m);
  cout << "here 2" << endl;
  PLANE temp3(points, Vec3f(0,1,0), Vec3f(0, 0, 1));
  temp3.setTexture(wallTex, ceilingTex, floorTex);

  cout << "here now" << endl;

  items.push_back (&temp);  
  items.push_back (&temp2);  
  items.push_back (&temp3);  
 

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glEnable(GL_DEPTH_TEST);
  glutInitWindowSize(600,600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Final OpenGL");


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
