#include "primitives.cpp"
#include "kinect.cpp"

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
  glutSolidSphere(1.0,10.0,10.0);
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
