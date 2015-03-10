/***************** 
	 Benji Lampel 
	 Project - Fire
	 CS290I W15
*****************/

#include <GL/glut.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

#define WIN_W 640
#define WIN_H 480
#define NUM_PARTICLES 100
#define X NUM_PARTICLES / 2
#define Y NUM_PARTICLES / 2

using namespace std;

float Perlin		[NUM_PARTICLES];
float pos_table	[NUM_PARTICLES][2];
float colors		[NUM_PARTICLES][3]; 
float Gradient	[X][Y][2];

void initGrad() {
	srand(time(NULL));
	//COMPUTE GRADIENT
	int x = 0;
	int y = 0;
	while(x + y < X + Y) {
		//generate values from -1 to 1
		float x1 = (rand() % 201) * 0.01 - 1;
		float y1 = (rand() % 201) * 0.01 - 1;
		float r_sqr = pow(x1, 2) + pow(y1, 2);
		if(r_sqr > 0 && r_sqr <= 1)
		{
			Gradient[x][y][0] = x1 / (x1 + y1);
			Gradient[x][y][1] = y1 / (x1 + y1);
			y++;
		
			if(y == Y) {
				y = 0;
				x++;
			}
		}
	}
}

float lerp(float a0, float a1, float w) {
	return (1.0 - w) * a0 + w * a1;
}

float dotGridGrad(int ix, int iy, float x, float y) {
	float dx = x - (double) ix;
	float dy = y - (double) iy;
	return (dx*Gradient[ix][iy][0] + dy*Gradient[ix][iy][1]);
}

void perlin(float x, float y, int z) {
	//grid cell coords:
	int x0 = (x > 0.0 ? (int) x : (int) x - 1);
	int x1 = x0 + 1;
	int y0 = (y > 0.0 ? (int) y : (int) y - 1);
	int y1 = y0 + 1;

	//determine interpolation weights
	float wx = x - (double) x0;
	float wy = y - (double) y0;

	//interpolate between grid points and gradients
	float n0, n1, ix0, ix1, value;
	n0 = dotGridGrad(x0, y0, x, y);
	n1 = dotGridGrad(x1, y0, x, y);
	ix0 = lerp(n0, n1, wx);

	n0 = dotGridGrad(x0, y1, x, y);
	n1 = dotGridGrad(x1, y1, x, y);
	ix1 = lerp(n0, n1, wx);
	value = lerp(ix0, ix1, wy);
	Perlin[z] = value;
}

void idle(void) {
	//update all particle positions
	for(int i = 0; i < NUM_PARTICLES; i++) {
		perlin(pos_table[i][0]*10, pos_table[i][1]*10, i);
	}
	//update all particle colors using result from perlin
	for(int i = 0; i < NUM_PARTICLES; i++) {
		if(Perlin[i] == 0) {
			colors[i][0] = 0.0f;
			colors[i][1] = 1.0f;
			colors[i][2] = 0.0f;
		}
		else if(Perlin[i] < 0) {
			colors[i][0] = 1.0f;
			colors[i][1] = 0.0f;
			colors[i][2] = 0.0f;
		}
		else {
			colors[i][0] = 0.0f;
			colors[i][1] = 0.0f;
			colors[i][2] = 1.0f;
		}
			
	}
}

void init(void) {
	GLfloat mat_specular  [] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mat_shininess [] = { 50.0 };
	GLfloat light_ambient [] = { 1.0, 1.0, 1.0, 0.0 };
  GLfloat light_specular[] = { 0.5, 1.0, 1.0, 0.0 };
  GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
  
	glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_SMOOTH);

  glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	
	glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_TEXTURE_2D);

	initGrad();
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_COLOR_MATERIAL);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//draw some fire particles
		for(int i = 0; i < NUM_PARTICLES; i++) {
			glColor3f(colors[i][0], colors[i][1], colors[i][2]);
			//	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			//	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
			glPushMatrix();
				glTranslatef(pos_table[i][0], pos_table[i][1], 0);
				glutSolidSphere(0.02, 20.0, 20.0);
			glPopMatrix();
		}

	glDisable(GL_COLOR_MATERIAL);
	glFlush();
	glutSwapBuffers;
	glutPostRedisplay();
}

void reshape(int w, int h) {
	if(w == 0 || h == 0)
		return;
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
}

void keydown(unsigned char key, int x, int y) {
	switch(key) {
		case 27:
			exit(0);
			break;
	}
}

int main(int argc, char** argv) {

	float height = -1;
	for(int i = 0; i < NUM_PARTICLES; i++) {
		//pos_table[i][0] = rand() % 2 - (0.1 * (rand() % 11));
		//pos_table[i][1] = (rand() % 11) * 0.01;
		if(i % 10 == 0)
		{
			height += 0.1;
		}
		pos_table[i][0] = i*0.1 - floor(i*0.1);
		pos_table[i][1] = height;
		
		colors[i][0] = 0.0f;
		colors[i][1] = 0.0f;
		colors[i][2] = 0.0f;
	}

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize (WIN_W, WIN_H); 
	glutInitWindowPosition (100, 100);
	glutCreateWindow("fire");
	init();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keydown);
	glutMainLoop();
	return 0;
}
