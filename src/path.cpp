#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include "path.h"


Eigen::Vector3f
Path::getCurrPoint(void) {
    float rad = (rad1 * rad2)/sqrt(rad1 * rad1 * pow(sin(degree * PI/180.0f), 2) + rad2 * rad2 * pow(cos(degree * PI/180.0f), 2));
    float x = rad * cos(degree * PI/180.0f);
    float y = rad * sin(degree * PI/180.0f);
    float z = a * pow(x, 3) + b * pow(y, 3);
    Eigen::Vector3f point;
    point << x, y, z - 0.5;
    return point;
}

void
Path::render(void) {
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_POLYGON);
	for (int i = 0; i < 360; i++) {
		Eigen::Vector3f curr = getCurrPoint();
		glVertex3f(curr(0), curr(1), curr(2));
		addDegree(1);
	}
	glEnd();

	//Draw ball at current point
	glPushMatrix();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	Eigen::Vector3f currPoint = getCurrPoint();
	glTranslatef(currPoint(0), currPoint(1), currPoint(2));
	GLUquadric* quad = gluNewQuadric();
	gluSphere(quad, 0.05, 10, 10);
	glPopMatrix();
}
