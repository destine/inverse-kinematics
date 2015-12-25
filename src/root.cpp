#include "root.h"
#include <Eigen/Dense>
#include <time.h>
#include <iostream>
#include <cstring>
#include <list>
#include "assert.h"

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

#ifndef DEBUG
#define NDEBUG
#endif

#define MAX_LINE_LENGTH 1000
#define DEFAULT_WIDTH 720
#define DEFAULT_HEIGHT 720

void
Root::init(int argc, char** argv, FILE* input)
{
    m_pArm = new Arm();
    m_pArmPath = new Path();
    m_pCameraPath = new Path();
    m_maxSize = 0;

    parse(input);
    m_maxSize *= 1.15;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    glutInitWindowPosition(0,0);

    m_isInitialized = true;
    return;
}

void
Root::parse(FILE* input) {
    const char* flags[] = {"-mod", "-arm", "-path", "-cir", "-ell"};
    char lineBuffer[MAX_LINE_LENGTH];
    char* token = NULL;
    int mode = 0;
    while (fgets(lineBuffer, MAX_LINE_LENGTH, input)) {
        //Get first argument and check for flag, assign mode according to flag
        char* tokenSrc = lineBuffer;
        token = strtok(tokenSrc, " \n");
        bool validFlag = false;

        for (int i = 0; i < sizeof(flags)/sizeof(char*); i++) {
            if (!strcmp(token, flags[i])) {
                mode = i;
                validFlag = true;
                break;
            }
        }

        if (!validFlag) {
            std::cout << "Invalid flag ignored: " << token << std::endl;
        } else {
            std::list<char*> args;
            token = strtok(NULL, " \n");
            while (token != NULL) {
                tokenSrc = NULL;
                args.push_back(token);
                token = strtok(NULL, " \n");
            }

            std::list<char*>::iterator iter;
            int counter;
            float a, b, rad, x, y;

            switch(mode) {
                case 0: //-mod input.obj
                    //TODO
                    break;
                case 1: //-arm [joint/]length ...
                    for (iter = args.begin(); iter != args.end(); ++iter) {
                        std::string expr(*iter);
                        if (expr.find("/") != -1) {
                            //Handle [pivot]/length
                            Body* b = new Body(std::stof(expr.substr(expr.find("/") + 1)));
                            if (expr.find("pm") != -1) { //prismatic
                                PrismJoint* j;
                                if (m_pArm->getLastJoint()) {
                                    j = new PrismJoint(m_pArm->getLastJoint()->getOutboardBody(), b);
                                } else {
                                    j = new PrismJoint(NULL, b);
                                }
                                m_pArm->appendJoint(j);
                            } else if (expr.find("pn") != -1) { //pin
                                PinJoint* j;
                                if (m_pArm->getLastJoint()) {
                                    j = new PinJoint(m_pArm->getLastJoint()->getOutboardBody(), b);
                                } else {
                                    j = new PinJoint(NULL, b);
                                }
                                m_pArm->appendJoint(j);
                            } else if (expr.find("ba") != -1) { //ball
                                BallJoint* j;
                                if (m_pArm->getLastJoint()) {
                                    j = new BallJoint(m_pArm->getLastJoint()->getOutboardBody(), b);
                                } else {
                                    j = new BallJoint(NULL, b);
                                }
                                m_pArm->appendJoint(j);
                            } else if (expr.find("dp") != -1) { //double pin
                                DoublePinJoint* j;
                                if (m_pArm->getLastJoint()) {
                                    j = new DoublePinJoint(m_pArm->getLastJoint()->getOutboardBody(), b);
                                } else {
                                    j = new DoublePinJoint(NULL, b);
                                }
                                m_pArm->appendJoint(j);
                            }
                        } else { //default = ball joint
                            Body* b = new Body(std::atof(*iter));
                            BallJoint* j;
                            if (m_pArm->getLastJoint()) {
                                j = new BallJoint(m_pArm->getLastJoint()->getOutboardBody(), b);
                            } else {
                                j = new BallJoint(NULL, b);
                            }
                            m_pArm->appendJoint(j);
                        }
                    }
                    break;
                case 2: //-path [a b] for path
                    counter = 0;
                    a = 0, b = 0;
                    for (iter = args.begin(); iter != args.end(); ++iter) {
                        if (counter == 0) a = std::atof(*iter);
                        else if (counter == 1) b = std::atof(*iter);
                        counter++;
                        if (counter > 1) {
                            if (a == 0 || b == 0) {
                                std::cout << "Error parsing -path" << std::endl;
                            }
                            break;
                        }
                    }
                    m_pArmPath->setCoeff(a, b);
                    break;
                case 3: //-cir [radius]
                    counter = 0;
                    rad = 0;
                    for (iter = args.begin(); iter != args.end(); ++iter) {
                        rad = std::atof(*iter);
                        counter++;
                        if (counter > 0) {
                            if (!rad) {
                                std::cout << "Error parsing -cir" << std::endl;
                            }
                            break;
                        }
                    }
                    m_pArmPath->setRad(rad, rad);
                    m_maxSize = std::max(m_maxSize, rad);
                    break;
                case 4: //-ell [x rad] [y rad]
                    counter = 0;
                    x = 0, y = 0;
                    for (iter = args.begin(); iter != args.end(); ++iter) {
                        if (counter == 0) x = std::atof(*iter);
                        else if (counter == 1) y = std::atof(*iter);
                        counter++;
                        if (counter > 1) {
                            if (x == 0 || y == 0) {
                                std::cout << "Error parsing -ell" << std::endl;
                            }
                            break;
                        }
                    }
                    m_pArmPath->setRad(x, y);
                    m_maxSize = std::max(m_maxSize, std::max(x, y));
                    break;
                default:
                    break;
            }
        }
    }
    return;
}

void
Root::run(void (*render)(void),
          void (*reshape)(int, int),
          void (*idle)(void),
          void (*input)(unsigned char, int, int)) {
    assert(m_isInitialized);
    m_pArm->print();
    m_pArmPath->print();

    glutCreateWindow("root");
    glutDisplayFunc(render);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(input);

    glMatrixMode(GL_MODELVIEW);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLoadIdentity();

    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);

    glOrtho(-m_maxSize, m_maxSize, -m_maxSize, m_maxSize, m_maxSize, -m_maxSize);

    //Lighting
    GLfloat position0[] = {5.0, 5.0, 0.0, 1.0};
    GLfloat ambient0[] = {0.5, 1.0, 1.0, 1.0};
    GLfloat diffuse0[] = {1.0, 0.1, 0.3, 1.0};
    GLfloat specular0[] = {0.1, 0.1, 0.1, 1.0};
    glLightfv(GL_LIGHT0, GL_POSITION, position0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);

    //Lighting
    GLfloat position1[] = {-5.0, -5.0, 0.0, 1.0};
    GLfloat ambient1[] = {0.0, 0.0, 0.3, 1.0};
    GLfloat diffuse1[] = {0.05, 0.2, 0.8, 1.0};
    GLfloat specular1[] = {0.05, 0.1, 0.1, 1.0};
    glLightfv(GL_LIGHT1, GL_POSITION, position1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular1);

    GLfloat position2[] = {-5.0, 0.0, 0.0, 1.0};
    GLfloat ambient2[] = {0.0, 0.3, 0.0, 1.0};
    GLfloat diffuse2[] = {0.05, 0.8, 0.2, 1.0};
    GLfloat specular2[] = {0.1, 0.2, 0.1, 1.0};
    glLightfv(GL_LIGHT2, GL_POSITION, position2);
    glLightfv(GL_LIGHT2, GL_AMBIENT, ambient2);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse2);
    glLightfv(GL_LIGHT2, GL_SPECULAR, specular2);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);

    m_updateClock = clock();
    m_renderClock = clock();

    glColor3f(rand()/(float)RAND_MAX, rand()/(float)RAND_MAX, rand()/(float)RAND_MAX);
    glutMainLoop();
}

float absDifference(float a, float b)
{
    float difference = a - b;
    if (difference < 0)
        return -difference;
    return difference;
}

void
Root::update(void)
{
    Eigen::Vector3f goalPoint = m_pArmPath->getNextPoint(1.5);
    Eigen::Vector3f err = goalPoint - m_pArm->getEndEffector();
    float prevError = 1000;
    float currError = err(0) * err(0) + err(1) * err(1) + err(2) * err(2);

    float b = 1;
    while (currError > 0.0001 && absDifference(prevError, currError) > 0.000001)
    {
        prevError = currError;
        m_pArm->approachPoint(goalPoint, b); 
        err = goalPoint - m_pArm->getEndEffector(); 
        currError = err(0) * err(0) + err(1) * err(1) + err(2) * err(2);
        if (currError > prevError)
            b /= 2;
    }
}

void
Root::render(float interpolation)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glTranslatef(0, -0.5, 0);
    glRotatef(90, 1, 0, 0);
    glRotatef(m_pArmPath->getDegree(), 0, 0, -1);
    m_pArmPath->render();
    m_pArm->render(interpolation);
    glPopMatrix();

    glFlush();
    glutSwapBuffers();
    return;
}

void
Root::halt(void)
{
    delete m_pArm;
    delete m_pArmPath;
    delete m_pCameraPath;
}

//---------------OpenGL Helper Functions---------------

void
Root::handleInput(unsigned char key, int x, int y) {
    switch (key) {
        case ' ':
            halt();
            exit(EXIT_SUCCESS);
            break;
        default:
            break;
    }
    return;
}

void
Root::render() { //OpenGL glDisplayFunc
    return;
}

void
Root::idle() {
    const int UPDATE_RATE = 24;
    const int FRAME_RATE = 60;

    clock_t t = clock();

    if ((t - m_updateClock)/(float)CLOCKS_PER_SEC > 1/(float)UPDATE_RATE) {
        m_updateClock = clock();
        update();
    }

    if ((t - m_renderClock)/(float)CLOCKS_PER_SEC > 1/(float)FRAME_RATE) {
        m_renderClock = clock();
        float interpolation = (t - m_updateClock)/(float)CLOCKS_PER_SEC * UPDATE_RATE;
        render(interpolation);
    }

    return;
}

void
Root::reshape(int width, int height) {
    glViewport(0, 0, width, height);
    return;
}
