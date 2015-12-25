#ifndef __incl_root__
#define __incl_root__

#include <cstdio>
#include <ctime>
#include "arm.h"
#include "path.h"

class Root
{
    Arm* m_pArm;
    Path* m_pArmPath;
    Path* m_pCameraPath;

    float m_maxSize;
    bool m_isInitialized;
    clock_t m_updateClock;
    clock_t m_renderClock;

public:
    Root(void):m_isInitialized(false) {}
    virtual ~Root(void) { halt(); }

    virtual void init(int argc, char** argv, FILE* input);

    virtual void run(void (*render)(void),
          void (*reshape)(int, int),
          void (*idle)(void),
          void (*input)(unsigned char, int, int));
    virtual void update(void);
    virtual void render(float interpolation);

    virtual void halt(void);

    //OpenGL helper functions
    virtual void handleInput(unsigned char key, int x, int y);
    virtual void reshape(int width, int height);
    virtual void idle();
    virtual void render();

protected:
    virtual void parse(FILE* input);
};

#endif
