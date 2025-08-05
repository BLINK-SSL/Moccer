#ifndef DSTAR_DRAW_H
#define DSTAR_DRAW_H

#ifdef MACOS
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <stdlib.h>
#include <unistd.h>
#include "Dstar.h"
#include "../models/robot.h"

class DstarDraw {
public:
    DstarDraw(int width, int height, int fieldWidth, int fieldHeight);
    ~DstarDraw();

    void initGL();
    void resizeGLScene(int width, int height);
    void drawGLScene(Robot* blueRobots, Robot* yellowRobots);
    void keyPressed(unsigned char key, int x, int y);
    void mouseFunc(int button, int state, int x, int y);
    void mouseMotionFunc(int x, int y);
    void drawCell(state s, float z = 0.45);

    float radian;

private:
    int hh, ww, fw, fh;
    int window;
    int mbutton;
    int mstate;
    bool b_autoreplan;
    Dstar* dstar;

    ds_pq openList;
    ds_ch cellHash;
    ds_oh openHash;

    float dxRatio;
    float dyRatio;
    float dRatio;
};

#endif // DSTAR_DRAW_H
