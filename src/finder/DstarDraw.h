#ifndef DSTAR_DRAW_H
#define DSTAR_DRAW_H

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <stdlib.h>
#include <unistd.h>
#include "Dstar.h"

class DstarDraw {
public:
    DstarDraw(int width, int height, int fieldWidth, int fieldHeight);
    ~DstarDraw();

    void initGL();
    void resizeGLScene(int width, int height);
    void drawGLScene();
    void keyPressed(unsigned char key, int x, int y);
    void mouseFunc(int button, int state, int x, int y);
    void mouseMotionFunc(int x, int y);
    void draw();
    void drawCell(state s, float z = 0.45);

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
};

#endif // DSTAR_DRAW_H
