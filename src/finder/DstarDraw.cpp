#include "DstarDraw.h"

DstarDraw::DstarDraw(int width, int height, int fieldWidth, int fieldHeight)
    : hh(height), ww(width), fw(fieldWidth), fh(fieldHeight), mbutton(0), mstate(0), b_autoreplan(true)
{
    dstar = new Dstar();
    dxRatio = static_cast<float>(ww) / fw;
    dyRatio = static_cast<float>(hh) / fh;
    dRatio = 50;
}

DstarDraw::~DstarDraw() {
    delete dstar;
}

void DstarDraw::initGL() {
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0);
    // glViewport(0, 0, ww, hh);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(-6700 / dRatio, 6700 / dRatio, -5200 / dRatio, 5200 / dRatio, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}

void DstarDraw::resizeGLScene(int width, int height) {
    ww = width;
    hh = height;
    // glViewport(0, 0, ww, hh);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-6700 / dRatio, 6700 / dRatio, -5200 / dRatio, 5200 / dRatio, -1, 1);
    // glOrtho(0, ww, 0, hh, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}

Pair DstarDraw::drawGLScene(Robot* blueRobots, Robot* yellowRobots) {
    // usleep(100);
    dstar->resetMap();
    dstar->updateStart(static_cast<int>(blueRobots[0].x / dRatio), static_cast<int>(blueRobots[0].y / dRatio));
    for (int i = 0; i < 11; ++i) {
        if (blueRobots[i].confidence > 0.5) {
            if (i == 0) continue;
            dstar->addCircularObstacle(static_cast<int>(blueRobots[i].x / dRatio), static_cast<int>(blueRobots[i].y / dRatio), 200, 0);
        }
        if (yellowRobots[i].confidence > 0.5) {
            dstar->addCircularObstacle(static_cast<int>(yellowRobots[i].x / dRatio), static_cast<int>(yellowRobots[i].y / dRatio), 200, 0);
        }
    }
    dstar->addFieldObstacle();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glPushMatrix();

    if (b_autoreplan) dstar->replan();
    Pair pair = dstar->draw(dRatio, blueRobots, yellowRobots);
    // std::cout << "Degree: " << radian * 180 / M_PI << std::endl;
    glPopMatrix();
    glutSwapBuffers();
    return pair;
}