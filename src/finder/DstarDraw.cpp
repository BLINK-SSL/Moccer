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
    for (int i = 0; i < 16; ++i) {
        if (blueRobots[i].confidence > 0.5) {
            if (i == 0) continue;
            dstar->addCircularObstacle(static_cast<int>(blueRobots[i].x / dRatio), static_cast<int>(blueRobots[i].y / dRatio), 200, 100);
        }
        if (yellowRobots[i].confidence > 0.5) {
            dstar->addCircularObstacle(static_cast<int>(yellowRobots[i].x / dRatio), static_cast<int>(yellowRobots[i].y / dRatio), 200, 100);
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

void DstarDraw::keyPressed(unsigned char key, int x, int y) {
    usleep(100);
    switch (key) {
        case 'q':
        case 'Q':
            glutDestroyWindow(window);
            exit(0);
        case 'r':
        case 'R':
            dstar->replan();
            break;
        case 'a':
        case 'A':
            b_autoreplan = !b_autoreplan;
            break;
        case 'c':
        case 'C':
            dstar->init(-5500 / dRatio, -4000 / dRatio, 5500 / dRatio, 4000 / dRatio, dRatio);
            dstar->addFieldObstacle();
            dstar->addCircularObstacle(0, 0, 90, 70);
            break;
    }
}

void DstarDraw::mouseFunc(int button, int state, int x, int y) {
    x -= ww / 2;
    y -= hh / 2; // Invert y-axis
    x = static_cast<int>(x / dxRatio);
    y = -static_cast<int>(y / dyRatio);

    mbutton = button;
    mstate = state;
    if (mstate == GLUT_DOWN) {
        if (button == GLUT_LEFT_BUTTON) {
            dstar->updateCell(x, y, -1);
        } else if (button == GLUT_RIGHT_BUTTON) {
            dstar->updateStart(x, y);
        } else if (button == GLUT_MIDDLE_BUTTON) {
            dstar->updateGoal(x, y);
        }
    }
}

void DstarDraw::mouseMotionFunc(int x, int y) {
    x -= ww / 2;
    y -= hh / 2; // Invert y-axis
    x = static_cast<int>(x / dxRatio);
    y = -static_cast<int>(y / dyRatio);

    if (mstate == GLUT_DOWN) {
        if (mbutton == GLUT_LEFT_BUTTON) {
            dstar->updateCell(x, y, -1);
        } else if (mbutton == GLUT_RIGHT_BUTTON) {
            dstar->updateStart(x, y);
        } else if (mbutton == GLUT_MIDDLE_BUTTON) {
            dstar->updateGoal(x, y);
        }
    }
}
