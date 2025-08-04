#include "DstarDraw.h"

DstarDraw::DstarDraw(int width, int height, int fieldWidth, int fieldHeight)
    : hh(height), ww(width), fw(fieldWidth), fh(fieldHeight), mbutton(0), mstate(0), b_autoreplan(true)
{
    dstar = new Dstar();
    dxRatio = static_cast<float>(ww) / fw;
    dyRatio = static_cast<float>(hh) / fh;
    dRatio = 75;
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
    
    glOrtho(-6000 / dRatio, 6000 / dRatio, -4500 / dRatio, 4500 / dRatio, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}

void DstarDraw::resizeGLScene(int width, int height) {
    ww = width;
    hh = height;
    // glViewport(0, 0, ww, hh);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-6000 / dRatio, 6000 / dRatio, -4500 / dRatio, 4500 / dRatio, -1, 1);
    // glOrtho(0, ww, 0, hh, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}

void DstarDraw::drawGLScene() {
    usleep(100);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glPushMatrix();

    if (b_autoreplan) dstar->replan();
    dstar->draw();

    glPopMatrix();
    glutSwapBuffers();
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
            dstar->addCircularObstacle(0, 0, 10, 9);
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

    std::cout << "Mouse clicked at: " << x << ", " << y << std::endl;
    std::cout << "Ratio: " << dxRatio << ", " << dyRatio << std::endl;
    std::cout << "x: " << x << ", y: " << y << std::endl;
    std::cout << std::endl;

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
