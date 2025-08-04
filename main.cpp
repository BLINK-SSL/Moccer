#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <tuple>
#include <chrono>
#include <thread>
// #include "src/observer.h"
#include "src/finder/DstarDraw.h"

using namespace std::chrono;

const int dRatio = 75; // Adjust this value as needed for your grid size

int windowWidth = 1000;
int windowHeight = 800;
int fieldWidth = 12000 / dRatio;
int fieldHeight = 9000 / dRatio;
DstarDraw* drawer = new DstarDraw(windowWidth, windowHeight, fieldWidth, fieldHeight);

void display() { drawer->drawGLScene(); }
void reshape(int w, int h) { drawer->resizeGLScene(w, h); }
void keyboard(unsigned char key, int x, int y) { drawer->keyPressed(key, x, y); }
void mouse(int button, int state, int x, int y) { drawer->mouseFunc(button, state, x, y); }
void motion(int x, int y) { drawer->mouseMotionFunc(x, y); }

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    int window = glutCreateWindow("Dstar Visualizer");
    drawer->initGL();

    drawer->keyPressed('c', 0, 0);

    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    glutMainLoop();
    delete drawer;
    return 0;
}
