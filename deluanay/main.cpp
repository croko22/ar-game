#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

// Estructura para almacenar un punto
struct Point
{
    float x, y, z;
};

// Clase para manejar la cámara
class Camera
{
public:
    float angleX, angleY;
    float posX, posY, posZ;
    float lastMouseX, lastMouseY;

    Camera() : angleX(0), angleY(0), posX(0), posY(0), posZ(5), lastMouseX(0), lastMouseY(0) {}

    void apply()
    {
        glTranslatef(0.0f, 0.0f, -posZ);
        glRotatef(angleY, 1.0f, 0.0f, 0.0f);
        glRotatef(angleX, 0.0f, 1.0f, 0.0f);
    }

    void update(int x, int y)
    {
        angleX += (x - lastMouseX) * 0.1f;
        angleY += (y - lastMouseY) * 0.1f;
        lastMouseX = x;
        lastMouseY = y;
        glutPostRedisplay();
    }

    void zoom(float amount)
    {
        posZ += amount;
        if (posZ < 1.0f)
            posZ = 1.0f; // Limitar el zoom para evitar inversión
        glutPostRedisplay();
    }
};

std::vector<Point> points;
Camera camera;

bool loadPCD(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line == "DATA ascii")
        {
            break;
        }
    }

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        Point point;
        if (!(iss >> point.x >> point.y >> point.z))
        {
            break;
        }
        points.push_back(point);
    }

    return true;
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    camera.apply();

    // Dibujar la nube de puntos
    glBegin(GL_POINTS);
    glColor3f(1.0, 1.0, 1.0);
    for (const auto &point : points)
    {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    glFlush();
    glutSwapBuffers();
}

void init()
{
    glClearColor(0.1, 0.39, 0.88, 1.0);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(5.0, 1.0, 1.0, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void mouseMotion(int x, int y)
{
    camera.update(x, y);
}

void mouseWheel(int button, int dir, int x, int y)
{
    if (dir > 0)
    {
        camera.zoom(-0.5f); // Acercar
    }
    else
    {
        camera.zoom(0.5f); // Alejar
    }
}

void mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        camera.lastMouseX = x;
        camera.lastMouseY = y;
    }
}

int main(int argc, char **argv)
{
    if (!loadPCD("bunny.pcd"))
    {
        return -1;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Dynamic Camera with Mouse Control");
    init();
    glutDisplayFunc(display);
    glutMotionFunc(mouseMotion);
    glutMouseFunc(mouse);
    glutMouseWheelFunc(mouseWheel);
    glutMainLoop();
    return 0;
}
