#include <GL/freeglut.h>

class Camera
{
public:
    float angleX, angleY;
    float posX, posY, posZ;
    float lastMouseX, lastMouseY;

    Camera() : angleX(0), angleY(0), posX(0), posY(0), posZ(0), lastMouseX(0), lastMouseY(0) {}

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
        if (posZ < -.0f)
            posZ = -1.0f;
        glutPostRedisplay();
    }
};
