#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include"EpsilonWorld.h"
void InputsGL(EpsilonWorld& world, GLFWwindow* window, float deltatime, bool& ispressed, int& contype, float& timer, EpsilonVector& origin, int width, int height, float zoom) {

    //ZoneScoped;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (!ispressed) {
            if (contype == 0) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);

                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;

                world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(spawnX, spawnY), 0.7f, 0.5f, 4, 4, false, none));
            }
            else if (contype == 1) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                EpsilonBody body = EpsilonBody::CreateBoxBody(EpsilonVector(spawnX, spawnY), 0.7f, 0.5f, 2, 2, false, spring);
                body.CreateConnection(origin);
                world.AddBody(body);
                contype = 0;
            }
            else if (contype == 2) {

                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                EpsilonBody body = EpsilonBody::CreateBoxBody(EpsilonVector(spawnX, spawnY), 0.7f, 0.5f, 2, 2, false, thr);
                body.CreateConnection(origin);
                world.AddBody(body);
                contype = 0;
            }
        }

        ispressed = true;
    }
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {

        if (!ispressed) {
            if (contype == 0) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                world.AddBody(EpsilonBody::CreateCircleBody(EpsilonVector(spawnX, spawnY), 1.5f, 1, 1, false, none));
            }
            else if (contype == 1) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                EpsilonBody body = EpsilonBody::CreateCircleBody(EpsilonVector(spawnX, spawnY), 1.5f, 1, 1, false, spring);
                body.CreateConnection(origin);
                world.AddBody(body);
                contype = 0;
            }
            else if (contype == 2) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                EpsilonBody body = EpsilonBody::CreateCircleBody(EpsilonVector(spawnX, spawnY), 1.5f, 1, 1, false, thr);
                body.CreateConnection(origin);
                world.AddBody(body);
                contype = 0;
            }
        }
        ispressed = true;
    }
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {

        if (!ispressed) {
            if (contype == 0) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                world.AddBody(EpsilonBody::CreateTriangleBody(EpsilonVector(spawnX, spawnY), 1.5f, 0.5f, 2, false, none));
            }
            else if (contype == 1) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                EpsilonBody body = EpsilonBody::CreateTriangleBody(EpsilonVector(spawnX, spawnY), 1, 0.5f, 2, false, spring);
                body.CreateConnection(origin);
                world.AddBody(body);
                contype = 0;
            }
            else if (contype == 2) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
                float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
                float targetNDC_X = mouseNDC_X * zoom;
                float targetNDC_Y = mouseNDC_Y * zoom;
                float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
                float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
                EpsilonBody body = EpsilonBody::CreateTriangleBody(EpsilonVector(spawnX, spawnY), 1, 0.5f, 2, false, thr);
                body.CreateConnection(origin);
                world.AddBody(body);
                contype = 0;
            }
        }
        ispressed = true;
    }
    else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        if (!ispressed) {
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
            float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
            float targetNDC_X = mouseNDC_X * zoom;
            float targetNDC_Y = mouseNDC_Y * zoom;
            float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
            float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
            world.Explosion(EpsilonVector(spawnX, spawnY), 10, 100);
        }
        ispressed = true;

    }
    else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        contype = 1;
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
        float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
        float targetNDC_X = mouseNDC_X * zoom;
        float targetNDC_Y = mouseNDC_Y * zoom;
        float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
        float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
        origin = EpsilonVector(spawnX, spawnY);
    }
    else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
        contype = 2;
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
        float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
        float targetNDC_X = mouseNDC_X * zoom;
        float targetNDC_Y = mouseNDC_Y * zoom;
        float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
        float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
        origin = EpsilonVector(spawnX, spawnY);
    }
    else if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
        timer -= deltatime;
        if (timer <= 0) {
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            float mouseNDC_X = (xpos / width) * 2.0f - 1.0f;
            float mouseNDC_Y = 1.0f - (ypos / height) * 2.0f; // Inverted Y
            float targetNDC_X = mouseNDC_X * zoom;
            float targetNDC_Y = mouseNDC_Y * zoom;
            float spawnX = ((targetNDC_X + 1.0f) / 2.0f) * width;
            float spawnY = ((1.0f - targetNDC_Y) / 2.0f) * height;
            world.AddBody(EpsilonBody::CreateCircleBody(EpsilonVector(spawnX, spawnY), 1.5f, 0.5f, 1, false, none));
            timer = 0.02f;
        }
    }
    else if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
    else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        cout << world.GetBodyCount() << endl;
    }
    else {
        ispressed = false;
    }
}