#pragma once
#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<fstream>
#include<string>
#include<sstream>
#include<streambuf>
#include<iostream>
#include<vector>
#include<cmath>
#include<ctime>
#include<thread>
#include<Tracy.hpp>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"EpsilonWorld.h"
#include"AABB.h"
#include"EpsilonVector.h"
#include"EpsilonRenderer.h"
#include"EpsilonInputs.h"
#include"UnitTests.h"
using namespace std;
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

int main() {
    EpsilonRenderer renderer;
    UnitTestVector();
    UnitTestAABB();
    UnitTestUpdateMovement();
    srand(time(0));
    bool ispressed = false;
    int contype = 0;
    float timer = 0.02f;
    float fixedDt = 1 / 60.f;
    float accumulator = 0;
    EpsilonVector origin;  
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWmonitor* monitorGL = glfwGetPrimaryMonitor();
    const GLFWvidmode* modeGL = glfwGetVideoMode(monitorGL);
    const int width = modeGL->width;
    const int height = modeGL->height;
    const float zoom = 0.2f;
    EpsilonWorld world(width, height, zoom);
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(960, 540), 1.f, 0.5f, 300, 3, true, none));
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(1080, 540), 1.f, 0.5f, 3, 300, true, none));
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(840, 540), 1.f, 0.5f, 3, 300, true, none));   
    world.CreateWater(EpsilonVector(960, 580), 150, 30, 1);
    GLFWwindow* windowGL = glfwCreateWindow(width, height, "Epsilon", NULL, NULL);
    if (windowGL == NULL) {
        cout << "Failed to create GLFW window" << endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(windowGL);
    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        cout << "Failed to initialize GLAD" << endl;
        glfwTerminate();
    }
    glViewport(0, 0, width, height);
    glfwSetFramebufferSizeCallback(windowGL, framebuffer_size_callback);
    renderer.Initialize();
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(windowGL)) {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        renderer.Render(world, width, height, zoom);
        InputsGL(world, windowGL, deltaTime, ispressed, contype, timer, origin, width, height, zoom);
        world.Update(deltaTime, 6);
        glfwSwapBuffers(windowGL);
        glfwPollEvents();
        //FrameMark;
    }
   
    glfwTerminate();

    return 0;
}
