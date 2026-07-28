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
    const int worldWidth = 10000;
	const int worldHeight = 10000;
    const float PPM = 64.f;
    const float zoom = 1.f;
    EpsilonWorld world(width, height, worldWidth, worldHeight, zoom);
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(width/2.f, height/2.f), 1.f, 0.5f, 10, 1, true, false, none));
    //world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(width/2.f+width/16.f, height / 2.f), 1.f, 0.5f, 3, 300, true, false, none));
    //world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(width / 2.f - width / 16.f, height / 2.f), 1.f, 0.5f, 3, 300, true, false, none));
    Entity ent2;
    Position pos2;
    pos2.value = EpsilonVector(width/(2.f*PPM), 900/PPM);
    Angle ang2;
    ang2.value = 0;
    AddStaticBox(world, ent2, pos2, ang2, 10, 1, 1000, 0.5, 0.9, 0.5);
   
    world.CreateWater(EpsilonVector(width / 2.f, height / 2.f+height/25.f), 150, 30, 1);
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
        int counter = 0;
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        accumulator += deltaTime;
        //renderer.Render(world, width, height, zoom);
        renderer.RenderDOD(world, width, height, zoom, PPM);
        //InputsGL(world, windowGL, deltaTime, ispressed, contype, timer, origin, width, height, zoom);
        InputsDOD(world, windowGL, deltaTime, ispressed, contype, timer, origin, width, height, zoom, PPM);
        while (accumulator > fixedDt&&counter<3) {
            //world.Update(fixedDt, 6);
            WorldStep(world, fixedDt, 8);
            accumulator -= fixedDt;
            counter++;
        }
        
        if (counter >= 3) {
            accumulator = 0;
        }

       
        glfwSwapBuffers(windowGL);
        glfwPollEvents();
        FrameMark;
    }
   
    glfwTerminate();

    return 0;
}
