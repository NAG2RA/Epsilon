#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include"EpsilonWorld.h"
void InputsGL(EpsilonWorld& world, GLFWwindow* window, float deltatime, bool& ispressed, int& contype, float& timer, EpsilonVector& origin, int width, int height, float zoom);
void InputsDOD(EpsilonWorld& world, GLFWwindow* window, float deltatime, bool& ispressed, int& contype, float& timer, EpsilonVector& origin, int width, int height, float zoom, const float& PPM);