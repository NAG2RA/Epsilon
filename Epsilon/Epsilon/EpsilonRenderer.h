#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include<vector>
#include"EpsilonWorld.h"
#include<fstream>
#include<string>
#include<sstream>
#include<streambuf>
struct EpsilonRenderer
{
public:
    vector<float> allVert;
    vector<float> waterVertices;
    vector<int> allIndic;
    vector<int> waterIndices;
    GLint colorLoc;
    GLint screenSize;
    GLint zoomGL;
    unsigned int shaderProgram;
    unsigned int VAO, VBO, EBO;
    unsigned int vertexShader;
    unsigned int fragmentShader;
    string loadShaderSrc(const char* filename);
    void Initialize();
    void Render(EpsilonWorld& world, int width, int height, float zoom);
    void RenderDOD(EpsilonWorld& world, int width, int height, float zoom, const float& PPM);
    ~EpsilonRenderer() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteProgram(shaderProgram);
    }
};

