#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include<vector>
#include"EpsilonWorld.h"
#include<fstream>
#include<string>
#include<sstream>
#include<streambuf>
class EpsilonRenderer
{
public:
    unsigned int shaderProgram;
    unsigned int VAO, VBO, EBO;
    unsigned int vertexShader;
    unsigned int fragmentShader;
    vector<float> allVert;
    vector<int> allIndic;
    vector<float> waterVertices;
    vector<int> waterIndices;
    GLint colorLoc;
    GLint screenSize;
    GLint zoomGL;
    string loadShaderSrc(const char* filename);
    void Initialize();
    void Render(EpsilonWorld& world, int width, int height, float zoom);
    ~EpsilonRenderer() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteProgram(shaderProgram);
    }
};

