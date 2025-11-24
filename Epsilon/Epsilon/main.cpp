#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include<fstream>
#include<string>
#include<sstream>
#include<streambuf>
#include<iostream>
#include<vector>
#include<cmath>
#include<ctime>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"EpsilonWorld.h"
#include"AABB.h"
#include"EpsilonVector.h"
#include"EpsilonScheduler.h"
using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

string loadShaderSrc(const char* filename) {
    ifstream file;
    stringstream buf;
    string ret = "";
    file.open(filename);

    if (file.is_open()) {
        buf << file.rdbuf();
        ret = buf.str();
    }
    else {
        cout << "Could not open " << filename << endl;
    }

    file.close();

    return ret;
}
void InputsGL(EpsilonWorld& world, GLFWwindow* window, float deltatime, bool& ispressed, int& contype, float& timer, EpsilonVector& origin, int width, int height,float zoom) {



    if (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
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

                world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(spawnX,spawnY), 0.7f, 0.5f, 2, 2, false, none));
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
                world.AddBody(EpsilonBody::CreateTriangleBody(EpsilonVector(spawnX,spawnY), 1.5f, 0.5f, 2, false, none));
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
    else if (glfwGetKey(window,GLFW_KEY_A) == GLFW_PRESS) {
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

void Draw(EpsilonWorld& world, int width, int height, float zoom) {
    int success;
    char infoLog[512];
    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    string vertexShaderSrc = loadShaderSrc("vertex_core.glsl");
    const GLchar* vert = vertexShaderSrc.c_str();
    glShaderSource(vertexShader, 1, &vert, NULL);
    glCompileShader(vertexShader);

    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        cout << "Could not compile vertexShader " << "Log:" << endl << infoLog << endl;
    }

    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    string fragmentShaderSrc = loadShaderSrc("fragment_core.glsl");
    const GLchar* frag = fragmentShaderSrc.c_str();
    glShaderSource(fragmentShader, 1, &frag, NULL);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        cout << "Could not compile fragmentShader " << "Log:" << endl << infoLog << endl;
    }


    unsigned int shaderProgram;
    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        cout << "Could not link shaderProgram " << "Log:" << endl << infoLog << endl;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    vector<float> allVert;
    vector<int> allIndic;
    for (int i = 0; i < world.GetBodyCount(); i++) {

        if (world.GetBody(i).shapetype == triangle) {
            vector<EpsilonVector> v = world.GetBody(i).GetTransformedVertices();
            vector<float> vertices = {
                 (1/zoom) * ((v[1].x / width) * 2 - 1), (1/zoom) * (1 - (v[1].y / height) * 2),0.0f,
                 (1/zoom) * ((v[0].x / width) * 2 - 1), (1/zoom) * (1 - (v[0].y / height) * 2),0.0f,
                 (1/zoom) * ((v[2].x / width) * 2 - 1), (1/zoom) * (1 - (v[2].y / height) * 2),0.0f
            };
            int offset = allVert.size() / 3;
            vector<int> indices = {
                offset + 0,offset + 1,offset + 2
            };
            allVert.insert(allVert.end(), vertices.begin(), vertices.end());
            allIndic.insert(allIndic.end(), indices.begin(), indices.end());


        }
        else if (world.GetBody(i).shapetype == box) {
            vector<EpsilonVector> v = world.GetBody(i).GetTransformedVertices();
            vector<float> vertices = {

                 (1/zoom) * ((v[3].x / width) * 2 - 1), (1/zoom) * (1 - (v[3].y / height) * 2),0.0f,
                 (1/zoom) * ((v[0].x / width) * 2 - 1), (1/zoom) * (1 - (v[0].y / height) * 2),0.0f,
                 (1/zoom) * ((v[1].x / width) * 2 - 1), (1/zoom) * (1 - (v[1].y / height) * 2),0.0f,
                 (1/zoom) * ((v[2].x / width) * 2 - 1), (1/zoom) * (1 - (v[2].y / height) * 2),0.0f
            };
            int offset = allVert.size() / 3;
            vector<int> indices = {
                offset + 0,offset + 1,offset + 2,
                offset + 0,offset + 3,offset + 2
            };
            allVert.insert(allVert.end(), vertices.begin(), vertices.end());
            allIndic.insert(allIndic.end(), indices.begin(), indices.end());

        }
        else if (world.GetBody(i).shapetype == circle) {
            EpsilonBody body = world.GetBody(i);
            float pi = 3.1415926;
            vector<float> vertices;
            vector<int> indices;
            vertices.push_back((1/zoom) * ((body.position.x / width) * 2 - 1));
            vertices.push_back((1/zoom) * (1 - (body.position.y / height) * 2));
            vertices.push_back(0.0f);
            int segments = 10;
            float rad = body.radius;
            EpsilonVector pos = body.position;
            for (int i = 0; i <= segments; i++) {
                float angle = 2.0f * pi * (float)i / (float)segments;
                vertices.push_back((1/zoom) * (((pos.x + cos(angle) * rad) / width) * 2 - 1));
                vertices.push_back((1/zoom) * (1 - ((pos.y + sin(angle) * rad) / height) * 2));
                vertices.push_back(0.0f);
            }
            int offset = allVert.size() / 3;
            for (int i = 0; i < segments; i++) {
                indices.push_back(offset + 0);
                indices.push_back(offset + i + 1);
                indices.push_back(offset + i + 2);
            }
            allVert.insert(allVert.end(), vertices.begin(), vertices.end());
            allIndic.insert(allIndic.end(), indices.begin(), indices.end());
        }
        if (world.GetBody(i).connectiontype == spring) {

            float vertices[] = {
                 (1/zoom) * ((world.GetBody(i).connectionPosition.x / width) * 2 - 1), (1/zoom) * (1 - (world.GetBody(i).connectionPosition.y / height) * 2),0.0f,
                 (1/zoom) * ((world.GetBody(i).originPosition.x / width) * 2 - 1), (1/zoom) * (1 - (world.GetBody(i).originPosition.y / height) * 2),0.0f
            };
            int indices[] = {
                 0,  1
            };
            glUseProgram(shaderProgram);
            glUniform4f(glGetUniformLocation(shaderProgram, "Color"), 0.0f, 1.0f, 0.0f, 1.0f);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_DYNAMIC_DRAW);
            glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
        }
        else if (world.GetBody(i).connectiontype == thr) {
            float vertices[] = {
                 (1/zoom) * ((world.GetBody(i).connectionPosition.x / width) * 2 - 1), (1/zoom) * (1 - (world.GetBody(i).connectionPosition.y / height) * 2),0.0f,
                 (1/zoom) * ((world.GetBody(i).originPosition.x / width) * 2 - 1), (1/zoom) * (1 - (world.GetBody(i).originPosition.y / height) * 2),0.0f
            };
            int indices[] = {
                 0,  1
            };
            glUseProgram(shaderProgram);
            glUniform4f(glGetUniformLocation(shaderProgram, "Color"), 0.0f, 0.0f, 1.0f, 1.0f);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_DYNAMIC_DRAW);
            glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
        }
    }

    glUseProgram(shaderProgram);
    glUniform4f(glGetUniformLocation(shaderProgram, "Color"), 1.0f, 1.0f, 1.0f, 1.0f);
    glBufferData(GL_ARRAY_BUFFER, allVert.size() * sizeof(float), allVert.data(), GL_DYNAMIC_DRAW);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, allIndic.size() * sizeof(unsigned int), allIndic.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, allIndic.size(), GL_UNSIGNED_INT, 0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    vector<float> waterVertices;
    vector<int> waterIndices;
    for (int i = 0; i < world.GetWaterCount(); i++) {

        Water w = world.GetWater(i);
        vector<float> vertices = {
                (1/zoom) * (((w.surfacePosition.x - (w.width / 2.f)) / width) * 2 - 1), (1/zoom) * (1 - ((w.surfacePosition.y + (w.depth)) / height) * 2),0.0f,
                (1/zoom) * (((w.surfacePosition.x - (w.width / 2.f)) / width) * 2 - 1), (1/zoom) * (1 - ((w.surfacePosition.y) / height) * 2),0.0f,
                (1/zoom) * (((w.surfacePosition.x + (w.width / 2.f)) / width) * 2 - 1), (1/zoom) * (1 - ((w.surfacePosition.y) / height) * 2),0.0f,
                (1/zoom) * (((w.surfacePosition.x + (w.width / 2.f)) / width) * 2 - 1), (1/zoom) * (1 - ((w.surfacePosition.y + (w.depth)) / height) * 2),0.0f
        };
        int offset = waterVertices.size() / 3;
        vector<int> indices = {
            offset + 0,offset + 1,offset + 2,
            offset + 0,offset + 3,offset + 2
        };
        waterVertices.insert(waterVertices.end(), vertices.begin(), vertices.end());
        waterIndices.insert(waterIndices.end(), indices.begin(), indices.end());
    }
    glUniform4f(glGetUniformLocation(shaderProgram, "Color"), 0.06f, 0.44f, 0.99f, 0.4f);
    glUseProgram(shaderProgram);
    glBufferData(GL_ARRAY_BUFFER, waterVertices.size() * sizeof(float), waterVertices.data(), GL_DYNAMIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, waterIndices.size() * sizeof(unsigned int), waterIndices.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, waterIndices.size(), GL_UNSIGNED_INT, 0);

    glDisable(GL_BLEND);
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteProgram(shaderProgram);
}


int main() {  
    srand(time(0));
    bool ispressed = false;
    int contype = 0;
    float timer = 0.02f;
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
    EpsilonScheduler scheduler;
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(960, 540), 1.f, 0.5f, 300, 3, true, none));
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(1080, 540), 1.f, 0.5f, 3, 300, true, none));
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(840, 540), 1.f, 0.5f, 3, 300, true, none));   
    world.CreateWater(EpsilonVector(960, 580), 150, 30, 1);
    GLFWwindow* windowGL = glfwCreateWindow(width, height, "Epsilon", monitorGL, NULL);
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
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(windowGL)) {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        scheduler.Schedule([&]() {
            InputsGL(world, windowGL, deltaTime, ispressed, contype, timer, origin, width, height, zoom);
        });
        //InputsGL(world, windowGL, deltaTime, ispressed, contype, timer, origin,width,height,zoom);
        scheduler.Schedule([&]() {
            world.Update(deltaTime, 8);
        });
        //world.Update(deltaTime, 8);
        scheduler.Schedule([&]() {
            Draw(world,width,height,zoom);
        });
        
        scheduler.Run();
        glfwSwapBuffers(windowGL);
        glfwPollEvents();
    }
   
    glfwTerminate();


    return 0;
}
