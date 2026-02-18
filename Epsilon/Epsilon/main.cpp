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
#include<thread>
#include<Tracy.hpp>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"EpsilonWorld.h"
#include"AABB.h"
#include"EpsilonVector.h"
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
void UnitTestVector()
{
    EpsilonVector a(5, 4);
    EpsilonVector b(12, -37);
    EpsilonVector zero(0,0);
    assert((a + b).x == 17 && (a + b).y == -33 && (a - b).x == -7 && (a - b).y == 41 && (a.Dot(b)) == -88 && (a.Cross(b)) == -233
        && !isnan(zero.Normalized().x) && !isnan(zero.Normalized().y));
}

void UnitTestAABB()
{
    AABB a(EpsilonVector(0,0), EpsilonVector(2, 2));
    AABB b(EpsilonVector(1, 1), EpsilonVector(3, 3));
    AABB c(EpsilonVector(0, 0), EpsilonVector(1, 1));
    AABB d(EpsilonVector(1, 0), EpsilonVector(2, 1));
    assert(Collisions::IntersectAABB(a, b) && Collisions::IntersectAABB(c, d));

}

void UnitTestUpdateMovement() {
    EpsilonBody bd = EpsilonBody::CreateBoxBody(EpsilonVector(0, 0), 0.7f, 0.5f, 4, 4, false, none);
    float dt = 1.0f;
    EpsilonVector gravity(0, 9.8f);
    bd.updateMovement(dt, gravity, 1);
    assert(Collisions::NearlyEqual(bd.position.y, 4.9f));

}
class Renderer {
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
    void Initialize() {
        int success;
        char infoLog[512];
       
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
        colorLoc = glGetUniformLocation(shaderProgram, "Color");
        screenSize = glGetUniformLocation(shaderProgram, "screenSize");
        zoomGL = glGetUniformLocation(shaderProgram, "zoom");
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    }
    void Render(EpsilonWorld& world, int width, int height, float zoom) {
        //ZoneScoped;
        allVert.clear();
        allIndic.clear();

        allVert.reserve(world.GetBodyCount() * 12);
        allIndic.reserve(world.GetBodyCount() * 6);
        
        glUniform2f(screenSize, width, height);
        glUniform1f(zoomGL, zoom);
        for (int i = 0; i < world.GetBodyCount(); i++) {
            //ZoneScoped;
           // ZoneScopedN("Body");
            if (world.GetBody(i).shapetype == triangle) {
                vector<EpsilonVector> v = world.GetBody(i).GetTransformedVertices();
                vector<float> vertices = {
                    v[1].x, v[1].y,0.0f,
                    v[0].x, v[0].y,0.0f,
                    v[2].x, v[2].y,0.0f
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

                     v[3].x, v[3].y,0.0f,
                     v[0].x, v[0].y,0.0f,
                     v[1].x, v[1].y,0.0f,
                     v[2].x, v[2].y,0.0f
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
                vertices.push_back(body.position.x);
                vertices.push_back(body.position.y);
                vertices.push_back(0.0f);
                int segments = 10;
                float rad = body.radius;
                EpsilonVector pos = body.position;
                for (int i = 0; i <= segments; i++) {
                    float angle = 2.0f * pi * (float)i / (float)segments;
                    vertices.push_back(pos.x + cos(angle) * rad);
                    vertices.push_back(pos.y + sin(angle) * rad);
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
                     world.GetBody(i).connectionPosition.x, world.GetBody(i).connectionPosition.y,0.0f,
                     world.GetBody(i).originPosition.x, world.GetBody(i).originPosition.y,0.0f
                };
                int indices[] = {
                     0,  1
                };
                glUseProgram(shaderProgram);
                glUniform4f(colorLoc, 0.0f, 1.0f, 0.0f, 1.0f);
                glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), NULL, GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ARRAY_BUFFER, 0,sizeof(vertices), vertices);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), NULL, GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0, sizeof(indices), indices);
                glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
            }
            else if (world.GetBody(i).connectiontype == thr) {
                float vertices[] = {
                    world.GetBody(i).connectionPosition.x, world.GetBody(i).connectionPosition.y,0.0f,
                     world.GetBody(i).originPosition.x, world.GetBody(i).originPosition.y,0.0f
                };
                int indices[] = {
                     0,  1
                };
                glUseProgram(shaderProgram);
                glUniform4f(colorLoc, 0.0f, 0.0f, 1.0f, 1.0f);
                glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), NULL, GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), NULL, GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(indices), indices);
                glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
            }
        }
        {
            //ZoneScopedN("RenderBody");
            glUseProgram(shaderProgram);
            glUniform4f(colorLoc, 1.0f, 1.0f, 1.0f, 1.0f);
            glBufferData(GL_ARRAY_BUFFER, allVert.size() * sizeof(float), NULL, GL_DYNAMIC_DRAW);
            glBufferSubData(GL_ARRAY_BUFFER, 0, allVert.size() * sizeof(float), allVert.data());

            glBufferData(GL_ELEMENT_ARRAY_BUFFER, allIndic.size() * sizeof(unsigned int), NULL, GL_DYNAMIC_DRAW);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0, allIndic.size() * sizeof(unsigned int), allIndic.data());
            glDrawElements(GL_TRIANGLES, allIndic.size(), GL_UNSIGNED_INT, 0);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        waterVertices.clear();
        waterIndices.clear();
        waterVertices.reserve(world.GetWaterCount() * 12);
        waterIndices.reserve(world.GetWaterCount() * 6);
        for (int i = 0; i < world.GetWaterCount(); i++) {
            //ZoneScopedN("Water");
            Water w = world.GetWater(i);
            vector<float> vertices = {
                    w.surfacePosition.x - (w.width / 2.f), w.surfacePosition.y + (w.depth),0.0f,
                    w.surfacePosition.x - (w.width / 2.f), w.surfacePosition.y,0.0f,
                    w.surfacePosition.x + (w.width / 2.f), w.surfacePosition.y,0.0f,
                    w.surfacePosition.x + (w.width / 2.f), w.surfacePosition.y + (w.depth),0.0f
            };
            int offset = waterVertices.size() / 3;
            vector<int> indices = {
                offset + 0,offset + 1,offset + 2,
                offset + 0,offset + 3,offset + 2
            };
            waterVertices.insert(waterVertices.end(), vertices.begin(), vertices.end());
            waterIndices.insert(waterIndices.end(), indices.begin(), indices.end());
        }
        {
            //ZoneScopedN("RenderWater");
            glUniform4f(colorLoc, 0.06f, 0.44f, 0.99f, 0.4f);
            glUseProgram(shaderProgram);
            glBufferData(GL_ARRAY_BUFFER, waterVertices.size() * sizeof(float), NULL, GL_DYNAMIC_DRAW);
            glBufferSubData(GL_ARRAY_BUFFER,0, waterVertices.size() * sizeof(float), waterVertices.data());
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, waterIndices.size() * sizeof(unsigned int), NULL, GL_DYNAMIC_DRAW);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0, waterIndices.size() * sizeof(unsigned int), waterIndices.data());
            glDrawElements(GL_TRIANGLES, waterIndices.size(), GL_UNSIGNED_INT, 0);

            glDisable(GL_BLEND);
        }
    }
    ~Renderer() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteProgram(shaderProgram);
    }
};
void InputsGL(EpsilonWorld& world, GLFWwindow* window, float deltatime, bool& ispressed, int& contype, float& timer, EpsilonVector& origin, int width, int height,float zoom) {
    
    //ZoneScoped;

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

                world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(spawnX,spawnY), 0.7f, 0.5f, 4, 4, false, none));
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
int main() {
    Renderer renderer;
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
