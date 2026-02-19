#include "EpsilonRenderer.h"
string EpsilonRenderer::loadShaderSrc(const char* filename)
{
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
void EpsilonRenderer::Initialize() {
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
void EpsilonRenderer::Render(EpsilonWorld& world, int width, int height, float zoom)
{
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
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), NULL, GL_DYNAMIC_DRAW);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(indices), indices);
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
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, allIndic.size() * sizeof(unsigned int), allIndic.data());
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
        glBufferSubData(GL_ARRAY_BUFFER, 0, waterVertices.size() * sizeof(float), waterVertices.data());
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, waterIndices.size() * sizeof(unsigned int), NULL, GL_DYNAMIC_DRAW);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, waterIndices.size() * sizeof(unsigned int), waterIndices.data());
        glDrawElements(GL_TRIANGLES, waterIndices.size(), GL_UNSIGNED_INT, 0);

        glDisable(GL_BLEND);
    }
}