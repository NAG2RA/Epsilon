#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include<ctime>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"EpsilonWorld.h"
#include"AABB.h"
#include"EpsilonVector.h"
using namespace sf;
using namespace std;

Vector2f EpToVec2(EpsilonVector c) {
    return Vector2f(c.x, c.y);
}
EpsilonVector Vec2ToEp(Vector2f c) {
    return EpsilonVector(c.x, c.y);
}
int main() {  
    srand(time(0));
    EpsilonWorld world;
    EpsilonVector a(1, 0);
    EpsilonVector b(2, 3);
    float c =2;
    a +=c*b;
    Vector2f acceleration(0, 0);
    Vector2f velocity(0, 0);
    Clock dt;
    float drag = 0.99f;
    RenderWindow window(VideoMode({ 1280, 720 }), "mywindow");
    View v = window.getDefaultView();
    v.zoom(.05f);
    window.setView(v);
    window.setFramerateLimit(320);   
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(640, 370), 0.5f, 0.5f, 30, 3, true));
    RectangleShape r({ 30,3 });
    r.setPosition(EpToVec2(world.bodyList[0].position));
    r.setOrigin({ 15,1.5 });
    bool ispressed = false;
    while (window.isOpen()) {   
        Time d = dt.restart();
        float deltatime = d.asSeconds();
        while (const optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        } 
        if (Mouse::isButtonPressed(Mouse::Button::Left)) {
            if (!ispressed) {
                Vector2i postemp = Mouse::getPosition(window);
                Vector2f pos = window.mapPixelToCoords(postemp);
                world.AddBody(EpsilonBody::CreateBoxBody(Vec2ToEp(pos), 1, 0.5f, 2, 2, false));
            }
            
            ispressed = true;
        }
        else if (Mouse::isButtonPressed(Mouse::Button::Right)) {

            if (!ispressed) {
                Vector2i postemp = Mouse::getPosition(window);
                Vector2f pos = window.mapPixelToCoords(postemp);
                world.AddBody(EpsilonBody::CreateCircleBody(Vec2ToEp(pos), 1, 0.5f, 1, false));
            }
            ispressed = true;
        }
        else if (Mouse::isButtonPressed(Mouse::Button::Middle)) {

            if (!ispressed) {
                Vector2i postemp = Mouse::getPosition(window);
                Vector2f pos = window.mapPixelToCoords(postemp);
                world.AddBody(EpsilonBody::CreateTriangleBody(Vec2ToEp(pos), 1, 0.5f, 2, false));
            }
            ispressed = true;
        }
        else {
            ispressed = false;
        }
        r.setPosition(EpToVec2(world.bodyList[0].position));
        world.Update(deltatime, 20);
        window.clear(Color::Black);
        for (size_t i = 1; i < world.bodyList.size(); i++) {
            if (world.bodyList[i].shapetype == box) {
                RectangleShape rc({ 2,2 });
                rc.setOrigin({ 1,1 });
                rc.setPosition(EpToVec2(world.bodyList[i].position));  
                Angle angle = radians(world.bodyList[i].angle);
                rc.setRotation(angle);
                window.draw(rc);
            }
            else if (world.bodyList[i].shapetype == triangle) {
                float rad = world.bodyList[i].width / sqrt(3);
                CircleShape c(rad,3);
                c.setOrigin({rad, world.bodyList[i].height/2.f });
                c.setPosition(EpToVec2(world.bodyList[i].position));
                Angle angle = radians(world.bodyList[i].angle);
                c.setRotation(angle);
                RectangleShape rc({ 0.1,0.1 });
                rc.setPosition(EpToVec2(world.bodyList[i].GetTransformedVertices()[0]));
                rc.setRotation(angle);
                rc.setFillColor(Color::Red);
                RectangleShape rc2({ 0.1,0.1 });
                rc2.setPosition(EpToVec2(world.bodyList[i].GetTransformedVertices()[1]));
                rc2.setRotation(angle);
                rc2.setFillColor(Color::Red);
                RectangleShape rc3({ 0.1,0.1 });
                rc3.setPosition(EpToVec2(world.bodyList[i].GetTransformedVertices()[2]));
                rc3.setRotation(angle);
                rc3.setFillColor(Color::Red);
                window.draw(c);
                window.draw(rc);
                window.draw(rc2);
                window.draw(rc3);
            }
            else {
                CircleShape c(1);
                c.setOrigin({ 1,1 });
                c.setPosition(EpToVec2(world.bodyList[i].position));
                Angle angle = radians(world.bodyList[i].angle);
                c.setRotation(angle);
                RectangleShape rc({ 1,0.1 });
                rc.setPosition(EpToVec2(world.bodyList[i].position));
                rc.setRotation(angle);
                rc.setFillColor(Color::Red);
                window.draw(c);
                window.draw(rc);
            }
            AABB box = world.bodyList[i].GetAABB();
            if (box.min.y > 400) {
                world.RemoveBody(i);
            }
            
        }
        window.draw(r);
        window.display();
    }

    return 0;
}