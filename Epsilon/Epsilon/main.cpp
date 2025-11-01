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
    float timer = 0.02f;
    EpsilonWorld world;
    EpsilonVector origin;
    Clock dt;
    RenderWindow window(VideoMode({ 1280,720 }), "Epsilon");
    View v = window.getDefaultView();
    v.zoom(.5f);
    window.setView(v);
    ContextSettings settings;
    settings.antiAliasingLevel = 0;
    window.setFramerateLimit(320);   
   world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(640, 370), 1.f, 0.5f, 300, 3, true, none));
   RectangleShape r({ 300,3 });
    r.setPosition(EpToVec2(world.GetBody(0).position));
    r.setOrigin({ 150,1.5 });
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(740, 370), 1.f, 0.5f, 3, 300, true, none));
    RectangleShape r1({ 3,300 });
    r1.setPosition(EpToVec2(world.GetBody(1).position));
    r1.setOrigin({ 1.5,150 });
    world.AddBody(EpsilonBody::CreateBoxBody(EpsilonVector(540, 370), 1.f, 0.5f, 3, 300, true, none));
    RectangleShape r2({ 3,300 });
    r2.setPosition(EpToVec2(world.GetBody(2).position));
    r2.setOrigin({ 1.5,150 });
    bool ispressed = false;
    int contype = 0;
    RectangleShape water({ 30 , 3 });
    water.setOrigin({ 15, 0 });
    water.setPosition(Vector2f(640, 375));
    world.CreateWater(EpsilonVector(640, 375), 30, 3, 1);
    water.setFillColor(Color(16, 112, 222, 100));
    
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
                if (contype == 0) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    world.AddBody(EpsilonBody::CreateBoxBody(Vec2ToEp(pos), 0.7f, 0.5f, 2, 2, false, none));
                }
                else if (contype == 1) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    EpsilonBody body = EpsilonBody::CreateBoxBody(Vec2ToEp(pos), 0.7f, 0.5f, 2, 2, false, spring);
                    body.CreateConnection(origin);
                    world.AddBody(body);
                    contype = 0;
                }
                else if (contype == 2) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    EpsilonBody body = EpsilonBody::CreateBoxBody(Vec2ToEp(pos), 0.7f, 0.5f, 2, 2, false, thr);
                    body.CreateConnection(origin);
                    world.AddBody(body);
                    contype = 0;
                }
            }
            
            ispressed = true;
        }
        else if (Mouse::isButtonPressed(Mouse::Button::Right)) {
            
            if (!ispressed) {
                if (contype == 0) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    world.AddBody(EpsilonBody::CreateCircleBody(Vec2ToEp(pos), 1.5f, 1, 1, false, none));
                }
                else if (contype == 1) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    EpsilonBody body = EpsilonBody::CreateCircleBody(Vec2ToEp(pos), 1.5f, 1, 1, false, spring);
                    body.CreateConnection(origin);
                    world.AddBody(body);
                    contype = 0;
                }
                else if (contype == 2) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    EpsilonBody body = EpsilonBody::CreateCircleBody(Vec2ToEp(pos), 1.5f, 1, 1, false, thr);
                    body.CreateConnection(origin);
                    world.AddBody(body);
                    contype = 0;
                }
            }
            ispressed = true;
        }
        else if (Mouse::isButtonPressed(Mouse::Button::Middle)) {

            if (!ispressed) {
                if (contype == 0) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    world.AddBody(EpsilonBody::CreateTriangleBody(Vec2ToEp(pos), 1.5f, 0.5f, 2, false, none));
                }
                else if (contype == 1) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    EpsilonBody body = EpsilonBody::CreateTriangleBody(Vec2ToEp(pos), 1, 0.5f, 2, false, spring);
                    body.CreateConnection(origin);
                    world.AddBody(body);
                    contype = 0;
                }
                else if (contype == 2) {
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    EpsilonBody body = EpsilonBody::CreateTriangleBody(Vec2ToEp(pos), 1, 0.5f, 2, false, thr);
                    body.CreateConnection(origin);
                    world.AddBody(body);
                    contype = 0;
                }
            }
            ispressed = true;
        }
        else if (Keyboard::isKeyPressed(Keyboard::Key::A)) {
            if (!ispressed) {
                Vector2i postemp = Mouse::getPosition(window);
                Vector2f pos = window.mapPixelToCoords(postemp);
                world.Explosion(Vec2ToEp(pos), 10, 100);
            }
            ispressed = true;
          
        }
        else if (Keyboard::isKeyPressed(Keyboard::Key::C) && Keyboard::isKeyPressed(Keyboard::Key::S)) {
            contype = 1;
            Vector2i postemp = Mouse::getPosition(window);
            Vector2f pos = window.mapPixelToCoords(postemp);
            origin = Vec2ToEp(pos);
        }
        else if (Keyboard::isKeyPressed(Keyboard::Key::C) && Keyboard::isKeyPressed(Keyboard::Key::T)) {
            contype = 2;
            Vector2i postemp = Mouse::getPosition(window);
            Vector2f pos = window.mapPixelToCoords(postemp);
            origin = Vec2ToEp(pos);
        }
        else if (Keyboard::isKeyPressed(Keyboard::Key::H)) {
            timer -= deltatime;
            if (timer <= 0) {
                Vector2i postemp = Mouse::getPosition(window);
                Vector2f pos = window.mapPixelToCoords(postemp);
                world.AddBody(EpsilonBody::CreateCircleBody(Vec2ToEp(pos), 1.5f, 1, 1, false, none));
                timer = 0.02f;
            }
        }
        else if (Keyboard::isKeyPressed(Keyboard::Key::Q)) {
            cout << world.GetBodyCount() << endl;
        }
        else {
            ispressed = false;
        }
        r.setPosition(EpToVec2(world.GetBody(0).position));
        world.Update(deltatime, 16);
        window.clear(Color::Black);
        for (int i = 0; i < world.GetBodyCount(); i++) {
           
                if (world.GetBody(i).shapetype == box) {
                    RectangleShape rc({ 2,2 });
                    rc.setOrigin({ 1,1 });
                    rc.setPosition(EpToVec2(world.GetBody(i).position));
                    Angle angle = radians(world.GetBody(i).angle);
                    rc.setRotation(angle);
                    window.draw(rc);
                }
                else if (world.GetBody(i).shapetype == triangle) {
                    float rad = world.GetBody(i).width / sqrt(3);
                    CircleShape c(rad, 3);
                    c.setOrigin({ rad, world.GetBody(i).height * 2.f / 3.f });
                    c.setPosition(EpToVec2(world.GetBody(i).position));
                    Angle angle = radians(world.GetBody(i).angle);
                    c.setRotation(angle);
                    window.draw(c);
                }
                else {
                    CircleShape c(1);
                    c.setOrigin({ 1,1 });
                    c.setPosition(EpToVec2(world.GetBody(i).position));
                    Angle angle = radians(world.GetBody(i).angle);
                    c.setRotation(angle);
                    RectangleShape rc({ 1,0.1 });
                    rc.setPosition(EpToVec2(world.GetBody(i).position));
                    rc.setRotation(angle);
                    rc.setFillColor(Color::Red);
                    window.draw(c);
                    window.draw(rc);
                }
                if (world.GetBody(i).connectiontype == thr) {
                    VertexArray line(PrimitiveType::Lines, 2);
                    line[0].position = EpToVec2(world.GetBody(i).originPosition);
                    line[0].color = Color::Blue;
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    line[1].position = EpToVec2(world.GetBody(i).connectionPosition);
                    line[1].color = Color::Blue;
                    window.draw(line);
                }
                else if (world.GetBody(i).connectiontype == spring) {
                    VertexArray line(PrimitiveType::Lines, 2);
                    line[0].position = EpToVec2(world.GetBody(i).originPosition);
                    line[0].color = Color::Green;
                    Vector2i postemp = Mouse::getPosition(window);
                    Vector2f pos = window.mapPixelToCoords(postemp);
                    line[1].position = EpToVec2(world.GetBody(i).connectionPosition);
                    line[1].color = Color::Green;
                    window.draw(line);
                }

            
            
            AABB box = world.GetBody(i).GetAABB();
            if (box.min.y > 400) {
                world.RemoveBody(i);
            }
            
        }
        window.draw(r);
        window.draw(r1);
        window.draw(r2);
        window.draw(water);
        window.display();
    }

    return 0;
}