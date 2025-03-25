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
using namespace sf;
using namespace std;

int main() {  
    srand(time(0));
    EpsilonWorld world;
    Vector2f acceleration(0, 0);
    Vector2f velocity(0, 0);
    Clock dt;
    float drag = 0.99f;
    RenderWindow window(VideoMode({ 1280, 720 }), "mywindow");
    window.setFramerateLimit(320);   
    world.AddBody(EpsilonBody::CreateBoxBody(Vector2f(100, 500), 0.5f, 0.5f, 1000, 50, true));
    RectangleShape r({ 1000,50 });
    r.setPosition(world.bodyList[0].position);
    r.setOrigin({ 500,25 });
    bool ispressed = false;
    while (window.isOpen()) {   
        Time d = dt.restart();
        float deltatime = d.asMicroseconds();
        while (const optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        } 
        if (Mouse::isButtonPressed(Mouse::Button::Left)) {
            if (!ispressed) {
                Vector2f pos = static_cast<Vector2f>(Mouse::getPosition(window));
                world.AddBody(EpsilonBody::CreateBoxBody(pos, 0.5f, 0.5f, 50, 50, false));
            }
            
            ispressed = true;
        }
        else if (Mouse::isButtonPressed(Mouse::Button::Right)) {

            if (!ispressed) {
                Vector2f pos = static_cast<Vector2f>(Mouse::getPosition(window));
                world.AddBody(EpsilonBody::CreateCircleBody(pos, 0.5f, 0.5f, 25, false));
            }
            ispressed = true;
        }
        else {
            ispressed = false;
        }
        r.setPosition(world.bodyList[0].position);
        world.Update(deltatime);
        window.clear(Color::Black);
        for (size_t i = 1; i < world.bodyList.size(); i++) {
            if (world.bodyList[i].shapetype == box) {
                RectangleShape rc({ 50,50 });
                rc.setOrigin({ 25,25 });
                rc.setPosition(world.bodyList[i].position);  
                window.draw(rc);
                
            }
            else {
                CircleShape c(25);
                c.setOrigin({ 25,25 });
                c.setPosition(world.bodyList[i].position);
                window.draw(c);
            }
            AABB box = world.bodyList[i].GetAABB();
            if (box.min.y > 720) {
                world.RemoveBody(i);
            }
            
        }
        
        window.draw(r);
        window.display();
    }
   
    return 0;
}