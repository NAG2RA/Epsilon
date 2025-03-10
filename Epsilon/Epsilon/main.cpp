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
    for (size_t i = 0; i < 10; i++) {
        int randint = rand() % 2;
        float posX = rand() % 501;
        float posY = rand() % 501;
        if (randint == 0) {
            world.AddBody(EpsilonBody::CreateCircleBody(Vector2f(posX, posY), 1, 1, 20, false));
        }
        else {
            world.AddBody(EpsilonBody::CreateBoxBody(Vector2f(posX, posY), 1, 1, 50, 50, false));
        }
    }
    while (window.isOpen()) {   
        Time d = dt.restart();
        float deltatime = d.asMicroseconds();
        while (const optional event = window.pollEvent())
        {
            // "close requested" event: we close the window
            if (event->is<sf::Event::Closed>())
                window.close();
        }  
        if (Keyboard::isKeyPressed(Keyboard::Key::A)) {
            world.bodyList[0].acceleration.x = -.1;
        }
        if (Keyboard::isKeyPressed(Keyboard::Key::D)) {
            world.bodyList[0].acceleration.x = .1;
        }
        if (Keyboard::isKeyPressed(Keyboard::Key::W)) {
            world.bodyList[0].acceleration.y = -.1;
        }
        if (Keyboard::isKeyPressed(Keyboard::Key::S)) {
            world.bodyList[0].acceleration.y = .1;
        }
        world.Update(deltatime);
        world.bodyList[0].updateMovement(deltatime);
        
        window.clear(Color::Black);
        for (size_t i = 0; i < world.bodyList.size(); i++) {
            if (world.bodyList[i].shapetype == box) {
                RectangleShape r({ 50,50 });
                r.setPosition(world.bodyList[i].position);
                r.setOrigin({ 25,25 });
                window.draw(r);
            }
            else {
                CircleShape c(20);
                c.setPosition(world.bodyList[i].position);
                c.setOrigin({ 20,20 });
                window.draw(c);
            }
            if (i == 0) {
                continue;
            }
            world.bodyList[i].updateMovement(deltatime);
        }
        window.display();
        world.bodyList[0].acceleration = Vector2f({ 0,0 });
    }
    return 0;
}