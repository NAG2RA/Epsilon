#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"EpsilonBody.h"
#include"Collisions.h"
using namespace sf;
using namespace std;

int main() {  
    Clock dt;
    Vector2f normal(0, 0);
    float depth = 0;
    float drag = 0.99f;
    Vector2f velocity(0, 0);
    Vector2f acceleration(0, 0);
    Vector2f velocity2(0, 0);
    Vector2f acceleration2(0, 0);
    Vector2f pos(0, 0);
    Vector2f posb(100, 100);
    EpsilonBody circle = EpsilonBody::CreateCircleBody(pos, 1, 1, 10, false);
    CircleShape crc(circle.radius);
    crc.setPosition(circle.position);
    crc.setOrigin({ circle.radius / 2 ,circle.radius / 2 });
    EpsilonBody circle2 = EpsilonBody::CreateCircleBody(posb, 1, 1, 10, false);
    CircleShape crc2(circle2.radius);
    crc2.setPosition(circle2.position);
    crc2.setOrigin({ circle2.radius / 2 ,circle2.radius / 2 });
    RenderWindow window(VideoMode({ 1280, 720 }), "mywindow");
    window.setFramerateLimit(320);   
    while (window.isOpen()) {   
        Time d = dt.restart();
        float deltatime = d.asMicroseconds();
        while (const optional event = window.pollEvent())
        {
            // "close requested" event: we close the window
            if (event->is<sf::Event::Closed>())
                window.close();
        }  
        crc.setPosition(circle.position);
        crc2.setPosition(circle2.position);
        if (Keyboard::isKeyPressed(Keyboard::Key::A)) {
            acceleration.x = -.1f;
        }
        if (Keyboard::isKeyPressed(Keyboard::Key::D)) {
            acceleration.x = .1f;
        }
        if (Keyboard::isKeyPressed(Keyboard::Key::W)) {
            acceleration.y = -.1f;
        }
        if (Keyboard::isKeyPressed(Keyboard::Key::S)) {
            acceleration.y = .1f;
        }
        if (Collisions::IntersectCircles(circle.radius, circle2.radius, circle.position, circle2.position, normal, depth)) 
        {
            velocity = -normal * (float)(depth/2);
            velocity2 = normal * (float)(depth/2);
            acceleration = acceleration * 10.f;
            circle2.updateMovement(circle2.position, deltatime, velocity2, acceleration2, drag);
        }
        circle.updateMovement(circle.position, deltatime, velocity, acceleration, drag);

        window.clear(Color::Black);
        window.draw(crc);
        window.draw(crc2);
        window.display();
        acceleration = Vector2f({ 0,0 });
    }
    return 0;
}