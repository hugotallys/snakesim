#ifndef LINK_H
#define LINK_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#define LINK_WIDTH 10.0f
#define JOINT_RADIUS 8.0f


float rad2deg(float rad);

float deg2rad(float deg);


class Link {
private:
    float length;
    float angle;
    sf::Vector2f iniPos;
    sf::Vector2f endPos;
    Link* prevLink;
    Link* nextLink;
public:
    Link(float x, float y, float length, float angle);
    Link(Link* prevLink, float length, float angle);
    void draw_rectangle(sf::RenderWindow& window);
    void draw_circle(sf::RenderWindow& window);
    void update(float angle);
    void printPositions();
};

#endif