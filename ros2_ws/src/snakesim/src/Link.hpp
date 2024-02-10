#ifndef LINK_H
#define LINK_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <memory>

#define LINK_WIDTH 10.0f
#define JOINT_RADIUS 8.0f


float rad2deg(float rad);

float deg2rad(float deg);


class Link : public std::enable_shared_from_this<Link> {
private:
    float length;
    float angle;
    sf::Vector2f iniPos;
    sf::Vector2f endPos;
    std::shared_ptr<Link> prevLink;
    std::shared_ptr<Link> nextLink; 
public:
    Link(float x, float y, float length, float angle);
    Link(std::shared_ptr<Link> prevLink, float length, float angle);
    void set_next_link(std::shared_ptr<Link> nextLink);
    void draw_rectangle(sf::RenderWindow& window);
    void draw_circle(sf::RenderWindow& window);
    void update(float angle);
    void printPositions();
    sf::Vector2f getEndPos();
    float getAngle();
};

#endif