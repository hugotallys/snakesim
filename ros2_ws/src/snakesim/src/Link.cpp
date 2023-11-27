#include "Link.hpp"

#define LINK_WIDTH 10.0f
#define JOINT_RADIUS 8.0f


float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}


Link::Link(float x, float y, float length, float angle) {
    this->length = length;
    this->angle = angle;
    this->iniPos = sf::Vector2f(x, y);
    this->endPos = sf::Vector2f(
        x + length * cos(angle),
        y + length * sin(angle)
    );
    this->prevLink = nullptr;
}

Link::Link(Link* prevLink, float length, float angle) {
    this->length = length;
    this->angle = angle + prevLink->angle;
    this->prevLink = prevLink;
    this->iniPos = prevLink->endPos;
    this->endPos = sf::Vector2f(
        iniPos.x + this->length * cos(this->angle),
        iniPos.y + this->length * sin(this->angle)
    );
    this->prevLink->nextLink = this;
}

// Draw the link as a rectengle
void Link::draw_rectangle(sf::RenderWindow& window) {
    sf::RectangleShape link(sf::Vector2f(this->length, LINK_WIDTH));
    link.setOrigin(0.0f, LINK_WIDTH / 2.0f);
    link.setPosition(this->iniPos);
    link.setRotation(rad2deg(this->angle));
    window.draw(link);
}

// Draw the link as a circle to represent the joint
void Link::draw_circle(sf::RenderWindow& window) {
    sf::CircleShape joint(JOINT_RADIUS);
    joint.setOrigin(JOINT_RADIUS, JOINT_RADIUS);
    joint.setPosition(this->iniPos);
    // sets color to red
    joint.setFillColor(sf::Color(255, 0, 0));
    window.draw(joint);
}

void Link::update(float angle) {
    if (this->prevLink == nullptr) {
        this->angle = angle;
        this->endPos = sf::Vector2f(
            iniPos.x + length * cos(angle),
            iniPos.y + length * sin(angle)
        );
    } else {
        this->angle = angle + prevLink->angle;
        this->iniPos = prevLink->endPos;
        this->endPos = sf::Vector2f(
            iniPos.x + length * cos(this->angle),
            iniPos.y + length * sin(this->angle)
        );
    }
}

void Link::printPositions() {
    std::cout << "iniPos: " << iniPos.x << ", " << iniPos.y << std::endl;
    std::cout << "endPos: " << endPos.x << ", " << endPos.y << std::endl;
}
