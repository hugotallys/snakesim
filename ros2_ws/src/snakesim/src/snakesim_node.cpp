#include <SFML/Graphics.hpp>
#include "Link.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


#define WINDOW_WIDTH 700
#define WINDOW_HEIGHT 700

#define ARM_LENGTH 100.0f

// Main function
int main() {
    // Create a window
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "SnakeSim");
    window.setFramerateLimit(60);
    
    float q0[3] = {0.0f, 0.0f, 0.0f};

    // Creates a link
    Link link(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f, ARM_LENGTH, q0[0]);
    Link link2(&link, ARM_LENGTH, q0[1]);
    Link link3(&link2, ARM_LENGTH, q0[2]);

    // Creates snake head by reading a png sprite:
    sf::Texture texture;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("snakesim");
    std::string full_path_to_image = package_share_directory + "/resources/snake.png";

    if (!texture.loadFromFile(full_path_to_image)) {
        std::cout << "Error loading texture" << std::endl;
    }

    sf::Sprite sprite;
    sprite.setTexture(texture);

    // gets the size of the sprite
    sf::Vector2u size = texture.getSize();
    // sets the origin of the sprite 0 pixels from the left and sprite height / 2 pixels from the top
    sprite.setOrigin(0.0f, size.y / 2.0f);
    // scales down the sprite to 0.5 of its original size
    sprite.setScale(0.2f, 0.2f);

    // Main loop
    while (window.isOpen()) {
        // Event handling
        sf::Event event;
        
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }  
        
        // Clear the window
        window.clear();

        // Sets background color to RGB(89, 36, 224)
        window.clear(sf::Color(89, 36, 224));
        
        // Draw rectangles
        link.draw_rectangle(window);
        link2.draw_rectangle(window);
        link3.draw_rectangle(window);

        // Draw circles
        link.draw_circle(window);
        link2.draw_circle(window);
        link3.draw_circle(window);

        // Draw the sprite at the and of link3
        sprite.setPosition(link3.getEndPos());
        sprite.setRotation(rad2deg(link3.getAngle()));
        window.draw(sprite);

        // Updates the links
        q0[0] += 0.01f;
        q0[1] += 0.02f;
        q0[2] += 0.03f;

        link.update(q0[0]);
        link2.update(M_PI_2*cos(q0[1]));
        link3.update(M_PI_2*sin(q0[2]));

        // Display the window
        window.display();
    }

    return 0;
}
