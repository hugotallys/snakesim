#include <SFML/Graphics.hpp>
#include "Link.hpp"

// Main function
int main() {
    // Create a window
    sf::RenderWindow window(sf::VideoMode(800, 600), "SnakeSim");
    window.setFramerateLimit(60);
    
    float initialAngle = deg2rad(45.0f);
    float initialAngle2 = deg2rad(-45.0f);
    float initialAngle3 = deg2rad(45.0f);

    // Creates a link
    Link link(400.0f, 300.0f, 100.0f, deg2rad(initialAngle));
    Link link2(&link, 100.0f, deg2rad(initialAngle2));
    Link link3(&link2, 100.0f, deg2rad(initialAngle3));
    
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
        
        // Draw rectangles
        link.draw_rectangle(window);
        link2.draw_rectangle(window);
        link3.draw_rectangle(window);

        // Draw circles
        link.draw_circle(window);
        link2.draw_circle(window);
        link3.draw_circle(window);

        // Updates the links
        initialAngle += deg2rad(1.0f);
        initialAngle2 -= deg2rad(1.0f);
        initialAngle3 += deg2rad(1.0f);

        link.update(initialAngle);
        link2.update(initialAngle2);
        link3.update(initialAngle3);

        // Display the window
        window.display();
    }

    return 0;
}
