#include "SnakeSim.hpp"

SnakeSim::SnakeSim() : /*initialAngles{0, -M_PI_2, -M_PI_2},*/ manEllipse(0.0f, 0.0f), running(true), snakeCount(0) {
    // Create a window
    window = std::make_unique<sf::RenderWindow>(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "SnakeSim");
    window->setFramerateLimit(60);

    // Creates links
    // links.push_back(std::make_shared<Link>(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f, ARM_LENGTH, initialAngles[0]));
    // links.push_back(std::make_shared<Link>(links[0], ARM_LENGTH, initialAngles[1]));
    // links.push_back(std::make_shared<Link>(links[1], ARM_LENGTH, initialAngles[2]));

    // links[0]->set_next_link(links[1]);
    // links[1]->set_next_link(links[2]);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("snakesim");
    std::string full_path_to_image = package_share_directory + "/resources/snake.png";

    if (texture.loadFromFile(full_path_to_image)) {
        sf::Image icon = texture.copyToImage();
        window->setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    }

    sprite.setTexture(texture);

    sf::Vector2u size = texture.getSize();
    // sets the origin of the sprite 0 pixels from the left and sprite height / 2 pixels from the top
    sprite.setOrigin(0.0f, size.y / 2.0f);
    // scales down the sprite to 0.5 of its original size
    sprite.setScale(0.2f, 0.2f);

    std::cout << "SnakeSim initialized" << std::endl;
}

void SnakeSim::run() {
    // Main loop
    while (running) {

        // Process events
        sf::Event event;
        while (window->pollEvent(event)) {}

        // Clear the window
        window->clear();

        // Sets background color to RGB(89, 36, 224)
        window->clear(sf::Color(89, 36, 224));
        
        if (links.size() > 0) {
            // Draw rectangles
            for (auto& link : links) {
                link->draw_rectangle(*window);
            }

            // Draw circles
            for (auto& link : links) {
                link->draw_circle(*window);
            }

            // Draw trajectory
            this->pushPointToTrajectory();

            for (const auto& point : trajectory) {
                // Display the point as a white circle
                sf::CircleShape circle(2.0f);
                circle.setFillColor(sf::Color::White);
                circle.setPosition(point.position);
                window->draw(circle);
            }

            // Draw sprite

            sf::Vector2f endPos = links[2]->getEndPos();
            float angle = links[2]->getAngle();

            endPos.y = endPos.y + 0.5f * sin(-angle) * sprite.getGlobalBounds().height;
            endPos.x = endPos.x - 0.5f * cos(-angle) * sprite.getGlobalBounds().height;

            sprite.setPosition(endPos);
            sprite.setRotation(rad2deg(angle));
            
            window->draw(sprite);

            // Draw manipulability ellipse

            sf::CircleShape ellipse(ARM_LENGTH);

            ellipse.setOrigin(ARM_LENGTH, ARM_LENGTH);
            ellipse.setPosition(links[2]->getEndPos());
            ellipse.setScale(manEllipse);
            ellipse.setRotation(rad2deg(eigenAngle));
            
            // Set the outline
            ellipse.setFillColor(sf::Color::Transparent); // Make the inside of the ellipse transparent
            ellipse.setOutlineThickness(2); // Set the thickness of the outline
            ellipse.setOutlineColor(sf::Color::White); // Set the color of the outline

            window->draw(ellipse);

            for (int i = 0; i < 3; i++) {
                links[i]->update(initialAngles[i]);
            }
        }
        
        // Display the window
        window->display();
    }
}

int SnakeSim::getSnakeCount() {
    return snakeCount;
}

void SnakeSim::createSnake(const std::array<float, 2>& origin, const std::array<float, 3>& angles) {
    links.clear();
    
    float x = origin[0] + WINDOW_WIDTH / 2.0f;
    float y = origin[1] + WINDOW_HEIGHT / 2.0f;

    links.push_back(std::make_shared<Link>(x, y, ARM_LENGTH, angles[0]));
    links.push_back(std::make_shared<Link>(links[0], ARM_LENGTH, angles[1]));
    links.push_back(std::make_shared<Link>(links[1], ARM_LENGTH, angles[2]));

    links[0]->set_next_link(links[1]);
    links[1]->set_next_link(links[2]);

    this->setAngles(angles);

    snakeCount++;
}


void SnakeSim::setAngles(const std::array<float, 3>& angles) {
    for (int i = 0; i < 3; i++) {
        initialAngles[i] = -angles[i];
    }
}

void SnakeSim::setEigenValues(const std::array<float, 2>& eigenValues) {
    manEllipse.x = eigenValues[0];
    manEllipse.y = eigenValues[1];
}

void SnakeSim::setEigenAngle(float eigenAngle) {
    this->eigenAngle = eigenAngle;
}

void SnakeSim::stop() {
    running = false;
}

void SnakeSim::pushPointToTrajectory() {
    if (trajectory.size() == TRAJECTORY_SIZE) {
        trajectory.pop_front();
    }
    trajectory.push_back(links[2]->getEndPos());
}
