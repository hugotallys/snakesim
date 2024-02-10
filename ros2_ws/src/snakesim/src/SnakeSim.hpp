#include <SFML/Graphics.hpp>
#include "Link.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <atomic>
#include <deque>

#define WINDOW_WIDTH 750
#define WINDOW_HEIGHT 750
#define ARM_LENGTH 100.0f
#define TRAJECTORY_SIZE 100

class SnakeSim {
public:
    SnakeSim();
    void run();
    void setAngles(const std::array<float, 3>& angles);
    void setEigenValues(const std::array<float, 2>& eigenValues);
    void setEigenAngle(float eigenAngle);
    void stop();
    void pushPointToTrajectory();
    void createSnake(const std::array<float, 2>& origin, const std::array<float, 3>& angles);
    int getSnakeCount();

private:
    std::unique_ptr<sf::RenderWindow> window;
    std::vector<std::shared_ptr<Link>> links;
    sf::Sprite sprite;
    sf::Texture texture;
    std::array<float, 3> initialAngles;
    std::atomic<bool> running;
    std::deque<sf::Vertex> trajectory;
    sf::Vector2f manEllipse;
    float eigenAngle;
    int snakeCount;
};
