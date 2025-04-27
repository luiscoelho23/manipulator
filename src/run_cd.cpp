#include "collision_detection_env.hpp"
#include <iostream>
#include <string>
#include <iomanip>

/*
    Number of arg: 11
    
    arg 0: Executable name
    agr 1-7: vector 7dof joints
    agr 8-10: vector Obstacle position in 3D
    agr 11: /robot_description
*/  

int main(int argc, char** argv) {
    if (argc != 12) {
        std::cerr << "Usage: " << argv[0] << " <7 joint values>" << " <3 sphere position values>" << " [robot_description]" << std::endl;
        return 1;
    }

    // Parse joint values
    std::vector<double> vector_7dof;
    for (int i = 1; i < 8; ++i) { 
        try {
            double joint_val = std::stod(argv[i]);
            vector_7dof.push_back(joint_val);
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid argument '" << argv[i] << "'. Must be a valid double." << std::endl;
            return 1;
        }
    }

    // Create collision detection environment
    CollisionDetectionEnv Obj(argv[11]);

    // Set robot position
    Obj.setToPosition(vector_7dof);

    // Check self-collision
    bool self_collision = Obj.checkCollision();
    std::cout << "SELF_COLLISION: " << (self_collision ? "true" : "false") << std::endl;

    // Parse sphere position
    std::vector<double> obstacle_pos = {
        std::stod(argv[8]),
        std::stod(argv[9]),
        std::stod(argv[10])
    };
    
    // Sphere properties
    double sphere_radius = 0.10; // 10cm radius
    
    // Check collision with obstacle
    bool obstacle_collision = Obj.checkCollisionWithObject(obstacle_pos, sphere_radius);
    std::cout << "OBSTACLE_COLLISION: " << (obstacle_collision ? "true" : "false") << std::endl;
    
    return 0;
}