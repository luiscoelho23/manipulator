#ifndef __COLLISION_DETECTION_ENV__
#define __COLLISION_DETECTION_ENV__

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <fcl/fcl.h>
#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>



/*
// Original MoveIt-based header
#include <rclcpp/rclcpp.hpp>

#include "moveit/collision_detection/collision_common.hpp"

#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include <moveit/robot_model_loader/robot_model_loader.hpp>

#include "moveit/collision_detection_fcl/collision_common.hpp"
#include "moveit/collision_detection_fcl/collision_env_fcl.hpp"

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <geometric_shapes/shape_operations.h>

class CollisionDetectionEnv{
    
    public:

        CollisionDetectionEnv(const std::string robot_description);

        ~CollisionDetectionEnv();

        void  setToPosition(moveit::core::RobotState& panda_state, std::vector<double> joints);

    public: //protected:

        bool robot_model_ok_;

        moveit::core::RobotModelPtr robot_model_;

        collision_detection::CollisionEnvPtr c_env_;

        collision_detection::AllowedCollisionMatrixPtr acm_;

        moveit::core::RobotStatePtr robot_state_;
};
*/

// FCL-based header

/**
 * CollisionDetectionEnv - A collision detection environment for robotic manipulators
 * 
 * This class provides collision detection functionality for:
 * 1. Self-collision checking between robot links
 * 2. Collision checking with external objects (e.g., spheres)
 * 
 * It uses the Flexible Collision Library (FCL) for efficient collision detection.
 */
class CollisionDetectionEnv {
public:
    /**
     * Constructor - Creates a collision detection environment from a URDF robot description
     * 
     * @param robot_description URDF string describing the robot
     */
    CollisionDetectionEnv(const std::string robot_description);
    
    /**
     * Destructor - Cleans up collision objects
     */
    ~CollisionDetectionEnv();

    /**
     * Set robot position based on joint values
     * 
     * @param joints Vector of joint values (7 for a 7-DOF robot)
     */
    void setToPosition(const std::vector<double>& joints);
    
    /**
     * Check for self-collisions between robot links
     * 
     * @return true if the robot is in self-collision, false otherwise
     */
    bool checkCollision();
    
    /**
     * Check for collisions between the robot and a sphere
     * 
     * @param object_pos Position of the sphere [x,y,z]
     * @param radius Radius of the sphere
     * @return true if the robot is in collision with the sphere, false otherwise
     */
    bool checkCollisionWithObject(const std::vector<double>& object_pos, double radius);

private:
    /**
     * Load a mesh file into an FCL collision model
     * 
     * @param filename Path to the mesh file
     * @param scale Scale to apply to the mesh
     * @param model FCL model to populate
     * @return true if loading was successful, false otherwise
     */
    bool loadMeshToFCL(const std::string& filename, const urdf::Vector3& scale, 
                      std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> model);
                      
    /** FCL collision manager for efficient collision queries */
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerd> collision_manager_;
    
    /** Map of collision objects for each robot link */
    std::map<std::string, fcl::CollisionObjectd*> collision_objects_;
    
    /**
     * Allowed Collision Matrix (ACM)
     * 
     * This matrix specifies which pairs of links are allowed to be in collision.
     * - acm_[link1][link2] = true means collisions between link1 and link2 are allowed (ignored)
     * - acm_[link1][link2] = false means collisions between link1 and link2 are not allowed (detected)
     * 
     * This is used to:
     * 1. Ignore expected collisions between adjacent links in the kinematic chain
     * 2. Improve performance by skipping collision checks for allowed pairs
     */
    std::map<std::string, std::map<std::string, bool>> acm_;
};

#endif