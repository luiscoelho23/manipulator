#include "collision_detection_env.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>

/*
// Original MoveIt-based implementation
CollisionDetectionEnv::CollisionDetectionEnv(const std::string robot_description){

  auto urdf_model = std::make_shared<urdf::Model>();
  if (!urdf_model->initString(robot_description))
      throw std::runtime_error("Failed to parse URDF string");

  // Create MoveIt RobotModel
  moveit::core::RobotModel robot_model(urdf_model,nullptr);
  robot_model_ok_ = static_cast<bool>(robot_model_);

  std::cout << "Robot Model: "<< robot_model_ok_ << std::endl;

  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(robot_model_->getLinkModelNames(), false);

  acm_->setEntry("panda_link0", "panda_link1", true);
  acm_->setEntry("panda_link1", "panda_link2", true);
  acm_->setEntry("panda_link2", "panda_link3", true);
  acm_->setEntry("panda_link3", "panda_link4", true);
  acm_->setEntry("panda_link4", "panda_link5", true);
  acm_->setEntry("panda_link5", "panda_link6", true);
  acm_->setEntry("panda_link6", "panda_link7", true);
  acm_->setEntry("panda_link7", "panda_hand", true);
  acm_->setEntry("panda_hand", "panda_rightfinger", true);
  acm_->setEntry("panda_hand", "panda_leftfinger", true);
  acm_->setEntry("panda_rightfinger", "panda_leftfinger", true);
  acm_->setEntry("panda_link5", "panda_link7", true);
  acm_->setEntry("panda_link6", "panda_hand", true);

  c_env_ = std::make_shared<collision_detection::CollisionEnvFCL>(robot_model_);

  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
}

CollisionDetectionEnv::~CollisionDetectionEnv(){

}

void CollisionDetectionEnv::setToPosition(moveit::core::RobotState& panda_state, std::vector<double> joints)
{
  panda_state.setJointPositions("panda_joint1", &joints[0]);
  panda_state.setJointPositions("panda_joint2", &joints[1]);
  panda_state.setJointPositions("panda_joint3", &joints[2]);
  panda_state.setJointPositions("panda_joint4", &joints[3]);
  panda_state.setJointPositions("panda_joint5", &joints[4]);
  panda_state.setJointPositions("panda_joint6", &joints[5]);
  panda_state.setJointPositions("panda_joint7", &joints[6]);

  panda_state.update();
}
*/

// FCL-based implementation
CollisionDetectionEnv::CollisionDetectionEnv(const std::string robot_description) {
    // Parse URDF
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(robot_description);
    if (!urdf_model)
        throw std::runtime_error("Failed to parse URDF string");

    // Create FCL collision objects for each link
    for (const auto& link : urdf_model->links_) {
        if (link.second->collision) {
            // Create FCL collision geometry directly from URDF
            std::shared_ptr<fcl::CollisionGeometryd> geom = nullptr;
            
            if (link.second->collision->geometry->type == urdf::Geometry::BOX) {
                auto box = std::dynamic_pointer_cast<urdf::Box>(link.second->collision->geometry);
                if (box) {
                    geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
                }
            } 
            else if (link.second->collision->geometry->type == urdf::Geometry::SPHERE) {
                auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link.second->collision->geometry);
                if (sphere) {
                    geom = std::make_shared<fcl::Sphered>(sphere->radius);
                }
            } 
            else if (link.second->collision->geometry->type == urdf::Geometry::CYLINDER) {
                auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(link.second->collision->geometry);
                if (cylinder) {
                    geom = std::make_shared<fcl::Cylinderd>(cylinder->radius, cylinder->length);
                }
            } 
            else if (link.second->collision->geometry->type == urdf::Geometry::MESH) {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link.second->collision->geometry);
                if (mesh) {
                    // Load mesh directly using FCL's built-in mesh loader
                    // Note: This requires the actual file path, scale handling, etc.
                    try {
                        // Load mesh directly using FCL
                        std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
                        
                        // Use FCL's mesh loader or a direct mesh loading library
                        if (!loadMeshToFCL(mesh->filename, mesh->scale, bvh)) {
                            // Fallback to a simple bounding box if mesh loading fails
                            std::cerr << "Failed to load mesh " << mesh->filename 
                                      << " for link " << link.first << ", using bounding box instead" << std::endl;
                            
                            // Create a simple box as a fallback
                            geom = std::make_shared<fcl::Boxd>(0.1, 0.1, 0.1);
                        } else {
                            geom = bvh;
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception when loading mesh: " << e.what() << std::endl;
                        // Create a simple box as a fallback
                        geom = std::make_shared<fcl::Boxd>(0.1, 0.1, 0.1);
                    }
                }
            }

            if (geom) {
                auto obj = new fcl::CollisionObjectd(geom, fcl::Transform3d::Identity());
                collision_objects_[link.first] = obj;
            }
        }
    }

    // Initialize Allowed Collision Matrix (ACM)
    for (const auto& link1 : collision_objects_) {
        for (const auto& link2 : collision_objects_) {
            acm_[link1.first][link2.first] = false;  // By default, no collisions allowed
        }
    }

    // Set allowed collisions based on the original implementation
    acm_["panda_link0"]["panda_link1"] = true;
    acm_["panda_link1"]["panda_link2"] = true;
    acm_["panda_link2"]["panda_link3"] = true;
    acm_["panda_link3"]["panda_link4"] = true;
    acm_["panda_link4"]["panda_link5"] = true;
    acm_["panda_link5"]["panda_link6"] = true;
    acm_["panda_link6"]["panda_link7"] = true;
    acm_["panda_link7"]["panda_hand"] = true;
    acm_["panda_hand"]["panda_rightfinger"] = true;
    acm_["panda_hand"]["panda_leftfinger"] = true;
    acm_["panda_rightfinger"]["panda_leftfinger"] = true;
    acm_["panda_link5"]["panda_link7"] = true;
    acm_["panda_link6"]["panda_hand"] = true;

    // Create FCL collision manager
    collision_manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    for (const auto& obj : collision_objects_) {
        collision_manager_->registerObject(obj.second);
    }
    collision_manager_->setup();
}

// Helper function to load mesh into FCL without using shapes library
bool CollisionDetectionEnv::loadMeshToFCL(const std::string& filename, const urdf::Vector3& scale, 
                                         std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> model) {
    try {
        // Check if filename is valid
        if (filename.empty()) {
            std::cerr << "Empty mesh filename provided" << std::endl;
            return false;
        }
        
        // Handle package:// URIs in ROS
        std::string resolved_filename = filename;
        if (filename.substr(0, 10) == "package://") {
            // In a real implementation, you would resolve the package:// URI
            // using something like ament_index_cpp::get_package_share_directory
            std::cerr << "Warning: package:// URIs not fully supported. Trying direct path." << std::endl;
            resolved_filename = filename.substr(10); // Remove package:// prefix as fallback
        }
        
        // Load the mesh using Assimp
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(
            resolved_filename,
            aiProcess_Triangulate | 
            aiProcess_JoinIdenticalVertices |
            aiProcess_SortByPType
        );
        
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cerr << "Failed to load mesh: " << importer.GetErrorString() << std::endl;
            // Try to load as STL if initial load fails
            scene = importer.ReadFile(
                resolved_filename,
                aiProcess_Triangulate | 
                aiProcess_JoinIdenticalVertices | 
                aiProcess_SortByPType |
                aiProcess_GenNormals
            );
            
            if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
                std::cerr << "Failed to load mesh even as STL: " << importer.GetErrorString() << std::endl;
                return false;
            }
        }
        
        // Process the mesh
        if (scene->mNumMeshes == 0) {
            std::cerr << "No meshes found in file" << std::endl;
            return false;
        }
        
        // Extract vertices and triangles from all meshes in the scene
        std::vector<fcl::Vector3d> vertices;
        std::vector<fcl::Triangle> triangles;
        
        size_t vertex_offset = 0;
        
        for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
            const aiMesh* mesh = scene->mMeshes[m];
            
            if (!mesh->HasFaces()) {
                std::cerr << "Mesh " << m << " has no faces, skipping" << std::endl;
                continue;
            }
            
            // Reserve space for vertices
            vertices.reserve(vertices.size() + mesh->mNumVertices);
            
            // Add vertices
            for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
                vertices.push_back(fcl::Vector3d(
                    mesh->mVertices[i].x * scale.x,
                    mesh->mVertices[i].y * scale.y,
                    mesh->mVertices[i].z * scale.z
                ));
            }
            
            // Add triangles
            for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
                const aiFace& face = mesh->mFaces[i];
                
                if (face.mNumIndices == 3) {
                    triangles.push_back(fcl::Triangle(
                        face.mIndices[0] + vertex_offset,
                        face.mIndices[1] + vertex_offset,
                        face.mIndices[2] + vertex_offset
                    ));
                }
            }
            
            // Update vertex offset for the next mesh
            vertex_offset += mesh->mNumVertices;
        }
        
        if (vertices.empty() || triangles.empty()) {
            std::cerr << "No valid geometry data in mesh file" << std::endl;
            return false;
        }
        
        // Build the BVH model
        model->beginModel();
        model->addSubModel(vertices, triangles);
        model->endModel();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception when loading mesh: " << e.what() << std::endl;
        return false;
    }
}

CollisionDetectionEnv::~CollisionDetectionEnv() {
    for (auto& obj : collision_objects_) {
        delete obj.second;
    }
}

void CollisionDetectionEnv::setToPosition(const std::vector<double>& joints) {
    // Check if we have the expected number of joints
    if (joints.size() != 7) {
        std::cerr << "Error: Expected 7 joint values, but got " << joints.size() << std::endl;
        return;
    }

    // DH parameters for Panda (alpha, a, d, theta)
    std::vector<std::array<double, 4>> dh_params = {
        {0.0, 0.0, 0.333, joints[0]},
        {-M_PI/2, 0.0, 0.0, joints[1]},
        {M_PI/2, 0.0, 0.316, joints[2]},
        {M_PI/2, 0.0825, 0.0, joints[3]},
        {-M_PI/2, -0.0825, 0.384, joints[4]},
        {M_PI/2, 0.0, 0.0, joints[5]},
        {M_PI/2, 0.088, 0.0, joints[6]},
        {0.0, 0.0, 0.107, 0.0}  // End effector
    };
    
    // Calculate transforms for each link
    std::vector<Eigen::Matrix4d> transforms(dh_params.size() + 1);
    transforms[0] = Eigen::Matrix4d::Identity(); // Base transform
    
    for (size_t i = 0; i < dh_params.size(); ++i) {
        double alpha = dh_params[i][0];
        double a = dh_params[i][1];
        double d = dh_params[i][2];
        double theta = dh_params[i][3];
        
        double ct = cos(theta);
        double st = sin(theta);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        Eigen::Matrix4d transform;
        transform << ct, -st*ca, st*sa, a*ct,
                     st, ct*ca, -ct*sa, a*st,
                     0, sa, ca, d,
                     0, 0, 0, 1;
        
        transforms[i+1] = transforms[i] * transform;
    }
    
    // Map link names to transform indices
    std::map<std::string, int> link_indices = {
        {"panda_link0", 0},
        {"panda_link1", 1},
        {"panda_link2", 2},
        {"panda_link3", 3},
        {"panda_link4", 4},
        {"panda_link5", 5},
        {"panda_link6", 6},
        {"panda_link7", 7},
        {"panda_hand", 8},
        {"panda_leftfinger", 8},
        {"panda_rightfinger", 8}
    };
    
    // Apply transforms to collision objects
    for (auto& obj_pair : collision_objects_) {
        const std::string& link_name = obj_pair.first;
        fcl::CollisionObjectd* obj = obj_pair.second;
        
        // Find transform index for this link
        auto it = link_indices.find(link_name);
        if (it != link_indices.end()) {
            int idx = it->second;
            if (idx >= 0 && static_cast<size_t>(idx) < transforms.size()) {
                // Convert Eigen::Matrix4d to fcl::Transform3d
                Eigen::Matrix4d& mat = transforms[idx];
                Eigen::Matrix3d rot = mat.block<3,3>(0,0);
                Eigen::Vector3d trans = mat.block<3,1>(0,3);
                
                fcl::Transform3d transform;
                transform.setIdentity();
                transform.rotate(Eigen::Quaterniond(rot));
                transform.translate(trans);
                
                obj->setTransform(transform);
            } else {
                obj->setTransform(fcl::Transform3d::Identity());
            }
        } else {
            obj->setTransform(fcl::Transform3d::Identity());
        }
    }
    
    // Update collision manager
    collision_manager_->update();
}

bool CollisionDetectionEnv::checkCollision() {
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    
    // Check self-collision by checking all pairs of objects, respecting the ACM
    for (auto it1 = collision_objects_.begin(); it1 != collision_objects_.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != collision_objects_.end(); ++it2) {
            // Skip if collision is allowed between these links
            if (acm_[it1->first][it2->first]) {
                continue;
            }
            
            fcl::collide(it1->second, it2->second, request, result);
            if (result.isCollision()) {
                return true; // Return immediately on first collision
            }
            result.clear();
        }
    }
    
    return false;
}

bool CollisionDetectionEnv::checkCollisionWithObject(const std::vector<double>& object_pos, double radius) {
    if (object_pos.size() != 3) {
        std::cerr << "Error: Expected 3 values for object position, but got " << object_pos.size() << std::endl;
        return false;
    }
    
    // Create sphere for obstacle
    auto sphere = std::make_shared<fcl::Sphered>(radius);
    fcl::CollisionObjectd obstacle(sphere, fcl::Transform3d::Identity());
    obstacle.setTranslation(fcl::Vector3d(object_pos[0], object_pos[1], object_pos[2]));
    
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    
    // Check collision with obstacle
    for (const auto& obj : collision_objects_) {
        fcl::collide(obj.second, &obstacle, request, result);
        if (result.isCollision()) {
            return true; // Return immediately on first collision
        }
        result.clear();
    }
    
    return false;
}