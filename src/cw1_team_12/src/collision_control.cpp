#include <collision_control.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>

// Default base frame for collision objects.
static const std::string BASE_FRAME = "panda_link0";

namespace cw1_collision
{

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface &psi,
                          const std::string &object_name,
                          const geometry_msgs::Point &centre,
                          const geometry_msgs::Vector3 &dimensions,
                          const geometry_msgs::Quaternion &orientation)
{
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    collision_object.id = object_name;
    collision_object.header.frame_id = BASE_FRAME;

    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = dimensions.x;
    collision_object.primitives[0].dimensions[1] = dimensions.y;
    collision_object.primitives[0].dimensions[2] = dimensions.z;

    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position = centre;
    collision_object.primitive_poses[0].orientation = orientation;

    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    psi.applyCollisionObjects(collision_objects);
}

void removeCollisionObject(moveit::planning_interface::PlanningSceneInterface &psi,
                             const std::string &object_name)
{
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    collision_object.id = object_name;
    collision_object.header.frame_id = BASE_FRAME;
    collision_object.operation = collision_object.REMOVE;

    collision_objects.push_back(collision_object);
    psi.applyCollisionObjects(collision_objects);
}

void removeAllCollisions(moveit::planning_interface::PlanningSceneInterface &psi)
{
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    collision_object.operation = collision_object.REMOVE;
    collision_objects.push_back(collision_object);
    psi.applyCollisionObjects(collision_objects);
}

void addPlate(moveit::planning_interface::PlanningSceneInterface &psi,
              const std::string &plate_name)
{
    geometry_msgs::Point plate_position;
    plate_position.x = 0;
    plate_position.y = 0;
    plate_position.z = 0;

    geometry_msgs::Vector3 plate_dimension;
    plate_dimension.x = 5;
    plate_dimension.y = 5;
    plate_dimension.z = 0.08;

    geometry_msgs::Quaternion plate_orientation;
    plate_orientation.w = 1;
    plate_orientation.x = 0;
    plate_orientation.y = 0;
    plate_orientation.z = 0;

    addCollisionObject(psi, plate_name, plate_position, plate_dimension, plate_orientation);
}

void addCube(moveit::planning_interface::PlanningSceneInterface &psi,
             const std::vector<geometry_msgs::Point> &positions,
             const std::string &base_name)
{
    geometry_msgs::Vector3 cube_dimension;
    cube_dimension.x = 0.045;
    cube_dimension.y = 0.045;
    cube_dimension.z = 0.05;

    geometry_msgs::Quaternion cube_orientation;
    cube_orientation.w = 1;
    cube_orientation.x = 0;
    cube_orientation.y = 0;
    cube_orientation.z = 0;

    for (size_t i = 0; i < positions.size(); ++i)
    {
        std::string cube_name = base_name + std::to_string(i + 1);
        addCollisionObject(psi, cube_name, positions[i], cube_dimension, cube_orientation);
    }
}

void addBasket(moveit::planning_interface::PlanningSceneInterface &psi,
               const std::vector<geometry_msgs::Point> &positions,
               const std::string &base_name)
{
    geometry_msgs::Vector3 basket_dimension;
    basket_dimension.x = 0.12;
    basket_dimension.y = 0.12;
    basket_dimension.z = 0.16;

    geometry_msgs::Quaternion basket_orientation;
    basket_orientation.w = 1;
    basket_orientation.x = 0;
    basket_orientation.y = 0;
    basket_orientation.z = 0;

    for (size_t i = 0; i < positions.size(); ++i)
    {
        std::string basket_name = base_name + std::to_string(i + 1);
        addCollisionObject(psi, basket_name, positions[i], basket_dimension, basket_orientation);
    }
}

void addCollisions(moveit::planning_interface::PlanningSceneInterface &psi,
                   const std::vector<geometry_msgs::Point> &red_cubes,
                   const std::vector<geometry_msgs::Point> &blue_cubes,
                   const std::vector<geometry_msgs::Point> &purple_cubes,
                   const std::vector<geometry_msgs::Point> &red_baskets,
                   const std::vector<geometry_msgs::Point> &blue_baskets,
                   const std::vector<geometry_msgs::Point> &purple_baskets)
{
    // Add plate
    addPlate(psi, "plate");

    // Add cubes
    addCube(psi, red_cubes, "red_cube");
    addCube(psi, blue_cubes, "blue_cube");
    addCube(psi, purple_cubes, "purple_cube");

    // Add baskets
    addBasket(psi, red_baskets, "red_basket");
    addBasket(psi, blue_baskets, "blue_basket");
    addBasket(psi, purple_baskets, "purple_basket");
}
}
