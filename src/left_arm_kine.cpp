#include <hmcik/left_arm_kine.h>

#include <Eigen/Dense>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision.h>
#include <octomap/octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <unistd.h>

using namespace std;
using namespace fcl;

namespace
{
enum LinkIndex { P1P2 = 1, P2P3, P3P4, P3GoalP4 };
constexpr bool VERBOSE = false;
}

struct LeftArmKine::CollisionData
{
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    std::unique_ptr<fcl::CollisionObjectd> tree_obj;
    std::unique_ptr<fcl::CollisionObjectd> box_obj1;
    std::unique_ptr<fcl::CollisionObjectd> box_obj2;
    std::unique_ptr<fcl::CollisionObjectd> box_obj3;
    std::unique_ptr<fcl::CollisionObjectd> box_obj_body;
};

/*==================== 构造与析构 ====================*/

LeftArmKine::LeftArmKine(const std::string &config_filename, bool publish_ros, unsigned int seed)
    : collision_data(std::make_unique<CollisionData>()), rng(seed), dist01(0.0, 1.0)
{
    config.resize(14);
    config.at(0)=0.0;
    config.at(1)=0.0;
    config.at(2)=0.0;
    config.at(3)=0.0;
    config.at(4)=0.0;
    config.at(5)=0.0;
    config.at(6)=0.0;
    pw<<0,car_to_foot,waist_to_ground;
    pf<<0,car_to_foot,waist_to_ground-foot_to_ground;
    po<<0,car_to_foot,chest_to_ground;
    p1<<-chest_to_shoulder,car_to_foot,chest_to_ground;
    p2<<-chest_to_shoulder,car_to_foot,(chest_to_ground - shoulder_to_elbow);
    p3<<-chest_to_shoulder,car_to_foot,(chest_to_ground - shoulder_to_elbow-elbow_to_wrist);
    p4<<-chest_to_shoulder,car_to_foot,(chest_to_ground - shoulder_to_elbow-elbow_to_wrist-wrist_to_gripper);
    p1_start = p1;
    p2_start = p2;
    p3_start = p3;
    if (publish_ros)
    {
        nh = std::make_unique<ros::NodeHandle>();
        marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
        self_pointcloud_pub = nh->advertise<sensor_msgs::PointCloud2>("pointcloud_obs", 10);
    }
    const ros::Time stamp = publish_ros ? ros::Time::now() : ros::Time();
    Tend<<1,0,0,0,
          0,1,0,-wrist_to_gripper,
          0,0,1,0,
          0,0,0,1;

    Tend_1<<1,0,0,0,
    0,1,0,0,
    0,0,1,wrist_to_gripper,
    0,0,0,1;

    fcl::Boxd box1, box2, box3, body;

    //肩到肘部
    box1.side[0] = 0.1; // 修改长度
    box1.side[1] = 0.11; // 修改宽度
    box1.side[2] = shoulder_to_elbow/1000.0; // 修改高度
    //肘到腕部
    box2.side[0] = 0.1;
    box2.side[1] = 0.1;
    box2.side[2] = elbow_to_wrist/1000.0;
    //腕到末端执行器
    box3.side[0] = 0.1;
    box3.side[1] = 0.1;
    box3.side[2] = wrist_to_gripper/1000.0;
    //身体
    body.side[0] = 0.365;
    body.side[1] = 0.23;
    body.side[2] = 1.0;

    collision_data->box_obj1 = std::make_unique<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(box1));
    collision_data->box_obj2 = std::make_unique<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(box2));
    collision_data->box_obj3 = std::make_unique<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(box3));
    collision_data->box_obj_body = std::make_unique<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(body));
    collision_data->box_obj_body->setTranslation(pw/1000.0);
    Eigen::Vector3d pwpo = pw - po;
    Eigen::Quaterniond quat;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), pwpo.normalized());
    collision_data->box_obj_body->setQuatRotation(quat);

    box1_marker.header.frame_id = "base_link";
    box1_marker.header.stamp = stamp;
    box1_marker.ns = "left_p1p2";
    box1_marker.id = 1;
    // box1_marker.type = visualization_msgs::Marker::CYLINDER;
    box1_marker.type = visualization_msgs::Marker::CUBE;
    box1_marker.action = visualization_msgs::Marker::ADD;
    box1_marker.pose.position.x = (p1[0]+p2[0])/2000.0;
    box1_marker.pose.position.y = (p1[1]+p2[1])/2000.0;
    box1_marker.pose.position.z = (p1[2]+p2[2])/2000.0;
    Eigen::Vector3d p1p2 = p2 - p1;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p1p2.normalized());
    box1_marker.pose.orientation.x = quat.x();
    box1_marker.pose.orientation.y = quat.y();
    box1_marker.pose.orientation.z = quat.z();
    box1_marker.pose.orientation.w = quat.w();
    box1_marker.scale.x = box1.side[0];
    box1_marker.scale.y = box1.side[1];
    box1_marker.scale.z = box1.side[2];
    box1_marker.color.a = 0.99; // 透明度
    box1_marker.color.r = 1.0;
    box1_marker.color.g = 1.0;
    box1_marker.color.b = 1.0;

    box2_marker.header.frame_id = "base_link";
    box2_marker.header.stamp = stamp;
    box2_marker.ns = "left_p2p3";
    box2_marker.id = 2;
    // box2_marker.type = visualization_msgs::Marker::CYLINDER;
    box2_marker.type = visualization_msgs::Marker::CUBE;
    box2_marker.action = visualization_msgs::Marker::ADD;
    box2_marker.pose.position.x = (p2[0]+p3[0])/2000.0;
    box2_marker.pose.position.y = (p2[1]+p3[1])/2000.0;
    box2_marker.pose.position.z = (p2[2]+p3[2])/2000.0;
    Eigen::Vector3d p2p3 = p3 - p2;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p2p3.normalized());
    box2_marker.pose.orientation.x = quat.x();
    box2_marker.pose.orientation.y = quat.y();
    box2_marker.pose.orientation.z = quat.z();
    box2_marker.pose.orientation.w = quat.w();
    box2_marker.scale.x = box2.side[0];
    box2_marker.scale.y = box2.side[1];
    box2_marker.scale.z = box2.side[2];
    box2_marker.color.a = box1_marker.color.a;
    box2_marker.color.r = box1_marker.color.r;
    box2_marker.color.g = box1_marker.color.g;
    box2_marker.color.b = box1_marker.color.b;

    box3_marker.header.frame_id = "base_link";
    box3_marker.header.stamp = stamp;
    box3_marker.ns = "left_p3p4";
    box3_marker.id = 3;
    // box3_marker.type = visualization_msgs::Marker::CYLINDER;
    box3_marker.type = visualization_msgs::Marker::CUBE;
    box3_marker.action = visualization_msgs::Marker::ADD;
    box3_marker.pose.position.x = (p3[0]+p4[0])/2000.0;
    box3_marker.pose.position.y = (p3[1]+p4[1])/2000.0;
    box3_marker.pose.position.z = (p3[2]+p4[2])/2000.0;
    Eigen::Vector3d p3p4 = p4 - p3;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p3p4.normalized());
    box3_marker.pose.orientation.x = quat.x();
    box3_marker.pose.orientation.y = quat.y();
    box3_marker.pose.orientation.z = quat.z();
    box3_marker.pose.orientation.w = quat.w();
    box3_marker.scale.x = box3.side[0];
    box3_marker.scale.y = box3.side[1];
    box3_marker.scale.z = box3.side[2];
    box3_marker.color.a = box1_marker.color.a;
    box3_marker.color.r = box1_marker.color.r;
    box3_marker.color.g = box1_marker.color.g;
    box3_marker.color.b = box1_marker.color.b;

    body_marker.header.frame_id = "base_link";
    body_marker.header.stamp = stamp;
    body_marker.ns = "left_body";
    body_marker.id = 4;
    // body_marker.type = visualization_msgs::Marker::CYLINDER;
    body_marker.type = visualization_msgs::Marker::CUBE;
    body_marker.action = visualization_msgs::Marker::ADD;
    body_marker.pose.position.x = (pw[0]+po[0])/2000.0;
    body_marker.pose.position.y = (pw[1]+po[1])/2000.0;
    body_marker.pose.position.z = (pw[2]+po[2]+100.0)/2000.0;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), pwpo.normalized());
    body_marker.pose.orientation.x = quat.x();
    body_marker.pose.orientation.y = quat.y();
    body_marker.pose.orientation.z = quat.z();
    body_marker.pose.orientation.w = quat.w();
    body_marker.scale.x = body.side[0];
    body_marker.scale.y = body.side[1];
    // body_marker.scale.z = body.side[2];
    body_marker.scale.z = 0.5;
    body_marker.color.a = box1_marker.color.a;
    body_marker.color.r = box1_marker.color.r;
    body_marker.color.g = box1_marker.color.g;
    body_marker.color.b = box1_marker.color.b;

    waist_marker.header.frame_id = "base_link";
    waist_marker.header.stamp = stamp;
    waist_marker.ns = "left_waist";
    waist_marker.id = 5;
    // waist_marker.type = visualization_msgs::Marker::CYLINDER;
    waist_marker.type = visualization_msgs::Marker::CUBE;
    waist_marker.action = visualization_msgs::Marker::ADD;
    waist_marker.pose.position.x = (pw[0]+pf[0])/2000.0;
    waist_marker.pose.position.y = (pw[1]+pf[1])/2000.0;
    waist_marker.pose.position.z = (pw[2]+pf[2]-170.0)/2000.0;
    Eigen::Vector3d pwpf=pw-pf;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), pwpf.normalized());
    waist_marker.pose.orientation.x = quat.x();
    waist_marker.pose.orientation.y = quat.y();
    waist_marker.pose.orientation.z = quat.z();
    waist_marker.pose.orientation.w = quat.w();
    waist_marker.scale.x = 0.3;
    waist_marker.scale.y = 0.3;
    waist_marker.scale.z = 0.5;
    waist_marker.color.a = box1_marker.color.a;
    waist_marker.color.r = box1_marker.color.r;
    waist_marker.color.g = box1_marker.color.g;
    waist_marker.color.b = box1_marker.color.b;

    setConfig(config_filename);
    if (!publish_ros)
    {
        show = 0;
    }
    auto tree = std::make_shared<octomap::OcTree>(0.001);

    pcl::PLYReader ply_reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Affine3d transformation1 = Eigen::Affine3d::Identity();
    Eigen::Affine3d transformation2 = Eigen::Affine3d::Identity();

    switch (scene)
    {
        case 1:
            point_cloud_filename = resource_path + "scenes/tree0511SOR.ply";
            transformation2.translation()<<0.2100004,-0.3028386169,1.333280455; // tree0511SOR.ply
            break;
        case 2:
            point_cloud_filename = resource_path + "scenes/04_01.ply";
            transformation2.translation()<<0,0.15,1.333280455; // 04_01.ply
            break;
        case 3:
            point_cloud_filename = resource_path + "scenes/07_01.ply";
            transformation2.translation()<<0,0.15,1.333280455; // 07_01.ply
            break;
        case 4:
            point_cloud_filename = resource_path + "scenes/10_02.ply";
            transformation2.translation()<<0.08,0.15,1.333280455; // 10_02.ply
            break;
        case 5:
            point_cloud_filename = resource_path + "scenes/zhigan.ply";
            transformation2.translation()<<0.0,0.05,1.333280455; // zhigan.ply
            break;
        case 6:
            point_cloud_filename = resource_path + "scenes/full_tree.ply";
            transformation2.translation()<<0.15,1.35,0.0; // full_tree.ply
            break;
        default:
            break;
    }
    if (!point_cloud_filename.empty() && ply_reader.read(point_cloud_filename, *cloud) < 0)
    {
        throw std::runtime_error("Cannot read PLY scene: " + point_cloud_filename);
    }
    transformation1.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()));

    if(scene==6)
    {
        pcl::transformPointCloud(*cloud, *transformed_cloud, transformation2);
    }
    else
    {
        pcl::transformPointCloud(*cloud, *transformed_cloud, transformation1);
        pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transformation2);
    }

    if (pc_use)
    {
        for (const auto &point : transformed_cloud->points)
        {
            tree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }
    }
    collision_data->tree_obj = std::make_unique<fcl::CollisionObjectd>(std::make_shared<fcl::OcTreed>(tree));

    if (publish_ros)
    {
        plan_req = nh->advertise<sensor_msgs::JointState>("left_arm_planner", 1);
        if (pc_use)
        {
            pcl::toROSMsg(*transformed_cloud, self_pointcloud);
            self_pointcloud.header.frame_id = "base_link";
            self_pointcloud.header.stamp = stamp;
            self_pointcloud_pub.publish(self_pointcloud);
        }
        thread_marker_CB();
    }
}

LeftArmKine::~LeftArmKine() = default;


/*==================== 配置与位姿转换 ====================*/

Eigen::Matrix4d LeftArmKine::Pose2Matrix(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4d pose_matrix;
    Eigen::Quaterniond pose_quat;
    pose_quat.x()=pose.orientation.x;
    pose_quat.y()=pose.orientation.y;
    pose_quat.z()=pose.orientation.z;
    pose_quat.w()=pose.orientation.w;
    Eigen::Matrix3d pose_rotm=pose_quat.normalized().toRotationMatrix();
    pose_matrix.setIdentity();
    pose_matrix.block<3,3>(0, 0) = pose_rotm;
    Eigen::Vector3d pose_position(pose.position.x, pose.position.y, pose.position.z);
    pose_matrix.block<3,1>(0, 3) = pose_position;
    return pose_matrix;
}

geometry_msgs::Pose LeftArmKine::Matrix2Pose(const Eigen::Matrix4d &result_matrix)
{
    geometry_msgs::Pose res_pose;
    Eigen::Vector3d res_position=result_matrix.block<3,1>(0, 3);
    res_pose.position.x=res_position[0];
    res_pose.position.y=res_position[1];
    res_pose.position.z=res_position[2];
    Eigen::Matrix3d res_rotm=result_matrix.block<3,3>(0, 0);
    Eigen::Quaterniond res_pose_quat(res_rotm);
    res_pose.orientation.x=res_pose_quat.x();
    res_pose.orientation.y=res_pose_quat.y();
    res_pose.orientation.z=res_pose_quat.z();
    res_pose.orientation.w=res_pose_quat.w();
    return res_pose;
}

void LeftArmKine::setConfig(const std::string &config_filename)
{
    package_path = ros::package::getPath("hmcik");
    if (package_path.empty())
    {
        throw std::runtime_error("ROS package hmcik is not discoverable. Source the catkin workspace setup.");
    }
    resource_path = package_path + "/resource/";
    const std::string config_path = resource_path + config_filename;
    std::ifstream file(config_path);
    if (!file)
    {
        throw std::runtime_error("Cannot open IK config: " + config_path);
    }

    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(file, line))
    {
        const auto separator = line.find('=');
        if (separator != std::string::npos)
        {
            values[line.substr(0, separator)] = line.substr(separator + 1);
        }
    }

    try
    {
        segment = std::stoi(values.at("NGARR"));
        epsilon = std::stod(values.at("random"));
        little_count = std::stoi(values.at("Nposition"));
        rand_range = std::stod(values.at("rand_range"));
        max_count = std::stoi(values.at("max_count"));
        eps = std::stod(values.at("eps"));
        pc_use = std::stoi(values.at("pc_use"));
        show = std::stoi(values.at("show"));
        GARR = std::stoi(values.at("GARR")) != 0;
        scene = std::stoi(values.at("scene"));
    }
    catch (const std::exception &error)
    {
        throw std::runtime_error("Invalid IK config " + config_path + ": " + error.what());
    }
}

void LeftArmKine::getConfig(std::vector<double> &config)
{
    config=this->config;
}


/*==================== 正运动学FK ====================*/

Eigen::Matrix4d LeftArmKine::calculate_T9G(const double& theta8, const double& theta9)
{
    const double s8 = sin(theta8); // 三角缓存: 原表达式只算一次
    const double c8 = cos(theta8); // 三角缓存: 原表达式只算一次
    const double s9m = sin(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double c9m = cos(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    Eigen::Matrix4d T9;

    T9(0, 0) = s9m;
    T9(0, 1) = c9m;
    T9(0, 2) = 0;
    T9(0, 3) = -d1;
    T9(1, 0) = c9m * s8;
    T9(1, 1) = -s8 * s9m;
    T9(1, 2) = c8;
    T9(1, 3) = a3;
    T9(2, 0) = c8 * c9m;
    T9(2, 1) = -c8 * s9m;
    T9(2, 2) = -s8;
    T9(2, 3) = a6 + a7;

    T9(3, 0) = 0;
    T9(3, 1) = 0;
    T9(3, 2) = 0;
    T9(3, 3) = 1;

    return T9;
}

Eigen::Matrix4d LeftArmKine::calculate_T11G(const double& theta8, const double& theta9,
                                       const double& theta10, const double& theta11)
{
    const double s8 = sin(theta8); // 三角缓存: 原表达式只算一次
    const double c8 = cos(theta8); // 三角缓存: 原表达式只算一次
    const double s9m = sin(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double c9m = cos(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double s10p = sin(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double c10p = cos(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double s11 = sin(theta11); // 三角缓存: 原表达式只算一次
    const double c11 = cos(theta11); // 三角缓存: 原表达式只算一次
    Eigen::Matrix4d T11;

    T11(0, 0) = c9m * s11
                + c11 * c10p * s9m;
    T11(0, 1) = c11 * c9m
                - c10p * s11 * s9m;
    T11(0, 2) = s9m * s10p;
    T11(0, 3) = -d1 - d2 * c9m;
    T11(1, 0) = -c11 * (c8 * s10p
                                 - c9m * c10p * s8)
                - s8 * s11 * s9m;
    T11(1, 1) = s11 * (c8 * s10p
                                - c9m * c10p * s8)
                - c11 * s8 * s9m;
    T11(1, 2) = c8 * c10p
                + c9m * s8 * s10p;
    T11(1, 3) = a3 + d2 * s8 * s9m;
    T11(2, 0) = c11 * (s8 * s10p
                                + c8 * c9m * c10p)
                - c8 * s11 * s9m;
    T11(2, 1) = -s11 * (s8 * s10p
                                 + c8 * c9m * c10p)
                - c8 * c11 * s9m;
    T11(2, 2) = c8 * c9m * s10p
                - c10p * s8;
    T11(2, 3) = a6 + a7 + d2 * c8 * s9m;

    T11(3, 0) = 0;
    T11(3, 1) = 0;
    T11(3, 2) = 0;
    T11(3, 3) = 1;

    return T11;
}

Eigen::Matrix4d LeftArmKine::calculate_T13G(const double& theta8, const double& theta9,
                                       const double& theta10, const double& theta11,
                                       const double& theta12, const double& theta13)
{
    const double s8 = sin(theta8); // 三角缓存: 原表达式只算一次
    const double c8 = cos(theta8); // 三角缓存: 原表达式只算一次
    const double s9m = sin(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double c9m = cos(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double s10p = sin(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double c10p = cos(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double s11 = sin(theta11); // 三角缓存: 原表达式只算一次
    const double c11 = cos(theta11); // 三角缓存: 原表达式只算一次
    const double s12 = sin(theta12); // 三角缓存: 原表达式只算一次
    const double c12 = cos(theta12); // 三角缓存: 原表达式只算一次
    const double s13 = sin(theta13); // 三角缓存: 原表达式只算一次
    const double c13 = cos(theta13); // 三角缓存: 原表达式只算一次
    Eigen::Matrix4d T13;

    T13(0, 0) = c13 * (c12 * (c9m * s11
                                                + c11 * c10p * s9m)
                                 + s12 * s9m * s10p)
                + s13 * (c11 * c9m
                                  - c10p * s11 * s9m);
    T13(0, 1) = c13 * (c11 * c9m
                                - c10p * s11 * s9m)
                - s13 * (c12 * (c9m * s11
                                                  + c11 * c10p * s9m)
                                  + s12 * s9m * s10p);
    T13(0, 2) = c12 * s9m * s10p
                - s12 * (c9m * s11
                                  + c11 * c10p * s9m);
    T13(0, 3) = -d1
                - d3 * (c11 * c9m
                        - c10p * s11 * s9m)
                - d2 * c9m;
    T13(1, 0) = s13 * (s11 * (c8 * s10p
                                                - c9m * c10p * s8)
                                 - c11 * s8 * s9m)
                - c13 * (c12 * (c11 * (c8 * s10p
                                                                  - c9m * c10p * s8)
                                                   + s8 * s11 * s9m)
                                  - s12 * (c8 * c10p
                                                    + c9m * s8 * s10p));
    T13(1, 1) = s13 * (c12 * (c11 * (c8 * s10p
                                                                - c9m * c10p * s8)
                                                     + s8 * s11 * s9m)
                                 - s12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                + c13 * (s11 * (c8 * s10p
                                                  - c9m * c10p * s8)
                                   - c11 * s8 * s9m);
    T13(1, 2) = s12 * (c11 * (c8 * s10p
                                                - c9m * c10p * s8)
                                 + s8 * s11 * s9m)
                + c12 * (c8 * c10p
                                  + c9m * s8 * s10p);
    T13(1, 3) = a3
                - d3 * (s11 * (c8 * s10p
                                        - c9m * c10p * s8)
                        - c11 * s8 * s9m)
                + d2 * s8 * s9m;
    T13(2, 0) = c13 * (c12 * (c11 * (s8 * s10p
                                                                + c8 * c9m * c10p)
                                                     - c8 * s11 * s9m)
                                 - s12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                - s13 * (s11 * (s8 * s10p
                                                  + c8 * c9m * c10p)
                                   + c8 * c11 * s9m);
    T13(2, 1) = -s13 * (c12 * (c11 * (s8 * s10p
                                                                 + c8 * c9m * c10p)
                                                      - c8 * s11 * s9m)
                                  - s12 * (c10p * s8
                                                    - c8 * c9m * s10p))
                - c13 * (s11 * (s8 * s10p
                                                  + c8 * c9m * c10p)
                                   + c8 * c11 * s9m);
    T13(2, 2) = -s12 * (c11 * (s8 * s10p
                                                 + c8 * c9m * c10p)
                                  - c8 * s11 * s9m)
                - c12 * (c10p * s8
                                  - c8 * c9m * s10p);
    T13(2, 3) = a6 + a7
                + d3 * (s11 * (s8 * s10p
                                        + c8 * c9m * c10p)
                        + c8 * c11 * s9m)
                + d2 * c8 * s9m;

    T13(3, 0) = 0;
    T13(3, 1) = 0;
    T13(3, 2) = 0;
    T13(3, 3) = 1;

    return T13;
}

Eigen::Matrix4d LeftArmKine::calculate_T14G(const double& theta8, const double& theta9,
                                       const double& theta10, const double& theta11,
                                       const double& theta12, const double& theta13,
                                       const double& theta14)
{
    const double s8 = sin(theta8); // 三角缓存: 原表达式只算一次
    const double c8 = cos(theta8); // 三角缓存: 原表达式只算一次
    const double s9m = sin(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double c9m = cos(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double s10p = sin(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double c10p = cos(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double s11 = sin(theta11); // 三角缓存: 原表达式只算一次
    const double c11 = cos(theta11); // 三角缓存: 原表达式只算一次
    const double s12 = sin(theta12); // 三角缓存: 原表达式只算一次
    const double c12 = cos(theta12); // 三角缓存: 原表达式只算一次
    const double s13 = sin(theta13); // 三角缓存: 原表达式只算一次
    const double c13 = cos(theta13); // 三角缓存: 原表达式只算一次
    const double s14 = sin(theta14); // 三角缓存: 原表达式只算一次
    const double c14 = cos(theta14); // 三角缓存: 原表达式只算一次
    Eigen::Matrix4d T14;

    T14(0, 0) = c14 * (c13 * (c12 * (c9m * s11
                                                                + c11 * c10p * s9m)
                                                     + s12 * s9m * s10p)
                                 + s13 * (c11 * c9m
                                                   - c10p * s11 * s9m))
                - s14 * (s12 * (c9m * s11
                                                  + c11 * c10p * s9m)
                                  - c12 * s9m * s10p);
    T14(0, 1) = -s14 * (c13 * (c12 * (c9m * s11
                                                                 + c11 * c10p * s9m)
                                      + s12 * s9m * s10p)
                                 + s13 * (c11 * c9m
                                                   - c10p * s11 * s9m))
                - c14 * (s12 * (c9m * s11
                                                  + c11 * c10p * s9m)
                                  - c12 * s9m * s10p);
    T14(0, 2) = s13 * (c12 * (c9m * s11
                                                + c11 * c10p * s9m)
                                 + s12 * s9m * s10p)
                - c13 * (c11 * c9m
                                  - c10p * s11 * s9m);
    T14(0, 3) = -d1
                - d3 * (c11 * c9m
                        - c10p * s11 * s9m)
                - d2 * c9m;
    T14(1, 0) = s14 * (s12 * (c11 * (c8 * s10p
                                                                - c9m * c10p * s8)
                                     + s8 * s11 * s9m)
                                 + c12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                - c14 * (c13 * (c12 * (c11 * (c8 * s10p
                                                                                  - c9m * c10p * s8)
                                                       + s8 * s11 * s9m)
                                    - s12 * (c8 * c10p
                                                      + c9m * s8 * s10p))
                                 - s13 * (s11 * (c8 * s10p
                                                                   - c9m * c10p * s8)
                                                   - c11 * s8 * s9m));
    T14(1, 1) = c14 * (s12 * (c11 * (c8 * s10p
                                                                - c9m * c10p * s8)
                                     + s8 * s11 * s9m)
                                 + c12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                + s14 * (c13 * (c12 * (c11 * (c8 * s10p
                                                                                  - c9m * c10p * s8)
                                                       + s8 * s11 * s9m)
                                    - s12 * (c8 * c10p
                                                      + c9m * s8 * s10p))
                                 - s13 * (s11 * (c8 * s10p
                                                                   - c9m * c10p * s8)
                                                   - c11 * s8 * s9m));
    T14(1, 2) = -s13 * (c12 * (c11 * (c8 * s10p
                                                                 - c9m * c10p * s8)
                                      + s8 * s11 * s9m)
                                 - s12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                - c13 * (s11 * (c8 * s10p
                                                  - c9m * c10p * s8)
                                   - c11 * s8 * s9m);
    T14(1, 3) = a3
                - d3 * (s11 * (c8 * s10p
                                        - c9m * c10p * s8)
                        - c11 * s8 * s9m)
                + d2 * s8 * s9m;
    T14(2, 0) = c14 * (c13 * (c12 * (c11 * (s8 * s10p
                                                                                + c8 * c9m * c10p)
                                                     - c8 * s11 * s9m)
                                 - s12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                                 - s13 * (s11 * (s8 * s10p
                                                                   + c8 * c9m * c10p)
                                                   + c8 * c11 * s9m))
                - s14 * (s12 * (c11 * (s8 * s10p
                                                                  + c8 * c9m * c10p)
                                      - c8 * s11 * s9m)
                                 + c12 * (c10p * s8
                                                   - c8 * c9m * s10p));
    T14(2, 1) = -c14 * (s12 * (c11 * (s8 * s10p
                                                                 + c8 * c9m * c10p)
                                      - c8 * s11 * s9m)
                                 + c12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                - s14 * (c13 * (c12 * (c11 * (s8 * s10p
                                                                                 + c8 * c9m * c10p)
                                                      - c8 * s11 * s9m)
                                   - s12 * (c10p * s8
                                                     - c8 * c9m * s10p))
                                   - s13 * (s11 * (s8 * s10p
                                                                    + c8 * c9m * c10p)
                                                     + c8 * c11 * s9m));
    T14(2, 2) = s13 * (c12 * (c11 * (s8 * s10p
                                                                + c8 * c9m * c10p)
                                     - c8 * s11 * s9m)
                                 - s12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                + c13 * (s11 * (s8 * s10p
                                                  + c8 * c9m * c10p)
                                   + c8 * c11 * s9m);
    T14(2, 3) = a6 + a7
                + d3 * (s11 * (s8 * s10p
                                        + c8 * c9m * c10p)
                        + c8 * c11 * s9m)
                + d2 * c8 * s9m;

    T14(3, 0) = 0;
    T14(3, 1) = 0;
    T14(3, 2) = 0;
    T14(3, 3) = 1;

    return T14;
}

Eigen::Matrix4d LeftArmKine::calculate_T15G(const double& theta8, const double& theta9,
                                       const double& theta10, const double& theta11,
                                       const double& theta12, const double& theta13,
                                       const double& theta14)
{
    const double s8 = sin(theta8); // 三角缓存: 原表达式只算一次
    const double c8 = cos(theta8); // 三角缓存: 原表达式只算一次
    const double s9m = sin(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double c9m = cos(theta9 - M_PI_2); // 三角缓存: 原表达式只算一次
    const double s10p = sin(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double c10p = cos(theta10 + M_PI_2); // 三角缓存: 原表达式只算一次
    const double s11 = sin(theta11); // 三角缓存: 原表达式只算一次
    const double c11 = cos(theta11); // 三角缓存: 原表达式只算一次
    const double s12 = sin(theta12); // 三角缓存: 原表达式只算一次
    const double c12 = cos(theta12); // 三角缓存: 原表达式只算一次
    const double s13 = sin(theta13); // 三角缓存: 原表达式只算一次
    const double c13 = cos(theta13); // 三角缓存: 原表达式只算一次
    const double s14 = sin(theta14); // 三角缓存: 原表达式只算一次
    const double c14 = cos(theta14); // 三角缓存: 原表达式只算一次
    Eigen::Matrix4d T15;

    T15(0, 0) = c14 * (c13 * (c12 * (c9m * s11
                                                                + c11 * c10p * s9m)
                                                     + s12 * s9m * s10p)
                                 + s13 * (c11 * c9m
                                                   - c10p * s11 * s9m))
                - s14 * (s12 * (c9m * s11
                                                  + c11 * c10p * s9m)
                                  - c12 * s9m * s10p);
    T15(0, 1) = -s14 * (c13 * (c12 * (c9m * s11
                                                                 + c11 * c10p * s9m)
                                      + s12 * s9m * s10p)
                                 + s13 * (c11 * c9m
                                                   - c10p * s11 * s9m))
                - c14 * (s12 * (c9m * s11
                                                  + c11 * c10p * s9m)
                                  - c12 * s9m * s10p);
    T15(0, 2) = s13 * (c12 * (c9m * s11
                                                + c11 * c10p * s9m)
                                 + s12 * s9m * s10p)
                - c13 * (c11 * c9m
                                  - c10p * s11 * s9m);
    T15(0, 3) = d4 * (s13 * (c12 * (c9m * s11
                                                      + c11 * c10p * s9m)
                                       + s12 * s9m * s10p)
                      - c13 * (c11 * c9m
                                        - c10p * s11 * s9m))
                - d3 * (c11 * c9m
                        - c10p * s11 * s9m)
                - d2 * c9m
                - d1;
    T15(1, 0) = s14 * (s12 * (c11 * (c8 * s10p
                                                                - c9m * c10p * s8)
                                     + s8 * s11 * s9m)
                                 + c12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                - c14 * (c13 * (c12 * (c11 * (c8 * s10p
                                                                                  - c9m * c10p * s8)
                                                       + s8 * s11 * s9m)
                                    - s12 * (c8 * c10p
                                                      + c9m * s8 * s10p))
                                 - s13 * (s11 * (c8 * s10p
                                                                   - c9m * c10p * s8)
                                                   - c11 * s8 * s9m));
    T15(1, 1) = c14 * (s12 * (c11 * (c8 * s10p
                                                                - c9m * c10p * s8)
                                     + s8 * s11 * s9m)
                                 + c12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                + s14 * (c13 * (c12 * (c11 * (c8 * s10p
                                                                                  - c9m * c10p * s8)
                                                       + s8 * s11 * s9m)
                                    - s12 * (c8 * c10p
                                                      + c9m * s8 * s10p))
                                 - s13 * (s11 * (c8 * s10p
                                                                   - c9m * c10p * s8)
                                                   - c11 * s8 * s9m));
    T15(1, 2) = -s13 * (c12 * (c11 * (c8 * s10p
                                                                 - c9m * c10p * s8)
                                      + s8 * s11 * s9m)
                                 - s12 * (c8 * c10p
                                                   + c9m * s8 * s10p))
                - c13 * (s11 * (c8 * s10p
                                                  - c9m * c10p * s8)
                                   - c11 * s8 * s9m);
    T15(1, 3) = a3
                - d3 * (s11 * (c8 * s10p
                                        - c9m * c10p * s8)
                        - c11 * s8 * s9m)
                - d4 * (s13 * (c12 * (c11 * (c8 * s10p
                                                                        - c9m * c10p * s8)
                                             + s8 * s11 * s9m)
                         - s12 * (c8 * c10p
                                           + c9m * s8 * s10p))
                        + c13 * (s11 * (c8 * s10p
                                                          - c9m * c10p * s8)
                                          - c11 * s8 * s9m))
                + d2 * s8 * s9m;
    T15(2, 0) = c14 * (c13 * (c12 * (c11 * (s8 * s10p
                                                                                + c8 * c9m * c10p)
                                                     - c8 * s11 * s9m)
                                 - s12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                                 - s13 * (s11 * (s8 * s10p
                                                                   + c8 * c9m * c10p)
                                                   + c8 * c11 * s9m))
                - s14 * (s12 * (c11 * (s8 * s10p
                                                                  + c8 * c9m * c10p)
                                      - c8 * s11 * s9m)
                                 + c12 * (c10p * s8
                                                   - c8 * c9m * s10p));
    T15(2, 1) = -c14 * (s12 * (c11 * (s8 * s10p
                                                                 + c8 * c9m * c10p)
                                      - c8 * s11 * s9m)
                                 + c12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                - s14 * (c13 * (c12 * (c11 * (s8 * s10p
                                                                                 + c8 * c9m * c10p)
                                                      - c8 * s11 * s9m)
                                   - s12 * (c10p * s8
                                                     - c8 * c9m * s10p))
                                   - s13 * (s11 * (s8 * s10p
                                                                    + c8 * c9m * c10p)
                                                     + c8 * c11 * s9m));
    T15(2, 2) = s13 * (c12 * (c11 * (s8 * s10p
                                                                + c8 * c9m * c10p)
                                     - c8 * s11 * s9m)
                                 - s12 * (c10p * s8
                                                   - c8 * c9m * s10p))
                + c13 * (s11 * (s8 * s10p
                                                  + c8 * c9m * c10p)
                                   + c8 * c11 * s9m);
    T15(2, 3) = a6 + a7
                + d3 * (s11 * (s8 * s10p
                                        + c8 * c9m * c10p)
                        + c8 * c11 * s9m)
                + d4 * (s13 * (c12 * (c11 * (s8 * s10p
                                                                        + c8 * c9m * c10p)
                                             - c8 * s11 * s9m)
                         - s12 * (c10p * s8
                                           - c8 * c9m * s10p))
                        + c13 * (s11 * (s8 * s10p
                                                         + c8 * c9m * c10p)
                                          + c8 * c11 * s9m))
                + d2 * c8 * s9m;

    T15(3, 0) = 0;
    T15(3, 1) = 0;
    T15(3, 2) = 0;
    T15(3, 3) = 1;

    return T15;
}


/*==================== 基础数学工具 ====================*/

Eigen::Matrix3d LeftArmKine::rotate_x(double rad)
{
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0, 0, cos(rad), -sin(rad), 0, sin(rad), cos(rad);
    return Rx;
}

Eigen::Matrix3d LeftArmKine::rotate_y(double rad)
{
    Eigen::Matrix3d Ry;
    Ry << cos(rad), 0, sin(rad), 0, 1, 0, -sin(rad), 0, cos(rad);
    return Ry;
}

Eigen::Matrix3d LeftArmKine::rotate_z(double rad)
{
    Eigen::Matrix3d Rz;
    Rz << cos(rad), -sin(rad), 0, sin(rad), cos(rad), 0, 0, 0, 1;
    return Rz;
}

double LeftArmKine::distanceCost3(const Eigen::Vector3d &p1,Eigen::Vector3d &p2)
{
    Eigen::Vector3d diff = p1 - p2;
    return diff.norm();
}

double LeftArmKine::angle(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    double c = a.normalized().dot(b.normalized());
    c = std::max(-1.0, std::min(1.0, c));
    return acos(c);
}

Eigen::Vector3d LeftArmKine::angle_diff(Eigen::Matrix3d &R1, Eigen::Matrix3d &R2)
{
    double x_err = angle(R1.col(0), R2.col(0));
    double y_err = angle(R1.col(1), R2.col(1));
    double z_err = angle(R1.col(2), R2.col(2));

    #ifdef DEBUG
    std::cout << "X axis error = " << x_err << " rad\n";
    std::cout << "Y axis error = " << y_err << " rad\n";
    std::cout << "Z axis error = " << z_err << " rad\n";
    #endif

    Eigen::Vector3d angle_errors(x_err, y_err, z_err);
    return angle_errors;
}

double LeftArmKine::calculate_orientation_error(const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2)
{
    Eigen::Matrix3d R1 = T1.block<3,3>(0,0);
    Eigen::Matrix3d R2 = T2.block<3,3>(0,0);
    Eigen::Matrix3d delta_R = R1 * R2.transpose();
    double trace = delta_R.trace();
    double cos_theta = (trace - 1.0) / 2.0;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double angle_rad = std::acos(cos_theta);
    double angle_deg = angle_rad * 180.0 / M_PI;
    return angle_deg;
}


/*==================== 解析解算 pivot_hinge+verify ====================*/

Eigen::Vector2d LeftArmKine::pivot_hinge_solve(const double& pstart_x, const double& pstart_y, const double& pstart_z,
                                        const double& pmid_x, const double& pmid_y, const double& pmid_z,
                                        const double& pend_x, const double& pend_y, const double& pend_z,
                                        const double& pend_pre_x, const double& pend_pre_y, const double& pend_pre_z)
{
    double rad_pivot = 0.0;
    double rad_hinge = 0.0;
    Eigen::Vector3d pmidpend_pre(pend_pre_x - pmid_x, pend_pre_y - pmid_y, pend_pre_z - pmid_z);
    Eigen::Vector3d pstartpmid(pmid_x - pstart_x, pmid_y - pstart_y, pmid_z - pstart_z);
    Eigen::Vector3d pmidpend(pend_x - pmid_x, pend_y - pmid_y, pend_z - pmid_z);
    Eigen::Vector3d n = pstartpmid;
    Eigen::Vector3d n1 = pmidpend.cross(n);
    Eigen::Vector3d pmidpend_touying = n.cross(n1);
    // if(pmidpend_pre.norm() * pmidpend_touying.norm()< 1e-6||pmidpend.norm() * pmidpend_touying.norm()==0)
    const double EPS = 1e-6;
    if (pmidpend_pre.norm() < EPS || pmidpend.norm() < EPS || pmidpend_touying.norm() < EPS)
    {
        ROS_ERROR("pivot_hinge_solve: Norm of vector is too small, cannot compute angles.");
        Eigen::Vector2d res;
        // res << 0.0, 0.0;
        res << 0.0, M_PI_2;
        return res;
    }
    double pivot_cos = static_cast<double>(fabs(pmidpend_pre.dot(pmidpend_touying)) / (pmidpend_pre.norm() * pmidpend_touying.norm()));
    pivot_cos = std::min(1.0, std::max(0.0, pivot_cos));
    if(pivot_cos==0.0)
    {
        rad_pivot = M_PI_2;
    }
    else
    {
        double pivot_sin = 1-pivot_cos*pivot_cos;
        rad_pivot = atan2(sqrt(pivot_sin), pivot_cos);
    }
    double hinge_cos=static_cast<double>(fabs(pmidpend.dot(pmidpend_touying)) / (pmidpend.norm() * pmidpend_touying.norm()));
    hinge_cos = std::min(1.0, std::max(0.0, hinge_cos));
    if(hinge_cos==0.0)
    {
        rad_hinge = M_PI_2;
    }
    else
    {
        double hinge_sin=1-hinge_cos*hinge_cos;
        rad_hinge=atan2(sqrt(hinge_sin), hinge_cos);
    }
    Eigen::Vector2d res;
    res << rad_pivot, rad_hinge;
    return res;
}

Eigen::Vector2d LeftArmKine::calculate_qianhoutai_cetai_verify(
    Eigen::Vector3d p3, double theta_wanyao, double theta_ceyao,
    double theta_jiaobu, double theta_qianhoutai, double theta_cetai)
{
    // ROS_INFO("theta_qianhoutai = %.4f, theta_cetai = %.4f", theta_qianhoutai, theta_cetai);
    Eigen::Vector4d p29(0, -shoulder_to_elbow, 0, 1); // p2相对于9坐标系的坐标
    Eigen::Matrix4d T9G;
    Eigen::Vector4d p3G;
    Eigen::Vector3d p3G_3d;
    double distance_p3_to_p3G_3d;
    double set_qianhoutai_cetai[16]={theta_qianhoutai, theta_cetai,
                                     -theta_qianhoutai, theta_cetai,
                                     theta_qianhoutai, -theta_cetai,
                                     -theta_qianhoutai, -theta_cetai,
                                     M_PI - fabs(theta_qianhoutai), theta_cetai,
                                     -(M_PI - fabs(theta_qianhoutai)), theta_cetai,
                                     M_PI - fabs(theta_qianhoutai), -theta_cetai,
                                     -(M_PI - fabs(theta_qianhoutai)), -theta_cetai,
                                    };
    for(int i=0;i<16;i+=2)
    {
        theta_qianhoutai=set_qianhoutai_cetai[i];
        theta_cetai=set_qianhoutai_cetai[i+1];
        T9G = calculate_T9G(theta_qianhoutai, theta_cetai);
        p3G = T9G * p29;
        p3G_3d = p3G.head<3>();
        distance_p3_to_p3G_3d = (p3 - p3G_3d).norm();
        if (distance_p3_to_p3G_3d <= this->eps)
        {
            // ROS_INFO("qianhoutai_cetai distance_p3_to_p3G_3d: %f", distance_p3_to_p3G_3d);
            break;
        }
    }
    return Eigen::Vector2d(theta_qianhoutai, theta_cetai);
}

Eigen::Vector2d LeftArmKine::calculate_dabi_zhoubu_verify(
    Eigen::Vector3d p_wrist, double theta_wanyao, double theta_ceyao,
    double theta_jiaobu, double theta_qianhoutai, double theta_cetai,
    double theta_dabi, double theta_zhoubu)
{
    // ROS_INFO("theta_dabi = %.4f, theta_zhoubu = %.4f", theta_dabi, theta_zhoubu);
    Eigen::Vector3d p3G_3d;
    Eigen::Vector4d p311(0, -elbow_to_wrist, 0, 1); // p3相对于11坐标系的坐标
    Eigen::Matrix4d T11G;
    Eigen::Vector4d p3G;
    double distance_p_wrist_to_p4G_3d;
    double set_dabi_zhoubu[16]={theta_dabi, theta_zhoubu,
                                -theta_dabi, theta_zhoubu,
                                theta_dabi, -theta_zhoubu,
                                -theta_dabi, -theta_zhoubu,
                                M_PI - fabs(theta_dabi), theta_zhoubu,
                                -(M_PI - fabs(theta_dabi)), theta_zhoubu,
                                M_PI - fabs(theta_dabi), -theta_zhoubu,
                                -(M_PI - fabs(theta_dabi)), -theta_zhoubu,
                                };
    for(int i=0;i<16;i+=2)
    {
        theta_dabi=set_dabi_zhoubu[i];
        theta_zhoubu=set_dabi_zhoubu[i+1];
        T11G = calculate_T11G(theta_qianhoutai, theta_cetai, theta_dabi, theta_zhoubu);
        p3G = T11G * p311;
        p3G_3d = p3G.head<3>();
        distance_p_wrist_to_p4G_3d = (p3G_3d - p_wrist).norm();
        if (distance_p_wrist_to_p4G_3d <= this->eps)
        {
            // ROS_INFO("set_dabi_zhoubu distance_p_wrist_to_p4G_3d: %f", distance_p_wrist_to_p4G_3d);
            break;
        }
    }
    // if(theta_zhoubu>=0.523)return Eigen::Vector2d(theta_dabi, 0.523);
    return Eigen::Vector2d(theta_dabi, theta_zhoubu);
}

Eigen::Vector3d LeftArmKine::calculate_xiaobi_shouwan_verify(Eigen::Vector3d p4,
                                                 double theta_qianhoutai,
                                                 double theta_cetai,
                                                 double theta_dabi,
                                                 double theta_zhoubu,
                                                 double theta_xiaobi,
                                                 double theta_shouwan)
{
    // ROS_INFO("theta_xiaobi = %.4f, theta_shouwan = %.4f", theta_xiaobi, theta_shouwan);
    Eigen::Matrix4d T13G;
    Eigen::Vector3d T13G_p;
    double distance;
    double set_xiaobi_theta_shouwan[16]={theta_xiaobi, theta_shouwan,
                                         -theta_xiaobi, theta_shouwan,
                                         theta_xiaobi, -theta_shouwan,
                                         -theta_xiaobi, -theta_shouwan,
                                         M_PI - fabs(theta_xiaobi), theta_shouwan,
                                         -(M_PI - fabs(theta_xiaobi)), theta_shouwan,
                                         M_PI - fabs(theta_xiaobi), -theta_shouwan,
                                         -(M_PI - fabs(theta_xiaobi)), -theta_shouwan,
                                        };
    for(int i=0;i<16;i+=2)
    {
        theta_xiaobi=set_xiaobi_theta_shouwan[i];
        theta_shouwan=set_xiaobi_theta_shouwan[i+1];
        T13G = calculate_T13G(theta_qianhoutai, theta_cetai, theta_dabi, theta_zhoubu, theta_xiaobi, theta_shouwan)*Tend;
        T13G_p = T13G.block<3,1>(0, 3);
        distance = distanceCost3(T13G_p, p4);
        if (distance <= this->eps)
        {
            // ROS_INFO("set_xiaobi_theta_shouwan distance: %f", distance_p3_to_p3G_3d);
            break;
        }
    }
    Eigen::Vector3d theta_xiaobi_shouwan_vertify;
    theta_xiaobi_shouwan_vertify << theta_xiaobi, theta_shouwan, distance;
    return theta_xiaobi_shouwan_vertify;
}

double LeftArmKine::ConfigCalculate(Eigen::Matrix4d T14G,Eigen::Vector3d p1,Eigen::Vector3d p2,Eigen::Vector3d p3,Eigen::Vector3d p4)
{
    static const Eigen::Matrix4d T11G_zero = calculate_T11G(0.0, 0.0, 0.0, 0.0); // 常量, 只算一次
    Eigen::Vector2d theta_qianhoutai_cetai = pivot_hinge_solve(po(0),po(1),po(2), p1(0),p1(1),p1(2), p2(0),p2(1),p2(2),T11G_zero(0,3),T11G_zero(1,3),T11G_zero(2,3));
    double theta_qianhoutai = theta_qianhoutai_cetai(0);
    double theta_cetai = theta_qianhoutai_cetai(1);
    Eigen::Vector2d theta_qianhoutai_cetai_vertify = calculate_qianhoutai_cetai_verify(p2,0.0, 0.0, 0.0, theta_qianhoutai, theta_cetai);
    theta_qianhoutai = theta_qianhoutai_cetai_vertify(0);
    theta_cetai = theta_qianhoutai_cetai_vertify(1);
    Eigen::Matrix4d T13G=calculate_T13G(theta_qianhoutai,theta_cetai,0,M_PI_2-0,0,0);
    Eigen::Vector4d p3G_pre(T13G(0,3),T13G(1,3),T13G(2,3),1);
    Eigen::Vector2d theta_dabi_zhoubu = pivot_hinge_solve(p1(0),p1(1),p1(2), p2(0),p2(1),p2(2),p3(0),p3(1),p3(2), p3G_pre(0),p3G_pre(1),p3G_pre(2));
    double theta_dabi = theta_dabi_zhoubu(0);
    double theta_zhoubu = M_PI_2-theta_dabi_zhoubu(1);
    Eigen::Vector2d theta_dabi_zhoubu_vertify = calculate_dabi_zhoubu_verify(p3,0.0,0.0,0.0,theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu);
    theta_dabi = theta_dabi_zhoubu_vertify(0);
    theta_zhoubu = theta_dabi_zhoubu_vertify(1);
    T13G=calculate_T13G(theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu,0,M_PI_2-0);
    T13G=T13G*Tend;
    Eigen::Vector4d p4G_pre(T13G(0,3),T13G(1,3),T13G(2,3),1);
    Eigen::Vector2d theta_xiaobi_shouwan=pivot_hinge_solve(p2(0),p2(1),p2(2), p3(0),p3(1),p3(2),p4(0),p4(1),p4(2), p4G_pre(0),p4G_pre(1),p4G_pre(2));
    double theta_xiaobi = theta_xiaobi_shouwan(0);
    double theta_shouwan = M_PI_2-theta_xiaobi_shouwan(1);
    Eigen::Vector3d theta_xiaobi_shouwan_vertify = calculate_xiaobi_shouwan_verify(p4,theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu,theta_xiaobi,theta_shouwan);
    theta_xiaobi = theta_xiaobi_shouwan_vertify(0);
    theta_shouwan = theta_xiaobi_shouwan_vertify(1);
    double dis_res=theta_xiaobi_shouwan_vertify(2);

    config.at(7)=theta_qianhoutai;
    config.at(8)=theta_cetai;
    config.at(9)=theta_dabi;
    config.at(10)=theta_zhoubu;
    config.at(11)=theta_xiaobi;
    config.at(12)=theta_shouwan;

    Eigen::Matrix4d TEEG_desire=T14G;
    Eigen::Matrix3d desire_rotation = TEEG_desire.block<3, 3>(0, 0);// 提取旋转矩阵
    Eigen::Matrix4d T13G_result=calculate_T13G(config.at(7),config.at(8),config.at(9),config.at(10),config.at(11),config.at(12));
    Eigen::Matrix4d TEEG_result=T13G_result;
    Eigen::Matrix3d result_rotation = TEEG_result.block<3, 3>(0, 0);// 提取旋转矩阵

    #ifdef DEBUG
    cout<<"TEEG_desire: \n"<<TEEG_desire<<endl;
    cout<<"TEEG_result: \n"<<TEEG_result<<endl;
    #endif

    auto diff = angle_diff(desire_rotation,result_rotation);
    Eigen::Matrix4d TEEG_1=calculate_T15G(config.at(7),config.at(8),config.at(9),config.at(10),config.at(11),config.at(12),diff(0));
    Eigen::Matrix4d TEEG_2=calculate_T15G(config.at(7),config.at(8),config.at(9),config.at(10),config.at(11),config.at(12),-diff(0));

    double error_1 = calculate_orientation_error(TEEG_desire,TEEG_1);
    double error_2 = calculate_orientation_error(TEEG_desire,TEEG_2);
    if (error_1 < error_2)
    {
        config.at(13)=diff(0);
    }
    else
    {
        config.at(13)=-diff(0);
    }

    #ifdef DEBUG
    Eigen::Matrix4d T15G=calculate_T15G(config.at(7),config.at(8),config.at(9),config.at(10),config.at(11),config.at(12),config.at(13));
    Eigen::Matrix4d T15G_1=calculate_T15G(config.at(7),config.at(8),config.at(9),config.at(10),config.at(11),config.at(12),-config.at(13));
    cout<<"T15G: \n"<<T15G<<endl;
    cout<<"T15G_1: \n"<<T15G_1<<endl;
    #endif

    if(dis_res>1)
    {
        ROS_WARN("Result Angles (rad): QianHouTai: %.4f, CeTai: %.4f, DaBi: %.4f, ZhouBu: %.4f, XiaoBi: %.4f, ShouWan: %.4f, Final Rotate: %.4f, dis_res: %.4f",
                    config.at(7), config.at(8), config.at(9), config.at(10), config.at(11), config.at(12), config.at(13),dis_res);
    }


    return dis_res;
}


/*==================== FABRIK求解 ====================*/

bool LeftArmKine::retryRotateAvoid(Eigen::Vector3d &p, const Eigen::Vector3d &q, double d,
                              int link_index, int random_value, double random_jiaodu)
{
    const double rad = random_jiaodu * M_PI / 180.0; // 循环不变量: 只算一次
    Eigen::Matrix3d R = (random_value == 0) ? rotate_x(rad)
                      : (random_value == 1) ? rotate_y(rad)
                      :                      rotate_z(rad);
    for (int i = 1; i < segment; i++)
    {
        p = R * p;
        if (isCollision(link_index))
        {
            continue;
        }
        const double l = distanceCost3(q, p);
        p = (l - d) / l * q + d / l * p; // 与原 (l-d)/l*q + d/l*p 重投影一致
        return true;
    }
    return false;
}

bool LeftArmKine::retryRotateDirAvoid(Eigen::Vector3d &p, const Eigen::Vector3d &q, double d,
                                 int link_index, bool &collision)
{
    for (int i = 0; i < segment; i++)
    {
        if (isCollision(link_index))
        {
            collision = 1;
            int random_value = static_cast<int>(rand01() * 3.0);
            double random_jiaodu = rand01() * rand_range;
            const double rad = random_jiaodu * M_PI / 180.0;
            Eigen::Vector3d dir = p - q;
            if (random_value == 0)
            {
                dir = rotate_x(rad) * dir;
            }
            else if (random_value == 1)
            {
                dir = rotate_y(rad) * dir;
            }
            else
            {
                dir = rotate_z(rad) * dir;
            }
            dir.normalize();
            p = q + dir * d;
        }
        else
        {
            collision = 0;
            return true;
        }
    }
    return false;
}

int LeftArmKine::solve(int maxFailedAttempts, double eps, const Eigen::Matrix4d &T14G, int &iter_count)
{
    ROS_INFO("solve start");
    this->maxFailedAttempts = maxFailedAttempts;
    this->eps = eps;
    int failedAttempts = 0;
    double dis_res=1000;
    bool collision;
    TgoalB=T14G*Tend_1;
    p3_goal(0)=T14G(0,3);
    p3_goal(1)=T14G(1,3);
    p3_goal(2)=T14G(2,3);

    if(!set_p3_goal())
    {
        ROS_ERROR("goal with collision");
        return 2;
    }

    Eigen::Vector3d p3_0;
    Eigen::Vector3d p2_0;
    Eigen::Vector3d p1_0;
    double rad=0.0;
    int random_value=0;
    double random_jiaodu=0;
    double l=0.0;

    while(failedAttempts <= maxFailedAttempts && dis_res>eps)
    {
        if(rand01()<epsilon)
        {
            ROS_WARN("Reset to start position");
            p1 = p1_start;
            p2 = p2_start;
            p3 = p3_start;
        }
        else
        {
            p1 = p1_start;
            // double rand_theta8=-M_PI_2+rand01()*(M_PI);
            // double rand_theta9=-M_PI_2+rand01()*(M_PI_2);
            // double rand_theta10=(-15*M_PI/180.0)+rand01()*((85*M_PI/180.0)-(-15*M_PI/180.0));
            // double rand_theta11=(0*M_PI/180.0)+rand01()*((89*M_PI/180.0)-(0*M_PI/180.0));
            // double rand_theta12=(-60*M_PI/180.0)+rand01()*((60*M_PI/180.0)-(-60*M_PI/180.0));
            // double rand_theta13=(-15*M_PI/180.0)+rand01()*((15*M_PI/180.0)-(-15*M_PI/180.0));

            double theta_qianhoutai=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));
            double theta_cetai=(0*M_PI/180.0)+rand01()*((90*M_PI/180.0)-(0*M_PI/180.0));
            double theta_dabi=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));
            double theta_zhoubu=(-90*M_PI/180.0)+rand01()*((29*M_PI/180.0)-(-90*M_PI/180.0));
            double theta_xiaobi=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));
            double theta_shouwan=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));

            Eigen::Matrix4d p2_matrix = calculate_T11G(theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu);
            Eigen::Matrix4d p3_matrix = calculate_T13G(theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu,theta_xiaobi,theta_shouwan);
            p2[0]=p2_matrix(0,3);
            p2[1]=p2_matrix(1,3);
            p2[2]=p2_matrix(2,3);
            p3[0]=p3_matrix(0,3);
            p3[1]=p3_matrix(1,3);
            p3[2]=p3_matrix(2,3);
        }

        for(int i=0;i<little_count;i++)
        {
            if(distanceCost3(p3_goal,p3)<=eps||failedAttempts > maxFailedAttempts)break;
            if(show && failedAttempts % 5 == 0)thread_marker_CB();
            //确定P2P3连杆
            p3_0=p3;
            p3=p3_goal;
            l=distanceCost3(p2,p3);
            p2_0=p2;
            p2(0) = (l - d3) / l * p3(0) + d3 / l * p2(0);
            p2(1) = (l - d3) / l * p3(1) + d3 / l * p2(1);
            p2(2) = (l - d3) / l * p3(2) + d3 / l * p2(2);
            if(isCollision(P2P3))
            {
                collision=1;
                random_value = static_cast<int>(rand01() * 3.0);
                random_jiaodu=rand01()*rand_range;
                if(retryRotateAvoid(p2, p3, d3, P2P3, random_value, random_jiaodu))
                {
                    collision = 0;
                }
            }
            else
            {
                collision=0;
            }
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }

            if(show && failedAttempts % 5 == 0)thread_marker_CB();

            //确定P1P2连杆
            l = distanceCost3(p1, p2);
            p1_0 = p1; //将原先的p1记录下来
            p1(0) = (l - d2) / l * p2(0) + d2 / l * p1(0);
            p1(1) = (l - d2) / l * p2(1) + d2 / l * p1(1);
            p1(2) = (l - d2) / l * p2(2) + d2 / l * p1(2);
            if(isCollision(P1P2))
            {
                collision=1;
                random_value = static_cast<int>(rand01() * 3.0);
                random_jiaodu=rand01()*rand_range;
                if(retryRotateAvoid(p1, p2, d2, P1P2, random_value, random_jiaodu))
                {
                    collision = 0;
                }
            }
            else
            {
                collision=0;
            }
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                p1 = p1_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }

            if(show && failedAttempts % 5 == 0)thread_marker_CB();

            //后向过程
            //先确定P1P2连杆
            p1 = p1_start; //将原先的p1记录下来
            l = distanceCost3(p1, p2);
            p2(0) = (l - d2) / l * p1(0) + d2 / l * p2(0);
            p2(1) = (l - d2) / l * p1(1) + d2 / l * p2(1);
            p2(2) = (l - d2) / l * p1(2) + d2 / l * p2(2);
            if(isCollision(P1P2))
            {
                collision=1;
                random_value = static_cast<int>(rand01() * 3.0);
                random_jiaodu=rand01()*rand_range;
                if(retryRotateAvoid(p2, p1, d2, P1P2, random_value, random_jiaodu))
                {
                    collision = 0;
                }
            }
            else
            {
                collision=0;
            }
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                p1 = p1_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }

            if(show && failedAttempts % 5 == 0)thread_marker_CB();

            //确定P2P3连杆
            l=distanceCost3(p2,p3);
            p3(0) = (l - d3) / l * p2(0) + d3 / l * p3(0);
            p3(1) = (l - d3) / l * p2(1) + d3 / l * p3(1);
            p3(2) = (l - d3) / l * p2(2) + d3 / l * p3(2);
            if(isCollision(P2P3))
            {
                collision=1;
                random_value = static_cast<int>(rand01() * 3.0);
                random_jiaodu=rand01()*rand_range;
                if(retryRotateAvoid(p3, p2, d3, P2P3, random_value, random_jiaodu))
                {
                    collision = 0;
                }
            }
            else
            {
                collision=0;
            }
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                p1 = p1_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }
            failedAttempts = failedAttempts + 1;
            if(show && failedAttempts % 5 == 0)thread_marker_CB();

        }//while
        dis_res=ConfigCalculate(T14G,p1,p2,p3,p4);
        if(dis_res>eps)failedAttempts++;
    }//while

    if(failedAttempts<maxFailedAttempts)
    {
        iter_count=failedAttempts;
        ROS_INFO("solve successfully");
        ROS_INFO("Result Angles (rad): QianHouTai: %f, CeTai: %f, DaBi: %f, ZhouBu: %f, XiaoBi: %f, ShouWan: %f, Final Rotate: %f, dis_res: %f",config.at(7),
        config.at(8),config.at(9),config.at(10),config.at(11),config.at(12),config.at(13),dis_res);
        if(show)
        {
            box1_marker.color.a = 0.9;
            box1_marker.color.r = 0.0;
            box1_marker.color.g = 0.0;
            box1_marker.color.b = 1.0;
            box2_marker.color.a = 0.9;
            box2_marker.color.r = 0.0;
            box2_marker.color.g = 0.0;
            box2_marker.color.b = 1.0;
            box3_marker.color.a = 0.9;
            box3_marker.color.r = 0.0;
            box3_marker.color.g = 0.0;
            box3_marker.color.b = 1.0;
            thread_marker_CB();
        }
        return 1;
    }
    else
    {
        iter_count=-1;
        ROS_ERROR("solve failed\n");
        return 0;
    }

}

int LeftArmKine::solveSelfRotation(int maxFailedAttempts, double eps, const Eigen::Matrix4d &T14G, int &iter_count)
{
    ROS_INFO("solveSelfRotation start");
    this->maxFailedAttempts = maxFailedAttempts;
    this->eps = eps;
    int failedAttempts = 0;
    double dis_res=1000;
    bool collision;
    TgoalB=T14G*Tend_1;
    p3_goal(0)=T14G(0,3);
    p3_goal(1)=T14G(1,3);
    p3_goal(2)=T14G(2,3);

    if(!set_p3_goal())
    {
        ROS_ERROR("goal with collision");
        return 2;
    }

    Eigen::Vector3d p3_0;
    Eigen::Vector3d p2_0;
    Eigen::Vector3d p1_0;
    double rad=0.0;
    int random_value=0;
    double random_jiaodu=0;
    double l=0.0;

    while(failedAttempts <= maxFailedAttempts && dis_res>eps)
    {
        if(rand01()<epsilon)
        {
            p1 = p1_start;
            p2 = p2_start;
            p3 = p3_start;
        }
        else
        {
            p1 = p1_start;
            double theta_qianhoutai=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));
            double theta_cetai=(0*M_PI/180.0)+rand01()*((90*M_PI/180.0)-(0*M_PI/180.0));
            double theta_dabi=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));
            double theta_zhoubu=(-90*M_PI/180.0)+rand01()*((29*M_PI/180.0)-(-90*M_PI/180.0));
            double theta_xiaobi=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));
            double theta_shouwan=(-180*M_PI/180.0)+rand01()*((180*M_PI/180.0)-(-180*M_PI/180.0));

            Eigen::Matrix4d p2_matrix = calculate_T11G(theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu);
            Eigen::Matrix4d p3_matrix = calculate_T13G(theta_qianhoutai,theta_cetai,theta_dabi,theta_zhoubu,theta_xiaobi,theta_shouwan);
            p2[0]=p2_matrix(0,3);
            p2[1]=p2_matrix(1,3);
            p2[2]=p2_matrix(2,3);
            p3[0]=p3_matrix(0,3);
            p3[1]=p3_matrix(1,3);
            p3[2]=p3_matrix(2,3);
        }

        // thread_marker_CB();

        for(int i=0;i<100;i++)
        {
            if(distanceCost3(p3_goal,p3)<=eps||failedAttempts > maxFailedAttempts)break;
            p3_0=p3;
            p2_0=p2;
            p3=p3_goal;
            // 尝试segment次随机旋转避障; 失败时collision保持1, 由下方恢复逻辑处理
            retryRotateDirAvoid(p2, p3, d3, P2P3, collision);
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }

            // thread_marker_CB();

            //确定P1P2连杆
            p1_0 = p1; //将原先的p1记录下来
            // 尝试segment次随机旋转避障; 失败时collision保持1, 由下方恢复逻辑处理
            retryRotateDirAvoid(p1, p2, d2, P1P2, collision);
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                p1 = p1_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }

            // thread_marker_CB();

            //后向过程
            //先确定P1P2连杆
            p1 = p1_start; //将原先的p1记录下来
            // 尝试segment次随机旋转避障; 失败时collision保持1, 由下方恢复逻辑处理
            retryRotateDirAvoid(p2, p1, d2, P1P2, collision);
            // thread_marker_CB();
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                p1 = p1_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }

            //确定P2P3连杆
            // 尝试segment次随机旋转避障; 失败时collision保持1, 由下方恢复逻辑处理
            retryRotateDirAvoid(p3, p2, d3, P2P3, collision);
            if (collision == 1)
            {
                p3 = p3_0;
                p2 = p2_0;
                p1 = p1_0;
                failedAttempts = failedAttempts + 1;
                continue;
            }
            failedAttempts = failedAttempts + 1;
            // thread_marker_CB();
        }//while
        dis_res=ConfigCalculate(T14G,p1,p2,p3,p4);
        if(dis_res>eps)failedAttempts++;
        if(VERBOSE)
        {
            ROS_INFO("dis_res: %f",dis_res);
        }
    }

    if(failedAttempts<maxFailedAttempts)
    {
        iter_count=failedAttempts;
        ROS_INFO("solve successfully");
        ROS_INFO("Result Angles (rad): QianHouTai: %f, CeTai: %f, DaBi: %f, ZhouBu: %f, XiaoBi: %f, ShouWan: %f, Final Rotate: %f, dis_res: %f",config.at(7),
        config.at(8),config.at(9),config.at(10),config.at(11),config.at(12),config.at(13),dis_res);
        if(show)
        {
            box1_marker.color.a = 1.0;
            box1_marker.color.r = 1.0;
            box1_marker.color.g = 1.0;
            box1_marker.color.b = 1.0;
            box2_marker.color.a = 1.0;
            box2_marker.color.r = 1.0;
            box2_marker.color.g = 1.0;
            box2_marker.color.b = 1.0;
            box3_marker.color.a = 1.0;
            box3_marker.color.r = 1.0;
            box3_marker.color.g = 1.0;
            box3_marker.color.b = 1.0;
            thread_marker_CB();
        }
        return 1;
    }
    else
    {
        iter_count=-1;
        ROS_ERROR("solve failed");
        return 0;
    }
}


/*==================== 碰撞检测 ====================*/

bool LeftArmKine::isCollision(int index)
{
    if(!pc_use)return false; // pc_use==0时跳过八叉树碰撞(空树碰撞结果恒为false, 与原行为一致)
    Eigen::Quaterniond quat;
    bool body_collision, pc_collision;
    switch (index)
    {
        case P1P2:
        {
            collision_data->box_obj1->setTranslation((p1+p2)/2000.0);
            Eigen::Vector3d p1p2 = p2 - p1;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p1p2.normalized());
            collision_data->box_obj1->setQuatRotation(quat);
            collision_data->result.clear();
            collide(collision_data->box_obj1.get(), collision_data->box_obj_body.get(), collision_data->request, collision_data->result);
            body_collision=collision_data->result.isCollision();
            // collision_data->result.getContacts(contacts);
            // ROS_INFO("P1P2 body_collision contact size: %lu",contacts.size());
            collision_data->result.clear();
            collide(collision_data->box_obj1.get(), collision_data->tree_obj.get(), collision_data->request, collision_data->result);
            pc_collision=collision_data->result.isCollision();
            // collision_data->result.getContacts(contacts);
            // ROS_INFO("P1P2 pc_collision contact size: %lu",contacts.size());
            if (body_collision||pc_collision)
            {
                box1_marker.color.r = 1.0; // 碰撞时，标记为红色
                box1_marker.color.g = 0.0;
                box1_marker.color.b = 0.0;
            }
            else
            {
                box1_marker.color.r = 0.0; // 无碰撞时，标记为绿色
                box1_marker.color.g = 1.0;
                box1_marker.color.b = 0.0;
            }
        }
            break;
        case P2P3:
        {
            collision_data->box_obj2->setTranslation((p2+p3)/2000.0);
            Eigen::Vector3d p2p3 = p3 - p2;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p2p3.normalized());
            collision_data->box_obj2->setQuatRotation(quat);
            collision_data->result.clear();
            collide(collision_data->box_obj2.get(), collision_data->box_obj_body.get(), collision_data->request, collision_data->result);
            body_collision=collision_data->result.isCollision();
            collision_data->result.clear();
            collide(collision_data->box_obj2.get(), collision_data->tree_obj.get(), collision_data->request, collision_data->result);
            pc_collision=collision_data->result.isCollision();
            if (body_collision||pc_collision)
            {
                box2_marker.color.r = 1.0; // 碰撞时，标记为红色
                box2_marker.color.g = 0.0;
                box2_marker.color.b = 0.0;
            }
            else
            {
                box2_marker.color.r = 0.0; // 无碰撞时，标记为绿色
                box2_marker.color.g = 1.0;
                box2_marker.color.b = 0.0;
            }
        }
            break;
        case P3P4:
        {
            collision_data->box_obj3->setTranslation((p3+p4)/2000.0);
            Eigen::Vector3d p3p4 = p4 - p3;
            // collision_data->box_obj3->setTranslation((p3_goal+p4)/2000.0);
            // Eigen::Vector3d p3p4 = p4 - p3_goal;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p3p4.normalized());
            collision_data->box_obj3->setQuatRotation(quat);
            collision_data->result.clear();
            collide(collision_data->box_obj3.get(), collision_data->box_obj_body.get(), collision_data->request, collision_data->result);
            body_collision=collision_data->result.isCollision();
            collision_data->result.clear();
            collide(collision_data->box_obj3.get(), collision_data->tree_obj.get(), collision_data->request, collision_data->result);
            pc_collision=collision_data->result.isCollision();
            if (body_collision||pc_collision)
            {
                box3_marker.color.r = 1.0; // 碰撞时，标记为红色
                box3_marker.color.g = 0.0;
                box3_marker.color.b = 0.0;
            }
            else
            {
                box3_marker.color.r = 0.0; // 无碰撞时，标记为绿色
                box3_marker.color.g = 1.0;
                box3_marker.color.b = 0.0;
            }
            break;
        }
        case P3GoalP4:
        {
            collision_data->box_obj3->setTranslation((p3_goal+p4)/2000.0);
            Eigen::Vector3d p3p4 = p4 - p3_goal;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p3p4.normalized());
            collision_data->box_obj3->setQuatRotation(quat);
            collision_data->result.clear();
            collide(collision_data->box_obj3.get(), collision_data->box_obj_body.get(), collision_data->request, collision_data->result);
            body_collision=collision_data->result.isCollision();
            collision_data->result.clear();
            collide(collision_data->box_obj3.get(), collision_data->tree_obj.get(), collision_data->request, collision_data->result);
            pc_collision=collision_data->result.isCollision();
            if (body_collision||pc_collision)
            {
                box3_marker.color.r = 1.0; // 碰撞时，标记为红色
                box3_marker.color.g = 0.0;
                box3_marker.color.b = 0.0;
            }
            else
            {
                box3_marker.color.r = 0.0; // 无碰撞时，标记为绿色
                box3_marker.color.g = 1.0;
                box3_marker.color.b = 0.0;
            }
            break;
        }
        default:
            ROS_ERROR("the link is not found");
            return false;
    }
    // ROS_INFO("collision information:\nbody_collision: %d, pc_collision, %d",body_collision,pc_collision);
    return body_collision||pc_collision;
}

bool LeftArmKine::set_p3_goal()
{
    Eigen::Vector3d goal(TgoalB(0,3),TgoalB(1,3),TgoalB(2,3));
    p4=goal;
    // cout<<"P4:\n"<<p4<<endl;
    // Eigen::Vector3d dir(TgoalB(0,1),TgoalB(1,1),TgoalB(2,1));
    // p3_goal = goal - wrist_to_gripper*dir;
    // cout<<"P3_goal:\n"<<p3_goal<<endl;
    if(isCollision(P3GoalP4))
    {
        ROS_ERROR("set goal failed");
        if(show && nh)
        {
            box3_marker.pose.position.x = (p3_goal[0]+p4[0])/2000.0;
            box3_marker.pose.position.y = (p3_goal[1]+p4[1])/2000.0;
            box3_marker.pose.position.z = (p3_goal[2]+p4[2])/2000.0;
            Eigen::Vector3d p3p4 = p4 - p3_goal;
            Eigen::Quaterniond quat;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p3p4.normalized());
            box3_marker.pose.orientation.x = quat.x();
            box3_marker.pose.orientation.y = quat.y();
            box3_marker.pose.orientation.z = quat.z();
            box3_marker.pose.orientation.w = quat.w();
            marker_pub.publish(box3_marker);
        }
        return 0;
    }
    else
    {
        ROS_INFO("set goal successfully");
        if(show && nh)
        {
            box3_marker.pose.position.x = (p3_goal[0]+p4[0])/2000.0;
            box3_marker.pose.position.y = (p3_goal[1]+p4[1])/2000.0;
            box3_marker.pose.position.z = (p3_goal[2]+p4[2])/2000.0;
            Eigen::Vector3d p3p4 = p4 - p3_goal;
            Eigen::Quaterniond quat;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p3p4.normalized());
            box3_marker.pose.orientation.x = quat.x();
            box3_marker.pose.orientation.y = quat.y();
            box3_marker.pose.orientation.z = quat.z();
            box3_marker.pose.orientation.w = quat.w();
            marker_pub.publish(box3_marker);
        }
        return 1;
    }
}


/*==================== 可视化 ====================*/

void LeftArmKine::thread_marker_CB()
{
    if (!nh) return;
    Eigen::Quaterniond quat;
    box1_marker.pose.position.x = (p1[0]+p2[0])/2000.0;
    box1_marker.pose.position.y = (p1[1]+p2[1])/2000.0;
    box1_marker.pose.position.z = (p1[2]+p2[2])/2000.0;
    Eigen::Vector3d p1p2 = p2 - p1;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p1p2.normalized());
    box1_marker.pose.orientation.x = quat.x();
    box1_marker.pose.orientation.y = quat.y();
    box1_marker.pose.orientation.z = quat.z();
    box1_marker.pose.orientation.w = quat.w();

    box2_marker.pose.position.x = (p2[0]+p3[0])/2000.0;
    box2_marker.pose.position.y = (p2[1]+p3[1])/2000.0;
    box2_marker.pose.position.z = (p2[2]+p3[2])/2000.0;
    Eigen::Vector3d p2p3 = p3 - p2;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p2p3.normalized());
    box2_marker.pose.orientation.x = quat.x();
    box2_marker.pose.orientation.y = quat.y();
    box2_marker.pose.orientation.z = quat.z();
    box2_marker.pose.orientation.w = quat.w();

    box3_marker.pose.position.x = (p3[0]+p4[0])/2000.0;
    box3_marker.pose.position.y = (p3[1]+p4[1])/2000.0;
    box3_marker.pose.position.z = (p3[2]+p4[2])/2000.0;
    Eigen::Vector3d p3p4 = p4 - p3;
    quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), p3p4.normalized());
    box3_marker.pose.orientation.x = quat.x();
    box3_marker.pose.orientation.y = quat.y();
    box3_marker.pose.orientation.z = quat.z();
    box3_marker.pose.orientation.w = quat.w();

    marker_pub.publish(box1_marker);
    usleep(1*1000);
    marker_pub.publish(box2_marker);
    usleep(1*1000);
    marker_pub.publish(box3_marker);
    usleep(1*1000);
    marker_pub.publish(body_marker);
    usleep(1*1000);
    marker_pub.publish(waist_marker);
    usleep(1*1000);

    usleep(50*1000);
}


double LeftArmKine::rand01()
{
    return dist01(rng);
}
