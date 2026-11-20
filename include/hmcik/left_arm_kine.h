#pragma once

#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

class LeftArmKine
{

private:
    std::unique_ptr<ros::NodeHandle> nh;
    Eigen::Vector3d pw; //point_waist
    Eigen::Vector3d pf; //point_foot
    Eigen::Vector3d po; //point_chest
    Eigen::Vector3d p1; //point_shoulder
    Eigen::Vector3d p2; //point_elbow
    Eigen::Vector3d p3; //point_wrist
    Eigen::Vector3d p4; //point_gripper
    Eigen::Vector3d p1_start;   //肩部初始位置
    Eigen::Vector3d p2_start;   //肘部初始位置
    Eigen::Vector3d p3_start;   //手腕初始位置
    Eigen::Matrix4d TgoalB; //目标位姿
    Eigen::Vector3d p3_goal;    //手腕目标位置
    visualization_msgs::Marker box1_marker;
    visualization_msgs::Marker box2_marker;
    visualization_msgs::Marker box3_marker;
    visualization_msgs::Marker body_marker;
    visualization_msgs::Marker waist_marker;
    int maxFailedAttempts; //最大错误次数

    double car_to_foot = 237.0;
    double foot_to_ground = 349.18;
    double waist_to_foot = 641.7; // 加动平台厚度 1.0168-0.34918
    double waist_to_chest = 352.02; //351.5, 1.3676-1.0168
    double chest_to_shoulder = 262.94; //
    double shoulder_to_elbow = 340.50;
    double elbow_to_wrist = 226.98;
    double wrist_to_gripper = 316.27;

    double waist_to_ground = foot_to_ground + waist_to_foot;
    double chest_to_ground = waist_to_ground + waist_to_chest;

    double a3 = car_to_foot;
    double a6 = waist_to_ground;
    double a7 = waist_to_chest;
    double d1 = chest_to_shoulder; // 胸部中心与肩部的距离
    double d2 = shoulder_to_elbow; // 肩部与肘部的距离
    double d3 = elbow_to_wrist; // 肘部与腕部的距离
    double d4 = wrist_to_gripper; // 腕部与手爪的距离

    ros::Publisher marker_pub;
    struct CollisionData;
    std::unique_ptr<CollisionData> collision_data;
    sensor_msgs::PointCloud2 self_pointcloud;
    ros::Publisher self_pointcloud_pub;
    std::vector<double> config;
    std::mt19937 rng;                                   // 随机数发生器(random_device播种)
    std::uniform_real_distribution<double> dist01;      // [0,1) 均匀分布

    void setConfig(const std::string &config_filename);
    double angle(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
    Eigen::Vector3d angle_diff(Eigen::Matrix3d &R1, Eigen::Matrix3d &R2);

    Eigen::Matrix4d calculate_T9G(const double& theta8, const double& theta9); //计算T9G
    Eigen::Matrix4d calculate_T11G(const double& theta8, const double& theta9, const double& theta10,
                                    const double& theta11); //计算T11G
    Eigen::Vector2d calculate_dabi_zhoubu_verify(
        Eigen::Vector3d p_wrist, double theta_wanyao, double theta_ceyao,
        double theta_jiaobu, double theta_qianhoutai, double theta_cetai,
        double theta_dabi, double theta_zhoubu); //计算大臂肘部验证正负
    Eigen::Vector2d calculate_qianhoutai_cetai_verify(
        Eigen::Vector3d p3, double theta_wanyao, double theta_ceyao,
        double theta_jiaobu, double theta_qianhoutai, double theta_cetai); //计算前后抬侧抬验证正负
    Eigen::Vector3d calculate_xiaobi_shouwan_verify(Eigen::Vector3d p4,
                                                 double theta_qianhoutai,
                                                 double theta_cetai,
                                                 double theta_dabi,
                                                 double theta_zhoubu,
                                                 double theta_xiaobi,
                                                 double theta_shouwan); //计算小臂手腕验证正负
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LeftArmKine(const std::string &config_filename = "config_double.txt",
                         bool publish_ros = false,
                         unsigned int seed = std::random_device{}());
    ~LeftArmKine();
    Eigen::Matrix4d Tend;
    Eigen::Matrix4d Tend_1;
    //超参数设置
    int segment;
    double epsilon;
    int little_count;
    double rand_range;
    int max_count;
    double eps;
    int pc_use;
    int show;
    int test_count;
    bool GARR;
    int scene;
    std::string point_cloud_filename;
    std::string package_path;
    std::string resource_path;
    ros::Publisher plan_req;
    //功能函数
    void thread_marker_CB(); //发布marker线程回调函数
    bool set_p3_goal(); //设置手腕目标位置
    bool isCollision(int index); //碰撞检测
    int solve(int maxFailedAttempts, double eps, const Eigen::Matrix4d &T14G, int &iter_count); //求解函数
    int solveSelfRotation(int maxFailedAttempts, double eps, const Eigen::Matrix4d &T14G, int &iter_count); //FABRIK求解
    bool retryRotateAvoid(Eigen::Vector3d &p, const Eigen::Vector3d &q, double d,
                          int link_index, int random_value, double random_jiaodu); //solve()碰撞重试辅助
    bool retryRotateDirAvoid(Eigen::Vector3d &p, const Eigen::Vector3d &q, double d,
                             int link_index, bool &collision); //solveSelfRotation()碰撞重试辅助
    double distanceCost3(const Eigen::Vector3d &p1,Eigen::Vector3d &p2); //距离函数
    double rand01(); //生成0-1随机数
    double ConfigCalculate(Eigen::Matrix4d desire_pose,Eigen::Vector3d p1,Eigen::Vector3d p2,Eigen::Vector3d p3,Eigen::Vector3d p4); //计算当前构型对应的关节角度
    Eigen::Matrix3d rotate_x(double rad); //绕x轴旋转矩阵
    Eigen::Matrix3d rotate_y(double rad); //绕y轴旋转矩阵
    Eigen::Matrix3d rotate_z(double rad); //绕z轴旋转矩阵
    Eigen::Vector2d pivot_hinge_solve(const double& pstart_x, const double& pstart_y, const double& pstart_z,
                                        const double& pmid_x, const double& pmid_y, const double& pmid_z,
                                        const double& pend_x, const double& pend_y, const double& pend_z,
                                        const double& pend_pre_x, const double& pend_pre_y, const double& pend_pre_z); //枢轴铰链求解
    Eigen::Matrix4d calculate_T13G(const double& theta8, const double& theta9, const double& theta10,
                                const double& theta11, const double& theta12, const double& theta13); //计算T13G

    Eigen::Matrix4d calculate_T14G(const double& theta8, const double& theta9, const double& theta10,
    const double& theta11, const double& theta12, const double& theta13, const double& theta14); //计算T14G

    Eigen::Matrix4d calculate_T15G(const double& theta8, const double& theta9, const double& theta10,
        const double& theta11, const double& theta12, const double& theta13, const double& theta14);
    Eigen::Matrix4d Pose2Matrix(const geometry_msgs::Pose &pose);
    geometry_msgs::Pose Matrix2Pose(const Eigen::Matrix4d &result_matrix);
    double calculate_orientation_error(const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2);
    void getConfig(std::vector<double> &config); //获取目标构型
};
