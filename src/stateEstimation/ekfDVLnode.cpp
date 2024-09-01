//
// Created by jurobotics on 13.09.21.

#include "ekfDVL.h"
#include "rclcpp/rclcpp.hpp"

// just for tricking compiler
//#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "waterlinked_a50/msg/transducer_report_stamped.hpp"
#include "waterlinked_a50/msg/position_report_stamped.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

//#include <geometry_msgs/msg/PoseWithCovarianceStamped.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
//#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
//#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/sensor_baro.hpp"
#include "px4_msgs/msg/vehicle_air_data.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

//#include "../slamTools/generalHelpfulTools.h"
//#include "waterlinked_dvl/TransducerReportStamped.h"
//#include "commonbluerovmsg/srv/reset_ekf.hpp"
//#include "commonbluerovmsg/msg/height_stamped.hpp"
//#include <chrono>
#include <thread>
//#include "bluerov2common/ekfParameterConfig.h"
//#include <dynamic_reconfigure/server.h>
//#include <commonbluerovmsg/msg/state_of_blue_rov.hpp>
#include "dynamic_reconfigure/server.h"

static constexpr double CONSTANTS_ONE_G = 9.80665;

class RosClassEKF : public rclcpp::Node {
public:
    RosClassEKF() : Node("ekfStateEstimationASV"), currentEkf(rclcpp::Clock(RCL_ROS_TIME).now()) {


        this->declareParameters();
        this->getParameter();
        this->callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&RosClassEKF::parametersCallback, this, std::placeholders::_1));

        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


        // External = 0; Mavros = 1; Gazebo = 2
        this->firstMessage = true;
        this->currentInputDVL = 0;
        this->currentInputIMU = 0;
//        this->subscriberDVL.shutdown();
        this->rotationOfDVL = Eigen::AngleAxisd(2.35619449019,
                                                Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data; quick fix set to default
        this->positionIMU = Eigen::Vector3d(0, 0, 0);
        this->positionDVL = Eigen::Vector3d(0, 0, 0);


        this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", qos,
                                                                               std::bind(&RosClassEKF::imuCallback,
                                                                                         this, std::placeholders::_1));

//        this->subscriberPX4IMU = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
//                                                                               std::bind(&RosClassEKF::imuCallbackPX4,
//                                                                                         this, std::placeholders::_1));
        std::cout << "test DVL:" << std::endl;
        this->subscriberDVL = this->create_subscription<waterlinked_a50::msg::TransducerReportStamped>(
                "/velocity_estimate", qos, std::bind(&RosClassEKF::DVLCallbackDVL, this, std::placeholders::_1));


//        this->subscriberDepthOwnTopic = this->create_subscription<commonbluerovmsg::msg::HeightStamped>("height_baro",
//                                                                                                        qos,
//                                                                                                        std::bind(
//                                                                                                                &RosClassEKF::depthSensorCallback,
//                                                                                                                this,
//                                                                                                                std::placeholders::_1));

//        this->subscriberDepthSensorBaroPX4 = this->create_subscription<px4_msgs::msg::SensorBaro>("/fmu/out/sensor_baro", qos,
//                                                                                                        std::bind(
//                                                                                                        &RosClassEKF::depthSensorBaroPX4,
//                                                                                                        this,
//                                                                                                        std::placeholders::_1));
//        this->subscriberDepthSensorBaroSensorTube = this->create_subscription<sensor_msgs::msg::FluidPressure>(
//                "/pressure", qos,
//                std::bind(
//                        &RosClassEKF::depthSensorBaroSensorTubeCallback,
//                        this,
//                        std::placeholders::_1));
//        this->subscriberDepthSensorVehicleAirData = this->create_subscription<px4_msgs::msg::VehicleAirData>("/fmu/out/vehicle_air_data", qos,
//                                                                                                  std::bind(
//                                                                                                          &RosClassEKF::depthSensorVehicleAir,
//                                                                                                          this,
//                                                                                                          std::placeholders::_1));


        this->subscriberHeading = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("magnetic_heading", qos,
                                                                                                std::bind(
                                                                                                        &RosClassEKF::headingCallback,
                                                                                                        this,
                                                                                                        std::placeholders::_1));

//        this->serviceResetEkf = this->create_service<commonbluerovmsg::srv::ResetEkf>("resetCurrentEKF",
//                                                                                      std::bind(&RosClassEKF::resetEKF,
//                                                                                                this,
//                                                                                                std::placeholders::_1,
//                                                                                                std::placeholders::_2));

        this->publisherPoseEkf = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos);
        this->publisherTwistEkf = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "publisherTwistEkf", qos);

        this->publisherVehicleOdometry = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry", qos);

        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(1.0 / 30.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&RosClassEKF::timer_callback, this));


        this->timeAtStart = rclcpp::Time();
    }

private:
//    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
//    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
//    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClassDVL currentEkf;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriberIMU;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriberPX4IMU;

    //subscriber
//    rclcpp::Subscription<commonbluerovmsg::msg::HeightStamped>::SharedPtr subscriberDepthOwnTopic;
    rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr subscriberDepthSensorBaroPX4;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriberDepthSensorBaroSensorTube;
    rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr subscriberDepthSensorVehicleAirData;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscriberHeading;
    rclcpp::Subscription<waterlinked_a50::msg::TransducerReportStamped>::SharedPtr subscriberDVL;

    //publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisherPoseEkf;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisherTwistEkf;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisherVehicleOdometry;

    //Timer

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex updateSlamMutex;
    Eigen::Quaterniond rotationOfDVL;
    Eigen::Vector3d positionIMU, positionDVL;

    int currentInputDVL;
    int currentInputIMU;
    double pressureWhenStarted;
    bool firstMessage;
    rclcpp::Time timeAtStart;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    //Parameters of EKF
    double measurementNoiseImuPitch;
    double measurementNoiseImuRoll;

    double measurementNoiseImuVelocityVelRoll;
    double measurementNoiseImuVelocityVelPitch;
    double measurementNoiseImuVelocityVelYaw;

    double measurementNoiseDVLVX;
    double measurementNoiseDVLVY;
    double measurementNoiseDVLVZ;

    double processNoisePitch;
    double processNoiseRoll;
    double processNoiseYaw;

    double processNoiseVX;
    double processNoiseVY;
    double processNoiseVZ;

    double processNoiseVelPitch;
    double processNoiseVelRoll;
    double processNoiseVelYaw;

    double processNoiseX;
    double processNoiseY;
    double processNoiseZ;

    //Position of Sensors
    //DVL:
    double xPositionDVL;
    double yPositionDVL;
    double zPositionDVL;
    double yawRotationDVL;

    //IMU
    double xPositionIMU;
    double yPositionIMU;
    double zPositionIMU;
    double yawRotationIMU;

    void declareParameters() {
        //measurement noise
        this->declare_parameter("measurementNoiseImuPitch", 100.0);
        this->declare_parameter("measurementNoiseImuRoll", 100.0);

        this->declare_parameter("measurementNoiseImuVelocityVelRoll", 0.1);
        this->declare_parameter("measurementNoiseImuVelocityVelPitch", 0.1);
        this->declare_parameter("measurementNoiseImuVelocityVelYaw", 0.1);

        this->declare_parameter("measurementNoiseDVLVX", 0.1);
        this->declare_parameter("measurementNoiseDVLVY", 0.1);
        this->declare_parameter("measurementNoiseDVLVZ", 0.1);

        //process noise
        this->declare_parameter("processNoisePitch", 0.01);
        this->declare_parameter("processNoiseRoll", 0.01);
        this->declare_parameter("processNoiseYaw", 0.02);

        this->declare_parameter("processNoiseVX", 0.005);
        this->declare_parameter("processNoiseVY", 0.005);
        this->declare_parameter("processNoiseVZ", 0.01);

        this->declare_parameter("processNoiseVelPitch", 0.001);
        this->declare_parameter("processNoiseVelRoll", 0.001);
        this->declare_parameter("processNoiseVelYaw", 0.001);

        this->declare_parameter("processNoiseX", 0.02);
        this->declare_parameter("processNoiseY", 0.02);
        this->declare_parameter("processNoiseZ", 0.005);

        //Position of Sensors
        //DVL:
        this->declare_parameter("xPositionDVL", 0.0);
        this->declare_parameter("yPositionDVL", 0.0);
        this->declare_parameter("zPositionDVL", 0.0);
        this->declare_parameter("yawRotationDVL", 0.0);

        //IMU
        this->declare_parameter("xPositionIMU", 0.0);
        this->declare_parameter("yPositionIMU", 0.0);
        this->declare_parameter("zPositionIMU", 0.0);
        this->declare_parameter("yawRotationIMU", 0.0);

        //potential GPS stuff, maybe second IMU for redundence

    }

    void getParameter() {
        this->measurementNoiseImuPitch = this->get_parameter("measurementNoiseImuPitch").as_double();
        this->measurementNoiseImuRoll = this->get_parameter("measurementNoiseImuRoll").as_double();

        this->measurementNoiseImuVelocityVelRoll = this->get_parameter(
                "measurementNoiseImuVelocityVelRoll").as_double();
        this->measurementNoiseImuVelocityVelPitch = this->get_parameter(
                "measurementNoiseImuVelocityVelPitch").as_double();
        this->measurementNoiseImuVelocityVelYaw = this->get_parameter("measurementNoiseImuVelocityVelYaw").as_double();

        this->measurementNoiseDVLVX = this->get_parameter("measurementNoiseDVLVX").as_double();
        this->measurementNoiseDVLVY = this->get_parameter("measurementNoiseDVLVY").as_double();
        this->measurementNoiseDVLVZ = this->get_parameter("measurementNoiseDVLVZ").as_double();

        this->processNoisePitch = this->get_parameter("processNoisePitch").as_double();
        this->processNoiseRoll = this->get_parameter("processNoiseRoll").as_double();
        this->processNoiseYaw = this->get_parameter("processNoiseYaw").as_double();

        this->processNoiseVX = this->get_parameter("processNoiseVX").as_double();
        this->processNoiseVY = this->get_parameter("processNoiseVY").as_double();
        this->processNoiseVZ = this->get_parameter("processNoiseVZ").as_double();

        this->processNoiseVelPitch = this->get_parameter("processNoiseVelPitch").as_double();
        this->processNoiseVelRoll = this->get_parameter("processNoiseVelRoll").as_double();
        this->processNoiseVelYaw = this->get_parameter("processNoiseVelYaw").as_double();

        this->processNoiseX = this->get_parameter("processNoiseX").as_double();
        this->processNoiseY = this->get_parameter("processNoiseY").as_double();
        this->processNoiseZ = this->get_parameter("processNoiseZ").as_double();

        this->xPositionDVL = this->get_parameter("xPositionDVL").as_double();
        this->yPositionDVL = this->get_parameter("yPositionDVL").as_double();
        this->zPositionDVL = this->get_parameter("zPositionDVL").as_double();
        this->yawRotationDVL = this->get_parameter("yawRotationDVL").as_double();

        this->xPositionIMU = this->get_parameter("xPositionIMU").as_double();
        this->yPositionIMU = this->get_parameter("yPositionIMU").as_double();
        this->zPositionIMU = this->get_parameter("zPositionIMU").as_double();
        this->yawRotationIMU = this->get_parameter("yawRotationIMU").as_double();

    }


    rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";
        for (const auto &param: parameters) {
            if (param.get_name() == "measurementNoiseImuPitch") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseImuPitch = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseImuRoll") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseImuRoll = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseImuVelocityVelRoll") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseImuVelocityVelRoll = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseImuVelocityVelPitch") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseImuVelocityVelPitch = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseImuVelocityVelYaw") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseImuVelocityVelYaw = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseDVLVX") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseDVLVX = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseDVLVY") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseDVLVY = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "measurementNoiseDVLVZ") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->measurementNoiseDVLVZ = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoisePitch") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoisePitch = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseRoll") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseRoll = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseYaw") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseYaw = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseVX") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseVX = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseVY") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseVY = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseVZ") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseVZ = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseVelPitch") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseVelPitch = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseVelRoll") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseVelRoll = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseVelYaw") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseVelYaw = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseX") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseX = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseY") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseY = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "processNoiseZ") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->processNoiseZ = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "xPositionDVL") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->xPositionDVL = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "yPositionDVL") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->yPositionDVL = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "zPositionDVL") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->zPositionDVL = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "yawRotationDVL") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->yawRotationDVL = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "xPositionIMU") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->xPositionIMU = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "yPositionIMU") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->yPositionIMU = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "zPositionIMU") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->zPositionIMU = param.as_double();
                    result.successful = true;
                }
            }
            if (param.get_name() == "yawRotationIMU") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    this->yawRotationIMU = param.as_double();
                    result.successful = true;
                }
            }
        }
        return result;
    }


    void imuCallbackHelper(const sensor_msgs::msg::Imu::SharedPtr msg) {

//        std::cout << "takin IMU message"<< std::endl;
        Eigen::Matrix3d transformationX180DegreeRotationMatrix;
//        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));

        transformationX180DegreeRotationMatrix = rotation_vector2.toRotationMatrix();

        Eigen::Vector3d acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                     msg->linear_acceleration.z);
        acceleration = transformationX180DegreeRotationMatrix * acceleration;

        Eigen::Vector3d rotationVel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        rotationVel = transformationX180DegreeRotationMatrix * rotationVel;


        sensor_msgs::msg::Imu newMsg;
        newMsg.header = msg->header;
        newMsg.angular_velocity.x = rotationVel.x();
        newMsg.angular_velocity.y = rotationVel.y();
        newMsg.angular_velocity.z = rotationVel.z();

        newMsg.linear_acceleration.x = acceleration.x();
        newMsg.linear_acceleration.y = acceleration.y();
        newMsg.linear_acceleration.z = acceleration.z();
        Eigen::Quaterniond rotationRP(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        Eigen::Vector3d rollPitchYaw = this->getRollPitchYaw(rotationRP.inverse());
        // @TODO create low pass filter
        double rollIMUACCEL = atan2(-msg->linear_acceleration.y, msg->linear_acceleration.z);
        double pitchIMUACCEL = atan2(msg->linear_acceleration.x,
                                     sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y +
                                          msg->linear_acceleration.z * msg->linear_acceleration.z));

        rotationRP = getQuaternionFromRPY(rollIMUACCEL, pitchIMUACCEL, 0);
//        rotationRP = getQuaternionFromRPY(-rollIMUACCEL, pitchIMUACCEL, 0);

        newMsg.orientation.x = rotationRP.x();//not sure if correct
        newMsg.orientation.y = rotationRP.y();//not sure if correct
        newMsg.orientation.z = rotationRP.z();//not sure if correct
        newMsg.orientation.w = rotationRP.w();//not sure if correct





        Eigen::Quaterniond tmpRot;
        tmpRot.x() = newMsg.orientation.x;
        tmpRot.y() = newMsg.orientation.y;
        tmpRot.z() = newMsg.orientation.z;
        tmpRot.w() = newMsg.orientation.w;

//        std::cout << msg->linear_acceleration.x<< " " << msg->linear_acceleration.y<< " " << msg->linear_acceleration.z << std::endl;

        currentEkf.predictionImu(newMsg.linear_acceleration.x, newMsg.linear_acceleration.y,
                                 newMsg.linear_acceleration.z,
                                 tmpRot, this->positionIMU,
                                 newMsg.header.stamp);

        Eigen::Vector3d euler = generalHelpfulTools::getRollPitchYaw(tmpRot);// roll pitch yaw

        //calculate roll pitch from IMU accel data
//        std::cout << "my Roll: "<< euler.x()*180/M_PI<< std::endl;
//        std::cout << "my Pitch: "<< euler.y()*180/M_PI<< std::endl;
//        std::cout << "my Yaw: "<< euler.z()*180/M_PI<< std::endl;
        currentEkf.updateIMU(euler.x(), euler.y(), newMsg.angular_velocity.x, newMsg.angular_velocity.y,
                             newMsg.angular_velocity.z, tmpRot, newMsg.header.stamp);
        pose currentStateEkf = currentEkf.getState();
        geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.frame_id = "map_ned";
        poseMsg.pose.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = currentEkf.getRotationVector();
        poseMsg.pose.pose.orientation.x = rotDiff.x();
        poseMsg.pose.pose.orientation.y = rotDiff.y();
        poseMsg.pose.pose.orientation.z = rotDiff.z();
        poseMsg.pose.pose.orientation.w = rotDiff.w();
//        std::cout << msg->header.stamp << std::endl;
        poseMsg.header.stamp = msg->header.stamp;
        this->publisherPoseEkf->publish(poseMsg);
        geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;
        twistMsg.header.frame_id = "map_ned";
        twistMsg.twist.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.twist.angular.z = currentStateEkf.angleVelocity.z();
        twistMsg.header.stamp = newMsg.header.stamp;
        this->publisherTwistEkf->publish(twistMsg);
    }


    void imuCallbackPX4(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        //change the orientation of the IMU message
        sensor_msgs::msg::Imu newMsg{};
        newMsg.linear_acceleration.x = msg->accelerometer_m_s2[0];
        newMsg.linear_acceleration.y = msg->accelerometer_m_s2[1];
        newMsg.linear_acceleration.z = msg->accelerometer_m_s2[2];

        newMsg.angular_velocity.x = msg->gyro_rad[0];
        newMsg.angular_velocity.y = msg->gyro_rad[1];
        newMsg.angular_velocity.z = msg->gyro_rad[2];
        newMsg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();


        this->updateSlamMutex.lock();
        this->imuCallbackHelper(std::make_shared<sensor_msgs::msg::Imu>(newMsg));
        this->updateSlamMutex.unlock();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        //change the orientation of the IMU message


        this->updateSlamMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
//        std::cout << "getting DVL message" << std::endl;
        if (!msg->report.velocity_valid || msg->report.status != 0) {
            //if we don't know anything, the ekf should just go to 0, else the IMU gives direction.
            this->currentEkf.updateDVL(0, 0, 0, this->rotationOfDVL, this->positionDVL, rclcpp::Time(msg->timestamp));
        } else {
            this->currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz, this->rotationOfDVL,
                                       this->positionDVL,
                                       rclcpp::Time(msg->timestamp));
        }
        return;
    }

    void DVLCallbackDVL(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackSimulationHelper(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->currentEkf.updateDVL(msg->vector.x, msg->vector.y, msg->vector.z, Eigen::Quaterniond(1, 0, 0, 0),
                                   this->positionDVL,
                                   msg->header.stamp);
    }

    void DVLCallbackSimulation(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackSimulationHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackMavrosHelper(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->currentEkf.updateDVL(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                                   Eigen::Quaterniond(1, 0, 0, 0), this->positionDVL,
                                   msg->header.stamp);
    }

    void DVLCallbackMavros(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackMavrosHelper(msg);
        this->updateSlamMutex.unlock();
    }


    bool resetEKF(const std::shared_ptr<asvcommonmsg::srv::ResetEkf::Request> req,
                  std::shared_ptr<asvcommonmsg::srv::ResetEkf::Response> res) {
        this->updateSlamMutex.lock();
        this->currentEkf.resetToPos(req->x_pos, req->y_pos, req->yaw, req->reset_covariances);
        this->updateSlamMutex.unlock();
        res->reset_done = true;
        return true;
    }

    void headingCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

        this->updateSlamMutex.lock();
        this->headingHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void headingHelper(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->currentEkf.updateHeading(msg->vector.z, msg->header.stamp);
    }


    Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
        tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
        tf2::Matrix3x3 m(tmp);
        double r, p, y;
        m.getRPY(r, p, y);
        Eigen::Vector3d returnVector(r, p, y);
        return returnVector;
    }

    Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw) {
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
        tf2::Quaternion qtf2;
        qtf2.setRPY(roll, pitch, yaw);
        Eigen::Quaterniond q;
        q.x() = qtf2.x();
        q.y() = qtf2.y();
        q.z() = qtf2.z();
        q.w() = qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        return q;
    };

    void timer_callback() {
        px4_msgs::msg::VehicleOdometry odometryPose;

        pose currentEKFPose = this->currentEkf.getState();


        odometryPose.pose_frame = odometryPose.POSE_FRAME_NED;
        odometryPose.velocity_frame = odometryPose.VELOCITY_FRAME_NED;

        odometryPose.position[0] = currentEKFPose.position.x();
        odometryPose.position[1] = currentEKFPose.position.y();
        odometryPose.position[2] = currentEKFPose.position.z();
        Eigen::Quaterniond currentQuat = generalHelpfulTools::getQuaternionFromRPY(0, 0, currentEKFPose.rotation[2]);
        odometryPose.q[0] = currentQuat.w();
        odometryPose.q[1] = currentQuat.x();
        odometryPose.q[2] = currentQuat.y();
        odometryPose.q[3] = currentQuat.z();

        odometryPose.velocity[0] = NAN;
        odometryPose.velocity[1] = NAN;
        odometryPose.velocity[2] = NAN;

        odometryPose.angular_velocity[0] = NAN;
        odometryPose.angular_velocity[1] = NAN;
        odometryPose.angular_velocity[2] = NAN;

        odometryPose.position_variance[0] = NAN;
        odometryPose.position_variance[1] = NAN;
        odometryPose.position_variance[2] = NAN;

        odometryPose.orientation_variance[0] = NAN;
        odometryPose.orientation_variance[1] = NAN;
        odometryPose.orientation_variance[2] = NAN;

        odometryPose.velocity_variance[0] = NAN;
        odometryPose.velocity_variance[1] = NAN;
        odometryPose.velocity_variance[2] = NAN;

        odometryPose.velocity[0] = NAN;
        odometryPose.velocity[1] = NAN;
        odometryPose.velocity[2] = NAN;

        odometryPose.timestamp = (rclcpp::Time() - this->timeAtStart).nanoseconds() * 0.001;

        this->publisherVehicleOdometry->publish(odometryPose);
    }

public:
    pose getPoseOfEKF() {
        return this->currentEkf.getState();
    }


    Eigen::Quaterniond getQuaternionForMavrosFromRPY(double roll, double pitch, double yaw) {
        return generalHelpfulTools::getQuaternionFromRPY(roll, -pitch, -yaw + M_PI / 2);
    }

    Eigen::Vector3f getPositionForMavrosFromXYZ(Eigen::Vector3f inputPosition) {
        Eigen::AngleAxisf rotation_vector180X(180.0 / 180.0 * 3.14159, Eigen::Vector3f(1, 0, 0));
        Eigen::AngleAxisf rotation_vector90Z(90.0 / 180.0 * 3.14159, Eigen::Vector3f(0, 0, 1));
        Eigen::Vector3f positionRotatedForMavros =
                rotation_vector90Z.toRotationMatrix() * rotation_vector180X.toRotationMatrix() * inputPosition;
        return positionRotatedForMavros;

    }
};

int main(int argc, char **argv) {


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosClassEKF>());


    rclcpp::shutdown();

    return (0);
}
