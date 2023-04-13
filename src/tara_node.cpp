//
// Created by tp on 08/04/23.
//
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "Tara.h"

namespace tara_ros {
    class TaraNode : public rclcpp::Node {
    public:
        TaraNode();
        void initCamera();

        void run();
        void stop();
    private:
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoPublisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftImagePublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightImagePublisher;

        rclcpp::TimerBase::SharedPtr cameraInfoTimer;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

        Tara::Disparity disparity;
        TaraRev g_eRev;

        sensor_msgs::msg::CameraInfo leftInfo;
        sensor_msgs::msg::CameraInfo rightInfo;

        std::mutex imuQueueMux;
        std::condition_variable imuQueueCond;
        std::queue<IMUDATAOUTPUT_TypeDef *> imuQueue;
        std::future<BOOL> imuReaderFuture;

        volatile bool running = true;

        void cameraInfoPublisherTimerCallback();
        int imuPublisherAsync();
    };

    TaraNode::TaraNode() : Node("tara_node") {
        declare_parameter<bool>("rectify_images", true);

        leftInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>("left_info", 1);
        rightInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>("right_info", 1);
        leftImagePublisher = create_publisher<sensor_msgs::msg::Image>("left_image", 1);
        rightImagePublisher = create_publisher<sensor_msgs::msg::Image>("right_image", 1);

        imuPublisher = create_publisher<sensor_msgs::msg::Imu>("imu", 100);

        declare_parameters<int>("", {
                {"usb_device_id", -1},
                {"image_width", 0},
                {"image_height", 0}
        });

        initCamera();
    }

    void TaraNode::initCamera() {
        int usbDeviceId = -1;
        get_parameter("usb_device_id", usbDeviceId);
        disparity.SetDeviceID(usbDeviceId);

        int width = 0;
        int height = 0;
        get_parameter("image_width", width);
        get_parameter("image_height", height);
        disparity.SetResolution(width, height);

        if (!disparity.InitCamera(false, false, false)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Camera Initialisation Failed!");
            rclcpp::shutdown();
            return;
        }

        disparity.SetAutoExposure();
        SetAutoExposureStereo();

        if (!GetRevision(&g_eRev)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Init GetRevision Failed");
            rclcpp::shutdown();
            return;
        }

        //Configuring IMU rates
        IMUCONFIG_TypeDef lIMUConfig {
                IMU_ACC_GYRO_ENABLE,
                IMU_ACC_X_Y_Z_ENABLE,
                IMU_ACC_SENS_8G,
                IMU_GYRO_X_Y_Z_ENABLE,
                IMU_GYRO_SENS_500DPS,
                IMU_ODR_208HZ
        };

        //Setting the configuration using HID command
        BOOL uStatus = SetIMUConfig(lIMUConfig);
        if(!uStatus) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "SetIMUConfig Failed");
            rclcpp::shutdown();
            return;
        }

        leftInfo.header.frame_id = "";
        rightInfo.header.frame_id = "";
        leftInfo.header.stamp = now();
        rightInfo.header.stamp = leftInfo.header.stamp;
        leftInfo.width = rightInfo.width = width;
        leftInfo.height = rightInfo.height = height;
        for (int i = 0; i < 12; i++) {
            leftInfo.p[i] = disparity._TaraCamParameters.PRect1.at<double>(i);
            rightInfo.p[i] = disparity._TaraCamParameters.PRect2.at<double>(i);
        }

        // Publish camera info every 2 seconds.
        cameraInfoTimer = create_wall_timer(std::chrono::seconds(2), std::bind(&TaraNode::cameraInfoPublisherTimerCallback, this));
    }

    void TaraNode::cameraInfoPublisherTimerCallback() {
        leftInfoPublisher->publish(leftInfo);
        rightInfoPublisher->publish(rightInfo);
    }

    int TaraNode::imuPublisherAsync() {
        IMUDATAOUTPUT_TypeDef *imuSample = NULL;

        sensor_msgs::msg::Imu imuMessage;
        imuMessage.header.frame_id = "imu";

        while (running) {
            std::unique_lock<std::mutex> lk(imuQueueMux);
            if (!imuQueueCond.wait_for(lk, std::chrono::seconds(1),[&]{return !imuQueue.empty();})) {
                continue;
            }
            imuSample = imuQueue.front();
            imuQueue.pop();
            lk.unlock();

            // Only milliseconds accuracy!?
            rclcpp::Time stamp(imuSample->millisecond / 1000, imuSample->millisecond % 1000);
            imuMessage.header.stamp = now();

            // imuSample contains linear acceleration in CM/s?!, and angular rate in DPS. (at what scale??)
            imuMessage.linear_acceleration.x = imuSample->accX / 100;
            imuMessage.linear_acceleration.y = imuSample->accY / 100;
            imuMessage.linear_acceleration.z = imuSample->accZ / 100;
            imuMessage.angular_velocity.x = imuSample->gyroX * M_PI / 180;
            imuMessage.angular_velocity.y = imuSample->gyroY * M_PI / 180;
            imuMessage.angular_velocity.z = imuSample->gyroZ * M_PI / 180;
            imuPublisher->publish(imuMessage);
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "imuPublisherAsync exiting..");
        return imuSample ? imuSample->imuSampleIndex : 0;
    }

    void TaraNode::stop() {
        if (running) {
            running = false;

            IMUDATAINPUT_TypeDef imuInputDisable = {IMU_CONT_UPDT_DIS, IMU_AXES_VALUES_MIN};
            if (!ControlIMUCapture(&imuInputDisable)) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ControlIMUCapture Failed");
            }

            if (imuReaderFuture.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Timeout waiting for IMU reader to finish.");
            } else if (!imuReaderFuture.get()) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "IMU reader finish error!!");
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "IMU reader stopped.");
        }
    }

    void TaraNode::run() {
        // IMU...
        std::future<int> imuWriterFuture;

        imuReaderFuture = std::async(&GetIMUValueBuffer, &imuQueueMux, &imuQueueCond, &imuQueue);
        //Configuring IMU update mode
        IMUDATAINPUT_TypeDef imuInput = {IMU_CONT_UPDT_EN, IMU_AXES_VALUES_MIN};
        if(!ControlIMUCapture(&imuInput)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ControlIMUCapture Failed!");
            rclcpp::shutdown();
            running = false;
            return;
        }
        imuWriterFuture = std::async(std::bind(&TaraNode::imuPublisherAsync, this));

        // Images...
        cv::Mat leftImage;
        cv::Mat rightImage;
        timeval timeVal;

        sensor_msgs::msg::Image leftImageMsg;
        get_parameter("image_width", leftImageMsg.width);
        get_parameter("image_height", leftImageMsg.height);
        leftImageMsg.is_bigendian = false;
        leftImageMsg.step = leftImageMsg.width;
        sensor_msgs::msg::Image rightImageMsg(leftImageMsg);
        leftImageMsg.header.frame_id = "left_image";
        rightImageMsg.header.frame_id = "right_image";

        int m = 0;
        while(running) {
            if (!disparity.GrabFrame(&leftImage, &rightImage, &timeVal,true)) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Camera disparity.GrabFrame() Failed!");
                rclcpp::shutdown();
                break;
            }

            rclcpp::Time stamp(timeVal.tv_sec, timeVal.tv_usec * 1000);
            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, leftImage).toImageMsg(leftImageMsg);
            leftImageMsg.header.stamp = stamp;
            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, rightImage).toImageMsg(rightImageMsg);
            rightImageMsg.header.stamp = stamp;

            // Is the messages accessed after publish() returns?
            leftImagePublisher->publish(leftImageMsg);
            rightImagePublisher->publish(rightImageMsg);
        }

        stop();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Image Publishing exiting..");
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const std::shared_ptr<tara_ros::TaraNode>& node =
            std::make_shared<tara_ros::TaraNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_executor = [&executor, node]() {
        executor.spin();
        node->stop();
    };
    std::thread executor_thread(spin_executor);
    node->run();
    executor_thread.join();
    rclcpp::shutdown();
}