#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "image_transport/image_transport.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Depth AI
#include <depthai_ros_msgs/msg/tracked_features.hpp>

// Spectacular AI
#include <spectacularAI/vio.hpp>

#include "ros2_plugin.hpp"
#include "occupancy_grid.hpp"

namespace {
static double NANOS_TO_SECONDS = 1e-9;
static double SECONDS_TO_NANOS = 1e9;
static size_t IMU_QUEUE_SIZE = 100;
static size_t ODOM_QUEUE_SIZE = 100;
static uint32_t CAM_QUEUE_SIZE = 10;

//  message_filters::Synchronize duration for synchronizing different camera inputs
static rclcpp::Duration CAMERA_SYNC_INTERVAL = rclcpp::Duration(0, 10 * 1e6);

const char *oakYaml =
R"(imuAnomalyFilterGyroEnabled: True
imuStationaryEnabled: True
visualR: 0.01
skipFirstNFrames: 10
)";

const char *baseYaml =
R"(trackChiTestOutlierR: 3
trackOutlierThresholdGrowthFactor: 1.3
hybridMapSize: 2
sampleSyncLag: 1
outputCameraPose: False
trackingStatusInitMinSeconds: 0.5
)";

const char *slamYaml =
R"(useSlam: True
delayFrames: 2
noWaitingForResults: True
maxSlamResultQueueSize: 2
)";

constexpr size_t DIM = 4;

template <class M> M matMulGeneric(const M &a, const M &b) {
    M r;
    for (size_t i = 0; i < DIM; ++i) {
        for (size_t j = 0; j < DIM; ++j) {
            r.at(i).at(j) = 0;
            for (size_t k = 0; k < DIM; ++k) {
                r.at(i).at(j) += a.at(i).at(k) * b.at(k).at(j);
            }
        }
    }
    return r;
}

spectacularAI::Matrix4d matrixMul(const spectacularAI::Matrix4d &a, const spectacularAI::Matrix4d &b) {
    return matMulGeneric(a, b);
}

using Matrix4f = std::array<std::array<float, 4>, 4>;

spectacularAI::Vector3f matVectorMult(const Matrix4f &m, const spectacularAI::Vector3f &v) {
    return {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3],
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3],
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]
    };
}

Matrix4f matrixToFloat(const spectacularAI::Matrix4d &doubleArray) {
    std::array<std::array<float, 4>, 4> floatArray;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            floatArray[i][j] = static_cast<float>(doubleArray[i][j]);
        }
    }
    return floatArray;
}

spectacularAI::Matrix4d matrixConvert(const std::array<double, 9> &m) {
    return {{
        {m[0], m[1], m[2], 0.0},
        {m[3], m[4], m[5], 0.0},
        {m[6], m[7], m[8], 0.0},
        {0.0, 0.0, 0.0, 1.0}
    }};
}

spectacularAI::Matrix4d matrixConvert(geometry_msgs::msg::TransformStamped tf) {
    auto t = tf.transform.translation;
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    auto r = tf2::Matrix3x3(q);
    return {{
        {r[0][0], r[0][1], r[0][2], t.x},
        {r[1][0], r[1][1], r[1][2], t.y},
        {r[2][0], r[2][1], r[2][2], t.z},
        {0.0, 0.0, 0.0, 1.0}
    }};
}

double stampToSeconds(std_msgs::msg::Header::Header_::_stamp_type stamp) {
    return (int64_t)stamp.sec + (int64_t)stamp.nanosec * NANOS_TO_SECONDS;
}

std_msgs::msg::Header::Header_::_stamp_type secondsToStamp(double seconds) {
    std_msgs::msg::Header::Header_::_stamp_type stamp;
    stamp.sec = std::floor(seconds);
    stamp.nanosec = (seconds - stamp.sec) * SECONDS_TO_NANOS;
    return stamp;
}

std::string toJson(spectacularAI::Matrix4d m) {
    std::ostringstream ss;
    ss << std::setprecision(18);
    ss << "[";
    ss << "[" << m[0][0] << "," << m[0][1] << "," << m[0][2] << "," << m[0][3] << "],";
    ss << "[" << m[1][0] << "," << m[1][1] << "," << m[1][2] << "," << m[1][3] << "],";
    ss << "[" << m[2][0] << "," << m[2][1] << "," << m[2][2] << "," << m[2][3] << "],";
    ss << "[" << m[3][0] << "," << m[3][1] << "," << m[3][2] << "," << m[3][3] << "]";
    ss << "]";
    return ss.str();
}

geometry_msgs::msg::TransformStamped poseToTransformStampped(spectacularAI::Pose pose, std::string frameId, std::string childFrameId) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = secondsToStamp(pose.time);
    tf.header.frame_id = frameId;
    tf.child_frame_id = childFrameId;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation.x = pose.orientation.x;
    tf.transform.rotation.y = pose.orientation.y;
    tf.transform.rotation.z = pose.orientation.z;
    tf.transform.rotation.w = pose.orientation.w;
    return tf;
}

nav_msgs::msg::Odometry outputToOdometryMsg(spectacularAI::VioOutputPtr output, std::string frameId, std::string childFrameId) {
    auto& pose = output->pose;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = secondsToStamp(pose.time);
    odom.header.frame_id = frameId;
    odom.child_frame_id = childFrameId;

    odom.pose.pose.position.x = pose.position.x;
    odom.pose.pose.position.y = pose.position.y;
    odom.pose.pose.position.z = pose.position.z;
    odom.pose.pose.orientation.x = pose.orientation.x;
    odom.pose.pose.orientation.y = pose.orientation.y;
    odom.pose.pose.orientation.z = pose.orientation.z;
    odom.pose.pose.orientation.w = pose.orientation.w;

    // Row-major representation of the 6x6 covariance matrix
    auto& pc = output->positionCovariance;
    odom.pose.covariance = {
        pc[0][0], pc[0][1], pc[0][2], 0, 0, 0,
        pc[1][0], pc[1][1], pc[1][2], 0, 0, 0,
        pc[2][0], pc[2][1], pc[2][2], 0, 0, 0,
        0, 0, 0,                      0, 0, 0,
        0, 0, 0,                      0, 0, 0,
        0, 0, 0,                      0, 0, 0
    };

    odom.twist.twist.linear.x = output->velocity.x;
    odom.twist.twist.linear.y = output->velocity.y;
    odom.twist.twist.linear.z = output->velocity.z;
    odom.twist.twist.angular.x = output->angularVelocity.x;
    odom.twist.twist.angular.y = output->angularVelocity.y;
    odom.twist.twist.angular.z = output->angularVelocity.z;

    // Row-major representation of the 6x6 covariance matrix
    auto& vc = output->velocityCovariance;
    odom.twist.covariance = {
        vc[0][0], vc[0][1], vc[0][2], 0, 0, 0,
        vc[1][0], vc[1][1], vc[1][2], 0, 0, 0,
        vc[2][0], vc[2][1], vc[2][2], 0, 0, 0,
        0, 0, 0,                      0, 0, 0,
        0, 0, 0,                      0, 0, 0,
        0, 0, 0,                      0, 0, 0
    };

    return odom;
}

using StereoCameraPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

using StereoDepthCameraPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, depthai_ros_msgs::msg::TrackedFeatures>;

using StereoCameraSynchronizer = message_filters::Synchronizer<StereoCameraPolicy>;
using StereoDepthCameraSynchronizer = message_filters::Synchronizer<StereoDepthCameraPolicy>;

#pragma pack(push)
#pragma pack(1)
struct RosPointWithColor {
    spectacularAI::Vector3f pos;
    uint8_t rgba[4];
};
#pragma pack(pop)


} // namespace

namespace spectacularAI {
namespace ros2 {

class Ros2WrapperImpl : public Ros2Wrapper {
public:
    Ros2WrapperImpl() : Ros2Wrapper("spectacularAI"),
            nodeHandle(std::shared_ptr<Ros2WrapperImpl>(this, [](auto *) {})), // create the shared_ptr node handle for this node.
            imageTransport(nodeHandle), vioInitDone(false) {

        imuFrameId = declareAndReadParameterString("imu_frame_id", "");
        worldFrameId = declareAndReadParameterString("world_frame_id", "");
        cam0FrameId = declareAndReadParameterString("cam0_frame_id", "");
        cam1FrameId = declareAndReadParameterString("cam1_frame_id", "");
        depthScale = declareAndReadParameterDouble("depth_scale", 1.0 / 1000.0);
        recordingFolder = declareAndReadParameterString("recording_folder", "");
        enableMapping = declareAndReadParameterBool("enable_mapping", true);
        enableOccupancyGrid = declareAndReadParameterBool("enable_occupancy_grid", false);
        cameraInputType = declareAndReadParameterString("camera_input_type", "stereo_depth_features");

        transformListenerBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transformListener = std::make_shared<tf2_ros::TransformListener>(*transformListenerBuffer);
        transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "input/imu",
            IMU_QUEUE_SIZE,
            std::bind(&Ros2WrapperImpl::imuCallback, this, std::placeholders::_1));

        odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("output/odometry", ODOM_QUEUE_SIZE);
        if (enableOccupancyGrid) occupancyGridPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("output/occupancyGrid", ODOM_QUEUE_SIZE);
        if (enableMapping) pointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", ODOM_QUEUE_SIZE);

        if (cameraInputType == "stereo") {
            cameraInfo0Subscription.subscribe(this, "/input/cam0/camera_info");
            cameraInfo1Subscription.subscribe(this, "/input/cam1/camera_info");
            cameraImage0Subscription.subscribe(this, "/input/cam0/image_rect");
            cameraImage1Subscription.subscribe(this, "/input/cam1/image_rect");

            stereoCameraSynchronizer = std::make_unique<StereoCameraSynchronizer>(StereoCameraPolicy(CAM_QUEUE_SIZE),
                cameraInfo0Subscription, cameraInfo1Subscription,
                cameraImage0Subscription, cameraImage1Subscription);
            stereoCameraSynchronizer->setMaxIntervalDuration(CAMERA_SYNC_INTERVAL);
            stereoCameraSynchronizer->registerCallback(std::bind(&Ros2WrapperImpl::stereoCameraCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        } else if (cameraInputType == "stereo_depth_features") {
            cameraInfo0Subscription.subscribe(this, "/input/cam0/camera_info");
            cameraInfo1Subscription.subscribe(this, "/input/cam1/camera_info");
            cameraImage0Subscription.subscribe(this, "/input/cam0/image_rect");
            cameraImage1Subscription.subscribe(this, "/input/cam1/image_rect");
            cameraDepthSubscription.subscribe(this, "/input/depth/image");
            depthaiTrackedFeaturesCam0Subscription.subscribe(this, "/input/cam0/features");

            // TODO: cameraImage1Subscription shouldn't be needed, but Synchronizer failed to produce output without it
            stereoDepthCameraSynchronizer = std::make_unique<StereoDepthCameraSynchronizer>(StereoDepthCameraPolicy(CAM_QUEUE_SIZE),
                cameraInfo0Subscription, cameraInfo1Subscription,
                cameraImage0Subscription, cameraImage1Subscription,
                cameraDepthSubscription, depthaiTrackedFeaturesCam0Subscription);
            stereoDepthCameraSynchronizer->setMaxIntervalDuration(CAMERA_SYNC_INTERVAL);
            stereoDepthCameraSynchronizer->registerCallback(std::bind(&Ros2WrapperImpl::stereoDepthCameraCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported camera_input_type: %s", cameraInputType.c_str());
        }
    }

private:
    std::string createConfigYaml() {
        std::ostringstream oss;
        oss << baseYaml;
        oss << slamYaml;
        oss << oakYaml;

        oss << "alreadyRectified: True\n";


        bool useFeatureTracker = cameraInputType == "stereo_depth_features";

        if (useFeatureTracker) {
            // Not good with non-depth grayscale video inputs.
            oss << "depthErrorScale: 0.1\n";
            oss << "keyframeCandidateInterval: 0\n";
        } else {
            oss << "keyframeCandidateInterval: 6\n";
        }

        if (enableMapping) {
            oss << "computeStereoPointCloud: True\n";
            oss << "computeDenseStereoDepthKeyFramesOnly: True\n";
            oss << "stereoPointCloudStride: 5\n";
            oss << "stereoPointCloudMinDepth: " << minDepth << "\n";
        }

        return oss.str();
    }

    std::string rosToSaiDistortionModel(std::string distortion_model) {
        // Spectacular AI supported models pass through
        if (distortion_model == "brown-conrady"
            || distortion_model == "pinhole"
            || distortion_model == "kannala-brandt4"
            || distortion_model == "kannala-brandt18"
            || distortion_model == "omnidir") return distortion_model;
        // Match ROS names to SAI names
        if (distortion_model == "plumb_bob") return "unknown"; // TODO: Is this also brown-conrady with fewer params?
        if (distortion_model == "rational_polynomial") return "brown-conrady";
        if (distortion_model == "equidistant") return "unknown";
        // Unsupported model
        RCLCPP_WARN(this->get_logger(), "Unsupported camera model: %s", distortion_model.c_str());
        return "unknown";
    }

    bool cameraInfoToCalibrationJson(const std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> intrinsics, const std::vector<geometry_msgs::msg::TransformStamped> extrinsics, bool isRectified, std::string &calibrationJson) {
        // https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg
        std::ostringstream calib;
        calib << std::setprecision(18);
        calib << "{";
        calib << "\"cameras\": [";
        for (size_t i = 0; i < intrinsics.size(); i++) {
            auto& intr = intrinsics[i];
            auto& extr = extrinsics[i];

            std::string model = rosToSaiDistortionModel(intr->distortion_model);
            spectacularAI::Matrix4d imuToCamera = matrixConvert(extr);
            double fx, fy, px, py;

            if (isRectified) {
                // Projection/camera matrix
                //     [fx'  0  cx' Tx]
                // P = [ 0  fy' cy' Ty]
                //     [ 0   0   1   0]
                fx = intr->p[0];
                fy = intr->p[5];
                px = intr->p[2];
                py = intr->p[6];

                spectacularAI::Matrix4d rectificationRotation = matrixConvert(intr->r);
                imuToCamera = matrixMul(rectificationRotation, imuToCamera);
                model = "pinhole";
            } else {
                // Intrinsic camera matrix for the raw (distorted) images.
                //     [fx  0 cx]
                // K = [ 0 fy cy]
                //     [ 0  0  1]
                fx = intr->k[0];
                fy = intr->k[4];
                px = intr->k[2];
                py = intr->k[5];
            }

            if (model == "unknown") return false;

            calib << "{";
            calib << "\"focalLengthX\":" << fx << ",";
            calib << "\"focalLengthY\":" << fy << ",";
            calib << "\"principalPointX\":" << px << ",";
            calib << "\"principalPointY\":" << py << ",";
            calib << "\"imageHeight\":" << intr->height << ",";
            calib << "\"imageWidth\":" << intr->width << ",";
            calib << "\"imuToCamera\": " << toJson(imuToCamera) << ",";
            if (model != "pinhole") {
                calib << "\"distortionCoefficients\": [";
                for (size_t j = 0; j < intr->d.size(); j++) {
                    calib << intr->d[j];
                    if (j + 1 != intr->d.size()) calib << ",";
                }
                calib << "],";
            }
            calib << "\"model\": \"" << model << "\"";
            calib << "}";
            if (i + 1 != intrinsics.size()) calib << ",";
        }
        calib << "]}";
        calibrationJson = calib.str();
        return true;
    }

    bool rosEncodingToColorFormat(const std::string encoding, spectacularAI::ColorFormat &colorFormat) {
        colorFormat = spectacularAI::ColorFormat::NONE;
        if (encoding == sensor_msgs::image_encodings::RGB8) {
            colorFormat = spectacularAI::ColorFormat::RGB;
        } else if (encoding == sensor_msgs::image_encodings::BGR8) {
            colorFormat = spectacularAI::ColorFormat::BGR;
        } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
            colorFormat = spectacularAI::ColorFormat::RGBA;
        } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
            colorFormat = spectacularAI::ColorFormat::BGRA;
        } else if (encoding == sensor_msgs::image_encodings::MONO8) {
            colorFormat = spectacularAI::ColorFormat::GRAY;
        } else if (encoding == sensor_msgs::image_encodings::MONO16 || encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            colorFormat = spectacularAI::ColorFormat::GRAY16;
        }
        return colorFormat != spectacularAI::ColorFormat::NONE;
    }

    std::string declareAndReadParameterString(std::string name, std::string defaultValue) {
        this->declare_parameter<std::string>(name, defaultValue);
        return this->get_parameter(name).as_string();
    }

    double declareAndReadParameterDouble(std::string name, double defaultValue) {
        this->declare_parameter<double>(name, defaultValue);
        return this->get_parameter(name).as_double();
    }

    bool declareAndReadParameterBool(std::string name, bool defaultValue) {
        this->declare_parameter<bool>(name, defaultValue);
        return this->get_parameter(name).as_bool();
    }

    void imuCallback(const sensor_msgs::msg::Imu &msg) const {
        // RCLCPP_INFO(this->get_logger(), "Received: %s", msgToString(msg).c_str());
        if (!vioInitDone || !vioApi) return;
        double time = stampToSeconds(msg.header.stamp);
        vioApi->addGyro(time, {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z});
        vioApi->addAcc(time, {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z});
    }

    void stereoCameraCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo0, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo1,
        const sensor_msgs::msg::Image::ConstSharedPtr &img0, const sensor_msgs::msg::Image::ConstSharedPtr &img1) {

        if (!vioInitDone) {
            geometry_msgs::msg::TransformStamped imuToCam0, imuToCam1;
            try {
                imuToCam0 = transformListenerBuffer->lookupTransform(imuFrameId, cam0FrameId, tf2::TimePointZero);
                imuToCam1 = transformListenerBuffer->lookupTransform(imuFrameId, cam1FrameId, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get camera transforms: %s", ex.what());
                return;
            }

            // TODO: Support non rectified images, by ROS convention "image" topic is unrectified, "image_rect" is rectified
            constexpr bool IS_RECTIFIED = true;
            startVio({camInfo0, camInfo1}, {imuToCam0, imuToCam1}, IS_RECTIFIED);
        }

        if (vioInitDone && !vioApi) return;

        if (img0->width != img1->width || img0->height != img0->height) {
            RCLCPP_WARN(this->get_logger(), "Stereo image resolutions don't match");
            return;
        }

        spectacularAI::ColorFormat colorFormat;
        if (!rosEncodingToColorFormat(img0->encoding, colorFormat)) {
            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding");
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "Timestamp: %d, %d", img0->header.stamp.sec, img0->header.stamp.nanosec);

        double time = stampToSeconds(img0->header.stamp);
        vioApi->addFrameStereo(
            time,
            img0->width, img0->height,
            img0->data.data(), img1->data.data(),
            colorFormat
        );
    }

    void stereoDepthCameraCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo0, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo1,
        const sensor_msgs::msg::Image::ConstSharedPtr &img0, const sensor_msgs::msg::Image::ConstSharedPtr &img1,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth, const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr &features) {
        (void)img1;

        // RCLCPP_INFO(this->get_logger(), "Received all data: %f", stampToSeconds(img0->header.stamp));

        if (!vioInitDone) {
            geometry_msgs::msg::TransformStamped imuToCam0, imuToCam1;
            try {
                imuToCam0 = transformListenerBuffer->lookupTransform(imuFrameId, cam0FrameId, tf2::TimePointZero);
                imuToCam1 = transformListenerBuffer->lookupTransform(imuFrameId, cam1FrameId, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get camera transforms: %s", ex.what());
                return;
            }

            // TODO: Support non rectified images, by ROS convention "image" topic is unrectified, "image_rect" is rectified
            constexpr bool IS_RECTIFIED = true;
            startVio({camInfo0, camInfo1}, {imuToCam0, imuToCam1}, IS_RECTIFIED);
        }

        if (vioInitDone && !vioApi) return;

        spectacularAI::ColorFormat colorFormat;
        spectacularAI::ColorFormat depthFormat;
        if (!rosEncodingToColorFormat(img0->encoding, colorFormat)) {
            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", img0->encoding.c_str());
            return;
        }
        if (!rosEncodingToColorFormat(depth->encoding, depthFormat)) {
            RCLCPP_WARN(this->get_logger(), "Unsupported depth image encoding: %s", depth->encoding.c_str());
            return;
        }

        monoFeatures.clear();
        monoFeatures.resize(features->features.size());
        for (auto& ft : features->features) {
            monoFeatures.push_back({ int(ft.id), { float(ft.position.x), float(ft.position.y) } });
        }

        double time = stampToSeconds(img0->header.stamp);

        bool isSlamKf = frameNumber % 6 == 0; // TODO: Make configurable
        vioApi->addFrameStereoDepth(
            time,
            img0->width, img0->height,
            monoFeatures.data(),
            monoFeatures.size(),
            depth->data.data(),
            depthFormat,
            isSlamKf ? img0->data.data() : nullptr,
            nullptr,
            isSlamKf ? colorFormat : spectacularAI::ColorFormat::NONE,
            depthScale);

        frameNumber++;
    }

    void vioOutputCallback(spectacularAI::VioOutputPtr vioOutput) {
        if (vioOutput->status == spectacularAI::TrackingStatus::TRACKING) {
            transformBroadcaster->sendTransform(poseToTransformStampped(vioOutput->pose, worldFrameId, imuFrameId));
            odometryPublisher->publish(outputToOdometryMsg(vioOutput, worldFrameId, imuFrameId));
            // RCLCPP_INFO(this->get_logger(), "Output: %s", vioOutput->asJson().c_str());
        }
    }

    void mappingOutputCallback(spectacularAI::mapping::MapperOutputPtr mappingOutput) {
        if (latestKeyFrame == mappingOutput->updatedKeyFrames.back()) return;
        latestKeyFrame = mappingOutput->updatedKeyFrames.back();
        const auto &kf = mappingOutput->map->keyFrames.at(latestKeyFrame);
        if (!kf->pointCloud || kf->pointCloud->empty()) return;
        const auto &primary = kf->frameSet->primaryFrame;

        sensor_msgs::msg::PointCloud2 pc;
        pc.header.stamp = secondsToStamp(primary->cameraPose.pose.time);
        pc.header.frame_id = worldFrameId;

        std::string fieldNames[] = {"x", "y", "z", "rgb"};
        uint32_t totalOffset = 0;
        for (int i = 0; i < 4; i++) {
            sensor_msgs::msg::PointField f;
            f.name = fieldNames[i];
            f.offset = totalOffset;
            f.datatype = sensor_msgs::msg::PointField::FLOAT32; // 1 float == rgba8
            f.count = 1;
            pc.fields.push_back(f);
            totalOffset += sizeof(uint32_t);
        }

        const std::uint8_t* rgb24Data = kf->pointCloud->getRGB24Data();
        const spectacularAI::Vector3f* positionData = kf->pointCloud->getPositionData();
        Matrix4f camToWorld = matrixToFloat(primary->cameraPose.getCameraToWorldMatrix());
        bool hasColors = kf->pointCloud->hasColors();

        spectacularAI::Vector3f center = {
            (float)primary->cameraPose.pose.position.x, (float)primary->cameraPose.pose.position.y, (float)primary->cameraPose.pose.position.z
        };
        if (enableOccupancyGrid) {
            if (occupancyGrid.empty()) {
                occupancyGrid.reset(center, minDepth, maxDepth, cellSizeMeters, occupiedThreshold);
            }
            tempGrid.reset(center, minDepth, maxDepth, cellSizeMeters, occupiedThreshold); // TODO: Could use bounding box for visible area
        }

        pc.data.resize(sizeof(RosPointWithColor) * kf->pointCloud->size());
        RosPointWithColor* points = (RosPointWithColor*) pc.data.data();
        for (size_t i = 0; i < kf->pointCloud->size(); i++) {
            auto &p = points[i];
            p.pos = matVectorMult(camToWorld, positionData[i]);
            if (hasColors) {
                p.rgba[0] = rgb24Data[i * 3 + 0];
                p.rgba[1] = rgb24Data[i * 3 + 1];
                p.rgba[2] = rgb24Data[i * 3 + 2];
                p.rgba[3] = 255;
            } else {
                p.rgba[0] = 0;
                p.rgba[1] = 0;
                p.rgba[2] = 0;
                p.rgba[3] = 0;
            }
            if (enableOccupancyGrid) {
                float relativeZ = p.pos.z - center.z;
                if (relativeZ > minOccupancyHeight && relativeZ < maxOccupancyHeight) {
                    tempGrid.addPoint(p.pos);
                    // p.rgba[0] = 255;
                    // p.rgba[1] = 0;
                    // p.rgba[2] = 0;
                }
            }
        }

        pc.point_step = sizeof(RosPointWithColor);
        pc.height = 1;
        pc.width = kf->pointCloud->size();
        pc.row_step = pc.point_step * kf->pointCloud->size();
        pc.is_bigendian = false;
        pc.is_dense = true;
        pointCloudPublisher->publish(pc);

        if (enableOccupancyGrid && primary->image) {
            // Trace to all edge points to find unoccupied cells
            float cameraTestHeight = center.z + (maxOccupancyHeight + minOccupancyHeight) * 0.5;
            tempGrid.traceFreeSpace(center, *primary, cameraTestHeight);
            occupancyGrid.merge(tempGrid);

            nav_msgs::msg::OccupancyGrid og;
            og.header.stamp = pc.header.stamp;
            og.header.frame_id = pc.header.frame_id;

            og.info.map_load_time = og.header.stamp; // Seems redundant, duplicating header
            og.info.resolution = occupancyGrid.cellSize;
            og.info.width = occupancyGrid.width;
            og.info.height = occupancyGrid.height;

            og.info.origin.position.x = occupancyGrid.originX * occupancyGrid.cellSize;
            og.info.origin.position.y = occupancyGrid.originY * occupancyGrid.cellSize;
            og.info.origin.position.z = center.z;
            og.info.origin.orientation.x = 0.0f;
            og.info.origin.orientation.y = 0.0f;
            og.info.origin.orientation.z = 0.0f;
            og.info.origin.orientation.w = 1.0f;
            og.data = occupancyGrid.cells; // TODO: Copying entire map, perhaps occupancyGrid could internally wrap nav_msgs::msg::OccupancyGrid?
            occupancyGridPublisher->publish(og);
        }
    }

    void startVio(std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> intrinsics, std::vector<geometry_msgs::msg::TransformStamped> extrinsics, bool isRectified) {
        std::lock_guard<std::mutex> lock(vioStartup);
        if (vioInitDone) return;

        std::string calibrationJson;
        if (!cameraInfoToCalibrationJson(intrinsics, extrinsics, isRectified, calibrationJson)) {
            vioInitDone = true;
            return;
        }

        std::string config = createConfigYaml();

        // RCLCPP_INFO(this->get_logger(), "Calibration: %s", calibrationJson.c_str());
        // RCLCPP_INFO(this->get_logger(), "Configuration: %s", config.c_str());

        spectacularAI::Vio::Builder vioBuilder = spectacularAI::Vio::builder();
        vioBuilder.setCalibrationJSON(calibrationJson);
        vioBuilder.setConfigurationYAML(config);
        if (!recordingFolder.empty()) vioBuilder.setRecordingFolder(recordingFolder);
        if (enableMapping) vioBuilder.setMapperCallback(std::bind(&Ros2WrapperImpl::mappingOutputCallback, this, std::placeholders::_1));
        vioApi = vioBuilder.build();
        vioApi->setOutputCallback(std::bind(&Ros2WrapperImpl::vioOutputCallback, this, std::placeholders::_1));
        vioInitDone = true;
    }

    rclcpp::Node::SharedPtr nodeHandle;
    image_transport::ImageTransport imageTransport;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPublisher;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;

    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfo0Subscription;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfo1Subscription;
    message_filters::Subscriber<sensor_msgs::msg::Image> cameraImage0Subscription;
    message_filters::Subscriber<sensor_msgs::msg::Image> cameraImage1Subscription;
    message_filters::Subscriber<sensor_msgs::msg::Image> cameraDepthSubscription;
    message_filters::Subscriber<depthai_ros_msgs::msg::TrackedFeatures> depthaiTrackedFeaturesCam0Subscription;

    std::unique_ptr<tf2_ros::Buffer> transformListenerBuffer;
    std::shared_ptr<tf2_ros::TransformListener> transformListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

    std::unique_ptr<StereoCameraSynchronizer> stereoCameraSynchronizer;
    std::unique_ptr<StereoDepthCameraSynchronizer> stereoDepthCameraSynchronizer;

    std::unique_ptr<spectacularAI::Vio> vioApi;
    std::atomic_bool vioInitDone;
    std::mutex vioStartup;

    // Params
    std::string imuFrameId;
    std::string worldFrameId;
    std::string cam0FrameId;
    std::string cam1FrameId;
    double depthScale;
    std::string cameraInputType;
    std::string recordingFolder;
    bool enableMapping;

    int frameNumber = 0;
    int64_t latestKeyFrame = -1;
    std::vector<spectacularAI::MonocularFeature> monoFeatures;

    // Map
    bool enableOccupancyGrid;
    OccupancyGrid occupancyGrid;
    OccupancyGrid tempGrid;
    float maxDepth = 3.0; // Max distance from camera for occupancy map
    float minDepth = 0.2;
    float maxOccupancyHeight = 0.3; // Max height above device for point to be counted in occupancy map
    float minOccupancyHeight = 0.05; // Min height above device for point to be counted in occupancy map
    float cellSizeMeters = 0.07;
    int occupiedThreshold = 2;
};

std::unique_ptr<Ros2Wrapper> Ros2Wrapper::build() {
    return std::make_unique<Ros2WrapperImpl>();
}

Ros2Wrapper::~Ros2Wrapper() = default;

} // namespace ros2
} // namespace spectacularai
