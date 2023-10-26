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
#include "nav_msgs/msg/path.hpp"

// Depth AI
#include <depthai_ros_msgs/msg/tracked_features.hpp>

// Spectacular AI
#include <spectacularAI/vio.hpp>

// Helpers
#include "helpers.hpp"
#include "pose_helper.hpp"
#include "device_imu_calib.hpp"
#include "occupancy_grid.hpp"

namespace {
constexpr size_t IMU_QUEUE_SIZE = 1000;
constexpr size_t ODOM_QUEUE_SIZE = 1000;
constexpr uint32_t CAM_QUEUE_SIZE = 30;
constexpr size_t TRAJECTORY_LENGTH = 1000;
constexpr size_t UPDATE_TRAJECTORY_EVERY_NTH_POSE = 5;

// Group window for camera frames, should always be smaller than time between two frames
static const rclcpp::Duration CAMERA_SYNC_INTERVAL = rclcpp::Duration(0, 10 * 1e6);

// https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
static const rclcpp::QoS CAMERA_QOS = rclcpp::QoS(rclcpp::KeepLast(CAM_QUEUE_SIZE)).best_effort().durability_volatile();

const char *oakYaml =
R"(# set: wrapper-base
trackChiTestOutlierR: 3
trackOutlierThresholdGrowthFactor: 1.3
hybridMapSize: 2
sampleSyncLag: 1
trackingStatusInitMinSeconds: 0.5
cameraTrailLength: 12
cameraTrailHanoiLength: 8
delayFrames: 2
maxSlamResultQueueSize: 2
# set: oak-d
imuAnomalyFilterGyroEnabled: true
imuStationaryEnabled: true
visualR: 0.01
skipFirstNFrames: 10
# set: live
noWaitingForResults: true
# custom
useSlam: True
ffmpegVideoCodec: "libx264 -crf 15 -preset ultrafast"
)";


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

class TrajectoryPublisher {
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher;
    std::deque<geometry_msgs::msg::PoseStamped> poses;
    std::string frameId;
    int counter = 0;
public:
    TrajectoryPublisher(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher, std::string frameId)
        : publisher(publisher), frameId(frameId) {}

    void addPose(spectacularAI::Pose pose) {
        poses.push_front(poseToPoseStampped(pose, frameId));
        while (poses.size() > TRAJECTORY_LENGTH) poses.pop_back();
        if (counter % UPDATE_TRAJECTORY_EVERY_NTH_POSE == 0) {
            nav_msgs::msg::Path path;
            path.header.stamp = secondsToStamp(pose.time);
            path.header.frame_id = frameId;
            path.poses = std::vector<geometry_msgs::msg::PoseStamped>(poses.begin(), poses.end());
            publisher->publish(path);
        }
        counter++;
    }
};


} // namespace

namespace spectacularAI {
namespace ros2 {

class Node : public rclcpp::Node {
public:
    Node(const rclcpp::NodeOptions& options) : rclcpp::Node("spetacularAI", options), vioInitDone(false), receivedFrames(false) {

        deviceModel = declareAndReadParameterString("device_model", "");

        // Used for IMU-Camera extrinsics calibration
        imuFrameId = declareAndReadParameterString("imu_frame_id", "");
        cam0FrameId = declareAndReadParameterString("cam0_optical_frame_id", "");
        cam1FrameId = declareAndReadParameterString("cam1_optical_frame_id", "");

        // Output frames
        fixedFrameId = declareAndReadParameterString("fixed_frame_id", "map");
        odometryFrameId = declareAndReadParameterString("odometry_frame_id", "odom");
        baseLinkFrameId = declareAndReadParameterString("base_link_frame_id", "base_link");

        depthScale = declareAndReadParameterDouble("depth_scale", 1.0 / 1000.0);
        recordingFolder = declareAndReadParameterString("recording_folder", "");
        enableMapping = declareAndReadParameterBool("enable_mapping", true);
        enableOccupancyGrid = declareAndReadParameterBool("enable_occupancy_grid", false);
        cameraInputType = declareAndReadParameterString("camera_input_type", "stereo_depth_features");

        outputOnImuSamples = declareAndReadParameterBool("output_on_imu_samples", false);

        maxOdomFreq = declareAndReadParameterDouble("max_odom_freq", 100);
        maxOdomCorrectionFreq = declareAndReadParameterDouble("max_odom_correction_freq", 10);

        overrideImuToCam0  = declareAndReadParameterString("imu_to_cam0", "");
        overrideImuToCam1  = declareAndReadParameterString("imu_to_cam1", "");

        bool separateOdomTf = declareAndReadParameterBool("separate_odom_tf", false);

        bool publishPaths = declareAndReadParameterBool("publish_paths", false);

        transformListenerBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transformListener = std::make_shared<tf2_ros::TransformListener>(*transformListenerBuffer);
        transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "input/imu",
            IMU_QUEUE_SIZE,
            std::bind(&Node::imuCallback, this, std::placeholders::_1));

        if (separateOdomTf) {
            poseHelper = std::make_unique<PoseHelper>(1.0 / maxOdomCorrectionFreq);
        }

        // odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("output/odometry", ODOM_QUEUE_SIZE);
        if (enableOccupancyGrid) occupancyGridPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("output/occupancyGrid", ODOM_QUEUE_SIZE);
        if (enableMapping) pointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", ODOM_QUEUE_SIZE);
        if (publishPaths) {
            odometryPathPublisher = std::make_unique<TrajectoryPublisher>(this->create_publisher<nav_msgs::msg::Path>("output/odometry_path", ODOM_QUEUE_SIZE), fixedFrameId);
            correctedPathPublisher = std::make_unique<TrajectoryPublisher>(this->create_publisher<nav_msgs::msg::Path>("output/corrected_path", ODOM_QUEUE_SIZE), fixedFrameId);
            vioPathPublisher = std::make_unique<TrajectoryPublisher>(this->create_publisher<nav_msgs::msg::Path>("output/vio_path", ODOM_QUEUE_SIZE), fixedFrameId);
        }

        if (cameraInputType == "stereo") {
            cameraInfo0Subscription.subscribe(this, "/input/cam0/camera_info", CAMERA_QOS.get_rmw_qos_profile());
            cameraInfo1Subscription.subscribe(this, "/input/cam1/camera_info", CAMERA_QOS.get_rmw_qos_profile());
            cameraImage0Subscription.subscribe(this, "/input/cam0/image_rect", CAMERA_QOS.get_rmw_qos_profile());
            cameraImage1Subscription.subscribe(this, "/input/cam1/image_rect", CAMERA_QOS.get_rmw_qos_profile());

            stereoCameraSynchronizer = std::make_unique<StereoCameraSynchronizer>(StereoCameraPolicy(CAM_QUEUE_SIZE),
                cameraInfo0Subscription, cameraInfo1Subscription,
                cameraImage0Subscription, cameraImage1Subscription);
            stereoCameraSynchronizer->setMaxIntervalDuration(CAMERA_SYNC_INTERVAL);
            stereoCameraSynchronizer->registerCallback(std::bind(&Node::stereoCameraCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        } else if (cameraInputType == "stereo_depth_features") {
            cameraInfo0Subscription.subscribe(this, "/input/cam0/camera_info", CAMERA_QOS.get_rmw_qos_profile());
            cameraInfo1Subscription.subscribe(this, "/input/cam1/camera_info", CAMERA_QOS.get_rmw_qos_profile());
            cameraImage0Subscription.subscribe(this, "/input/cam0/image_rect", CAMERA_QOS.get_rmw_qos_profile());
            cameraImage1Subscription.subscribe(this, "/input/cam1/image_rect", CAMERA_QOS.get_rmw_qos_profile());
            cameraDepthSubscription.subscribe(this, "/input/depth/image", CAMERA_QOS.get_rmw_qos_profile());
            depthaiTrackedFeaturesCam0Subscription.subscribe(this, "/input/cam0/features", CAMERA_QOS.get_rmw_qos_profile());

            stereoDepthCameraSynchronizer = std::make_unique<StereoDepthCameraSynchronizer>(StereoDepthCameraPolicy(CAM_QUEUE_SIZE),
                cameraInfo0Subscription, cameraInfo1Subscription,
                cameraImage0Subscription, cameraImage1Subscription,
                cameraDepthSubscription, depthaiTrackedFeaturesCam0Subscription);
            stereoDepthCameraSynchronizer->setMaxIntervalDuration(CAMERA_SYNC_INTERVAL);
            stereoDepthCameraSynchronizer->registerCallback(std::bind(&Node::stereoDepthCameraCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported camera_input_type: %s", cameraInputType.c_str());
        }
    }

private:
    std::string createConfigYaml() {
        std::ostringstream oss;
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

        if (outputOnImuSamples) {
            oss << "outputOnFrames: False\n";
        }

        return oss.str();
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

    void imuCallback(const sensor_msgs::msg::Imu &msg) {
        // RCLCPP_INFO(this->get_logger(), "Received: %s", msgToString(msg).c_str());
        double time = stampToSeconds(msg.header.stamp);
        if (imuStartTime < 0) {
            imuStartTime = time;
        } else if (!receivedFrames && time - imuStartTime > 5.0 && time - lastCameraWarning > 5.0)  {
            RCLCPP_WARN(this->get_logger(), "Received IMU data for %f seconds, but haven't received any camera data. Are camera streams properly configured and synchronized?", time - imuStartTime);
            lastCameraWarning = time;
        }

        if (!vioInitDone || !vioApi) return;
        if (time < previousImuTime) {
            RCLCPP_WARN(this->get_logger(), "Received IMU sample (%f) that's older than previous (%f), skipping it.", time, previousImuTime);
            return;
        }
        vioApi->addGyro(time, {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z});
        vioApi->addAcc(time, {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z});
        if (outputOnImuSamples && (time - previousTriggerTime) > (1. / maxOdomFreq)) {
            vioApi->addTrigger(time, triggerCounter++);
            previousTriggerTime = time;
        }
        previousImuTime = time;
    }

    bool getStereoCameraExtrinsics(spectacularAI::Matrix4d &imuToCam0, spectacularAI::Matrix4d &imuToCam1, spectacularAI::Matrix4d &imuToOutput) {
        if ((!deviceModel.empty() || !overrideImuToCam0.empty() || !overrideImuToCam1.empty())
            && !cam0FrameId.empty()
            && !cam1FrameId.empty()) {

            spectacularAI::Matrix4d imuToCameraX;
            int camIndex = 0;

            if (!overrideImuToCam0.empty()) {
                camIndex = 0;
                if (!matrixFromJson(overrideImuToCam0, imuToCameraX)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse imuToCam0 matrix from given JSON string: %s", overrideImuToCam0.c_str());
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Using given imuToCam0 calibration");
            } else if (!overrideImuToCam1.empty()) {
                camIndex = 1;
                if (!matrixFromJson(overrideImuToCam1, imuToCameraX)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse imuToCam0 matrix from given JSON string: %s", overrideImuToCam0.c_str());
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Using given imuToCam1 calibration");
            } else if (!deviceModel.empty()) {
                if (!getDeviceImuToCameraMatrix(deviceModel, imuToCameraX, camIndex)) {
                    RCLCPP_WARN(this->get_logger(), "No stored camera calibration for %s", deviceModel.c_str());
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Using imuToCamera calibration based on device model");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to find any imuToCamera calibration");
                return false;
            }

            if (camIndex == 0) {
                spectacularAI::Matrix4d cam0ToCam1;
                try {
                    cam0ToCam1 = matrixConvert(transformListenerBuffer->lookupTransform(cam1FrameId, cam0FrameId, tf2::TimePointZero));
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(this->get_logger(), "Could not get camera transforms: %s", ex.what());
                    return false;
                }
                imuToCam0 = imuToCameraX;
                imuToCam1 = matrixMul(cam0ToCam1, imuToCam0);
            } else {
                spectacularAI::Matrix4d cam1ToCam0;
                try {
                    cam1ToCam0 = matrixConvert(transformListenerBuffer->lookupTransform(cam0FrameId, cam1FrameId, tf2::TimePointZero));
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(this->get_logger(), "Could not get camera transforms: %s", ex.what());
                    return false;
                }
                imuToCam1 = imuToCameraX;
                imuToCam0 = matrixMul(cam1ToCam0, imuToCam1);
            }


        } else if (!imuFrameId.empty() && !cam0FrameId.empty() && !cam1FrameId.empty()) {
            try {
                imuToCam0 = matrixConvert(transformListenerBuffer->lookupTransform(cam0FrameId, imuFrameId, tf2::TimePointZero));
                imuToCam1 = matrixConvert(transformListenerBuffer->lookupTransform(cam1FrameId, imuFrameId, tf2::TimePointZero));
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get camera transforms: %s", ex.what());
                return false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                "Failed to obtain camera extrinsics, cannot start tracking. You must provide a known device_model or imu frame ID");
            return false;
        }

        spectacularAI::Matrix4d cam0ToBaseLink;

        try {
            cam0ToBaseLink = matrixConvert(transformListenerBuffer->lookupTransform(baseLinkFrameId, cam0FrameId, tf2::TimePointZero));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get camera transforms for computing imuToOutput: %s", ex.what());
            return false;
        }
        imuToOutput = matrixMul(cam0ToBaseLink, imuToCam0);
        return true;
    }

    void stereoCameraCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo0, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo1,
        const sensor_msgs::msg::Image::ConstSharedPtr &img0, const sensor_msgs::msg::Image::ConstSharedPtr &img1) {
        receivedFrames = true;

        if (!vioInitDone) {
            spectacularAI::Matrix4d imuToCam0, imuToCam1, imuToOutput;
            if (!getStereoCameraExtrinsics(imuToCam0, imuToCam1, imuToOutput)) return;
            // TODO: Support non rectified images, by ROS convention "image" topic is unrectified, "image_rect" is rectified
            constexpr bool IS_RECTIFIED = true;
            startVio({camInfo0, camInfo1}, {imuToCam0, imuToCam1}, imuToOutput, IS_RECTIFIED);
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
        if (time < previousFrameTime) {
            RCLCPP_WARN(this->get_logger(), "Received frame (%f) that's older than previous (%f), skipping it.", time, previousFrameTime);
            return;
        }
        previousFrameTime = time;
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

        receivedFrames = true;
        //RCLCPP_INFO(this->get_logger(), "Received all data: %f", stampToSeconds(img0->header.stamp));

        if (!vioInitDone) {
            spectacularAI::Matrix4d imuToCam0, imuToCam1, imuToOutput;
            if (!getStereoCameraExtrinsics(imuToCam0, imuToCam1, imuToOutput)) return;
            // TODO: Support non rectified images, by ROS convention "image" topic is unrectified, "image_rect" is rectified
            constexpr bool IS_RECTIFIED = true;
            startVio({camInfo0, camInfo1}, {imuToCam0, imuToCam1}, imuToOutput, IS_RECTIFIED);
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
        if (time < previousFrameTime) {
            RCLCPP_WARN(this->get_logger(), "Received frame (%f) that's older than previous (%f), skipping it.", time, previousFrameTime);
            return;
        }
        previousFrameTime = time;

        bool isSlamKf = frameNumber % 6 == 0; // TODO: Make configurable
        vioApi->addFrameStereoDepth(
            time,
            img0->width, img0->height,
            monoFeatures.data(),
            monoFeatures.size(),
            depth->data.data(),
            depthFormat,
            isSlamKf ? img0->data.data() : nullptr,
            isSlamKf ? img1->data.data() : nullptr,
            isSlamKf ? colorFormat : spectacularAI::ColorFormat::NONE,
            depthScale);

        frameNumber++;
    }

    void vioOutputCallback(spectacularAI::VioOutputPtr vioOutput) {
        if (vioOutput->status == spectacularAI::TrackingStatus::TRACKING) {
            if (poseHelper) {
                spectacularAI::Pose odomPose;
                if (poseHelper->computeContinousTrajectory(vioOutput, odomPose, odomCorrection)) {
                    transformBroadcaster->sendTransform(poseToTransformStampped(odomCorrection, fixedFrameId, odometryFrameId));
                }
                transformBroadcaster->sendTransform(poseToTransformStampped(odomPose, odometryFrameId, baseLinkFrameId));
                if (odometryPathPublisher) odometryPathPublisher->addPose(odomPose);
                if (correctedPathPublisher) {
                    correctedPathPublisher->addPose(spectacularAI::Pose::fromMatrix(odomPose.time, matrixMul(odomCorrection.asMatrix(), odomPose.asMatrix())));
                }
            } else {
                transformBroadcaster->sendTransform(poseToTransformStampped(vioOutput->pose, fixedFrameId, baseLinkFrameId));
            }
            if (vioPathPublisher) vioPathPublisher->addPose(vioOutput->pose);

            // odometryPublisher->publish(outputToOdometryMsg(vioOutput, vioOutputParentFrameId, vioOutputChildFrameId));
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
        pc.header.frame_id = fixedFrameId;

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

    void startVio(std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> intrinsics, std::vector<spectacularAI::Matrix4d> extrinsics, spectacularAI::Matrix4d imuToOutput, bool isRectified) {
        std::lock_guard<std::mutex> lock(vioStartup);
        if (vioInitDone) return;

        std::string calibrationJson;
        if (!cameraInfoToCalibrationJson(intrinsics, extrinsics, imuToOutput, isRectified, calibrationJson)) {
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
        if (enableMapping) vioBuilder.setMapperCallback(std::bind(&Node::mappingOutputCallback, this, std::placeholders::_1));
        vioApi = vioBuilder.build();
        vioApi->setOutputCallback(std::bind(&Node::vioOutputCallback, this, std::placeholders::_1));
        vioInitDone = true;
    }

    std::unique_ptr<TrajectoryPublisher> odometryPathPublisher;
    std::unique_ptr<TrajectoryPublisher> correctedPathPublisher;
    std::unique_ptr<TrajectoryPublisher> vioPathPublisher;

    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
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
    std::unique_ptr<PoseHelper> poseHelper;

    double previousFrameTime = 0.0;
    double previousImuTime = 0.0;
    double previousTriggerTime = 0.0;

    int frameNumber = 0;
    int64_t latestKeyFrame = -1;
    std::vector<spectacularAI::MonocularFeature> monoFeatures;
    bool outputOnImuSamples;
    uint64_t triggerCounter = 1;

    std::atomic_bool receivedFrames;
    double imuStartTime = -1.;
    double lastCameraWarning = -1.;
    spectacularAI::Pose odomCorrection;

    // Params
    std::string fixedFrameId;
    std::string odometryFrameId;
    std::string baseLinkFrameId;

    std::string deviceModel;
    std::string imuFrameId;
    std::string cam0FrameId;
    std::string cam1FrameId;
    double depthScale;
    std::string cameraInputType;
    std::string recordingFolder;
    bool enableMapping;
    double maxOdomFreq;
    double maxOdomCorrectionFreq;
    std::string overrideImuToCam0;
    std::string overrideImuToCam1;

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

} // namespace ros2
} // namespace spectacularai

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(spectacularAI::ros2::Node)
