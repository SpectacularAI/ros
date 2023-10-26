#pragma once

#include <cstdint>
#include <vector>

// Spectacular AI
#include <spectacularAI/vio.hpp>

#include "helpers.hpp"

/**
 * Helper to form smooth odometry output trajectory
 */
class PoseHelper {
    bool intialized = false;
    spectacularAI::Vector3d odomPosition;
    double lastCorrection = -1.0;
    double lastTime;
    std::string mapFrameId;
    std::string odomFrameId;
    std::string baseFrameId;
    double correctionIntervalSeconds;

public:
    PoseHelper(std::string mapFrameId, std::string odomFrameId, std::string baseFrameId, double correctionIntervalSeconds)
        : mapFrameId(mapFrameId), odomFrameId(odomFrameId), baseFrameId(baseFrameId), correctionIntervalSeconds(correctionIntervalSeconds) {}

    bool computeContinousTrajectory(spectacularAI::VioOutputPtr vioOutput,
        geometry_msgs::msg::TransformStamped &odomPose,
        geometry_msgs::msg::TransformStamped &odomCorrection) {

        bool updatedCorrection = false;

        if (!intialized) {
            odomPosition = vioOutput->pose.position;
            intialized = true;
        } else {
            double deltaT = vioOutput->pose.time - lastTime;
            odomPosition.x += vioOutput->velocity.x * deltaT;
            odomPosition.y += vioOutput->velocity.y * deltaT;
            odomPosition.z += vioOutput->velocity.z * deltaT;
        }
        lastTime = vioOutput->pose.time;

        spectacularAI::Pose pose = spectacularAI::Pose {
            .time = vioOutput->pose.time,
            .position = odomPosition,
            .orientation = vioOutput->pose.orientation
        };
        odomPose = poseToTransformStampped(pose, mapFrameId, baseFrameId);

        if (vioOutput->pose.time - lastCorrection > correctionIntervalSeconds) {
            odomCorrection = poseToTransformStampped(spectacularAI::Pose::fromMatrix(
                vioOutput->pose.time,
                matrixMul(vioOutput->pose.asMatrix(), invertSE3(pose.asMatrix()))
            ), mapFrameId, odomFrameId);
            updatedCorrection = true;
            lastCorrection = vioOutput->pose.time;
        }

        return updatedCorrection;
    }
};

