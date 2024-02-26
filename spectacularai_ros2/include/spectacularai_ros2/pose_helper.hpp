#pragma once

#include <cstdint>
#include <vector>

// Spectacular AI
#include <spectacularAI/vio.hpp>

/**
 * Helper to form smooth odometry output trajectory
 */
class PoseHelper {
    bool intialized = false;
    spectacularAI::Vector3d odomPosition;
    double lastCorrection = -1.0;
    double lastTime;
    double correctionIntervalSeconds;

public:
    PoseHelper(double correctionIntervalSeconds) : correctionIntervalSeconds(correctionIntervalSeconds) {}

    bool computeContinousTrajectory(spectacularAI::VioOutputPtr vioOutput,
        spectacularAI::Pose &odomPose,
        spectacularAI::Pose &odomCorrection) {

        bool updatedCorrection = false;

        if (!intialized) {
            odomPosition = vioOutput->pose.position;
            intialized = true;
        } else {
            double deltaT = vioOutput->pose.time - lastTime;
            odomPosition.x += vioOutput->velocity.x * deltaT;
            odomPosition.y += vioOutput->velocity.y * deltaT;
            odomPosition.z = 0;
        }
        lastTime = vioOutput->pose.time;
        
        odomPose = spectacularAI::Pose {
            .time = vioOutput->pose.time,
            .position = odomPosition,
            .orientation = vioOutput->pose.orientation
        };

        if (vioOutput->pose.time - lastCorrection > correctionIntervalSeconds) {
            odomCorrection = spectacularAI::Pose::fromMatrix(
                vioOutput->pose.time,
                matrixMul(vioOutput->pose.asMatrix(), invertSE3(odomPose.asMatrix()))
            );
            updatedCorrection = true;
            lastCorrection = vioOutput->pose.time;
        }

        return updatedCorrection;
    }
};

