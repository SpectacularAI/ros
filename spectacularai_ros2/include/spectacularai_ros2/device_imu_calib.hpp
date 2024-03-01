#pragma once

#include <string>
#include <spectacularAI/types.hpp>

namespace {
bool stringStartsWith(const std::string &str, const std::string &query) {
    return str.length() >= query.length() && str.substr(0, query.length()) == query;
}
}

bool getDeviceImuToCameraMatrix(std::string deviceModel, spectacularAI::Matrix4d &imuToCamera, int &camIndex) {
    if (stringStartsWith(deviceModel, "BW") || deviceModel == "oakd" || deviceModel == "OAK-D") {
        camIndex = 1; // Left camera
        imuToCamera = {{
            { 0, -1, 0, 0.052 },
            { 1,  0, 0, 0.013662 },
            { 0,  0, 1, 0.0025 },
            { 0,  0, 0, 1 }
        }};
    } else if (stringStartsWith(deviceModel, "DM") || stringStartsWith(deviceModel, "OAK-D-S2")) {
        camIndex = 1; // Left camera
        // TODO: Inaccurate translation
        imuToCamera = {{
            { 0, 1, 0, 0.052 },
            { 1, 0, 0, 0.013662 },
            { 0, 0,-1, 0.0025 },
            { 0, 0, 0, 1 }
        }};
    } else if (stringStartsWith(deviceModel, "RAE") || stringStartsWith(deviceModel, "rae")) {
        camIndex = 0; // Right camera
        imuToCamera = {{
            { -0.0021768695008372863, 0.9999877375295445, 0.004448149021739123, -0.05771866767992802 },
            { 0.48302895198231616, -0.0028433421070772624, 0.8755997641345776, -0.02546937245057432 },
            { 0.8756016747277509, 0.0040546511817166064, -0.4830168392693499, -0.0264284136937599 },
            { 0.0, 0.0, 0.0, 1.0 }
        }};
    } else {
        return false;
    }
    return true;
}
