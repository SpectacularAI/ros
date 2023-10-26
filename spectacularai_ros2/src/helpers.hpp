
#pragma once

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <spectacularAI/types.hpp>

constexpr double NANOS_TO_SECONDS = 1e-9;
constexpr double SECONDS_TO_NANOS = 1e9;

template <class M, unsigned int DIM> M matMulGeneric(const M &a, const M &b) {
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
    return matMulGeneric<spectacularAI::Matrix4d, 4>(a, b);
}

spectacularAI::Matrix3d matrixMul(const spectacularAI::Matrix3d &a, const spectacularAI::Matrix3d &b) {
    return matMulGeneric<spectacularAI::Matrix3d, 3>(a, b);
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

spectacularAI::Matrix3d matrixConvert(std::array<double, 9> r) {
    return {{
        {r[0], r[1], r[2]},
        {r[3], r[4], r[5]},
        {r[6], r[7], r[8]}
    }};
}

spectacularAI::Matrix3d getRotation(spectacularAI::Matrix4d m) {
    return {{
        {m[0][0], m[0][1], m[0][2]},
        {m[1][0], m[1][1], m[1][2]},
        {m[2][0], m[2][1], m[2][2]}
    }};
}

spectacularAI::Matrix4d setRotation(spectacularAI::Matrix4d m, spectacularAI::Matrix3d r) {
    return {{
        {r[0][0], r[0][1], r[0][2], m[0][3]},
        {r[1][0], r[1][1], r[1][2], m[1][3]},
        {r[2][0], r[2][1], r[2][2], m[2][3]},
        {0.0, 0.0, 0.0, 1.0}
    }};
}

spectacularAI::Matrix4d invertSE3(spectacularAI::Matrix4d m) {
    return {{
        {m[0][0], m[1][0], m[2][0], -(m[0][0] * m[0][3] + m[1][0] * m[1][3] + m[2][0] * m[2][3]) },
        {m[0][1], m[1][1], m[2][1], -(m[0][1] * m[0][3] + m[1][1] * m[1][3] + m[2][1] * m[2][3]) },
        {m[0][2], m[1][2], m[2][2], -(m[0][2] * m[0][3] + m[1][2] * m[1][3] + m[2][2] * m[2][3]) },
        {0,0,0,1}
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

std::string toJson(spectacularAI::Matrix3d m) {
    std::ostringstream ss;
    ss << std::setprecision(18);
    ss << "[";
    ss << "[" << m[0][0] << "," << m[0][1] << "," << m[0][2] << "],";
    ss << "[" << m[1][0] << "," << m[1][1] << "," << m[1][2] << "],";
    ss << "[" << m[2][0] << "," << m[2][1] << "," << m[2][2] << "]";
    ss << "]";
    return ss.str();
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

void skipWhiteSpace(const std::string& json, size_t& pos) {
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' || json[pos] == '\r')) {
        pos++;
    }
}

double parseNumber(const std::string& json, size_t& pos) {
    skipWhiteSpace(json, pos);
    size_t start = pos;
    while (pos < json.length() && (isdigit(json[pos]) || json[pos] == '.' || json[pos] == '-')) {
        pos++;
    }
    return std::stod(json.substr(start, pos - start));
}

bool matrixFromJson(const std::string& json, spectacularAI::Matrix4d &m) {
    std::array<std::array<double, 4>, 4> result;
    size_t pos = 0;

    skipWhiteSpace(json, pos);
    if (json[pos] != '[') return false;
    pos++;

    for (int i = 0; i < 4; i++) {
        skipWhiteSpace(json, pos);
        if (json[pos] != '[') return false;
        pos++;

        for (int j = 0; j < 4; j++) {
            result[i][j] = parseNumber(json, pos);
            skipWhiteSpace(json, pos);
            if (j < 3) {
                if (json[pos] != ',') {
                    return false;
                }
                pos++;
            }
        }

        skipWhiteSpace(json, pos);
        if (json[pos] != ']') return false;
        pos++;

        if (i < 3) {
            if (json[pos] != ',') {
                return false;
            }
            pos++;
        }
    }

    if (json[pos] != ']') {
        return false;
    }

    m = result;
    return true;
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
