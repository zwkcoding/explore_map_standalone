/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "explore_large_map/rigid_transform.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glog/logging.h"

namespace cartographer {
    namespace transform {

        namespace {

        }  // namespace

        Eigen::Vector3d QuaterniondtoPitchRollYaw(const Eigen::Quaterniond &q) {
            Eigen::Vector3d retVector;

            const auto x = q.x();
            const auto y = q.y();
            const auto z = q.z();
            const auto w = q.w();

            retVector[0] = std::atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
            retVector[1] = -asin(-2.0 * (x * z - w * y));
            retVector[2] = std::atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
//     retVector[0] = -retVector[0];
            return retVector/**180/3.14159265*/;
        }

        Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch, const double yaw) {
            const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            return yaw_angle * pitch_angle * roll_angle;
        }

        Eigen::Quaterniond PitchRollYaw(double pitch, double roll, double yaw) {

            const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            return yaw_angle * roll_angle * pitch_angle;
        }

    }  // namespace transform
}  // namespace cartographer
