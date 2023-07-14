/*
 * FF-LINS: A Consistent Frame-to-Frame Solid-State-LiDAR-Inertial State Estimator 
 *
 * Copyright (C) 2023 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "common/logging.h"

void Logging::initialization(char **argv, bool logtostderr, bool logtofile) {
    if (logtostderr & logtofile) {
        FLAGS_alsologtostderr = true;
    } else if (logtostderr) {
        FLAGS_logtostderr = true;
    }

    if (logtostderr) {
        // 输出颜色
        FLAGS_colorlogtostderr = true;
    }

    // glog初始化
    google::InitGoogleLogging(argv[0]);
}

string Logging::doubleData(double data) {
    return absl::StrFormat("%0.6lf", data);
}

void Logging::shutdownLogging() {
    google::ShutdownGoogleLogging();
}
