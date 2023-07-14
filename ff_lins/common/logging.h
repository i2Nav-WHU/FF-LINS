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

#ifndef LOGGING_H
#define LOGGING_H

#include <Eigen/Geometry>
#include <absl/strings/str_format.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <iostream>
#include <sstream>
#include <string>

using std::string;

#define LOGI (LOG(INFO))
#define LOGW (LOG(WARNING))
#define LOGE (LOG(ERROR))
#define LOGF (LOG(FATAL))

#if !DCHECK_IS_ON()
#define DLOGI (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(INFO))
#define DLOGW (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(WARNING))
#define DLOGE (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(ERROR))
#define DLOGF (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(FATAL))
#else
#define DLOGI LOGI
#define DLOGW LOGW
#define DLOGE LOGE
#define DLOGF LOGF
#endif

class Logging {

public:
    static void initialization(char **argv, bool logtostderr = true, bool logtofile = false);
    static void shutdownLogging();

    template <typename T, int Rows, int Cols>
    static void printMatrix(const Eigen::Matrix<T, Rows, Cols> &matrix, const string &prefix = "Matrix: ") {
        std::cout << prefix << matrix.rows() << "x" << matrix.cols() << std::endl;
        if (matrix.cols() == 1) {
            std::cout << matrix.transpose() << std::endl;
        } else {
            std::cout << matrix << std::endl;
        }
    }

    static string doubleData(double data);

    template <typename... Args> static string sequenceData(const Args &...args) {
        std::ostringstream ss;

        printSequence(ss, args...);

        return ss.str();
    }

private:
    template <typename T> static void printSequence(std::ostringstream &ss, const T &t) {
        ss << t;
    }

    template <typename T, typename... Args>
    static void printSequence(std::ostringstream &ss, const T &t, const Args &...args) {
        ss << t << ", ";
        printSequence(ss, args...);
    }
};

#endif // LOGGING_H
