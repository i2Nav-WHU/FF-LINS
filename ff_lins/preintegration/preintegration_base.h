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

#ifndef PREINTEGRATION_BASE_H
#define PREINTEGRATION_BASE_H

#include "common/integration_state.h"
#include "common/rotation.h"
#include "common/types.h"

#include <memory>
#include <vector>

class PreintegrationBase {

public:
    PreintegrationBase(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0, IntegrationState state);

    virtual ~PreintegrationBase() = default;

    const IntegrationState &currentState() {
        return current_state_;
    }

    const IntegrationState &deltaState() {
        return delta_state_;
    }

    double deltaTime() const {
        return delta_time_;
    }

    double startTime() const {
        return start_time_;
    }

    double endTime() const {
        return end_time_;
    }

    const Vector3d &gravity() {
        return gravity_;
    }

    const vector<IMU> &imuBuffer() {
        return imu_buffer_;
    }

    void addNewImu(const IMU &imu);
    void reintegration(IntegrationState &state);

public:
    virtual Eigen::MatrixXd evaluate(const IntegrationState &state0, const IntegrationState &state1,
                                     double *residuals) = 0;

    virtual Eigen::MatrixXd residualJacobianPose0(const IntegrationState &state0, const IntegrationState &state1,
                                                  double *jacobian) = 0;
    virtual Eigen::MatrixXd residualJacobianPose1(const IntegrationState &state0, const IntegrationState &state1,
                                                  double *jacobian) = 0;
    virtual Eigen::MatrixXd residualJacobianMix0(const IntegrationState &state0, const IntegrationState &state1,
                                                 double *jacobian)  = 0;
    virtual Eigen::MatrixXd residualJacobianMix1(const IntegrationState &state0, const IntegrationState &state1,
                                                 double *jacobian)  = 0;

    virtual int numResiduals()                     = 0;
    virtual int numMixParametersBlocks()           = 0;
    virtual std::vector<int> numBlocksParameters() = 0;

    virtual void constructState(const double *const *parameters, IntegrationState &state0,
                                IntegrationState &state1) = 0;

protected:
    // need compensate bias
    virtual void updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) = 0;

    virtual void resetState(const IntegrationState &state) = 0;
    virtual void integrationProcess(unsigned long index)   = 0;

    static void stateToData(const IntegrationState &state, IntegrationStateData &data);
    static void stateFromData(const IntegrationStateData &data, IntegrationState &state);

    void integration(const IMU &imu_pre, const IMU &imu_cur);

    IMU compensationBias(const IMU &imu) const;

    IMU compensationScale(const IMU &imu) const;

public:
    static constexpr int NUM_POSE = 7;

protected:
    const std::shared_ptr<IntegrationParameters> parameters_;

    IntegrationState current_state_;
    IntegrationState delta_state_;

    vector<IMU> imu_buffer_;
    double delta_time_{0};
    double start_time_;
    double end_time_;

    Vector3d gravity_;

    Eigen::MatrixXd jacobian_, covariance_;
    Eigen::MatrixXd noise_;
    Eigen::MatrixXd sqrt_information_;

    Quaterniond corrected_q_;
    Vector3d corrected_p_, corrected_v_;
};

#endif // PREINTEGRATION_BASE_H