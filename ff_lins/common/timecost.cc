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

#include "common/timecost.h"

void TimeCost::restart() {
    start_     = absl::Now();
    is_finish_ = false;
}

void TimeCost::finish() {
    end_       = absl::Now();
    duration_  = end_ - start_;
    is_finish_ = true;
}

double TimeCost::costInSecond() {
    if (!is_finish_) {
        finish();
    }

    return absl::ToDoubleSeconds(duration_);
}

std::string TimeCost::costInSecond(const std::string &header) {
    auto cost = costInSecond();
    return absl::StrFormat("%s %0.6lf s", header.c_str(), cost);
}

double TimeCost::costInMillisecond() {
    if (!is_finish_) {
        finish();
    }
    return absl::ToDoubleMilliseconds(duration_);
}

std::string TimeCost::costInMillisecond(const std::string &header) {
    auto cost = costInMillisecond();
    return absl::StrFormat("%s costs %0.3lf ms", header.c_str(), cost);
}