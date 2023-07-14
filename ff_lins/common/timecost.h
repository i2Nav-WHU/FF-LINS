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

#ifndef TIMECOST_H
#define TIMECOST_H

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>

class TimeCost {

public:
    TimeCost() {
        restart();
    }

    void restart();
    void finish();

    double costInSecond();
    std::string costInSecond(const std::string &header);

    double costInMillisecond();
    std::string costInMillisecond(const std::string &header);

private:
    absl::Time start_, end_;
    absl::Duration duration_;

    bool is_finish_{false};
};

#endif // TIMECOST_H
