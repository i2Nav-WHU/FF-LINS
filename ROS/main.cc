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

#include "fusion.h"

#include "ff_lins/common/logging.h"
#include "ff_lins/common/timecost.h"

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<Fusion> fusion);

int main(int argc, char *argv[]) {
    // Glog初始化
    Logging::initialization(argv, true, true);

    // ROS节点初始化
    ros::init(argc, argv, "ff_lins_node", ros::InitOption::NoSigintHandler);

    // 注册信号处理函数
    std::signal(SIGINT, sigintHandler);

    auto fusion = std::make_shared<Fusion>();

    // 退出检测线程
    std::thread check_thread(checkStateThread, fusion);

    std::cout << "Fusion process is started..." << std::endl;

    // 进入消息循环
    TimeCost timecost;
    fusion->run();
    check_thread.join();
    LOGI << "FF-LINS process costs " << timecost.costInSecond() << " seconds";

    // 关闭日志
    Logging::shutdownLogging();

    return 0;
}

void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    global_finished = true;
}

void checkStateThread(std::shared_ptr<Fusion> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion);
    while (!global_finished) {
        sleep(1);
    }

    // 退出LINS处理线程
    fusion_ptr->setFinished();
    std::cout << "FF-LINS has been shutdown ..." << std::endl;

    // 关闭ROS
    ros::shutdown();
    std::cout << "ROS node has been shutdown ..." << std::endl;
}
