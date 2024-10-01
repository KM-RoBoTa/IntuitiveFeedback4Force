/**
 ******************************************************************************
 * @file            force_sensors.hpp
 * @brief           Header for the force_sensors.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 10/2023
 * @authors  kamilo.melo@km-robota.com, 10/2023
 ****************************************************************************
 */


#ifndef FORCE_SENSORS_HPP
#define FORCE_SENSORS_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#define NBR_SENSORS	4

/**
 * @brief   Container for a single force sensor
 */
struct ForceSensorStruct {
    float Fx = 0;
    float Fy = 0;
    float Fz = 0;
};


/**
 * @brief   Class handling all Intuitive force sensors (max 4)
 */
class ForceSensors {
public:
    ForceSensors(const char* sensors_portname);
    ~ForceSensors();
    float getForces(std::vector<ForceSensorStruct>& forces);
    void calibrate();

private:
    bool m_stopThread = false;

    float m_freq;
    std::thread m_thread;
    std::mutex m_mutex;
    std::vector<ForceSensorStruct> m_forces;
    std::vector<ForceSensorStruct> m_tareOffsets;

    void forceSensorsLoop(const char* sensors_portname);
    int openPort(const char* sensors_portname);
    void interpretData(char* read_buffer, int bytes_read);
};


#endif