#ifndef SLAM_SIM_YOUBOT_TEST_BASE_H
#define SLAM_SIM_YOUBOT_TEST_BASE_H

#include <thread>
#include <atomic>


#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbotethercatmaster.h"
#include "youbot/EthercatMaster.hpp"

using namespace youbot;

EthercatMasterInterface *ecatmaster;

const float PI = 3.14159265359;
const float G = 9.81;

//Data from Measurement
const float angle_limit0 [5] = {169, 155, -146, 192.5, 167.5};
const float angle_limit1[5] = {-169, 0, 151, -12.5, -167.5};
const int encoder_limit0 [5] = {-584, -269, -328, -163, -260};
const int encoder_limit1[5] = {-585507, -267271, -327681, -162220, -264626};

//Data from Internet
//const int encoder_low [5] = {-1000,-1000,-1000,-1000,-5000};
//const float angle_offset [5] = {169,65,151,102.5,167.5};
//const int encoder_range [5] = {579000,259000,319000,154000,250000};

//link length
const float L1 = 0.147;
const float L2 = 0.155;
const float L3 = 0.135;
const float L4 = 0.113;
const float L5 = 0.105;//including gripper
const float L45 = 0.218;

//center of mass
const float LC2 = 0.1063;
const float LC3 = 0.0589;
const float LC4 = 0.0165;
const float LC5 = 0.0289;

//link mass
const float M1 = 1.39;
const float M2 = 1.318;
const float M3 = 0.821;
const float M4 = 0.769;
const float M5 = 0.887;//including gripper

//gear ratio
const float r[5] = {156,156,100,71,71};
const float r_p = 26;
const float a = 0.158;
const float b = 0.228;
const float R_w = 0.0475;

//torque constant value
const float tau[5] = {0.0335,0.0335,0.0335,0.051,0.049};

//friction constant
const float B[5] = {0.0005048, 0.0007245, 0.0005029, 0.0006543, 0.0000247};
const float J[5] = {0.0000135, 0.0000135, 0.0000135, 0.00000925, 0.0000035};


#endif //SLAM_SIM_YOUBOT_TEST_BASE_H
