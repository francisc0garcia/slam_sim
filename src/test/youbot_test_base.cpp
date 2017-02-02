#include "youbot_test_base.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>

#include <unistd.h> // for usleep()

#define CONTROL_LOOP_CYCLE_PERIOD_MS 10


struct timespec cycletime = {0, CONTROL_LOOP_CYCLE_PERIOD_MS*ECATMASTER_PERIOD_NS}; //ECATMASTER_PERIOD_NS=1ms
struct timespec wakeupTime;

YouBotSetMsg ECatSendMsg;
YouBotGetMsgExt ECatGetMsg;
YouBotGlobals helperfunctions;

std::ofstream output_file_robot;
std::string filename_robot = "experiment_robot.txt";


bool is_control_loop_activated = false;

bool is_W_pressed = false;
bool is_S_pressed = false;
bool is_A_pressed = false;
bool is_D_pressed = false;
bool is_Q_pressed = false;
bool is_E_pressed = false;

float w_linear = 40;
float w_rotational = 30;
float Omega_M[4];

int32_t joint_encoder_get[5];
float joint_velocity_get[5];
float joint_current_get[5];

int32_t wheel_encoder_get[4];
float wheel_velocity_get[4];
float wheel_current_get[4];

std::vector<long> t_control_record;
std::vector<long> FL_encoder, FR_encoder, BL_encoder, BR_encoder;
std::vector<float> FL_velocity, FR_velocity, BL_velocity, BR_velocity;

void run_control()
{
    // initialising robot measurement storage
    t_control_record.clear();
    FL_encoder.clear();
    FL_velocity.clear();
    FR_encoder.clear();
    FR_velocity.clear();
    BL_encoder.clear();
    BL_velocity.clear();
    BR_encoder.clear();
    BR_velocity.clear();

    while (is_control_loop_activated)
    {
        // getting robot measurements
        ecatmaster-> GetData(WHEEL_FL, ECatGetMsg);
        wheel_encoder_get[0] = ECatGetMsg.actualPosition_EncTicks;
        wheel_velocity_get[0] = -ECatGetMsg.actualVelocity_RadPerSec/r_p;
        wheel_current_get[0] = ECatGetMsg.actualCurrent_mA;

        ecatmaster-> GetData(WHEEL_FR, ECatGetMsg);
        wheel_encoder_get[1] = ECatGetMsg.actualPosition_EncTicks;
        wheel_velocity_get[1] = ECatGetMsg.actualVelocity_RadPerSec/r_p;
        wheel_current_get[1] = ECatGetMsg.actualCurrent_mA;

        ecatmaster-> GetData(WHEEL_BL, ECatGetMsg);
        wheel_encoder_get[2] = ECatGetMsg.actualPosition_EncTicks;
        wheel_velocity_get[2] = -ECatGetMsg.actualVelocity_RadPerSec/r_p;
        wheel_current_get[2] = ECatGetMsg.actualCurrent_mA;

        ecatmaster-> GetData(WHEEL_BR, ECatGetMsg);
        wheel_encoder_get[3] = ECatGetMsg.actualPosition_EncTicks;
        wheel_velocity_get[3] = ECatGetMsg.actualVelocity_RadPerSec/r_p;
        wheel_current_get[3] = ECatGetMsg.actualCurrent_mA;

        // robot measurements >> storage vectors
        t_control_record.push_back(t_control);
        FL_encoder.push_back(wheel_encoder_get[0]);
        FL_velocity.push_back(wheel_velocity_get[0]);
        FR_encoder.push_back(wheel_encoder_get[1]);
        FR_velocity.push_back(wheel_velocity_get[1]);
        BL_encoder.push_back(wheel_encoder_get[2]);
        BL_velocity.push_back(wheel_velocity_get[2]);
        BR_encoder.push_back(wheel_encoder_get[3]);
        BR_velocity.push_back(wheel_velocity_get[3]);

        // No pressed key
        Omega_M[0] = 0;
        Omega_M[1] = 0;
        Omega_M[2] = 0;
        Omega_M[3] = 0;

        // Single DOF motion
        if (is_W_pressed)
        {
            Omega_M[0] = w_linear*r_p;
            Omega_M[1] = w_linear*r_p;
            Omega_M[2] = w_linear*r_p;
            Omega_M[3] = w_linear*r_p;
        }
        if (is_S_pressed)
        {
            Omega_M[0] = -w_linear*r_p;
            Omega_M[1] = -w_linear*r_p;
            Omega_M[2] = -w_linear*r_p;
            Omega_M[3] = -w_linear*r_p;
        }
        if (is_A_pressed)
        {
            Omega_M[0] = -w_linear*r_p;
            Omega_M[1] = w_linear*r_p;
            Omega_M[2] = w_linear*r_p;
            Omega_M[3] = -w_linear*r_p;
        }
        if (is_D_pressed)
        {
            Omega_M[0] = w_linear*r_p;
            Omega_M[1] = -w_linear*r_p;
            Omega_M[2] = -w_linear*r_p;
            Omega_M[3] = w_linear*r_p;
        }
        if (is_Q_pressed)
        {
            Omega_M[0] = -w_rotational*r_p;
            Omega_M[1] = w_rotational*r_p;
            Omega_M[2] = -w_rotational*r_p;
            Omega_M[3] = w_rotational*r_p;
        }
        if (is_E_pressed)
        {
            Omega_M[0] = w_rotational*r_p;
            Omega_M[1] = -w_rotational*r_p;
            Omega_M[2] = w_rotational*r_p;
            Omega_M[3] = -w_rotational*r_p;
        }

        // Linear + Left rotation
        if (is_W_pressed && is_Q_pressed)
        {
            Omega_M[0] = (w_linear - w_rotational)*r_p;
            Omega_M[1] = (w_linear + w_rotational)*r_p;
            Omega_M[2] = (w_linear - w_rotational)*r_p;
            Omega_M[3] = (w_linear + w_rotational)*r_p;
        }
        if (is_S_pressed && is_Q_pressed)
        {
            Omega_M[0] = (-w_linear - w_rotational)*r_p;
            Omega_M[1] = (-w_linear + w_rotational)*r_p;
            Omega_M[2] = (-w_linear - w_rotational)*r_p;
            Omega_M[3] = (-w_linear + w_rotational)*r_p;
        }
        if (is_A_pressed && is_Q_pressed)
        {
            Omega_M[0] = (-w_linear - w_rotational)*r_p;
            Omega_M[1] = (w_linear + w_rotational)*r_p;
            Omega_M[2] = (w_linear - w_rotational)*r_p;
            Omega_M[3] = (-w_linear + w_rotational)*r_p;
        }
        if (is_D_pressed && is_Q_pressed)
        {
            Omega_M[0] = (w_linear - w_rotational)*r_p;
            Omega_M[1] = (-w_linear + w_rotational)*r_p;
            Omega_M[2] = (-w_linear - w_rotational)*r_p;
            Omega_M[3] = (w_linear + w_rotational)*r_p;
        }

        // Linear + Right rotation
        if (is_W_pressed && is_E_pressed)
        {
            Omega_M[0] = (w_linear + w_rotational)*r_p;
            Omega_M[1] = (w_linear - w_rotational)*r_p;
            Omega_M[2] = (w_linear + w_rotational)*r_p;
            Omega_M[3] = (w_linear - w_rotational)*r_p;
        }
        if (is_S_pressed && is_E_pressed)
        {
            Omega_M[0] = (-w_linear + w_rotational)*r_p;
            Omega_M[1] = (-w_linear - w_rotational)*r_p;
            Omega_M[2] = (-w_linear + w_rotational)*r_p;
            Omega_M[3] = (-w_linear - w_rotational)*r_p;
        }
        if (is_A_pressed && is_E_pressed)
        {
            Omega_M[0] = (-w_linear + w_rotational)*r_p;
            Omega_M[1] = (w_linear - w_rotational)*r_p;
            Omega_M[2] = (w_linear + w_rotational)*r_p;
            Omega_M[3] = (-w_linear - w_rotational)*r_p;
        }
        if (is_D_pressed && is_E_pressed)
        {
            Omega_M[0] = (w_linear + w_rotational)*r_p;
            Omega_M[1] = (-w_linear - w_rotational)*r_p;
            Omega_M[2] = (-w_linear + w_rotational)*r_p;
            Omega_M[3] = (w_linear - w_rotational)*r_p;
        }

        // Linear + Linear (45 deg)
        if (is_W_pressed && is_A_pressed)
        {
            Omega_M[0] = 0;
            Omega_M[1] = w_linear*r_p;
            Omega_M[2] = w_linear*r_p;
            Omega_M[3] = 0;
        }
        if (is_W_pressed && is_D_pressed)
        {
            Omega_M[0] = w_linear*1.414*r_p;
            Omega_M[1] = 0;
            Omega_M[2] = 0;
            Omega_M[3] = w_linear*1.414*r_p;
        }
        if (is_S_pressed && is_A_pressed)
        {
            Omega_M[0] = -w_linear*1.414*r_p;
            Omega_M[1] = 0;
            Omega_M[2] = 0;
            Omega_M[3] = -w_linear*1.414*r_p;
        }
        if (is_S_pressed && is_D_pressed)
        {
            Omega_M[0] = 0;
            Omega_M[1] = -w_linear*1.414*r_p;
            Omega_M[2] = -w_linear*1.414*r_p;
            Omega_M[3] = 0;
        }

        ECatSendMsg.controllerMode = CONTROLLER_MODE_VELOCITY;
        ECatSendMsg.value = -Omega_M[0];
        ecatmaster-> SetData(WHEEL_FL, ECatSendMsg);
        ECatSendMsg.value = Omega_M[1];
        ecatmaster-> SetData(WHEEL_FR, ECatSendMsg);
        ECatSendMsg.value = -Omega_M[2];
        ecatmaster-> SetData(WHEEL_BL, ECatSendMsg);
        ECatSendMsg.value = Omega_M[3];
        ecatmaster-> SetData(WHEEL_BR, ECatSendMsg);
    }

    // releasing the wheels
    ECatSendMsg.controllerMode = CONTROLLER_MODE_CURRENT;
    ECatSendMsg.value = 0;
    ecatmaster-> SetData(WHEEL_FL, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_FR, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_BL, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_BR, ECatSendMsg);
}


int main(int argc, char **argv)
{
    // enabling EthercatMaster
    ecatmaster = new YouBotEtherCatMaster();

    // reserving memory for experimental data storage
    t_control_record.reserve(100000);
    FL_encoder.reserve(100000);
    FL_velocity.reserve(100000);
    FR_encoder.reserve(100000);
    FR_velocity.reserve(100000);
    BL_encoder.reserve(100000);
    BL_velocity.reserve(100000);
    BR_encoder.reserve(100000);
    BR_velocity.reserve(100000);

    // commanding no current in the motors
    ECatSendMsg.controllerMode = CONTROLLER_MODE_CURRENT;
    ECatSendMsg.value = 0;
    ecatmaster-> SetData(ARM_JOINT_1, ECatSendMsg);
    ecatmaster-> SetData(ARM_JOINT_2, ECatSendMsg);
    ecatmaster-> SetData(ARM_JOINT_3, ECatSendMsg);
    ecatmaster-> SetData(ARM_JOINT_4, ECatSendMsg);
    ecatmaster-> SetData(ARM_JOINT_5, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_FL, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_FR, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_BL, ECatSendMsg);
    ecatmaster-> SetData(WHEEL_BR, ECatSendMsg);

    // Emergency
    //  ecatmaster-> EmergencyStop();


}
