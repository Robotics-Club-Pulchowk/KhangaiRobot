/*
 * position_sensor.h
 * 
 * Created : 1/5/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "position_sensor.h"
#include "smoothing_algorithm.h"

#include "devs_config.h"

#include "error.h"

#include "robo_states.h"
#include "moore.h"

static const size_t gN_States = 9;
static const size_t gN_Inputs = 2;

extern Moore_Machine<gN_States, gN_Inputs> gBridge_Machine;

static Exp_Smooth gXLidarAlpha35(0.35);
static Exp_Smooth gYLidarAlpha35(0.35);

// Starting Y-Position
float gLast_YEncoderValue = 400;
// Starting X-Position
float gLast_XEncoderValue = 310;

static float gYEncoder_Dist = 0;

float gXLidar_Bias = 260;
float gYLidar_Bias = 0;

// Last Position
Vec3<float> gLastPosition(gLast_XEncoderValue, gLast_YEncoderValue, 0);

Kalman_Vars gEncoders_KV;
Kalman_Vars gXLidarEncoder_KV;


PositionSensor& PositionSensor::get_Instance()
{
        static PositionSensor gPSensor_Instance;

        return gPSensor_Instance;
}

int PositionSensor::init(uint32_t dt_millis)
{
        Encoders_Init();

        gXEncoder.init();
        gYEncoder.init();
        gXLidar.init();

        return 0;
}

Vec3<float> PositionSensor::read_Position(Vec3<float> ori, Vec3<float> base_state, const State_Vars *sv, uint32_t dt_millis)
{
        // First Collect Data from all available position sensors
        // 0 -> x, 1 -> y
        Vec3<float> free_wheel;
        float lidar[2];

        bool x_lidar_used(false), y_lidar_used(false);
        bool x_enc_used(false), y_enc_used(false);

        for (uint8_t i = 0; i < sensor_count_; ++i) {
                if (p_sensors_[i]->get_Name() == SensorName::XEncoder) {
                        free_wheel.setX(p_sensors_[i]->read());
                        x_enc_used = true;
                }
                else if (p_sensors_[i]->get_Name() == SensorName::YEncoder) {
                        free_wheel.setY(p_sensors_[i]->read());
                        y_enc_used = true;
                }
                else if (p_sensors_[i]->get_Name() == SensorName::XLidar) {
                        lidar[0] = p_sensors_[i]->read() + gXLidar_Bias;
                        x_lidar_used = true;
                        // printf("%ld\n", (int32_t)(lidar[0]));
                }
                else if (p_sensors_[i]->get_Name() == SensorName::YLidar) {
                        lidar[1] = p_sensors_[i]->read() + gYLidar_Bias;
                        y_lidar_used = true;
                }
        }
        if (!(x_lidar_used || x_enc_used)) {
                error(Error::DEVICE_ERROR);
                printf("No Data Available on x-direction!!\n");
        }
        if (!(y_lidar_used || y_enc_used)) {
                error(Error::DEVICE_ERROR);
                printf("No Data Available on y-direction!!\n");
        }

        if (x_lidar_used || y_lidar_used) {
                process_LidarData(lidar, sv);
        }
        // Smooth Lidar Data if used else clear the smoothie
        if (x_lidar_used) {
                lidar[0] = gXLidarAlpha35.smooth(lidar[0]);
        }
        else {
                gXLidarAlpha35.clear();
        }
        // if (y_lidar_used) {

        // }
        // else {

        // }

        // float dt = (float)(dt_millis) / 1000.0;
        // Calculate the movement of the body with respect to the body frame

        // Rotate the movement to the navigation frame
        // yaw-pitch-roll
        float roll = ori.getX() / 57.3;
        float pitch = ori.getY() / 57.3;
        float yaw = ori.getZ() / 57.3;

        float s_r = sin(roll);
        float c_r = cos(roll);
        float s_p = sin(pitch);
        float c_p = cos(pitch);
        float s_y = sin(yaw);
        float c_y = cos(yaw);

        // https://ocw.mit.edu/courses/mechanical-engineering/2-017j-design-of-electromechanical-robotic-systems-fall-2009/course-text/MIT2_017JF09_ch09.pdf
        // R(r, p, y)
        float R[3][3] = { { c_p*c_r,                c_p*s_r,               -s_p},
                          {-c_y*s_r + s_y*s_p*c_r,  c_y*c_r + s_y*s_p*s_r,  s_y*c_p},
                          { s_y*s_r + c_y*s_p*c_r, -s_y*c_r + c_y*s_p*s_r,  c_y*c_p}, };

        Mat Rm(R);

        Mat d_pos = Rm.trans() * free_wheel;

        Vec3<float> del_pos(d_pos.at(0,0), d_pos.at(1,0), d_pos.at(2,0));

        float ex = del_pos.getX();
        float ey = del_pos.getY();

        // float ex = free_wheel.getX();
        // float ey = free_wheel.getY() * 1.0117;

        // printf("%ld  ", (int32_t)(yaw*1000));

        // Mat SO2(2,2);
        // SO2.at(0,0) = c_y;
        // SO2.at(0,1) = -s_y;
        // SO2.at(1,0) = s_y;
        // SO2.at(1,1) = c_y;

        // Mat fw(2,1);
        // fw.at(0,0) = free_wheel.getX();
        // fw.at(1,0) = free_wheel.getY();
        // Mat so2_pos = SO2.trans() * fw;

        // float ex = so2_pos.at(0,0);
        // float ey = so2_pos.at(1,0);

        gYEncoder_Dist += ey;
        // printf("%ld\n", (int32_t)(gYEncoder_Dist));

        // Fuse the data with the data from lidar that gives movement with
        // respect to the navigation frame

        float x(0), y(0);
        if (!x_lidar_used) {
                x = gLastPosition.getX() + ex;
                xlidar_enc_fuser_.clear();
        }
        else {
                x = xlidar_enc_fuser_.filter(lidar[0], del_pos.getX(), dt_millis);
        }

        // y lidar not used
        if (!y_lidar_used) {
                y = gLastPosition.getY() + ey;
                // ylidar_enc_fuser_.clear();
        }
        else {
                // y = ylidar_enc_fuser_.filter(lidar[0], del_pos.getY(), dt_millis);
        }

        // x and y are in mm
        gLastPosition.set_Values(x, y, 0);

        // gLastPosition.print();
        // printf("\n");

        return gLastPosition;
}

void PositionSensor::process_LidarData(float (&lidar)[2], const State_Vars *sv)
{
        // Process Lidar Data according to the field the robot is in

        // Assumptions:
        // lidar[0] => XLidar Data
        // lidar[1] => YLidar Data

        // printf("%ld\n", (int32_t)lidar[0]);

        float jungle_pole_dist = 1255;
        float bridge_pole_dist = 900;
        float tol = 100;

        Field id = sv->id;

        // Processing XLidar data
        if ((int)id > (int)(Field::FIELD_J)) {
                // The fence is at most 2000 mm from robot
                if (lidar[0] < 2000) {
                        // Lower Fence distance
                        lidar[0] += 2500;
                }
        }
        else {
                if (id == Field::FIELD_B || id == Field::FIELD_F) {
                        if (lidar[0] < jungle_pole_dist) {
                                lidar[0] += jungle_pole_dist;

                                // Compensating the Y value based on lidar data
                                if (id == Field::FIELD_B) {
                                        gLast_YEncoderValue = 2030.0;
                                }
                                else if (id == Field::FIELD_F) {
                                        gLast_YEncoderValue = 5030.0;
                                }
                        }
                }
                else if (id == Field::FIELD_H) {

                        int state_id = -1;

                        if (lidar[0] < (bridge_pole_dist - tol)) {
                                // Feed 0
                                state_id = gBridge_Machine.feed(0);
                        }
                        else if (lidar[0] > (bridge_pole_dist + tol)) {
                                // Feed 1
                                state_id = gBridge_Machine.feed(1);
                        }

                        if (state_id == 7) {
                        }

                        // Correct the XLidar Value
                        if (lidar[0] < bridge_pole_dist) {
                                lidar[0] += 755;
                        }
                }
        }
}


int init_XLidarEncoderKalman(uint32_t dt_millis)
{
        // This is for data fusion of lidar and encoder
        Mat state_model(2,2);
        state_model.at(0,0) = 1;
        state_model.at(0,1) = -1;
        state_model.at(1,0) = 0;
        state_model.at(1,1) = 1;

        Mat control_model(2,1);
        control_model.at(0,0) = 1;
        control_model.at(1,0) = 0;
        
        Mat obs_model(1,2);
        obs_model.at(0,0) = 1;
        obs_model.at(0,1) = 0;

        Mat priori_error(2,2);
        priori_error.at(0,0) = 500;
        priori_error.at(0,1) = 0;
        priori_error.at(1,0) = 0;
        priori_error.at(1,1) = 500;
        
        Mat process_error(2,2);
        process_error.at(0,0) = 0.001;
        process_error.at(0,1) = 0;
        process_error.at(1,0) = 0;
        process_error.at(1,1) = 0.003;
        
        Mat measure_error(1,1);
        measure_error.at(0,0) = 200;

        gXLidarEncoder_KV.set_F(state_model);
        gXLidarEncoder_KV.set_B(control_model);
        gXLidarEncoder_KV.set_H(obs_model);
        gXLidarEncoder_KV.set_I(2);
        gXLidarEncoder_KV.set_P(priori_error);
        gXLidarEncoder_KV.set_Q(process_error);
        gXLidarEncoder_KV.set_R(measure_error);

        return 0;
}

int init_EncodersKalman(uint32_t dt_millis)
{
        return 0;
}

// *** Construction of Bridge Moore Machine ***

void BP1_f() { gLast_YEncoderValue = 6530; printf("State BP1\n"); }
void BP2_f() { gLast_YEncoderValue = 7030; printf("State BP2\n"); }
void BP3_f() { gLast_YEncoderValue = 7530; printf("State BP3\n"); }
void BP4_f() { gLast_YEncoderValue = 8030; printf("State BP4\n"); }

static State B(0);
static State BP1(1, BP1_f);
static State BR1(2);
static State BP2(3, BP2_f);
static State BR2(4);
static State BP3(5, BP3_f);
static State BR3(6);
static State BP4(7, BP4_f);
static State BR4(8);

static State* gStates[gN_States] = { &B, &BP1, &BR1, &BP2, &BR2, &BP3, &BR3, &BP4, &BR4 };
static int gInputs[gN_Inputs] = { 0, 1 };
static size_t gDel[gN_States][gN_Inputs] = { { 1, 0 },
                                             { 1, 2 },
                                             { 3, 2 },
                                             { 3, 4 },
                                             { 5, 4 },
                                             { 5, 6 },
                                             { 7, 6 },
                                             { 7, 8 },
                                             { 8, 8 } };

Moore_Machine<gN_States, gN_Inputs> gBridge_Machine(gStates, gInputs, gDel);

// ***     End Of Bridge Moore Machine      ***
