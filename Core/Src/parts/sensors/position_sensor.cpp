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
#include "bridge.h"

static Exp_Smooth gXLidarAlpha35(0.35);
static Exp_Smooth gYLidarAlpha35(0.35);

//* Required for changing field

// Starting Y-Position
float gLast_YEncoderValue = 300;
// Starting X-Position
static float gLast_XEncoderValue = 0;

//*

// Last Position
Vec3<float> gLastPosition(gLast_XEncoderValue, gLast_YEncoderValue, 0);

Kalman_Vars gEncoders_KV;
Kalman_Vars gXLidarEncoder_KV;
Kalman_Vars gYLidarEncoder_KV;


PositionSensor& PositionSensor::get_Instance()
{
        static PositionSensor sPSensor_Instance;

        return sPSensor_Instance;
}

int PositionSensor::init(uint32_t dt_millis)
{
        Encoders_Init();
        Lidars_Init();

        return 0;
}

static bool gY_Lidar_Used = false;

Vec3<float> PositionSensor::read_Position(Vec3<float> ori, Vec3<float> base_state, const State_Vars *sv, uint32_t dt_millis)
{
        // First Collect Data from all available position sensors
        // 0 -> x, 1 -> y
        Vec2<float> free_wheel;
        float lidar[2];

        bool x_lidar_used(false), y_lidar_used(false);
        bool x_enc_used(false), y_enc_used(false);
        gY_Lidar_Used = false;

        for (uint8_t i = 0; i < sensor_count_; ++i) {
                if (p_sensors_[i]->get_Name() == SensorName::XEncoder) {
                        free_wheel.setX(-p_sensors_[i]->read());
                        x_enc_used = true;
                }
                else if (p_sensors_[i]->get_Name() == SensorName::YEncoder) {
                        free_wheel.setY(p_sensors_[i]->read());
                        y_enc_used = true;
                }
                else if (p_sensors_[i]->get_Name() == SensorName::XLidar) {
                        lidar[0] = p_sensors_[i]->read();
                        x_lidar_used = true;
                        // printf("Time : %ld\tXLidar : %ld\n", HAL_GetTick(), (int32_t)lidar[0]);
                }
                else if (p_sensors_[i]->get_Name() == SensorName::YLidar) {
                        lidar[1] = p_sensors_[i]->read();
                        // printf("Time : %ld\tYLidar : %ld\t\t", HAL_GetTick(), (int32_t)lidar[1]);
                        y_lidar_used = true;
                        gY_Lidar_Used = true;
                }
        }
        // Report error if there are no available sensors in any axes
        if (!(x_lidar_used || x_enc_used)) {
                error(Error::DEVICE_ERROR);
                printf("No Data Available on x-direction!!\n");
        }
        if (!(y_lidar_used || y_enc_used)) {
                error(Error::DEVICE_ERROR);
                printf("No Data Available on y-direction!!\n");
        }

        if (x_lidar_used || y_lidar_used) {
                process_LidarData(ori, lidar, sv);
        }

        // Smooth Lidar Data if used else clear the smoothie
        if (x_lidar_used) {
                lidar[0] = gXLidarAlpha35.smooth(lidar[0]);
        }
        else {
                gXLidarAlpha35.clear();
        }
        if (gY_Lidar_Used) {
                lidar[1] = gYLidarAlpha35.smooth(lidar[1]);
        }
        else {
                gYLidarAlpha35.clear();
        }


        // Calculate the movement of the body with respect to the body frame
        Vec2<float> enc = rotate_EncData(ori, free_wheel);
        float ex = enc.getX();
        float ey = -enc.getY();

        // Fuse the data with the data from lidar that gives movement with
        // respect to the navigation frame

        float x(0), y(0);
        if (!x_lidar_used) {
                x = gLastPosition.getX() + ex;
                xlidar_enc_fuser_.clear();
        }
        else {
                x = xlidar_enc_fuser_.filter(lidar[0], ex, dt_millis);
        }
        if (!gY_Lidar_Used) {
                y = gLastPosition.getY() + ey;
                ylidar_enc_fuser_.clear();
        }
        else {
                y = ylidar_enc_fuser_.filter(lidar[1], ey, dt_millis);
        }

        // x and y are in mm
        gLastPosition.set_Values(x, y, 0);

        // gLastPosition.print();
        // printf("\n");

        return gLastPosition;
}

void PositionSensor::update_State(Vec3<float> state)
{
        gLastPosition = state;
}

void PositionSensor::process_LidarData(Vec3<float> ori, float (&lidar)[2], const State_Vars *sv)
{
        // Process Lidar Data according to the field the robot is in

        // Assumptions:
        // lidar[0] => XLidar Data
        // lidar[1] => YLidar Data

        // printf("%ld\n", (int32_t)lidar[0]);

        float yaw = ori.getZ();

        float jungle_pole_dist = 1255;
        float bridge_pole_dist = 900;
        float ylidar_lower_value = 100;
        float tol = 100;

        Field id = sv->id;

        // Processing XLidar data
        if ((int)id >= (int)(Field::FIELD_P)) {
                // The fence is at most 2000 mm from robot
                if (lidar[0] < 2000) {
                        // Lower Fence distance
                        lidar[0] += 2500;
                }
        }
        else if (fabs(yaw) < 5) {
                // Compensating y-values at poles using xlidar's data
                if (id == Field::FIELD_B || id == Field::FIELD_F ||
                    (id == Field::FIELD_C && lidar[0] < 400)) {
                        if (lidar[0] < 1000) {
                                lidar[0] += jungle_pole_dist;

                                // Compensating the Y value based on lidar data
                                if (id == Field::FIELD_B ||
                                    (id == Field::FIELD_C && lidar[0] < 400)) {
                                        gLast_YEncoderValue = 2030.0;
                                }
                                else if (id == Field::FIELD_F) {
                                        gLast_YEncoderValue = 5030.0;
                                }
                        }
                }
                // Bridge is in between field H and I
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

        if (fabs(yaw) < 5) {
                // Compensating y-values at poles using ylidar's data 
                if (id == Field::FIELD_A) {
                        if (lidar[1] < 1500 && lidar[1] > ylidar_lower_value) {
                                lidar[1] = 2000 - lidar[1];
                        }
                        else {
                                gY_Lidar_Used = false;
                        }
                }
                else if (id == Field::FIELD_C) {
                        if (lidar[1] < 1000 && lidar[1] > ylidar_lower_value) {
                                lidar[1] = 3500 - lidar[1];
                        }
                        else {
                                gY_Lidar_Used = false;
                        }
                }
                // else if (id == Field::FIELD_E) {
                //         if (lidar[1] < 1000 && lidar[1] > ylidar_lower_value) {
                //                 lidar[1] = 5000 - lidar[1];
                //         }
                //         else {
                //                 gY_Lidar_Used = false;
                //         }
                // }
                // else if (id == Field::FIELD_G) {
                //         if (lidar[1] < 1000 && lidar[1] > ylidar_lower_value) {
                //                 lidar[1] = 6500 - lidar[1];
                //         }
                //         else {
                //                 gY_Lidar_Used = false;
                //         }
                // }
                else if (id == Field::FIELD_I || id == Field::FIELD_J ||
                        id == Field::FIELD_O) {
                        // Need to compensate for shagai too
                        if (lidar[1] > ylidar_lower_value) {
                                lidar[1] = 10000 - lidar[1];
                        }
                }
                else {
                        gY_Lidar_Used = false;
                }
        }
        else {
                gY_Lidar_Used = false;
        }
}

Vec2<float> PositionSensor::rotate_EncData(Vec3<float> ori, Vec2<float> enc)
{
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

        // Converting to Vec3 since we only have support for multiplication
        // between nx3 matrix and a vec3
        Vec3<float> free_wheel(enc.getX(), enc.getY(), 0);

        // https://ocw.mit.edu/courses/mechanical-engineering/2-017j-design-of-electromechanical-robotic-systems-fall-2009/course-text/MIT2_017JF09_ch09.pdf
        // R(r, p, y)
        float R[3][3] = { { c_p*c_r,                c_p*s_r,               -s_p},
                          {-c_y*s_r + s_y*s_p*c_r,  c_y*c_r + s_y*s_p*s_r,  s_y*c_p},
                          { s_y*s_r + c_y*s_p*c_r, -s_y*c_r + c_y*s_p*s_r,  c_y*c_p}, };

        Mat Rm(R);

        Mat d_pos = Rm.trans() * free_wheel;

        float ex = d_pos.at(0,0);
        float ey = d_pos.at(1,0);

        return Vec2<float>(ex, ey);
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
        measure_error.at(0,0) = 50;

        gXLidarEncoder_KV.set_F(state_model);
        gXLidarEncoder_KV.set_B(control_model);
        gXLidarEncoder_KV.set_H(obs_model);
        gXLidarEncoder_KV.set_I(2);
        gXLidarEncoder_KV.set_P(priori_error);
        gXLidarEncoder_KV.set_Q(process_error);
        gXLidarEncoder_KV.set_R(measure_error);

        return 0;
}


int init_YLidarEncoderKalman(uint32_t dt_millis)
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
        measure_error.at(0,0) = 50;

        gYLidarEncoder_KV.set_F(state_model);
        gYLidarEncoder_KV.set_B(control_model);
        gYLidarEncoder_KV.set_H(obs_model);
        gYLidarEncoder_KV.set_I(2);
        gYLidarEncoder_KV.set_P(priori_error);
        gYLidarEncoder_KV.set_Q(process_error);
        gYLidarEncoder_KV.set_R(measure_error);

        return 0;
}

int init_EncodersKalman(uint32_t dt_millis)
{
        return 0;
}
