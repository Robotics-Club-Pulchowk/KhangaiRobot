/*
 * state_sensor.cpp
 *
 * Created : 1/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "state_sensor.h"
#include "smoothing_algorithm.h"
#include "kalman.h"
#include "devs_config.h"
#include "robo_states.h"

#include "error.h"
#include "defines.h"
#include "logger.h"


static Vec3<float> gOmega_Bias;

int init_OriFilters(uint32_t dt_millis);

State_Sensor& State_Sensor::get_Instance()
{
        static State_Sensor sState_Sensor_Instance;

        PositionSensor &p_sens = PositionSensor::get_Instance();
        sState_Sensor_Instance.p_sensor_ = &p_sens;

        Bound_Box &box = Bound_Box::get_Instance();
        sState_Sensor_Instance.bound_box_ = &box;

        return sState_Sensor_Instance;
}


/**
 * @brief Function that initializes all the required components for the robot's
 *        State Sensor(x, y and yaw)
 * @retval 0 if inititialized properly
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Call the devices initializations in correct order
 * 2) Call the software initializations for utilities like smoothies and filters
 * </pre>
 */
//! Not yet implemented
int State_Sensor::init(uint32_t dt_millis)
{

        IMU_Init();
        Stepper_Init();
        Lidars_Init();
        

        bool mpu_is_ready(false), hmc_is_ready(false);
        
        // Read MPU6050 if it is ready to be read
        if (MPU6050_Read_NormAxes(&Body_IMU) == HAL_OK) {
                mpu_is_ready = true;
        }

        // Read HMC5883 if it is ready to be read
        if (HMC5883_Read(&Body_HMC) == HAL_OK) {
                hmc_is_ready = true;
        }

        if (!mpu_is_ready || !hmc_is_ready) {
                //! Error Condition
                error(Error::PERIPHERAL_ERROR);
        }

        // init_OriFilters(dt_millis);

        // Use all the available sensors initially
        // Available Sensors : 
        //              1. XEncoder
        //              2. YEncoder
        //              3. XLidar

        add_PSensor(&gXEncoder);
        add_PSensor(&gYEncoder);
        add_PSensor(&gXLidar);

        p_sensor_->init(dt_millis);
        
        // HMC5883_Calibrate(&Body_HMC, &IMU_Stepper, 200);
        gOmega_Bias = MPU6050_Calc_OmegaBias(&Body_IMU, 1000);

        bound_box_->init();
        bounds_ = 0;

        // Initialize variables
        is_first_ori_ = true;
        
        return 0;
}


/**
 * @brief Function that provides the state of the robot
 * @param base_state This parameter is considered as one of the possible state
 *                   of robot. This will be passed to both Orientation_Sensor
 *                   and the Position_Sensor for further use.
 * @param dt_millis : The time period at which this function is called periodically
 * 
 * @retval state of the robot
 * 
 * @note The state of our robot is defined by x,y distance from fences and the
 *       Robot's heading
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Read Orientation of the robot with Orientation_Sensor
 * 2) Read Position of the robot with Position_Sensor
 * </pre>
 */
Vec3<float> State_Sensor::read_State(Vec3<float> base_state, const State_Vars *curr_sv, uint32_t dt_millis)
{
        Vec3<float> state;
        Vec3<float> pos;
        Vec3<float> dori;

        Vec3<float> ori = read_Orientation(base_state, dt_millis);
        // ori.print();
        // printf("\n");

        if (is_first_ori_) {
                is_first_ori_ = false;
                first_ori_ = ori;
        }
        else {
                dori = ori - first_ori_;
                pos = p_sensor_->read_Position(dori, base_state, curr_sv, dt_millis);

                float psi = dori.getZ() / 57.3;
                psi = atan2(sin(psi), cos(psi)) * 57.3;

                state.set_Values(pos.getX(), pos.getY(), psi);   // mm mm deg

                state = compensate_Bounds(state, ori, curr_sv);
        }

        return state;
}

/**
 ** Compensates the position values based on the readings from limit switches
 */
Vec3<float> State_Sensor::compensate_Bounds(Vec3<float> pos, Vec3<float> ori, const State_Vars *curr_sv)
{
        Field id = curr_sv->id;

        if ((int)id >= (int)(Field::FIELD_J)) {

                bound_box_->update();
                bounds_ = bound_box_->get_Bounds();

                // printf("%x\t", bounds_);

                if (id == Field::FIELD_J || id == Field::FIELD_L) {
                        //* Look for the robot to touch the fence with face 6
                        if (bounds_ & (1 << (int)(Face::_6))) {
                                //* Face 6 has touched the fence
                                pos.setY(8350);
                                
                                //* Read face 6 value and reset angle here
                                uint8_t face6 = bound_box_->get_Bound(6);
                                if ((face6 & (1 << 0)) && (face6 & (1 << 1))) {
                                        // first_ori_ = ori;
                                        // pos.setZ(0);
                                }
                        }
                }

                else if (id == Field::FIELD_K) {
                        //* Look for the robot to touch the fence with face 6 & 8
                        if (bounds_ & (1 << (int)(Face::_6))) {
                                //* Face 6 has touched the fence
                                pos.setY(8350);
                        }
                        if (bounds_ & (1 << (int)(Face::_8))) {
                                //* Face 8 has touched the fence
                                pos.setX(6000);
                        }
                        
                        //* Read face 6 & 8 value and reset angle here
                        uint8_t face6 = bound_box_->get_Bound(6);
                        if ((face6 & (1 << 0)) && (face6 & (1 << 1))) {
                                // first_ori_ = ori;
                                // pos.setZ(0);
                        }
                        uint8_t face8 = bound_box_->get_Bound(8);
                        if ((face8 & (1 << 0)) && (face8 & (1 << 1))) {
                                // first_ori_ = ori;
                                // pos.setZ(0);
                        }
                }
                else if (id == Field::FIELD_Q || id == Field::FIELD_Q1 || id == Field::FIELD_Q2) {
                        //* Look for the robot to touch the fence with face 6 & 8
                        if (bounds_ & (1 << (int)(Face::_6))) {
                                //* Face 6 has touched the fence
                                pos.setY(4350);
                        }
                        if (bounds_ & (1 << (int)(Face::_7))) {
                                //* Face 8 has touched the fence
                                pos.setX(3800);
                        }
                        
                }

                p_sensor_->update_State(pos);
        }

        return pos;
}


static Exp_Smooth gXAccelAlpha35(0.35);
static Exp_Smooth gYAccelAlpha35(0.35);
static Exp_Smooth gZAccelAlpha35(0.35);

static Exp_Smooth gXGyroAlpha35(0.35);
static Exp_Smooth gYGyroAlpha35(0.35);
static Exp_Smooth gZGyroAlpha35(0.35);

static Exp_Smooth gXMagAlpha35(0.35);
static Exp_Smooth gYMagAlpha35(0.35);
static Exp_Smooth gZMagAlpha35(0.35);

static Kalman_Vars gAccel_Gyro;

static Kalman_Filter gRoll_Filter(&gAccel_Gyro, init_OriFilters);
static Kalman_Filter gPitch_Filter(&gAccel_Gyro, init_OriFilters);
static Kalman_Filter gYaw_Filter(&gAccel_Gyro, init_OriFilters);

int init_OriFilters(uint32_t dt_millis)
{

        float dt = (float)dt_millis / 1000.0;

        // For Orientation Purpose
        Mat state_model(2,2);
        state_model.at(0,0) = 1;
        state_model.at(0,1) = -dt;
        state_model.at(1,0) = 0;
        state_model.at(1,1) = 1;

        Mat control_model(2,1);
        control_model.at(0,0) = dt;
        control_model.at(1,0) = 0;

        Mat obs_model(1,2);
        obs_model.at(0,0) = 1;
        obs_model.at(0,1) = 0;

        Mat priori_error(2,2);
        priori_error.at(0,0) = 10;
        priori_error.at(0,1) = 0;
        priori_error.at(1,0) = 0;
        priori_error.at(1,1) = 10;

        Mat process_error(2,2);
        process_error.at(0,0) = 0.001;
        process_error.at(0,1) = 0;
        process_error.at(1,0) = 0;
        process_error.at(1,1) = 0.003;

        Mat measure_error(1,1);
        measure_error.at(0,0) = 0.04;

        gAccel_Gyro.set_F(state_model);
        gAccel_Gyro.set_B(control_model);
        gAccel_Gyro.set_H(obs_model);
        gAccel_Gyro.set_I(2);
        gAccel_Gyro.set_P(priori_error);
        gAccel_Gyro.set_Q(process_error);
        gAccel_Gyro.set_R(measure_error);

        return 0;
}

/**
 * @brief Function that provides the orientation of the robot
 * @param base_state This parameter is considered as one of the possible state
 *                   of robot.
 * @param dt_millis : The time period at which this function is called periodically
 * 
 * @retval orientation of the robot in degrees
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Read the values from Accelerometer and the Gyroscope
 * </pre>
 */
Vec3<float> State_Sensor::read_Orientation(Vec3<float> base_state, uint32_t dt_millis)
{
        Vec3<float> accel;
        Vec3<float> gyro;
        Vec3<float> mag;
        Vec3<float> angles;

#ifdef _ENABLE_I2C_ERROR_DETECTION

        bool mpu_is_ready(true), hmc_is_ready(true);
        
        // Read MPU6050 if it is ready to be read
        if (MPU6050_Read_NormAxes(&Body_IMU) != HAL_OK) {
                mpu_is_ready = false;
        }

        // Read HMC5883 if it is ready to be read
        if (HMC5883_Read(&Body_HMC) != HAL_OK) {
                hmc_is_ready = false;
        }

        if (!mpu_is_ready || !hmc_is_ready) {
                //! Error Condition
                HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_SET);
                error(Error::PERIPHERAL_ERROR);
                IMU_Init();
                return angles;
        }
        else {
                HAL_GPIO_WritePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin, GPIO_PIN_RESET);
        }
#else
        MPU6050_Read_NormAxes(&Body_IMU);
        HMC5883_Read(&Body_HMC);
#endif  // _ENABLE_I2C_ERROR_DETECTION

        accel = Body_IMU.norm_a_axis;
        gyro = Body_IMU.norm_g_axis - gOmega_Bias;
        mag = Body_HMC.raw_axis;

        // (accel.mult_EW(1000)).print();
        // printf("    ");
        // (gyro.mult_EW(4)).print();
        // printf("    ");
        // mag.print();
        // printf("\n");

        // We will consider the rotation in order: psi-theta-phi
        float ax = gXAccelAlpha35.smooth(accel.getX());
        float ay = gYAccelAlpha35.smooth(accel.getY());
        float az = gZAccelAlpha35.smooth(accel.getZ());

        float gx = gXGyroAlpha35.smooth(gyro.getX());
        float gy = gYGyroAlpha35.smooth(gyro.getY());
        float gz = gZGyroAlpha35.smooth(gyro.getZ());

        float bx = gXMagAlpha35.smooth(mag.getX());
        float by = gYMagAlpha35.smooth(mag.getY());
        float bz = gZMagAlpha35.smooth(mag.getZ());
        
        float roll = atan2f(ay, az) * 57.3;

        roll = gRoll_Filter.filter(roll, gx, dt_millis);
        // printf("%ld,   ", (int32_t)(roll*1000));

        float sin_roll = sin(roll / 57.3);
        float cos_roll = cos(roll / 57.3);
        // Roll Compensated Pitch
        float pitch = atan2f((-ax), ay*sin_roll + az*cos_roll) * 57.3;

        pitch = gPitch_Filter.filter(pitch, gy, dt_millis);
        // printf("%ld,   ", (int32_t)(pitch*1000));

        float sin_pitch = sin(pitch / 57.3);
        float cos_pitch = cos(pitch / 57.3);
        // Tilt Compensated Yaw
        float yaw = atan2f((bz*sin_roll - by*cos_roll), (bx*cos_pitch + by*sin_roll*sin_pitch + bz*cos_roll*sin_pitch)) * 57.3;
        float yaw_before_kalman = yaw;

        // printf("%ld\t", (int32_t)(yaw));
        yaw = gYaw_Filter.filter(yaw, gz, dt_millis);
        // printf("%ld\n", (int32_t)(yaw));
        log_Angle(yaw_before_kalman, yaw);

        angles.set_Values(roll, pitch, yaw);

        return angles;
}

void State_Sensor::update_IMUOffsets(Vec3<float> offsets)
{
        Body_HMC.hard_iron_offset = offsets;
}

void State_Sensor::change_Sensors(Field field_id)
{
        switch (field_id) {
                case Field::FIELD_A : {      // State A
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_B : {      // State B
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_C : {      // State C
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_D : {      // State D
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_E : {      // State E
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_F : {      // State F
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_G : {      // State G
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_H : {      // State H
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_I : {      // State I
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->remove_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_J : {      // State J
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->remove_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_O : {      // State O
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->remove_Sensor(&gXLidar);
                        p_sensor_->add_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_P : {      // State I
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_Q : {      // State I
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->remove_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;
                
                case Field::FIELD_S : {      // State I
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->add_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                } break;

                default : {      // Default State
                        p_sensor_->add_Sensor(&gXEncoder);
                        p_sensor_->add_Sensor(&gYEncoder);
                        p_sensor_->remove_Sensor(&gXLidar);
                        p_sensor_->remove_Sensor(&gYLidar);
                }
        }
}

uint8_t State_Sensor::get_Bounds()
{
        return bounds_;
}
