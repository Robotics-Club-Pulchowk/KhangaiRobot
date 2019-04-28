/*
 * processor.cpp
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "processor.h"
#include "devs_config.h"

extern State_Vars gStateA_Data;
extern State_Vars gStateB_Data;
extern State_Vars gStateC_Data;
extern State_Vars gStateD_Data;
extern State_Vars gStateE_Data;
extern State_Vars gStateF_Data;
extern State_Vars gStateG_Data;
extern State_Vars gStateH_Data;
extern State_Vars gStateI_Data;
extern State_Vars gStateJ_Data;
extern State_Vars gStateK_Data;
extern State_Vars gStateL_Data;

extern State_Vars gStateO_Data;
extern State_Vars gStateP_Data;
extern State_Vars gStateQ_Data;
extern State_Vars gStateQ1_Data;
extern State_Vars gStateQ2_Data;
extern State_Vars gStateR_Data;
extern State_Vars gStateR1_Data;
extern State_Vars gStateR2_Data;


extern Robo_States gStateA;
extern Robo_States gStateB;
extern Robo_States gStateC;
extern Robo_States gStateD;
extern Robo_States gStateE;
extern Robo_States gStateF;
extern Robo_States gStateG;
extern Robo_States gStateH;
extern Robo_States gStateI;
extern Robo_States gStateJ;
extern Robo_States gStateK;
extern Robo_States gStateL;

extern Robo_States gStateO;
extern Robo_States gStateP;
extern Robo_States gStateQ;
extern Robo_States gStateQ1;
extern Robo_States gStateQ2;
extern Robo_States gStateR;
extern Robo_States gStateR1;
extern Robo_States gStateR2;


Robo_States gStateA(&gStateA_Data, &gStateB);
Robo_States gStateB(&gStateB_Data, &gStateC);
Robo_States gStateC(&gStateC_Data, &gStateD);
Robo_States gStateD(&gStateD_Data, &gStateE);
Robo_States gStateE(&gStateE_Data, &gStateF);
Robo_States gStateF(&gStateF_Data, &gStateG);
Robo_States gStateG(&gStateG_Data, &gStateH);
Robo_States gStateH(&gStateH_Data, &gStateI);
Robo_States gStateI(&gStateI_Data, &gStateJ);

Robo_States gStateJ(&gStateJ_Data, &gStateK);
Robo_States gStateK(&gStateK_Data, &gStateL);
Robo_States gStateL(&gStateL_Data, &gStateO);

Robo_States gStateO(&gStateO_Data, &gStateP);
Robo_States gStateP(&gStateP_Data, &gStateQ);
Robo_States gStateQ(&gStateQ_Data, &gStateR);
Robo_States gStateQ1(&gStateQ1_Data, &gStateR1);
Robo_States gStateQ2(&gStateQ2_Data, &gStateR2);
Robo_States gStateR(&gStateR_Data, &gStateR);
Robo_States gStateR1(&gStateR1_Data, &gStateR);
Robo_States gStateR2(&gStateR2_Data, &gStateR);


static uint8_t fill_Intensity(uint8_t red, uint8_t blue);


void init_GameField();

Processor& Processor::get_Instance(State_Sensor *sen)
{
        static Processor sRobo_CPU;
        
        init_GameField();

        sRobo_CPU.curr_state_ = &gStateA;
        sRobo_CPU.sensor_ = sen;

        JoyStick& joy = JoyStick::get_Instance(&huart2);
        sRobo_CPU.joy_stick_ = &joy;
        
        return sRobo_CPU;
}

int Processor::init(uint32_t dt_millis)
{
        sensor_->change_Sensors(curr_state_->get_ID());

        // Manual Components Initialization
        joy_stick_->init();

        return 0;
}

void Processor::process(Vec3<float> state, State_Vars *&robot_state_vars)
{
        uint8_t bounds = sensor_->get_Bounds();
        if (curr_state_->nextStateReached(state, bounds)) {
                update_State(bounds);
                sensor_->change_Sensors(curr_state_->get_ID());
        }

        robot_state_vars = curr_state_->get_State();
}

Vec3<float> Processor::auto_control(Vec3<float> state, Vec3<float> vel_from_base, uint32_t dt_millis)
{
        Vec3<float> vel;

        // Get new velocity for the robot
        Vec2<float> v_polar = curr_state_->calc_Velocity(state, vel_from_base, dt_millis);
        float v = v_polar.getX();
        float theta = v_polar.getY();

        float phi = state.getZ();

        phi /= (float)57.3;     // to rads
        // Ensuring: -pi < phi <= pi
        // Since sin and cos gives value between -1 and 1
        phi = atan2(sin(phi), cos(phi));

        // ! Need to look here more
        float vx = v*sin(theta - phi);
        float vy = v*cos(theta - phi);
        float rw = (phi)*0.3;

        vel.set_Values(vx, vy, rw);

        // This is for correcting units and the inverted co-ordinate system
        vx = -vel.getX() / (float)1000.0;
        vy = vel.getY()  / (float)1000.0;
        vel.setX(vx);
        vel.setY(vy);

        return vel;
}

Vec3<float> Processor::manual_control(JoyStick_Command& joy_cmd)
{
        Vec3<float> vels;

        vels = joy_cmd.vels;
        // Set rw to 0 for testing purpose
        vels.setZ(0);

        uint8_t brake = joy_cmd.brake;
        float factor = (255.0 - (float)brake)/255.0;

        if (factor < 0.2) {
                factor = 0;
        }

        vels = vels.mult_EW(factor);

        uint8_t accel = joy_cmd.accel;
        if (accel > 200) {
                factor = 1;
        }
        else {
                factor = 0.5;
        }

        vels = vels.mult_EW(factor);

        return vels;
}

static Control_Mode sLast_Mode = Control_Mode::NONE;
static bool gSend_LED_Data = false;
static uint8_t gSend_LED_Data_Num = 5;
const uint8_t gSend_LED_Data_Max = 5;

static bool gSend_Pneumatic_Data = false;
static uint8_t gSend_Pneumatic_Data_Num = 5;
const uint8_t gSend_Pneumatic_Data_Max = 5;

static uint8_t gSend_Extend_Num = 10;

Vec3<float> Processor::control(Vec3<float> state,
                               Vec3<float> vel_from_base,
                               Vec3<float> last_vel,
                               State_Vars *&robot_state_vars,
                               uint32_t dt_millis)
{
        Vec3<float> vels;
        // This is the second algorithm used for moving the robot.
        // Algorithm Info:
        //      1) Minimum Acceleration Trajectory
        //      2) Smooth Transition
        process(state, robot_state_vars);

        JoyStick_Command joy_command;
        Control_Mode mode = Control_Mode::MANUAL;
        bool grip_shagai = false;
        bool reset_pos = false;
        bool throw_shagai = false;

        bool just_read = false;

        //* Read the parsed joystick data
        if (!joy_stick_->is_Empty()) {
                joy_command = joy_stick_->parse();
                just_read = true;

                mode = joy_command.mode;
                
                grip_shagai = joy_command.grip_shagai;
                reset_pos = joy_command.reset_pos;
                throw_shagai = joy_command.throw_shagai;
        }

        //* Switch to manual mode automatically in state L
        Field id = robot_state_vars->id;
        if (id == Field::FIELD_L) {
                mode = Control_Mode::MANUAL;
        }
        else if (just_read && (mode == Control_Mode::NONE)) {
                mode = sLast_Mode;
        }

        uint8_t led_val = 0;

        //* Select Controller on the basis of control mode
        if (mode == Control_Mode::MANUAL) {

                float phi = state.getZ();
                phi /= (float)57.3;     // to rads

                if (!just_read) {
                        vels = last_vel.mult_EW(0.8);
                }
                else {
                        vels = manual_control(joy_command);
                }

                float rw = (phi)*0.1;
                vels.setZ(rw);

                led_val = fill_Intensity(0, 15);
        }
        else if (mode == Control_Mode::AUTO) {
                vels = auto_control(state, vel_from_base, dt_millis);
                led_val = fill_Intensity(15, 5);
        }

        //* Determine if there is change in control mode from the previous one
        if (sLast_Mode != mode) {
                gSend_LED_Data = true;
                sLast_Mode = mode;
        }

        //* Send LED data if there is change in control mode
        if (gSend_LED_Data) {
                if (--gSend_LED_Data_Num) {
                        gLED_Strip.write(&led_val, 1);
                }
                else {
                        gSend_LED_Data_Num = gSend_LED_Data_Max;
                        gSend_LED_Data = false;
                }
        }

        //* Send Extend command if robot is in field Q
        if (id == Field::FIELD_Q) {
                if (gSend_Extend_Num) {
                        uint8_t extend = 0x01;
                        gPneumatic.write(&extend, 1);
                        --gSend_Extend_Num;
                }
        }

        //* Throw Shagai if throw shagai flag obtained
        if (throw_shagai) {
                gSend_Pneumatic_Data = true;
        }

        //* Send Pneumatic data if there is change in control mode
        if (gSend_Pneumatic_Data) {
                if (--gSend_Pneumatic_Data_Num) {
                        uint8_t throw_shagai_cmd = 0x03;
                        gPneumatic.write(&throw_shagai_cmd, 1);
                }
                else {
                        gSend_Pneumatic_Data_Num = gSend_Pneumatic_Data_Max;
                        gSend_Pneumatic_Data = false;
                }
        }

        //* Reset Position to field O
        if (reset_pos) {
                curr_state_ = &gStateO;
                sensor_->change_Sensors(curr_state_->get_ID());
                robot_state_vars = curr_state_->get_State();
                Vec2<float> p = curr_state_->get_Centre();
                Vec3<float> pos(p.getX(), p.getY(), 0); 
                sensor_->update_Position(pos);
                // HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_SET);
        }

        return vels;
}

void Processor::update_State(uint8_t bounds)
{
        Field id = curr_state_->get_ID();

        if (id == Field::FIELD_Q) {
                if ((bounds & (1 << (int)(Face::_6))) && (bounds & (1 << (int)(Face::_7)))) {
                        // field R
                        curr_state_ = &gStateR;
                }
                else if (bounds & (1 << (int)(Face::_6))) {
                        // field Q1
                        curr_state_ = &gStateQ1;
                }
                else if (bounds & (1 << (int)(Face::_7))) {
                        // field Q2
                        curr_state_ = &gStateQ2;
                }
                return;
        }
        else if (id == Field::FIELD_Q1) {
                if ((bounds & (1 << (int)(Face::_6))) && (bounds & (1 << (int)(Face::_7)))) {
                        // field R
                        curr_state_ = &gStateR;
                }
                else if (bounds & (1 << (int)(Face::_7))) {
                        // field Q2
                        curr_state_ = &gStateQ2;
                }
                return;
        }
        else if (id == Field::FIELD_Q2) {
                if ((bounds & (1 << (int)(Face::_6))) && (bounds & (1 << (int)(Face::_7)))) {
                        // field R
                        curr_state_ = &gStateR;
                }
                else if (bounds & (1 << (int)(Face::_6))) {
                        // field Q1
                        curr_state_ = &gStateQ1;
                }
                return;
        }

        curr_state_ = curr_state_->get_NextState();
}

static uint8_t fill_Intensity(uint8_t red, uint8_t blue)
{
        return (red | (blue << 4));
}
