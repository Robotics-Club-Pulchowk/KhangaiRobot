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
// extern State_Vars gStateM_Data;
// extern State_Vars gStateN_Data;

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
// extern Robo_States gStateM;
// extern Robo_States gStateN;

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
Robo_States gStateL(&gStateL_Data, &gStateL);

// Robo_States gStateM(&gStateM_Data, &gStateN);
// Robo_States gStateN(&gStateN_Data, &gStateN);

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

void Processor::process(Vec3<float> state, State_Vars *&robot_state_vars_)
{
        uint8_t bounds = sensor_->get_Bounds();
        if (curr_state_->nextStateReached(state, bounds)) {
                update_State();
                sensor_->change_Sensors(curr_state_->get_ID());
        }

        robot_state_vars_ = curr_state_->get_State();
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

static uint8_t sLast_Mode = 0;

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
        bool auto_mode = true;
        bool manual_mode = false;
        bool grip_shagai = false;
        bool reset_pos = false;

        bool just_read = false;
        
        if (!joy_stick_->is_Empty()) {
                joy_command = joy_stick_->parse();
                just_read = true;

                manual_mode = joy_command.manual_mode;
                auto_mode = joy_command.auto_mode;
                
                grip_shagai = joy_command.grip_shagai;
                reset_pos = joy_command.reset_pos;
        }

        Field id = robot_state_vars->id;
        if (id == Field::FIELD_L) {
                manual_mode = true;
                auto_mode = false;
        }

        uint8_t led_val = 0;
        uint8_t curr_mode = 1;

        if (manual_mode) {
                curr_mode = 0;
                if (!just_read) {
                        vels = last_vel.mult_EW(0.8);
                }
                else {
                        vels = manual_control(joy_command);
                }
                led_val = fill_Intensity(0, 15);
        }
        else if (auto_mode) {
                curr_mode = 1;
                vels = auto_control(state, vel_from_base, dt_millis);
                led_val = fill_Intensity(15, 5);
        }

        if (sLast_Mode != curr_mode) {
                gLED_Strip.write(&led_val, 1);
        }

        sLast_Mode = curr_mode;

        return vels;
}

void Processor::update_State()
{
        curr_state_ = curr_state_->get_NextState();
}

static uint8_t fill_Intensity(uint8_t red, uint8_t blue)
{
        return (red | (blue << 4));
}
