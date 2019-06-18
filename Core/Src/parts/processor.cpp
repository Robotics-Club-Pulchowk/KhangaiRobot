/*
 * processor.cpp
 *
 * Created : 1/9/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "processor.h"
#include "devs_config.h"
#include "interpolation.h"

#include "defines.h"


extern float gAuto_Ratio;
extern uint32_t gMax_Robo_Manual_Velocity;

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
extern State_Vars gStateS_Data;
extern State_Vars gStateT_Data;


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
extern Robo_States gStateS;
extern Robo_States gStateT;


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
Robo_States gStateS(&gStateS_Data, &gStateT);
Robo_States gStateT(&gStateT_Data, &gStateO);


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

        Throwing& thr = Throwing::get_Instance();
        sRobo_CPU.thrower_ = &thr;
        
        return sRobo_CPU;
}

int Processor::init(uint32_t dt_millis)
{
        sensor_->change_Sensors(curr_state_->get_ID());

        // Manual Components Initialization
        joy_stick_->init();

        // Throwing Initialization
        thrower_->init();

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

        vel.set_Values(vx, vy, 0);

        // This is for correcting units and the inverted co-ordinate system
        vy = vel.getY()  / (float)1000.0;
        if (gCurrent_Field == GameField::RED) {
                vx = -vel.getX() / (float)1000.0;
        }
        else if (gCurrent_Field == GameField::BLUE) {
                vx = vel.getX() / (float)1000.0;
        }
        else {
                vx = 0;
                vy = 0;
        }

        vel.setX(vx);
        vel.setY(vy);

        Field id = curr_state_->get_ID();
        if (id == Field::FIELD_Q) {
                vel.set_Values(0,0,0);
        }

        return vel;
}

Vec3<float> Processor::manual_control(JoyStick_Command& joy_cmd)
{
        Vec3<float> vels;

        vels = joy_cmd.vels;
        float v = (float)(gMax_Robo_Manual_Velocity) / 1000.0;
        vels = vels.mult_EW(v);
        // Set rw to 0 for testing purpose
        vels.setZ(0);

        uint8_t brake = joy_cmd.brake;
        uint8_t accel = joy_cmd.accel;

        float factor = 0.5;

        if (brake > 200) {
                factor = 0.25;
        }
        else if (accel > 200) {
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

const static uint8_t gSend_Extend_Num_Max = 10;
static uint8_t gSend_Extend_Num = gSend_Extend_Num_Max;


const uint32_t gArm_Retrieve_Count = 100;
static uint32_t gShagai_Thrown_Counter = 0;
static bool gShagai_Thrown_Counter_Start = false;

static bool gWait_For_Arm_To_Return = false;
static uint32_t gWait_For_Arm_To_Return_Count = 0;
static uint32_t gWait_For_Arm_To_Return_Count_Max = 100;
static bool gArm_Returned = false;
static bool gShagai_Thrown = false;

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

        JoyStick_Command joy_command;
        Control_Mode mode = Control_Mode::MANUAL;
        bool grip_shagai = false;
        bool reset_pos = false;
        bool thr_shg = false;
        bool actuate_arm = false;

        bool just_read = false;

        //* Read the parsed joystick data
        if (!joy_stick_->is_Empty()) {
                joy_command = joy_stick_->parse();
                just_read = true;

                mode = joy_command.mode;
                
                grip_shagai = joy_command.grip_shagai;
                reset_pos = joy_command.reset_pos;
                thr_shg = joy_command.throw_shagai;
                actuate_arm = joy_command.actuate_arm;
        }
        
        //* Process the data if read
        process(state, robot_state_vars);
        
        Field id = robot_state_vars->id;

        //* Return back to pick another shagai if shagai is thrown and the robot
        //* has retrieved it's arm
        if (id >= Field::FIELD_Q) {

                if (thr_shg) {
                        gShagai_Thrown_Counter_Start = true;
                }
                if (gShagai_Thrown_Counter_Start) {
                        ++gShagai_Thrown_Counter;
                        if (gShagai_Thrown_Counter > gArm_Retrieve_Count) {
                                gWait_For_Arm_To_Return = true;
                                gShagai_Thrown_Counter = 0;
                                gShagai_Thrown_Counter_Start = false;
                                gShagai_Thrown = true;
                        }
                }

        }

        //* Switch to manual mode automatically in state L
        if (id == Field::FIELD_L) {
                mode = Control_Mode::MANUAL;
        }
        else if (just_read && (mode == Control_Mode::NONE)) {
                mode = sLast_Mode;
        }

        uint8_t led_val = 0;

        //* Select Controller on the basis of control mode
        if (mode == Control_Mode::MANUAL) {

                if (!just_read) {
                        vels = last_vel.mult_EW(0.8);
                }
                else {
                        vels = manual_control(joy_command);
                }

                led_val = fill_Intensity(0, 15);
        }
        else if (mode == Control_Mode::AUTO) {
                Vec3<float> vels_auto = auto_control(state, vel_from_base, dt_millis);
                Vec3<float> vels_manual;
                
                if (!just_read) {
                        vels_manual = last_vel.mult_EW(0.8);
                }
                else {
                        vels_manual = manual_control(joy_command);
                }

                vels = lerp(vels_manual, vels_auto, gAuto_Ratio);

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

        send_ThrowCommand(grip_shagai, thr_shg, actuate_arm);

        //* Reset Position to field O if reset_pos command is obtained
        if (reset_pos) {
                reset_Position(robot_state_vars);
        }

        //* Change orientation if in field Q - S (excluding S)
        if (id >= Field::FIELD_Q && id < Field::FIELD_S) {
                // sensor_->update_IMUOffsets(Vec3<float>(-55.5, -111.5, -276.5));
                
                if (gCurrent_Field == GameField::RED) {
                        vels.setZ(curr_state_->get_AngOffset() - 9);
                }
                else if (gCurrent_Field == GameField::BLUE) {
                        vels.setZ(curr_state_->get_AngOffset() + 9);
                }
        }
        else {
                vels.setZ(curr_state_->get_AngOffset());
                // sensor_->update_IMUOffsets(Vec3<float>(-55.5, 16.5, -276.5));
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


static uint32_t gSend_Retrieve_Num = 10;
void Processor::reset_Position(State_Vars *&robot_state_vars)
{
        curr_state_ = &gStateO;
        sensor_->change_Sensors(curr_state_->get_ID());
        robot_state_vars = curr_state_->get_State();

        Vec2<float> p = curr_state_->get_Centre();
        Vec3<float> pos(p.getX(), p.getY(), 0); 
        sensor_->update_Position(pos);
        
        gSend_Extend_Num = gSend_Extend_Num_Max;
        gSend_Retrieve_Num = 10;
        
        retrieve_Arm();
        throw_Shagai(false);
        pass_Gerege(true);
        throw_Shagai(false);
}

void Processor::send_ThrowCommand(bool grip, bool throw_shg, bool act_arm)
{
        Field id = curr_state_->get_ID();

        if (id == Field::FIELD_O) {
                gSend_Retrieve_Num = 10;
        }

        if (gShagai_Thrown) {
                if (gArm_Returned) {
                        curr_state_ = &gStateS;
                        gShagai_Thrown = false;
                        gWait_For_Arm_To_Return_Count = 0;
                        gArm_Returned = false;
                }
                else {
                        if (gWait_For_Arm_To_Return) {
                                retrieve_Arm();
                                ++gWait_For_Arm_To_Return_Count;
                                gArm_Returned = false;
                        }

                        if (gWait_For_Arm_To_Return_Count >= gWait_For_Arm_To_Return_Count_Max) {
                                gWait_For_Arm_To_Return = false;
                                gArm_Returned = true;
                        }
                        
                }
        }

        //* Send Extend command if robot is in field Q
        if (id == Field::FIELD_Q) {
                extend_Arm();
                // actuate_Platform(true);
        }

        //* Send Rotate Command When in Field I
        if (id == Field::FIELD_I) {
                rotate_Gerege();
        }

        // if (id >= Field::FIELD_Q) {
                throw_Shagai(throw_shg);
        // }

        if (id <= Field::FIELD_O) {
                actuate_Arm(act_arm);
        }
        else {
                actuate_Platform(act_arm);
        }

        if (id >= Field::FIELD_O) {
                grip_Shagai(grip);
                // printf("Grip Shagai");
        }
        else if (id < Field::FIELD_O) {
                pass_Gerege(grip);
        }
}


static bool gSend_Throw_Command = false;
static uint8_t gSend_Throw_Num = 3;
const uint8_t gSend_Throw_Max = 3;

void Processor::throw_Shagai(bool throw_shagai)
{
        //* Throw Shagai if throw shagai flag obtained
        if (throw_shagai) {
                gSend_Throw_Command = true;
        }

        //* Send Pneumatic data if there is change in control mode
        if (gSend_Throw_Command) {
                if (--gSend_Throw_Num) {
                        thrower_->write((uint8_t)(Throwing_Commands::THROW));
                        HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_SET);
                }
                else {
                        gSend_Throw_Num = gSend_Throw_Max;
                        gSend_Throw_Command = false;
                        HAL_GPIO_WritePin(B_RedLED_GPIO_Port, B_RedLED_Pin, GPIO_PIN_RESET);
                }
        }
}

void Processor::extend_Arm()
{
        if (gSend_Extend_Num) {
                thrower_->write((uint8_t)(Throwing_Commands::EXTEND));
                --gSend_Extend_Num;
        }
}

void Processor::retrieve_Arm()
{
        if (gSend_Retrieve_Num) {
                thrower_->write((uint8_t)(Throwing_Commands::RETRIEVE));
                --gSend_Retrieve_Num;
        }
}

static bool gSend_Grip_Command = false;
static uint8_t gSend_Grip_Command_Num = 3;
static uint8_t gSend_Grip_Command_Num_Max = 3;

void Processor::grip_Shagai(bool grip_shagai)
{
        //* Throw Shagai if throw shagai flag obtained
        if (grip_shagai) {
                gSend_Grip_Command = true;
        }

        //* Send Pneumatic data if there is change in control mode
        if (gSend_Grip_Command) {
                if (--gSend_Grip_Command_Num) {
                        thrower_->write((uint8_t)(Throwing_Commands::GRIP));
                }
                else {
                        gSend_Grip_Command_Num = gSend_Grip_Command_Num_Max;
                        gSend_Grip_Command = false;
                }
        }
}

static bool gSend_Gerege_Pass_Command = false;
static uint8_t gSend_Gerege_Pass_Command_Num = 3;
static uint8_t gSend_Gerege_Pass_Command_Num_Max = 3;

void Processor::pass_Gerege(bool pass)
{
        //* Throw Shagai if throw shagai flag obtained
        if (pass) {
                gSend_Gerege_Pass_Command = true;
        }

        //* Send Pneumatic data if there is change in control mode
        if (gSend_Gerege_Pass_Command) {
                if (--gSend_Gerege_Pass_Command_Num) {
                        thrower_->write((uint8_t)(Throwing_Commands::PASS_GEREGE));
                }
                else {
                        gSend_Gerege_Pass_Command_Num = gSend_Gerege_Pass_Command_Num_Max;
                        gSend_Gerege_Pass_Command = false;
                }
        }
}

static bool gSend_Gerege_Rotate_Command = false;
static uint8_t gSend_Gerege_Rotate_Command_Num = 3;
static uint8_t gSend_Gerege_Rotate_Command_Num_Max = 3;

void Processor::rotate_Gerege()
{
        if (--gSend_Gerege_Rotate_Command_Num) {
                thrower_->write((uint8_t)(Throwing_Commands::PASS_GEREGE));
        }
        else {
                gSend_Gerege_Rotate_Command_Num = gSend_Gerege_Rotate_Command_Num_Max;
                gSend_Gerege_Rotate_Command = false;
        }
}

static bool gSend_Actuate_Arm_Cmd = false;
static uint8_t gSend_Actuate_Arm_Cmd_Num = 3;
static uint8_t gSend_Actuate_Arm_Cmd_Num_Max = 3;
void Processor::actuate_Arm(bool act_arm)
{
        if (act_arm) {
                gSend_Actuate_Arm_Cmd = true;
        }

        if (gSend_Actuate_Arm_Cmd) {
                if (--gSend_Actuate_Arm_Cmd_Num) {
                        thrower_->write((uint8_t)(Throwing_Commands::ACTUATE));
                }
                else {
                        gSend_Actuate_Arm_Cmd_Num = gSend_Actuate_Arm_Cmd_Num_Max;
                        gSend_Actuate_Arm_Cmd = false;
                }
        }
}

static bool gSend_Actuate_Platform_Cmd = false;
static uint8_t gSend_Actuate_Platform_Cmd_Num = 3;
static uint8_t gSend_Actuate_Platform_Cmd_Num_Max = 3;
void Processor::actuate_Platform(bool act_arm)
{
        if (act_arm) {
                gSend_Actuate_Platform_Cmd = true;
        }

        if (gSend_Actuate_Platform_Cmd) {
                if (--gSend_Actuate_Platform_Cmd_Num) {
                        thrower_->write((uint8_t)(Throwing_Commands::MOVE_PLATFORM_RIGHT));
                }
                else {
                        gSend_Actuate_Platform_Cmd_Num = gSend_Actuate_Platform_Cmd_Num_Max;
                        gSend_Actuate_Platform_Cmd = false;
                }
        }
}

static uint8_t fill_Intensity(uint8_t red, uint8_t blue)
{
        return (red | (blue << 4));
}
