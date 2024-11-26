#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "ins_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "buzzer.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
static BuzzzerInstance *chassis_motor_error_buzzer;

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令，发布中心发给底盘的

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
//后续把用于时间的变量都放在一个结构体//
 static float cnt=0;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy,sin_theta,cos_theta;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,未进行限幅
static float vt_lf_limit, vt_rf_limit, vt_lb_limit, vt_rb_limit; // 已进行限幅
static float power_limit;// 限幅系数
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static attitude_t *chassis_IMU_data;     // 云台IMU数据
static referee_info_t* referee_data; // 用于获取裁判系统的数据
static SuperCapInstance *cap;
static float  rotate_speed_buff;
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = 
            {
                .Kp = 6, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = 
            {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },

        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id = 0x201;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x202;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x203;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x204;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);



    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x300,
            .rx_id = 0x301,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化

    //referee_data   = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI

}

/**
 * @brief 底盘启动与否
 *        
 */
static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    {
        // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
}

/**
 * @brief 底盘模式及旋转速度设定
 *        
 */
static void ChassisRotateSet()
{
    cnt = (float32_t)DWT_GetTimeline_s();//用于变速小陀螺
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
        case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_recv.wz = 0;
        break;
        //系数待调
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            chassis_cmd_recv.wz =0;
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            chassis_cmd_recv.wz =0;
        break;
        default:
        break;
    }
}
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        
 */
static void MecanumCalculate()
{
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta; 
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    vt_lf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
    vt_rf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
}

/**
 * @brief 根据裁判系统对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    rotate_speed_buff = 4;
    static float  chassis_power_buff = 1.5;
    //power_data[0]= referee_data->PowerHeatData.buffer_energy;        // 缓冲能量

/*具体两个buff是多少待测试
    switch (chassis_cmd_recv.chassis_power_robot_level)
    {
    case 1:
        power_data[1]=60;  
        rotate_speed_buff = 3.7;
        chassis_power_buff = 1.0;
        break;
    case 2:
        power_data[1]=65;
        rotate_speed_buff = 3.7;
        chassis_power_buff = 1.0;  
        break;
    case 3:
        power_data[1]=70;  
        break;
    case 4:
        power_data[1]=75;  
        break;
    case 5:
        power_data[1]=80;  
        break;
    case 6:
        power_data[1]=85;  
        break;
    case 7:
        power_data[1]=90;  
        break;
    case 8:
        power_data[1]=95;  
        break;
    case 9:
        power_data[1]=100;  
        break;
    case 10:
        power_data[1]=100;  
        break;

    default:
        power_data[1]=referee_data->GameRobotState.chassis_power_limit; 
        break; 
    }


    // rotate_speed_buff += 0.15*(cap->cap_msg.vol*0.001-19);
    // chassis_power_buff += 0.01*(cap->cap_msg.vol*0.001-19);

    // SuperCapSend(cap, (uint8_t *)&power_data);
    // chassis_cmd_recv.super_cap.chassis_power_mx = cap->cap_msg.vol;
    */
    DJIMotorSetRef(motor_lf, vt_lf*chassis_power_buff);
    DJIMotorSetRef(motor_rf, vt_rf*chassis_power_buff);
    DJIMotorSetRef(motor_lb, vt_lb*chassis_power_buff);
    DJIMotorSetRef(motor_rb, vt_rb*chassis_power_buff);
}

/**
 * @brief 将裁判系统发给发布中心，再通过发布中心发布给各个执行机构

 */
static void SendJudgeData()
{
    //to 发射
    chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_42mm_barrel_heat;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 获取新的控制信息
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    //底盘模式及参数设定
    ChassisStateSet();
    ChassisStateSet();
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
    //发送信息给云台
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
}