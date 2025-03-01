#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "ins_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "buzzer.h"
#include "rm_referee.h"
#include "referee_task.h"


/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
 // 使用板载IMU获取底盘转动角速度（但初始化pid那里写的会很乱，不适合代码的可读性）
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令（发布中心发给底盘的）
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; 

static referee_info_t* referee_data;        // 用于获取裁判系统的数据



/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,跟据功率的多少再乘上一个系数
static float sin_theta, cos_theta;//麦轮解算用
static float vx,vy;//获取车体信息要用到的中间变量

static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static float cnt=0;
static float chassis_power_buff= 1;
static float rotate_speed_buff = 1;

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

void ChassisInit()
{
    /***************************IMU_INIT******************************/

    /***************************MOTOR_INIT******************************/
    // 底盘电机的初始化，包括什么通信、什么id、pid及电机安装是正装还是反装（相当于给最终输出值添负号）及型号

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
    //电机id号一定一定得一一对应，不然看代码的人一头雾水！！
    chassis_motor_config.can_init_config.tx_id = 0x201;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x202;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);
    chassis_motor_config.can_init_config.tx_id = 0x203;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 0x204;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);



    /***************************REFEREE_COMM_INIT******************************/
    //裁判系统和巡航还没搞好，就先不开了
    referee_data   = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI

    /***************************BOARD_COMM_INIT******************************/
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            //云台的tx是底盘的rx，别搞错了！！！
            .tx_id = 0x209,
            .rx_id = 0x200,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
}


    /***************************SEND_TO_MOTOR******************************/
static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
}
/**
 * @brief  
 */
static void ChassisRotateSet()
{
    cnt = (float32_t)DWT_GetTimeline_s();//用于变速小陀螺
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
        //底盘跟随就不调了，懒
        case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_recv.wz = 0;
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            chassis_cmd_recv.wz = (1200+100*(float32_t)sin(cnt))*rotate_speed_buff;
        break;
        default:
        break;
    }
}
/**
 * @brief 计算每个底盘电机的输出,正运动学解算
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
 * @brief 功率高系数就大些（没裁判系统还没调）
 *
 */
static void LimitChassisOutput()
{ 
    rotate_speed_buff = 3;
    chassis_power_buff = 1.25;
    DJIMotorSetRef(motor_lf, vt_lf*chassis_power_buff);
    DJIMotorSetRef(motor_rf, vt_rf*chassis_power_buff);
    DJIMotorSetRef(motor_lb, vt_lb*chassis_power_buff);
    DJIMotorSetRef(motor_rb, vt_rb*chassis_power_buff);
}



/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 */
static void SendChassisData()
{
    //to 巡航
    vx = (motor_lf->measure.speed_aps +motor_lb->measure.speed_aps - motor_rb->measure.speed_aps - motor_rf->measure.speed_aps) / 4.0f / REDUCTION_RATIO_WHEEL / 360.0f * PERIMETER_WHEEL/1000 ;
    vy = (-motor_lf->measure.speed_aps +motor_lb->measure.speed_aps + motor_rb->measure.speed_aps - motor_rf->measure.speed_aps) / 4.0f / REDUCTION_RATIO_WHEEL / 360.0f * PERIMETER_WHEEL/1000  ;
    chassis_feedback_data.real_vx = vx * cos_theta + vy * sin_theta;
    chassis_feedback_data.real_vy = -vx * sin_theta + vy * cos_theta;
}

/**
 * @brief 靠这个函数将裁判系统发给发布中心，再通过发布中心发布给各个执行机构

 */
static void send_judge_data()
{
    chassis_feedback_data.Occupation=(referee_data->EventData.event_type >> 21) & 0x03;
    chassis_feedback_data.remain_time=referee_data->GameState.stage_remain_time;
    chassis_feedback_data.game_progress=referee_data->GameState.game_progress;
    
    //to 巡航
    if(referee_data->GameRobotState.robot_id>7)
    {
        chassis_feedback_data.remain_HP=referee_data->GameRobotHP.blue_7_robot_HP;
        chassis_feedback_data.self_hero_HP=referee_data->GameRobotHP.blue_1_robot_HP;
        chassis_feedback_data.self_infantry_HP=referee_data->GameRobotHP.blue_3_robot_HP;

        chassis_feedback_data.enemy_hero_HP=referee_data->GameRobotHP.red_1_robot_HP;
        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.red_3_robot_HP;
        chassis_feedback_data.enemy_sentry_HP=referee_data->GameRobotHP.red_7_robot_HP;
    }
    else
    {
        chassis_feedback_data.remain_HP=referee_data->GameRobotHP.red_7_robot_HP;
        chassis_feedback_data.self_hero_HP=referee_data->GameRobotHP.red_1_robot_HP;
        chassis_feedback_data.self_infantry_HP=referee_data->GameRobotHP.red_3_robot_HP;

        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.blue_1_robot_HP;
        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.blue_3_robot_HP;
        chassis_feedback_data.enemy_infantry_HP=referee_data->GameRobotHP.blue_7_robot_HP;
    }   
    chassis_feedback_data.left_bullet_heat= referee_data->PowerHeatData.shooter_17mm_2_barrel_heat;
    chassis_feedback_data.right_bullet_heat= referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    chassis_feedback_data.bullet_num=referee_data->ProjectileAllowance.projectile_allowance_17mm;
    chassis_feedback_data.bullet_speed=referee_data->ShootData.bullet_speed;

}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 获取新的控制信息
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    //底盘动与不动
    ChassisStateSet();
    //旋转速度设定
    ChassisRotateSet();
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系
    // 根据控制模式进行正运动学解算,计算底盘各个电机的速度
    MecanumCalculate();

    // 根据裁判系统的反馈数据设定闭环参考值
    LimitChassisOutput();
    send_judge_data();
    // 根据电机的反馈速度计算真实速度发给巡航
    SendChassisData(); 
     
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
}