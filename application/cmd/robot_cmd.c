// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_task.h"
#include "referee_UI.h"
#include "buzzer.h"
#include "can_comm.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信

static  BuzzzerInstance *rc_error_buzzer;
static  BuzzzerInstance *aim_success_buzzer;
static  BuzzzerInstance *motor_error_buzzer;

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static float chassis_speed_buff;   // 底盘速度增益

void RobotCMDInit()
{
    /*遥控器串口通信初始化*/
    rc_data = RemoteControlInit(&huart3);
    /*创建云台及发射机构的发布者和订阅者*/   
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    /*云台板和底盘板的can通信初始化*/
    CANComm_Init_Config_s comm_conf = {
        .can_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 0x301,
            .rx_id =  0x300,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
    /*让pitch水平*/
    gimbal_cmd_send.pitch = 0;
    /*警报初始化*/
    Buzzer_config_s rc_buzzer_config={
        .alarm_level=ALARM_LEVEL_HIGH,
        .loudness=0.3,
        .octave=OCTAVE_1,
    };
    /*启动机器人*/
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}
/**
 * @brief 根据云台反馈值计算差角
 */
static void CalcOffsetAngle()
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 基本功能设定
 */
static void BasicFunctionSet()
{
    /*云台基本模式设定*/
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    /*云台软件限位*/
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;

    /*发射机构基本模式设定*/
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    /*发射机构弹速、发射的间隔时间设定*/
    shoot_cmd_send.dead_time = 600;
}

/**
 * @brief 遥控器云台的角度控制
 */
static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.0005f * (float)rc_data[TEMP].rc.rocker_right_x;//0
    gimbal_cmd_send.pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_right_y;
}

/**
 * @brief 遥控器底盘模式及速度控制
 */
static void ChassisRC()
{
    /*底盘速度控制*/
    chassis_cmd_send.vx = 15.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy = -15.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向
    /*底盘模式设定*/
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
    }
    else
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
}

static void ShootRC()
{
    if(rc_data->rc.dial>200)
    {
        shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
        
    }
    else if(rc_data->rc.dial<(-200))
    {
        shoot_cmd_send.load_mode=LOAD_REVERSE;
    }
    else
    {
        shoot_cmd_send.load_mode=LOAD_STOP;
        if (switch_is_mid(rc_data[TEMP].rc.switch_left))
        {
            shoot_cmd_send.load_mode=LOAD_REVERSE;
        }
    }
}

/**
 * @brief 遥控器模式设定
 *
 */
static void RemoteControlSet()
{
    GimbalRC();
    ChassisRC();
    ShootRC();
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    /*底盘速度控制*/
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].a * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;
    /*云台角度控制*/
    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
    if(rc_data[TEMP].mouse.press_l||rc_data[TEMP].mouse.press_r)
    {
        if(rc_data[TEMP].mouse.press_l==1&&rc_data[TEMP].mouse.press_r==0)
        {
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
        }
        if(rc_data[TEMP].mouse.press_r)
        {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
        if(rc_data[TEMP].key_count[KEY_PRESS][Key_R])
        {
            shoot_cmd_send.load_mode=LOAD_REVERSE;
        }
    }


    switch (rc_data[TEMP].mouse.press_r) // 鼠标右键（暴走模式）
    {
    case 1:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void ControlDataDeal()
{

    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicFunctionSet();

        RemoteControlSet();
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        MouseKeySet();   
    }
    else
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}
/**
 * @brief  紧急停止,包括重要模块离线/双板通信失效等
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    if (robot_state == ROBOT_STOP) 
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[下]
    if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    ControlDataDeal();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 推送消息,双板通信,视觉通信等
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}