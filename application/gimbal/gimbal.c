#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *left_yaw_motor, *right_yaw_motor,*upper_pitch_motor,*lower_pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void GimbalInit()
{
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = 
        {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 12, // 6
                .Ki = 2,
                .Kd =0.4,//0.0.6
                .CoefA=3,
                .CoefB=2,
                .DeadBand = 0.01,
                .Improve = PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 260,  // 260
                .Ki = 0, // 200
                .Kd = 0,
                .Improve = PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020 
    };
   

    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 80
                .Ki = 0,//10
                .Kd = 0.2,//1
                .Improve = PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 100,  // 80
                .Ki = 0, // 350
                .Kd = 0,   // 0
                .Improve =PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[1]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    yaw_config.can_init_config.tx_id = 1;
    yaw_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    left_yaw_motor = DJIMotorInit (&yaw_config);


    yaw_config.can_init_config.tx_id = 2;
    yaw_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    right_yaw_motor = DJIMotorInit(&yaw_config);

    pitch_config.can_init_config.tx_id = 3;
    pitch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    lower_pitch_motor =DJIMotorInit (&pitch_config);

    pitch_config.can_init_config.tx_id = 4;
    pitch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    upper_pitch_motor =DJIMotorInit(&pitch_config);

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));//云台反馈出来的信息
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));    //云台接收的信息
}

static void GimbalStateSet()
{
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(upper_pitch_motor);
        DJIMotorStop(lower_pitch_motor);
        DJIMotorStop(right_yaw_motor);
        DJIMotorStop(left_yaw_motor);
        break;
    case GIMBAL_GYRO_MODE: 
         DJIMotorEnable(upper_pitch_motor);
        DJIMotorEnable(lower_pitch_motor);
        DJIMotorEnable(left_yaw_motor);
        DJIMotorEnable(right_yaw_motor);

        DJIMotorSetRef(right_yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(upper_pitch_motor, gimbal_cmd_recv.pitch);
        DJIMotorSetRef(left_yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(lower_pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }
}

static void SendGimbalData()
{
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = left_yaw_motor->measure.angle_single_round;
}

void GimbalTask()
{
    // 获取云台控制数据
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    //开启关闭云台
    GimbalStateSet();
    //将信息反馈给控制中心
    SendGimbalData();
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
