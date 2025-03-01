// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
#include "rm_referee.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信

static  BuzzzerInstance *cmd_error_buzzer;
static  BuzzzerInstance *aim_success_buzzer;

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回

static Minipc_Recv_s *minipc_recv_data; //小电脑接收数据指针,初始化时返回
static Minipc_Send_s  minipc_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static DataLebel_t DataLebel;
static float cnt1,cnt2;
#include "can_comm.h"

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 遥控器通信串口
    minipc_recv_data=minipcInit(&huart1); // 视觉通信串口
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    CANComm_Init_Config_s comm_conf = {
        .can_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 0x200,
            .rx_id =  0x209,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    Buzzer_config_s cmd_buzzer_config={
        .alarm_level=ALARM_LEVEL_HIGH,
        .octave=OCTAVE_1,
    };

    Buzzer_config_s aim_success_buzzer_config= {
        .alarm_level=ALARM_LEVEL_ABOVE_MEDIUM,
        .octave=OCTAVE_2,
    };
    cmd_error_buzzer = BuzzerRegister(&cmd_buzzer_config);
    aim_success_buzzer= BuzzerRegister(&aim_success_buzzer_config);
    shoot_fetch_data.over_heat_flag=0;
    gimbal_cmd_send.pitch = 0;
    memset(&DataLebel,0,sizeof(DataLebel));
    shoot_cmd_send.friction_mode = FRICTION_OFF;

}
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360
 *
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
 * @brief 每个模式的共性都放这了，不然代码太多了
 *
 */
static void BasicFunctionSet()
{
    //云台基本模式设定
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;

    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
}
/**
 * @brief 判断视觉有没有发信息，但因为现在发信息发的太慢了，头一直想走，所以给视觉3s的机会
 *
 */
static void VisionJudge()
{
    if(minipc_recv_data->Vision.deep!=0)//代表收到信息
    {
        DataLebel.vision_flag=1;
        AlarmSetStatus(aim_success_buzzer, ALARM_ON);
        if(abs(minipc_recv_data->Vision.yaw)>1&&aim_success_buzzer->loudness<0.5)
        {
            aim_success_buzzer->loudness=0.5*(1/abs(minipc_recv_data->Vision.yaw));
        }
        else if(abs(minipc_recv_data->Vision.yaw)<1 && abs(minipc_recv_data->Vision.pitch)<1)
        {
            aim_success_buzzer->loudness=0.5;
            DataLebel.fire_flag=1;
        }
    }
    else if(minipc_recv_data->Vision.deep==0 && DataLebel.vision_flag==1)       
    {
        DataLebel.fire_flag=0;
        DataLebel.t_shoot++;
        if(minipc_recv_data->Vision.deep!=0)
        DataLebel.t_shoot=0;
        if(DataLebel.t_shoot>=100)
        {
            DataLebel.t_shoot=0;
            DataLebel.vision_flag=0;
            AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
        }
    }
}
/**
 * @brief 遥控器控制
 *
 */
static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.0005f * (float)rc_data[TEMP].rc.rocker_right_x;//0
    gimbal_cmd_send.pitch -= 0.0003f * (float)rc_data[TEMP].rc.rocker_right_y;
}

static void ChassisRC()
{
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =-10.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
}

static void ShootRC()
{

   
    if(rc_data->rc.dial>200)
    {
        if(shoot_fetch_data.over_heat_flag==1)
        shoot_cmd_send.load_mode=LOAD_STOP;
        else
        {
            shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
        /*
            if(shoot_cmd_send.load_mode==LOAD_BURSTFIRE)
            {
                if(abs(shoot_fetch_data.speed)<100&&DWT_GetTimeline_s()>1)
                {
                    shoot_fetch_data.loader_error_flag++;
                }
            }

            else if(shoot_cmd_send.load_mode==LOAD_REVERSE)
            {
                shoot_fetch_data.recover_flag++;
            }
            if(shoot_fetch_data.loader_error_flag==1&&shoot_cmd_send.load_mode==LOAD_BURSTFIRE)
            {
                shoot_cmd_send.load_mode=LOAD_REVERSE;
                shoot_fetch_data.loader_error_flag=0;
            }
            if (shoot_fetch_data.recover_flag==1)
            {
                shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
                shoot_fetch_data.recover_flag=0;
            }

        }
        */
        }
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
 * @brief 小电脑控制
 *
 */
static void GimbalAC()
{
    VisionJudge();
    if(DataLebel.vision_flag==0)
    {
        gimbal_cmd_send.yaw-=0.33;
        DataLebel.t_pitch = (float32_t)DWT_GetTimeline_s();
        gimbal_cmd_send.pitch =20*abs(sin(2.5*DataLebel.t_pitch));
    }
    else
    {    
        if(abs(minipc_recv_data->Vision.pitch)<400&&abs(minipc_recv_data->Vision.yaw)<400)
        {
            gimbal_cmd_send.yaw-=0.014f*minipc_recv_data->Vision.yaw;   //往右获得的yaw是减
            gimbal_cmd_send.pitch -= 0.009f*minipc_recv_data->Vision.pitch;
        }
    }
}

static void ChassisAC()
{
    chassis_cmd_send.vx=minipc_recv_data->Nav.vx*4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL*1000;
    chassis_cmd_send.vy=minipc_recv_data->Nav.vy*4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL*1000;
    chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
}

static void ShootAC()
{
    if(DataLebel.fire_flag==1)
    {
        if(shoot_fetch_data.over_heat_flag==1)
        shoot_cmd_send.load_mode=LOAD_STOP;
        else
        shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
    }
    else
    {
        shoot_cmd_send.load_mode=LOAD_STOP;
    }
}
/**
 * @brief 停止
 *
 */
static void AnythingStop()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
}

/**
 * @brief 
 *
 */
static void RemoteDataDeal()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicFunctionSet();
        GimbalRC();
        ChassisRC();
        ShootRC();
        AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        BasicFunctionSet();
        GimbalAC();
        // //等巡航搞完改为ChassisAC();
         ChassisRC();
         ShootAC();
    }
    else
    {
        AnythingStop();
    }
}

/**
 * @brief  用于发送来自底盘的裁判信息给发射机构（害怕他传了些奇奇怪怪的信息给发射机构，先注释）
 */
static void SendJudgeShootData()
{
    shoot_cmd_send.bullet_real_speed=chassis_fetch_data.bullet_speed;
    shoot_cmd_send.bullet_num=chassis_fetch_data.bullet_num;
    shoot_cmd_send.left_bullet_heat=chassis_fetch_data.left_bullet_heat;
    shoot_cmd_send.right_bullet_heat=chassis_fetch_data.right_bullet_heat;

}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角
    CalcOffsetAngle();
    //控制
    RemoteDataDeal();
    // 设置视觉发送数据
    VisionSetFlag();
    VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.Yaw,gimbal_fetch_data.gimbal_imu_data.Pitch,
                        gimbal_fetch_data.gimbal_imu_data.Roll);
    // 设置巡航发送数据       
    NavSetMessage(chassis_fetch_data.real_vx,chassis_fetch_data.real_vy,gimbal_fetch_data.gimbal_imu_data.Yaw,chassis_fetch_data.Occupation
                    ,chassis_fetch_data.remain_HP,chassis_fetch_data.self_infantry_HP,chassis_fetch_data.self_hero_HP
                    ,chassis_fetch_data.enemy_color,chassis_fetch_data.enemy_infantry_HP,chassis_fetch_data.enemy_hero_HP
                    ,chassis_fetch_data.remain_time,shoot_cmd_send.bullet_num,chassis_fetch_data.game_progress
                    );
    SendJudgeShootData();
                        // 推送消息,双板通信,视觉通信
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    SendMinipcData();
}
