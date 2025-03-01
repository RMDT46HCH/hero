#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

static DJIMotorInstance *friction_l, *friction_r, *loader,*friction_u; // 拨盘电机
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static float loader_set_angle = 0;
//static cal_bullet_t cal_bullet;


void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = 
        {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 18000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 18000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 1; // 右摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    Motor_Init_Config_s friction_u_config = {
        .can_init_config = 
        {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 25, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M2006};
    friction_u_config.can_init_config.tx_id=4;
    friction_u = DJIMotorInit(&friction_u_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 50,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .MaxOut = 15000,
                .Derivative_LPF_RC = 0.001,
            },
            
            .speed_PID = {
                .Kp = 7, 
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP| ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508 
    };
    loader_config.can_init_config.tx_id=3;
    loader = DJIMotorInit(&loader_config);

    //后续增加一个切换枪口的电机实例

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

static void ShootStateSet()
{
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(friction_u);

        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }
}

static void ShootAngleSet()
{
    switch (shoot_cmd_recv.load_mode)
    {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
        case LOAD_1_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);// 切换到角度环
            loader_set_angle = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE*REDUCTION_RATIO_LOADER; // 控制量增加一发弹丸的角度
            DJIMotorSetRef(loader, loader_set_angle); 
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = shoot_cmd_recv.dead_time;    
            break;
        case LOAD_BURSTFIRE:
        /*
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            loader_set_angle = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE*REDUCTION_RATIO_LOADER*40; // 控制量增加20发弹丸的角度
            DJIMotorSetRef(loader, loader_set_angle); 
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = shoot_cmd_recv.dead_time;     
*/
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader,  360 * REDUCTION_RATIO_LOADER *4);

            break;
        // 拨盘反转,对速度闭环
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            loader_set_angle = loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE*REDUCTION_RATIO_LOADER; // 控制量增加一发弹丸的角度
            DJIMotorSetRef(loader, loader_set_angle); 
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = shoot_cmd_recv.dead_time;     
            break;
        default:
            while (1)
                ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
}

/**
 * @brief  未实测
 */
static void ShootSpeedSet()
{
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        DJIMotorSetRef(friction_l,50000);
        DJIMotorSetRef(friction_r, 50000);
        DJIMotorEnable(friction_u);
        DJIMotorSetRef(friction_u, 110000);

    }
    else // 关闭摩擦轮u
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        DJIMotorStop(friction_u);

    }
}

static void SendShootData()
{
    shoot_feedback_data.loader_speed_aps=loader->measure.speed_aps;
}
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    ShootStateSet();
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;
    ShootAngleSet();
    ShootSpeedSet();
    SendShootData();    
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
