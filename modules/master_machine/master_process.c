#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;
static DaemonInstance *minipc_daemon_instance;

void VisionSetFlag()
{
    minipc_send_data.Vision.detect_color=0;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    minipc_send_data.Vision.yaw = yaw;
    minipc_send_data.Vision.pitch = pitch;
    minipc_send_data.Vision.roll = roll;
}
void NavSetMessage(float vx, float vy, float yaw,uint8_t occupation,
					uint16_t self_sentry_HP,uint16_t self_infantry_HP,uint16_t self_hero_HP,
					uint16_t enermy_sentry_HP,uint16_t enermy_infantry_HP,uint16_t enermy_hero_HP,
                    uint16_t remain_time,uint16_t remain_bullet,uint8_t game_progress
					)
{
    minipc_send_data.Nav.header=0x4A;
    minipc_send_data.Nav.vx=vx;
    minipc_send_data.Nav.vy=vy;
    minipc_send_data.Nav.yaw=yaw;

    minipc_send_data.Nav.enemy_hero_HP=enermy_hero_HP;
    minipc_send_data.Nav.enemy_infantry_HP=enermy_infantry_HP;
    minipc_send_data.Nav.enemy_sentry_HP=enermy_sentry_HP;

    minipc_send_data.Nav.self_sentry_HP=self_sentry_HP;
    minipc_send_data.Nav.self_infantry_HP=self_infantry_HP;
    minipc_send_data.Nav.self_hero_HP=self_hero_HP;

    minipc_send_data.Nav.remain_time=remain_time;
    minipc_send_data.Nav.remain_bullet=remain_bullet;
    minipc_send_data.Nav.occupation =occupation;
    minipc_send_data.Nav.game_progress=game_progress;
    minipc_send_data.Nav.tail1=0x2B;
}
static USARTInstance *minipc_usart_instance;

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id minipc_usart_instance,此处没用.
 */
static void MiniPCOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(minipc_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"


/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeMinpc()
{
    //DaemonReload(minipc_daemon_instance); // 喂狗
    get_protocol_info_vision(minipc_usart_instance->recv_buff,&minipc_recv_data);
    get_protocol_info_odom(minipc_usart_instance->recv_buff, &minipc_recv_data);
}

Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeMinpc;
    conf.recv_buff_size = MINIPC_RECV_SIZE;
    conf.usart_handle = _handle;
    minipc_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断小电脑通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = MiniPCOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = minipc_usart_instance,
        .reload_count = 10,
    };
    minipc_daemon_instance = DaemonRegister(&daemon_conf);

    return &minipc_recv_data;
}


/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void SendMinipcData()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint8_t send_buff[MINIPC_SEND_SIZE];
    static uint16_t tx_len;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_Vision_data(&minipc_send_data, send_buff, &tx_len);
    get_protocol_send_Nav_data(&minipc_send_data, send_buff, &tx_len);

    USARTSend(minipc_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA);
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    uint16_t flag_register;
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
