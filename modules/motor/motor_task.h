#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H


/**
 * @brief 电机控制闭环任务,在RTOS中应该设定为1Khz运行
 *        舵机控制任务的频率设定为20Hz或更低
 * 
 */
void MotorControlTask();

#endif // !MOTOR_TASK_H

