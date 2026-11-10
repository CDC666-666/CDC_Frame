#ifndef FRAME_CDC_SYSTEM_FRAME_SYSTEM_H
#define FRAME_CDC_SYSTEM_FRAME_SYSTEM_H

/** @brief 初始化 MC02 外设与静态组件，在主循环前调用一次。 */
void frame_system_init(void);

/** @brief 主循环按接收、观测、估计、控制、发送、遥测的顺序推进组件。 */
void frame_system_poll(void);

#endif /* FRAME_CDC_SYSTEM_FRAME_SYSTEM_H */
