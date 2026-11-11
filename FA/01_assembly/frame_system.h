#ifndef FRAME_CDC_ASSEMBLY_FRAME_SYSTEM_H
#define FRAME_CDC_ASSEMBLY_FRAME_SYSTEM_H

/** 作用初始化 MC02 板级并装配静态组件，主循环前调用一次。
 * 输入无参数，读取生成配置，输出初始化后的内部实例和板级就绪状态，无返回值。 */
void frame_system_init(void);

/** 作用依次推进接收、观测、估计、控制、发送和遥测。
 * 输入无参数，读取内部实例与板级时钟，输出更新后的组件状态和通信请求，无返回值。 */
void frame_system_poll(void);

#endif /* FRAME_CDC_ASSEMBLY_FRAME_SYSTEM_H */
