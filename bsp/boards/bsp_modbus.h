#ifndef BSP_MODBUS_H
#define BSP_MODBUS_H

#include <stdint.h>

void modbus_init(void);
void modbus_poll(void);

/* RS485 串口（USART6）裸寄存器中断服务，由 Src/stm32f4xx_it.c 的
 * USART6_IRQHandler 调用。不经过 HAL_UART_IRQHandler。 */
void usart6_irq_handler(void);

/* CAN1 RX0 中断服务，由 Src/stm32f4xx_it.c 的 CAN1_RX0_IRQHandler 调用。
 * 走 HAL_CAN_IRQHandler → HAL_CAN_RxFifo0MsgPendingCallback（中断里只入队）。 */
void motor_can_irq_handler(void);

#endif
