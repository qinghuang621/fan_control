#ifndef BSP_MODBUS_H
#define BSP_MODBUS_H

#include <stdint.h>
#include "bsp_ads1256.h"

void modbus_init(void);
void modbus_poll(void);

/* AdcTask 调用：把 ADS1256 四路扫描快照写入只读寄存器 0x01A8~0x01C3 */
void modbus_ads_update(const ads1256_snapshot_t *snap);

/* RS485 串口（USART6）裸寄存器中断服务，由 Src/stm32f4xx_it.c 的
 * USART6_IRQHandler 调用。不经过 HAL_UART_IRQHandler。 */
void usart6_irq_handler(void);

/* CAN1 RX0 中断服务，由 Src/stm32f4xx_it.c 的 CAN1_RX0_IRQHandler 调用。
 * 走 HAL_CAN_IRQHandler → HAL_CAN_RxFifo0MsgPendingCallback（中断里只入队）。 */
void motor_can_irq_handler(void);

#endif
