#ifndef __FISH_KEYBOARD_CONFIG_H__
#define __FISH_KEYBOARD_CONFIG_H__

/*  USB HID 设备连接模式   */
#define USB_CONNECT_MODE                1

/*  键盘扫描最大行数   */
#define SCAN_KEYBOARD_LINE_MAX_SIZE 6

/*  键盘扫描单行最大行按键数   */
#define SCAN_KEYBOARD_LINE_KEY_MAX_SIZE 21

/*  键盘扫描最大层数   */
#define SCAN_KEYBOARD_LAYOUT_MAX_NUMBER 2

/*  键盘发送最大按键数   */
#define FISH_KEYBOARD_SEND_KEY_MAX_NUMBER 6

/*  灯光的个数   */
#define TIM_PWM_LED_STM32_LED_NUMBER 104

/* 灯的最大亮度（0-1.0） */
#define TIM_PWM_LED_STM32_LED_MAX_POWER    0.2f

#endif
