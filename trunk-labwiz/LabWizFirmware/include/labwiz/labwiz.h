#ifndef __LABWIZ_H__
#define __LABWIZ_H__

#define SW_A                0
#define SW_B                1
#define SW_C                2
#define SW_D                3
#define SW_E                4
#define SW_PWR              5

#define SW_MASK(S)          (1<<(S))

// Button callback
typedef void (*labwiz_btn_callback)(uint8_t);

void labwiz_init(void);

void labwiz_task_init(void);

void labwiz_set_btn_callback(labwiz_btn_callback cb);

#endif

//eof
