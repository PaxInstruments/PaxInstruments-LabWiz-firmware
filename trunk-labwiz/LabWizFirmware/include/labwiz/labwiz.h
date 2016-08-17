#ifndef __LABWIZ_H__
#define __LABWIZ_H__

#define SW_A                0
#define SW_B                1
#define SW_C                2
#define SW_D                3
#define SW_E                4
#define SW_PWR              5

#define SW_MASK(S)          (1<<(S))

typedef struct
{
  uint8_t Hours; /* Min_Data = 0 and Max_Data = 23 */
  uint8_t Minutes; /* Min_Data = 0 and Max_Data = 59 */
  uint8_t Seconds; /* Min_Data = 0 and Max_Data = 59 */
  uint8_t Month; /* Min_data=1 and Max_Data=12 */
  uint8_t Day; /* Min_data=1 and Max_Data=31 */
  uint8_t Year; /* Years since 2000, Min_data=00 and Max_Data=99 */
}labwiz_time_t;

// Button callback
typedef void (*labwiz_btn_callback)(uint8_t);

void labwiz_init(void);

void labwiz_task_init(void);

void labwiz_set_btn_callback(labwiz_btn_callback cb);

void labwiz_get_time(labwiz_time_t * tm);


#endif

//eof
