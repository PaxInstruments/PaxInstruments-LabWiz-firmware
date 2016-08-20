#ifndef __LABWIZ_H__
#define __LABWIZ_H__

typedef enum{
    SW_A = 0,
    SW_D,
    SW_B,
    SW_C,
    SW_E,
    SW_PWR,
}switch_names_e;

#define SW_MASK(S)          (1<<(S))

typedef enum{
    BATTERY_NOT_INSTALLED = 0,
    BATTERY_FULL,
    BATTERY_75,
    BATTERY_50,
    BATTERY_25,
    BATTERY_EMPTY,
}battery_status_e;

#define BATTERY_FULL_CHAR           '{'
#define BATTERY_50_CHAR             '}'
#define BATTERY_25_CHAR             '_'
#define BATTERY_NOT_INSTALLED_CHAR  '|'

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

battery_status_e labwiz_get_battery_status(void);

uint16_t labwiz_get_battery_mV(void);

bool labwiz_read(switch_names_e btn);

#endif

//eof
