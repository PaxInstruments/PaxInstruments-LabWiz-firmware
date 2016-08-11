/*****************************************************************************
 **
 ** t1000 related functions.
 ** August 2016
 **
 ****************************************************************************/

#include "string.h"

#include "labwiz/defs.h"
#include "labwiz/t1000.h"

#include "labwiz/port.h"
#include "labwiz/labwiz.h"
#include "labwiz/drv_lcd.h"
#include "labwiz/drv_filesystem.h"

// Definitions and types
// ----------------------------------------------------------------------------
#define STATE_INIT      0
#define STATE_OPERATING 1

#define PERIODIC_PERIOD_MS      1000

#define SENSOR_COUNT            4
#define MAXIMUM_GRAPH_POINTS    100

#define TEMP_MAX_VALUE_I        (32760)
#define TEMP_MIN_VALUE_I        (-32760)

// Int value representing an invalid temp. measurement
#define OUT_OF_RANGE_INT        32760

#define TEMPERATURE_UNITS_C     0
#define TEMPERATURE_UNITS_F     1
#define TEMPERATURE_UNITS_K     2

// Local variables
// ----------------------------------------------------------------------------
int m_button_mask = 0;
int m_state = STATE_INIT;

// Array to hold graph data, in temperature values
int16_t m_graphdata[SENSOR_COUNT][MAXIMUM_GRAPH_POINTS]={};
uint8_t m_graphdata_index=0;
uint8_t m_graphdata_head=0;
uint8_t m_graphdata_count=0;

uint32_t m_graphScale;

uint8_t m_axisDigits;     // Number of digits to display in the axis labels (ex: '80' -> 2, '1000' -> 4, '-999' -> 4)
int16_t m_minTempInt;
int16_t m_maxTempInt;

uint8_t m_temperatureUnit = TEMPERATURE_UNITS_C;

lcd_screen_t lcd_screen1;

static char m_scratch[100];

// Local prototypes
// ----------------------------------------------------------------------------
void _t1000_btn_press(uint8_t button);
void _t1000_updateGraphScaling(void);
int16_t _t1000_convertTemperatureInt(int16_t celcius);
uint8_t _t1000_temperature_to_pixel(int16_t temp);

// Public functions
// ----------------------------------------------------------------------------

void setup()
{
    int8_t x;
    lcd_blank();
    // Top Row section
    lcd_line(8,0, 8, MAX_COL);
    for(x=0;x<4;x++)
        lcd_line(0, (uint8_t)(x*33),7,(uint8_t)(x*33));
    lcd_line(0, 131,7,131);
    // Status bar
    lcd_line(18, 0,18,131);
    // Side bar
    lcd_line(19, 18,MAX_ROW,18);
    // Tick marks
    for(x=0;x<5;x++)
        lcd_set_pixel((uint8_t)((x*9)+19),17);

    // Test
#if 0
    lcd_print("123.4",0,(0*33)+2);
    lcd_print("123.4",0,(1*33)+2);
    lcd_print("123.4",0,(2*33)+2);
    lcd_print("123.4",0,(3*33)+2);
    lcd_print("TypK-Not Logging-1s-E",10,0);
#endif
    lcd_get_screen(&lcd_screen1);

    lcd_blank();
    labwiz_set_btn_callback(_t1000_btn_press);

    lcd_print("Pax Instruments t1000",30,3);

    return;
}

TickType_t m_last_tick;
void loop()
{
    switch(m_state){
    case STATE_INIT:
        vTaskDelay(portTICK_PERIOD_MS*2000);
        m_state = STATE_OPERATING;
        lcd_set_screen(&lcd_screen1);
        break;
    default:
    case STATE_OPERATING:
    {
        // This is the main loop
        // -------------------------------------------------------------------
        TickType_t current_tick = xTaskGetTickCount();

        // Do things every PERIODIC_PERIOD_MS
        if(current_tick >= (m_last_tick+(portTICK_PERIOD_MS*PERIODIC_PERIOD_MS)))
        {
            int num_points,x;

            m_last_tick = current_tick;
            led1(toggle());

            // Grab a data points and add them to the set

            // Copy over our display screen
            lcd_set_screen(&lcd_screen1);
            // Draw our datapoints
            // We have an area from 19----->130 and 19 ^---v 63 (111 x 44)

            // Increment the current graph point (it wraps around)
            if(m_graphdata_index == 0)
            {
                m_graphdata_index = MAXIMUM_GRAPH_POINTS - 1;
            }else{
                m_graphdata_index = (uint8_t)(m_graphdata_index-1);
            }
            // Increment the number of stored graph points
            if(m_graphdata_count < MAXIMUM_GRAPH_POINTS) {
                m_graphdata_count++;
            }

            // TODO: Save REAL values into m_graphdata[channel][m_graphdata_index]
            for(x=0;x<SENSOR_COUNT;x++)
                m_graphdata[x][m_graphdata_index] = 234;

            // TODO: Write the values to the top bar
            sprintf(m_scratch," %d",m_graphdata[0][m_graphdata_index]);
            lcd_print(m_scratch,0,(0*33)+2);
            sprintf(m_scratch," %d",m_graphdata[1][m_graphdata_index]);
            lcd_print(m_scratch,0,(1*33)+2);
            sprintf(m_scratch," %d",m_graphdata[2][m_graphdata_index]);
            lcd_print(m_scratch,0,(2*33)+2);
            sprintf(m_scratch," %d",m_graphdata[3][m_graphdata_index]);
            lcd_print(m_scratch,0,(3*33)+2);

            _t1000_updateGraphScaling();

            // Calculate how many graph points to display.
            // If the number of axis digits is >2, scale back how many
            // graph points to show
            num_points = m_graphdata_count;
            x = MAXIMUM_GRAPH_POINTS - ((m_axisDigits - 2)*5);
            if(x<num_points) num_points = x;

#if 1
            // Draw the temperature graph for each sensor
            for(uint8_t channel = 0; channel< SENSOR_COUNT; channel++)
            {
                int16_t tmp16;
                uint8_t p,index;

                // if the sensor is out of range, don't show it. If we are showing one
                // channel, ignore the others
                //if(m_graphdata[sensor][graphCurrentPoint] == OUT_OF_RANGE_INT || (sensor != graphChannel && graphChannel < 4) )
                if(m_graphdata[channel][m_graphdata_index] == OUT_OF_RANGE_INT)
                  continue;

                tmp16 = _t1000_convertTemperatureInt(m_graphdata[channel][m_graphdata_index]);

                // Get the position of the latest point
                //p = temperature_to_pixel(graph[sensor][graphCurrentPoint]);
                p = _t1000_temperature_to_pixel(tmp16);

                // Draw the channel number at the latest point
                sprintf(m_scratch,"%d",channel+1);
                lcd_print(m_scratch,(uint8_t)(3+p),(uint8_t)(112+5*channel));

                // Now, draw all the points
                index = m_graphdata_index;
                #if 1
                for(uint8_t point = 0; point < m_graphdata_count; point++)
                {
                    tmp16 = _t1000_convertTemperatureInt(m_graphdata[channel][index]);
                    //p = temperature_to_pixel(graph[sensor][index]);
                    p = _t1000_temperature_to_pixel(tmp16);
                    // Draw pixel at X, Y. X is # of pixels from the left
                    lcd_set_pixel(p,130-12-point);
                    // Go to next pixel
                    index++;
                    // Wrap when we hit the end of the array
                    if(index>=MAXIMUM_GRAPH_POINTS) index = 0;

                } // end for point
                #endif

            }// end for sensor
#endif

        }

        // Detect button pushes
        if(m_button_mask!=0)
        {
            if(m_button_mask&SW_MASK(SW_A))
            {
                m_button_mask&=~SW_MASK(SW_A);
            }
            if(m_button_mask&SW_MASK(SW_B))
            {
                m_button_mask&=~SW_MASK(SW_B);
            }
            if(m_button_mask&SW_MASK(SW_C))
            {
                m_button_mask&=~SW_MASK(SW_C);
            }
            if(m_button_mask&SW_MASK(SW_D))
            {
                m_button_mask&=~SW_MASK(SW_D);
            }
            if(m_button_mask&SW_MASK(SW_E))
            {
                lcd_backlight_toggle();
                m_button_mask&=~SW_MASK(SW_E);
            }
        }

        // -------------------------------------------------------------------
        // End main loop
    }
    break;
    } // End state select
    return;
}

// Private functions
// ----------------------------------------------------------------------------
void _t1000_btn_press(uint8_t button)
{
    // When called as a callback, the function is running in the labwiz
    // task, not the loop() task
    m_button_mask |= SW_MASK(button);
    // TODO: mutex?
    return;
}

void _t1000_updateGraphScaling()
{
  uint16_t delta;
  int16_t max=TEMP_MIN_VALUE_I;
  int16_t min=TEMP_MAX_VALUE_I;
  int16_t * ptr;
  int16_t p;
  uint8_t old_axisdigits = m_axisDigits;

  // Itterate over all the data and get the max & min
  for(uint8_t x = 0; x < SENSOR_COUNT; x++)
  {
     ptr = (int16_t*)&m_graphdata[x][0];
     for(uint8_t y=0; y < MAXIMUM_GRAPH_POINTS; y++)
     {
       p = *ptr;
       if(p!=OUT_OF_RANGE_INT)
       {
           p = _t1000_convertTemperatureInt(p);
           if(p>max) max = p;
           if(p<min) min = p;
       }
       ptr++;
     }
  }

  if(max==TEMP_MIN_VALUE_I) max=0;
  if(min==TEMP_MAX_VALUE_I) min=0;

  m_minTempInt = min;
  m_maxTempInt = max;
  delta = (uint16_t)(max - min);
  if(delta<4) m_maxTempInt=(int16_t)(m_minTempInt+4);

  m_graphScale = (uint32_t)((delta + 39) / 40);  // TODO: better rounding strategy
  if(m_graphScale==0) m_graphScale = 1;

  // graphScale is an int multiplier.  Normally we display 5 temperatures.
  // maxTempInt is the highest temp in the dataset
  // minTempInt is the lowest temp in the dataset

  // Calculate the number of axes digits to display
  m_axisDigits = 2;
  // These are in 1/10th, is min<-99.0 || max>999.9
  if(min<-999 || (max+(int16_t)(m_graphScale*4)) >9999) m_axisDigits = 4;
  else if(min<-100 || (max+(int16_t)(m_graphScale*4))>999) m_axisDigits = 3;


  if(m_axisDigits!=old_axisdigits)
  {
      // TODO: Redraw the display
  }

  return;
}

int16_t _t1000_convertTemperatureInt(int16_t celcius)
{
  switch(m_temperatureUnit){
  case TEMPERATURE_UNITS_F:
      celcius = (int16_t)(celcius*18);
      celcius = (int16_t)(celcius/10);
      celcius = (int16_t)(celcius + 320);
      return celcius;
  case TEMPERATURE_UNITS_K:
    return (int16_t)(celcius + 2732);
  default: break;
  }
  return celcius;
}

uint8_t _t1000_temperature_to_pixel(int16_t temp)
{
    uint16_t p;
    // This gets the delta between our measurement and the min value (which
    // is the bottom of the chart
    // Example: if minTempInt=300, p_int=325. p = 25, it is 2.5deg higher
    p = (uint16_t)(temp -  m_minTempInt);
    // Now we need to keep all points within the drawing window.  Each temperature step
    // takes up 10 pixels of height (But we are already in 1/10th of degrees!). But if
    // we scaled, scale our value down, this means we need to divide the delta by
    // the scale value
    p = (uint16_t)(p / m_graphScale);

    // This gets us a scaled pixel offset.  So at scale 1. 25/1 = 25
    // This means we put the pixel 25 pixels above the low. Since the
    // low is always 60 pixels from the top, we take this
    // value and remove our pos.
    p = (uint16_t)(60-p);
    return (uint8_t)p;
}

// eof
