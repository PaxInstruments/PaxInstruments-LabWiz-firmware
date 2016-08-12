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
#define STATE_INIT              0
#define STATE_OPERATING         1

#define PERIODIC_PERIOD_MS      250

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

bool m_logging=false;

int m_current_channel = SENSOR_COUNT;

lcd_screen_t lcd_screen1;

static char m_scratch[100];

// Local prototypes
// ----------------------------------------------------------------------------
void _t1000_btn_press(uint8_t button);
void _t1000_updateGraphScaling(void);
int16_t _t1000_convertTemperatureInt(int16_t celcius);
uint8_t _t1000_temperature_to_pixel(int16_t temp);
void _t1000_fake_data(void);
uint8_t _t1000_numlength(int16_t num);
char * _t1000_printtemp(char * buf, int16_t temp);
char _t1000_current_unit(void);
void _t1000_record_start(void);
void _t1000_record_stop(void);

// Public functions
// ----------------------------------------------------------------------------

void setup()
{
    int8_t x,y;
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
    for(x=1;x<5;x++)
        lcd_set_pixel((uint8_t)((x*10)+15),17);

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


    for(uint8_t x = 0; x < SENSOR_COUNT; x++)
    {
     for(uint8_t y=0; y < MAXIMUM_GRAPH_POINTS; y++)
     {
         m_graphdata[x][y] = OUT_OF_RANGE_INT;
     }
    }

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

            // Draw status
            lcd_print("K",10,0); // Thermocouple type
            lcd_print("B",10,127); // Battery
            sprintf(m_scratch,"`%c",_t1000_current_unit());
            lcd_print(m_scratch,10,20);
            lcd_print("Log Off",10,50);

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

            #if 0
            // TODO: Save REAL values into m_graphdata[channel][m_graphdata_index]
            #else
            _t1000_fake_data();
            #endif

            _t1000_updateGraphScaling();

            // Draw axis labels and marks
            for(uint8_t interval = 1; interval < 5; interval++)
            {
                uint8_t spaces=0,x;
                int16_t tmp16;
                tmp16 = (m_minTempInt/10) + (m_graphScale*interval);
                // TODO: Write a space string, then over write with number, drrr
                // Add spaces for right justified
                spaces = m_axisDigits-_t1000_numlength(tmp16);
                if(spaces>3) spaces=3;
                for(x=0;x<spaces;x++)
                    sprintf(&(m_scratch[x])," ");
                sprintf(&(m_scratch[spaces]), "%d", tmp16);
                lcd_print(m_scratch, 63 - interval*10, 0);
            }

            // Calculate how many graph points to display.
            // If the number of axis digits is >2, scale back how many
            // graph points to show
            num_points = m_graphdata_count;
            x = MAXIMUM_GRAPH_POINTS - ((m_axisDigits - 2)*5);
            if(x<num_points) num_points = x;

            // Draw the temperature graph for each sensor
            for(uint8_t channel = 0; channel< SENSOR_COUNT; channel++)
            {
                int16_t tmp16;
                uint8_t p,index;

                // if the sensor is out of range, don't show it. If we are showing one
                // channel, ignore the others
                if(m_graphdata[channel][m_graphdata_index] == OUT_OF_RANGE_INT ||
                    (channel != m_current_channel && m_current_channel < 4) )
                {
                  continue;
                }

                tmp16 = _t1000_convertTemperatureInt(m_graphdata[channel][m_graphdata_index]);

                _t1000_printtemp(m_scratch, tmp16);
                lcd_print(m_scratch,0,(channel*33)+2);

                // Get the position of the latest point
                //p = temperature_to_pixel(graph[sensor][graphCurrentPoint]);
                p = _t1000_temperature_to_pixel(tmp16);

                // Draw the channel number at the latest point
                sprintf(m_scratch,"%d",channel+1);
                lcd_print(m_scratch,(uint8_t)(p-3),(uint8_t)(112+5*channel));

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

        }

        // Detect button pushes
        if(m_button_mask!=0)
        {
            if(m_button_mask&SW_MASK(SW_A))
            {
                if(m_logging)
                    _t1000_record_stop();
                else
                    _t1000_record_start();
                m_button_mask&=~SW_MASK(SW_A);
            }
            if(m_button_mask&SW_MASK(SW_B))
            {
                m_button_mask&=~SW_MASK(SW_B);
            }
            if(m_button_mask&SW_MASK(SW_C))
            {
                if(m_temperatureUnit==TEMPERATURE_UNITS_K)
                    m_temperatureUnit=TEMPERATURE_UNITS_C;
                else
                    m_temperatureUnit++;
                m_button_mask&=~SW_MASK(SW_C);
            }
            if(m_button_mask&SW_MASK(SW_D))
            {
                m_current_channel++;
                if(m_current_channel>SENSOR_COUNT)
                    m_current_channel=0;
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
void _t1000_fake_data()
{
    // DEBUG: Fake some data
    m_graphdata[0][m_graphdata_index] = OUT_OF_RANGE_INT;
    m_graphdata[1][m_graphdata_index] = OUT_OF_RANGE_INT;
    m_graphdata[2][m_graphdata_index] = OUT_OF_RANGE_INT;
    m_graphdata[3][m_graphdata_index] = OUT_OF_RANGE_INT;
    #if 0
    // EXTREME!
    m_graphdata[0][m_graphdata_index] = 30000; // 3270.9 C
    m_graphdata[1][m_graphdata_index] = -2732; // -273.2C (abs zero)
    m_graphdata[2][m_graphdata_index] = OUT_OF_RANGE_INT;
    m_graphdata[3][m_graphdata_index] = OUT_OF_RANGE_INT;
    #endif
    #if 0
    m_graphdata[0][m_graphdata_index] = 1234;
    m_graphdata[1][m_graphdata_index] = -345;
    m_graphdata[2][m_graphdata_index] = OUT_OF_RANGE_INT;
    m_graphdata[3][m_graphdata_index] = OUT_OF_RANGE_INT;
    #endif
    #if 0
    #define OFFSET  30.0
    #define SCALE   10.0
    #define ADD     0.05
    static double val=0.0;
    double tmpdbl;
    tmpdbl = ((SCALE*sin(val))+OFFSET)*10;
    m_graphdata[0][m_graphdata_index] = (int16_t)tmpdbl;
    m_graphdata[2][m_graphdata_index] = (int16_t)tmpdbl+5.0;
    val += ADD;
    #endif

    #if 1
    static int16_t val=300;
    static int16_t step=5;
    m_graphdata[0][m_graphdata_index] = val;
    m_graphdata[1][m_graphdata_index] = val+50;
    m_graphdata[2][m_graphdata_index] = val+100;
    m_graphdata[3][m_graphdata_index] = val+150;
    val+=step;
    if(val>=400 || val<= 200) step=step*-1;
    #endif

#if 0
    temperatures_int[0] = convertTemperatureInt(temperatures_int[0]);
    temperatures_int[1] = convertTemperatureInt(temperatures_int[1]);
    temperatures_int[2] = convertTemperatureInt(temperatures_int[2]);
    temperatures_int[3] = convertTemperatureInt(temperatures_int[3]);
#endif


    return;
}
uint8_t _t1000_numlength(int16_t num)
{
    if(num>999 || num<-99) return 4;
    if(num>99 || num<-9) return 3;
    if(num>9 || num<0) return 2;
    return 1;
}

char * _t1000_printtemp(char * buf, int16_t temp)
{
    uint8_t tmp8;
    tmp8 = (uint8_t)abs(temp%10);
    if(temp>9999)
        sprintf(buf,"%d.%d",((temp)/10),tmp8);
    else
        sprintf(buf,"%3d.%d",((temp)/10),tmp8);
    return buf;
}
char _t1000_current_unit()
{
    switch(m_temperatureUnit){
    case TEMPERATURE_UNITS_F: return 'F';
    case TEMPERATURE_UNITS_K: return 'K';
    }
    return 'C';
}

#include "ff.h"
FIL * m_log_file = NULL;
void _t1000_record_start()
{
    char fileName[12];
    uint16_t i = 0;

    // Make sure we are in a known state
    if(m_logging)
        _t1000_record_stop();

      //Start logging
#if 0
      // Create LDxxxx.CSV for the lowest value of x.
      // Jump by 100 here
      do{
          i+=100;
          sprintf(fileName,"LD%04d.CSV",i);
      }while(sd.exists(fileName));
      // We now know that value doesn't exist, go back 100 and search from here
      i-=100;
      do{
          i+=10;
          sprintf(fileName,"LD%04d.CSV",i);
          // This could take a while, so reset the watchdog here
          wdt_reset();
      }while(sd.exists(fileName));
      // We now know that value doesn't exist, go back 10 and search from here
      i-=10;
      do{
          i+=1;
          sprintf(fileName,"LD%04d.CSV",i);
      }while(sd.exists(fileName));

      if(!file.open(fileName, O_CREAT | O_WRITE | O_EXCL))
      {
        return false;
      }
      file.clearWriteError();

      // write data header
      file.print("time (s)");

      #endif
      #if SERIAL_OUTPUT_ENABLED
      Serial.print("v");
      Serial.println(FIRMWARE_VERSION);

      Serial.print("File: ");
      Serial.println(fileName);
      Serial.print("time (s)");


      for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        #if SD_LOGGING_ENABLED
        file.print(", temp_");
        file.print(i, DEC);
        #endif
        #if SERIAL_OUTPUT_ENABLED
        Serial.print(", temp_");
        Serial.print(i, DEC);
        #endif

        switch(temperatureUnit) {
        case TEMPERATURE_UNITS_C:
          #if SD_LOGGING_ENABLED
          file.print(" (C)");
          #endif
          #if SERIAL_OUTPUT_ENABLED
          Serial.print(" (C)");
          #endif
          break;
        case TEMPERATURE_UNITS_F:
          #if SD_LOGGING_ENABLED
          file.print(" (F)");
          #endif
          #if SERIAL_OUTPUT_ENABLED
          Serial.print(" (F)");
          #endif
          break;
        case TEMPERATURE_UNITS_K:
          #if SD_LOGGING_ENABLED
          file.print(" (K)");
          #endif
          #if SERIAL_OUTPUT_ENABLED
          Serial.print(" (K)");
          #endif
          break;
        }
      }
      #if SD_LOGGING_ENABLED
      file.println();
      file.flush();
      #endif
      #if SERIAL_OUTPUT_ENABLED
      Serial.println();
      #endif

      return (file.getWriteError() == false);

#endif
    return;
}
void _t1000_record_stop()
{
    //FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);             /* Open or create a file */
    // Close file
    m_logging = false;
    return;
}
// eof
