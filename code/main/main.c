

/*
  Authors: Cristian Palencia, Yohan Kim, Zhilang Gui, Tanveer Dhilon
*/

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include <time.h>
#include "esp_sntp.h"
#include "driver/gptimer.h" 
#include "freertos/queue.h" 
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0x01 // i2c nack value (Was FF)

// ADXL343
#define AXL_ADDR                         ADXL343_ADDRESS // 0x53

// 14-Segment Display
#define SEGMENT_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Button
#define BUTTON 16

// Buzzer
#define ENABLE_PIN 16    // PWM control pin (LEDC)
#define INPUT_PIN1 19    // Input for buzzer on/off
#define INPUT_PIN2 18    // Input for buzzer on/off
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY 5000 // Frequency in Hz (adjust for tone)
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // 8-bit resolution

float xVal = 0.0, yVal = 0.0, zVal = 0.0; // acceleration values
float magnitude = 0.0;
int state = 0; // 0: not active, 1: active, 2: highly active
int timer_count = -1;
char alphanumeric_string[50] = "";


/* VARIABLES FOR TEMPERATURE CONVERSION */
#define T0 25
#define B 3435
#define R0 10000
#define NO_OF_SAMPLES   10
#define DEFAULT_VREF    1100
float Temperature = 0.0;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_temp = ADC_CHANNEL_6;        // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

void configure_pwm() {
    // Configure LEDC PWM for the ENABLE pin
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENABLE_PIN,
        .duty = 0, // Start with PWM off
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void configure_gpio() {
    // Configure GPIO for the input pins to control the buzzer
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INPUT_PIN1) | (1ULL << INPUT_PIN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void buzzer_on() {
    // Set GPIO to turn on the buzzer
    gpio_set_level(INPUT_PIN1, 1);
    gpio_set_level(INPUT_PIN2, 0);
}

void buzzer_off() {
    // Set GPIO to turn off the buzzer
    gpio_set_level(INPUT_PIN1, 0);
    gpio_set_level(INPUT_PIN2, 0);
}

// ChatGPT
void buzzer_task() { 
  while(1) {
    buzzer_on();  // Turn on the buzzer

    // Gradually increase PWM duty cycle to vary the tone
    for (int duty = 0; duty <= 255; duty++) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for smooth tone change
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second

    buzzer_off(); // Turn off the buzzer
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0); // Stop PWM
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
  }
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
  // Debug
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      count++;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AXL_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AXL_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SEGMENT_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SEGMENT_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SEGMENT_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

//////////////////////////////////////////

/// timer functions ///////
// A simple structure for queue elements
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a FIFO queue for timer-based events
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System log tags -- get logged when things happen, for debugging 
static const char *TAG_TIMER = "ec444: timer";       

// Timer interrupt handler -- callback timer function -- from GPTimer guide example
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;    // represents state info passed to callback, if needed
    example_queue_element_t ele = {
          .event_count = edata->count_value                   // Retrieve count value and send to queue
      };
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken); // Puts data into queue and alerts other recipients
    return (high_task_awoken == pdTRUE);  		      // pdTRUE indicates data posted successfully
}

// Timer configuration -- from GPTimer guide example
static void alarm_init() {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // instantiates timer
  
    gptimer_event_callbacks_t cbs = { // Set alarm callback
      .on_alarm = timer_on_alarm_cb,  // This is a specific supported callback from callbacks list
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue)); // This registers the callback
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                                      // Enables timer interrupt ISR

    ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload"); 
    gptimer_alarm_config_t alarm_config = { // Configure the alarm 
      .reload_count = 0,                    // counter will reload with 0 on alarm event
      .alarm_count = 1000000,            // period = 10*1s = 10s
      .flags.auto_reload_on_alarm = true,   // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));  // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                            // this starts the timer
}

///////////////////////////////


/* CHATGPT */
// Write one byte to a register
int writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXL_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* CHATGPT */
// Read one byte from a register
uint8_t readRegister(uint8_t reg) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXL_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXL_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

/* CHATGPT */
// Read 16 bits from a register
int16_t read16(uint8_t reg) {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXL_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXL_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, 2, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Combine the two bytes into one 16-bit integer
    return (int16_t)((data[1] << 8) | data[0]);
}


void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}


float calcMagnitude(float x, float y, float z) {
  return sqrt((x * x) + (y * y) + (z * z));
}

void modifyState(float magnitude) {
  int prev_state = state;

  // modify the state based on the input magnitude
  if(magnitude <= 10) {
    state = 0;
  }
  else if(10 < magnitude && magnitude <= 25) {
    state = 1;
  } 
  else if(magnitude > 25) {
    state = 2;
  }

  // if the state has changed reset timer
  if(timer_count >= 0) {
    if (state != prev_state) {
      timer_count = 0;
    }
  }
  

  // now that we checked if we had to reset the timer format the string (only do this if timer has been initialize!)
  if(timer_count >= 0) {
    if(state == 0) {
      sprintf(alphanumeric_string, "OTTO - INACTIVE - %d ", timer_count);
    }
    else if(state == 1) {
      sprintf(alphanumeric_string, "OTTO - ACTIVE - %d ", timer_count);
    }
    else if(state == 2) {
      sprintf(alphanumeric_string, "OTTO - HIGHLY ACTIVE - %d ", timer_count);
    }
  }

}

// Task to continuously poll acceleration and compute magnitude
// it will also format the current alphanumeric string based on computed magnitude
static void test_adxl343() {

  while (1) {
    getAccel(&xVal, &yVal, &zVal);
    magnitude = calcMagnitude(xVal, yVal, zVal);
    modifyState(magnitude);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


// Timer task -- what to do when the timer alarm triggers 
static void timer_evt_task(void *arg) {

    while (1) {
        // Transfer from queue and do something if triggered
        if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000))) {
            if(timer_count >= 0) {
                // ensure counter hasn't exceeded 99
                timer_count = (timer_count) % 99;
                                
                // increment the counter variables
                timer_count++;
            }
            else {
                printf("timer not started, press button\n");
            }
        }
    }
}

// Function to convert a given character to an 14 segment display output
/* USED: wikipedia for char to 16 bit conversion --> https://en.wikipedia.org/wiki/Fourteen-segment_display */
uint16_t charToSegment(char c) {
    switch (c) {
        // Uppercase letters
        case 'A': return 0xF7; 
        case 'B': return 0x128F; 
        case 'C': return 0x39; 
        case 'D': return 0x120F; 
        case 'E': return 0xF9; 
        case 'F': return 0xF1; 
        case 'G': return 0xBD; 
        case 'H': return 0xF6; 
        case 'I': return 0x1209; 
        case 'J': return 0x1E; 
        case 'K': return 0x2470; 
        case 'L': return 0x38; 
        case 'M': return 0x536; 
        case 'N': return 0x2136; 
        case 'O': return 0x3F; 
        case 'P': return 0xF3; 
        case 'Q': return 0x203F; 
        case 'R': return 0x20F3; 
        case 'S': return 0x18D; 
        case 'T': return 0x1201; 
        case 'U': return 0x3E; 
        case 'V': return 0xC30; 
        case 'W': return 0x2836; 
        case 'X': return 0x2D00; 
        case 'Y': return 0x1500; 
        case 'Z': return 0xC09; 
        // Lowercase letters
        case 'a': return 0b0001000001011000; 
        case 'b': return 0b0010000001111000;  
        case 'c': return 0b0000000011011000; 
        case 'd': return 0b0000100010001110; 
        case 'e': return 0b0000100001011000; 
        case 'f': return 0b0000000001110001; 
        case 'g': return 0b0000010010001110; 
        case 'h': return 0b0001000001110000; 
        case 'i': return 0b0001000000000000; 
        case 'j': return 0b0000000000001110;  
        case 'k': return 0b0011011000000000; 
        case 'l': return 0b0000000000110000;   
        case 'm': return 0b0001000011010100;  
        case 'n': return 0b0001000001010000;  
        case 'o': return 0b0000000011011100;   
        case 'p': return 0b0000000101110000;   
        case 'q': return 0b0000010010000110; 
        case 'r': return 0b0000000001010000;   
        case 's': return 0b0010000010001000;  
        case 't': return 0b0000000001111000;   
        case 'u': return 0b0000000000011100;   
        case 'v': return 0b0010000000000100;  
        case 'w': return 0b0010100000010100; 
        case 'x': return 0b0010100011000000; 
        case 'y': return 0b0010000000001100; 
        case 'z': return 0b0000100001001000;
        case '-': return 0b0000000011000000; 
        // Digits
        case '0': return 0x0C3F; 
        case '1': return 0x0406;
        case '2': return 0x00DB;
        case '3': return 0x008F;
        case '4': return 0x00E6; 
        case '5': return 0x00ED;
        case '6': return 0x00FD; 
        case '7': return 0x1401; 
        case '8': return 0x00FF; 
        case '9': return 0x00E7;
        default: return 0; // Return 0 for unsupported characters
    }
}

// Define button interrupt handler -- just sets the flag on interrupt
static void IRAM_ATTR gpio_isr_handler(void* arg){
    timer_count = 0;    
}

static void button_init() {
    // configure button
    gpio_reset_pin(BUTTON);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);

    // Configure the interrupt on the button pin (falling edge, active-low)
    gpio_set_intr_type(BUTTON, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, gpio_isr_handler, NULL);
}

// this function is in charge of reading the alphanumeric_string and writing data to it
static void alphanumeric_data() {
    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Write to characters to buffer
    char displaybuffer[50] = "";

    // Continually writes the same command
    while (1) {
        
        // copy the string over to a non changing variable so that we can properly print it out
        strcpy(displaybuffer, alphanumeric_string);

        // iterate through the entire string
        for (uint8_t i=0; i<strlen(displaybuffer); i++) {
            i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
            i2c_master_start(cmd4);
            i2c_master_write_byte(cmd4, ( SEGMENT_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);

            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[i]) & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[i]) >> 8, ACK_CHECK_EN);

            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[(i+1)%strlen(displaybuffer)]) & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[(i+1)%strlen(displaybuffer)]) >> 8, ACK_CHECK_EN);

            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[(i+2)%strlen(displaybuffer)]) & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[(i+2)%strlen(displaybuffer)]) >> 8, ACK_CHECK_EN);

            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[(i+3)%strlen(displaybuffer)]) & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, charToSegment(displaybuffer[(i+3)%strlen(displaybuffer)]) >> 8, ACK_CHECK_EN);

            i2c_master_stop(cmd4);
            ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd4);

            vTaskDelay(400 / portTICK_PERIOD_MS);
        }

    }

}


static void configure_ADC_temp(){
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel_temp, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel_temp, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    
}

static void report_temperature() {
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel_temp);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel_temp, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        adc_reading /= NO_OF_SAMPLES;

        /* Resistance Calculation */
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);  // get voltage reading
        float Resistance = (3300 - voltage) * 1000.0 / voltage;                 // using voltage divider equation solve for thermistor Resistance

        /* Temperature Conversion */
        Temperature = Resistance / R0;
        Temperature = log(Temperature);
        Temperature = (1.0 / B) * Temperature;
        Temperature = (1.0 / (T0 + 273.15)) + Temperature;
        Temperature = (1.0 / Temperature) - 273.15;
    }
}

// print data over serial
// Used by note app to print data using canvasjs
static void print_data() {
  while(1) {
    if(state == 0) printf("%02f Not Active\n", Temperature);
    else if(state == 1) printf("%02f Active\n", Temperature);
    else if(state == 2) printf("%02f Highly Active\n", Temperature);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void app_main() {
  // Routine
  i2c_master_init();
  i2c_scanner();

  // Check for ADXL343
  uint8_t deviceID;
  getDeviceID(&deviceID);
  if (deviceID != 0xE5) {
    printf("\n>>ADAXL343 Not Found\n");
  }

  // Timer queue initialize 
  timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
  if (!timer_queue) {
    ESP_LOGE(TAG_TIMER, "Creating queue failed");
    return;
  }

  alarm_init();
  configure_ADC_temp();
  button_init();

  configure_pwm();  // Initialize PWM for buzzer control
  configure_gpio(); // Initialize GPIO for controlling buzzer

  // Disable interrupts
  writeRegister(ADXL343_REG_INT_ENABLE, 0);

  // Set range
  setRange(ADXL343_RANGE_16_G);
  
  // Enable measurements
  writeRegister(ADXL343_REG_POWER_CTL, 0x08);

  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

  /* Tell VFS to use UART driver */
  esp_vfs_dev_uart_use_driver(UART_NUM_0);

  // Create task to poll ADXL343
  xTaskCreate(test_adxl343,"test_adxl343", 4096, NULL, 5, NULL);  
  xTaskCreate(print_data, "print_data", 4096, NULL, 5, NULL);
  xTaskCreate(timer_evt_task, "timer_evt_task", 8192, NULL, 5, NULL);
  xTaskCreate(report_temperature, "temperature_task", 4096, NULL, 5, NULL);
  xTaskCreate(alphanumeric_data, "alphanumeric_data", 8192, NULL, 5, NULL);
  xTaskCreate(buzzer_task, "buzzer_task", 4096, NULL, 5, NULL);
}
