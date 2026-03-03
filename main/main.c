#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "stdio.h"
#include "esp_adc/adc_cali.h"
#include "freertos/timers.h"

#define PSEAT_PIN         GPIO_NUM_4       // passenger seat button pin 4
#define ON_BUTTON         GPIO_NUM_2       // ON button pin 2
#define PBELT_PIN         GPIO_NUM_6       // passenger belt switch pin 6
#define DBELT_PIN         GPIO_NUM_7       // driver belt switch pin 7
#define IGNITION_BUTTON   GPIO_NUM_17      // ignition button pin 17
#define ON_LED            GPIO_NUM_21      // ON LED pin 21
#define REFILL_LED        GPIO_NUM_4       // refill LED pin 4
#define ALARM_PIN         GPIO_NUM_12      // alarm pin 12
#define ADC_ATTEN         ADC_ATTEN_DB_12  // set ADC attenuation
#define BITWIDTH          ADC_BITWIDTH_12  // set ADC bitwidth
#define DELAY_MS          250              // delay in milliseconds
#define DELAY2_MS         2000             // delay in milliseconds

//LEDC config variables (servo motor)
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (7)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//PWM signal frequency required by servo motor
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 50 Hz for a 20ms period.
//minimum and maximum servo pulse widths
#define LEDC_DUTY_MIN           (240) // Set duty to 2.7% (0 deg angle position)
#define LEDC_DUTY_MAX           (590) // Set duty to 7.2% to achieve an angle of 90% (max)
//servo motor step sizes to change how fast the servo motor rotates
#define STEP_HIGH_SPEED      (12.2) // fast speed -- 90 deg in 0.6 sec
#define STEP_LOW_SPEED       (4.92) // slow speed -- 90 deg in 1.5 sec

//ADC config vars (potentiometer)
#define MODE_SELECTOR           ADC_CHANNEL_6 //MUST BE ADC CHANNEL

// Keypad variables
#define LOOP_DELAY_MS           10      // Loop sampling time (ms)
#define DEBOUNCE_TIME           40      // Debounce time (ms)
#define NROWS                   4       // Number of keypad rows
#define NCOLS                   4       // Number of keypad columns
#define ACTIVE                  0       // Keypad active state (0 = low, 1 = high)
#define NOPRESS                 '\0'    // NOPRESS character
// Initialize keypad rows, columns, layout
int row_pins[] = {GPIO_NUM_3, GPIO_NUM_8, GPIO_NUM_18, GPIO_NUM_17};     // Pin numbers for rows
int col_pins[] = {GPIO_NUM_16, GPIO_NUM_15, GPIO_NUM_7, GPIO_NUM_6};   // Pin numbers for columns
char keypad_array[NROWS][NCOLS] = {   // Keypad layout
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Global state variables
bool dseat = false;     
bool pseat = false;     
bool dbelt = false;     
bool pbelt = false;     
bool ignition = false;  
bool engine_running = false;
bool last_ignition_button = false;
int executed = 0;
int ready_led = 0;
int ignition_off = 0;
int error = 0;

// Wiper system globals
int OFF = 1024;
int INT = 2048;
int LOW = 3072;
bool off_selected = false;
bool int_selected = false;
bool low_selected = false;
bool high_selected = false;
int timeDelaySel1 = 1365;
int timeDelaySel2 = 2730;
int INTtimeDelay = 1000;
int mode = 0;           // Current mode: 0=OFF, 1=INT, 2=LOW, 3=HIGH
int state = 0;          // Different wiper states: 0=PARKED, 1=WAIT, 2=UP, 3=DOWN
int timeInterval = 0;
float duty = LEDC_DUTY_MIN;
float current_step = STEP_LOW_SPEED;
float requested_step = STEP_LOW_SPEED;

// LCD global handling
hd44780_t lcd_display;

// Stop the servo motor at 0 degrees
void stop_servo_motor(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// LCD Task
void lcd_task(void *pvParameters)
{
    // Initialize LCD
    lcd_display.write_cb = NULL;
    lcd_display.font = HD44780_FONT_5X8;
    lcd_display.lines = 2;
    lcd_display.pins.rs = GPIO_NUM_38;
    lcd_display.pins.e  = GPIO_NUM_37;
    lcd_display.pins.d4 = GPIO_NUM_36;
    lcd_display.pins.d5 = GPIO_NUM_35;
    lcd_display.pins.d6 = GPIO_NUM_48;
    lcd_display.pins.d7 = GPIO_NUM_47;
    lcd_display.pins.bl = HD44780_NOT_USED;

    ESP_ERROR_CHECK(hd44780_init(&lcd_display));

    while (1)
    {
        // Clear the display
        hd44780_clear(&lcd_display);
        
        if (engine_running) {
            // Show "Wiper mode:" on first line
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Wiper mode:");
            
            // Show the current mode on second line
            hd44780_gotoxy(&lcd_display, 0, 1);
            if (mode == 0) {
                hd44780_puts(&lcd_display, "OFF");
            }
            else if (mode == 1) {
                // For INT mode, show the delay time too
                if (INTtimeDelay == 1000) {
                    hd44780_puts(&lcd_display, "INT-SHORT");
                }
                else if (INTtimeDelay == 3000) {
                    hd44780_puts(&lcd_display, "INT-MEDIUM");
                }
                else {
                    hd44780_puts(&lcd_display, "INT-LONG");
                }
            }
            else if (mode == 2) {
                hd44780_puts(&lcd_display, "LOW");
            }
            else if (mode == 3) {
                hd44780_puts(&lcd_display, "HIGH");
            }
        } else {
            // Engine is off
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Engine Off");
        }
        // Add delay
        vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
    }
}

//Initialize servo motor
static void ledc_init(void)
{
    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// initialize FSM states
typedef enum {
    Wait_for_press,
    Debounce,
    Wait_for_release,
} State_t;

void init_keypad() {
    // initialize row pins as outputs and set to inactive state
    for (int i = 0; i < NROWS; i++) {                           // incrementer to count through rows
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);      // set as output
        gpio_set_level(row_pins[i], !ACTIVE);                   // set high
    }

    // initialize column pins as inputs with pull-up resistors
    for (int j = 0; j < NCOLS; j++) {                           // incrementer to count through columns
        gpio_set_direction(col_pins[j], GPIO_MODE_INPUT);       // set as input
        gpio_pullup_en(col_pins[j]);                            // set internal pull up resistor
    }
}

char scan_keypad() {
    char new_key = NOPRESS;                     //declare variable to hold key that the function returns

    for (int i = 0; i < NROWS; i++) {           // scan each row, setting one row active at a time and then check each column to see if active
        gpio_set_level(row_pins[i], ACTIVE);    // set current row to active state

        for (int j = 0; j < NCOLS; j++) {                   // check each column for active state
            if (gpio_get_level(col_pins[j]) == ACTIVE) {    // if column is high,
                new_key = keypad_array[i][j];                   // get the pressed key from the array         
            }
        }
        gpio_set_level(row_pins[i], !ACTIVE);   // set current row back to inactive state
    }
    return new_key;                             // return key being pressed
}

void app_main(void)
{
    // Configure GPIO pins
    // set driver seat pin config to input and internal pullup
    gpio_reset_pin(ON_BUTTON);
    gpio_set_direction(ON_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(ON_BUTTON);

    // set passenger seat pin config to input and internal pullup
    gpio_reset_pin(PSEAT_PIN);
    gpio_set_direction(PSEAT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(PSEAT_PIN);

    // set driver belt pin config to input and internal pullup
    gpio_reset_pin(DBELT_PIN);
    gpio_set_direction(DBELT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DBELT_PIN);

    // set passenger belt pin config to input and internal pullup
    gpio_reset_pin(PBELT_PIN);
    gpio_set_direction(PBELT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(PBELT_PIN);

    // set ignition button config to input and internal pullup
    gpio_reset_pin(IGNITION_BUTTON);
    gpio_set_direction(IGNITION_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION_BUTTON);

    // set ready led pin config to output, level 0
    gpio_reset_pin(READY_LED);
    gpio_set_direction(READY_LED, GPIO_MODE_OUTPUT);

    // set success led pin config to output, level 0
    gpio_reset_pin(SUCCESS_LED);
    gpio_set_direction(SUCCESS_LED, GPIO_MODE_OUTPUT);

    // set alarm pin config to output, level 0
    gpio_reset_pin(ALARM_PIN);
    gpio_set_direction(ALARM_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize servo motor
    ledc_init();
    // Start with motor stopped
    stop_servo_motor();
    
    // POTENTIOMETER //
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;                              // Unit handle
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // Populate unit handle

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                  // Channel configuration

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MODE_SELECTOR, &chan_config));
    
    // Create Task for LCD Display
    xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 5, NULL);
    
    int modeSel_adc_bits;
    int delayTimeSel_adc_bits;
    int loop_count = 0;

    // KEYPAD //
    init_keypad();  // initialize the keypad

    // FSM variables
    State_t state = Wait_for_press; // initial state
    char new_key = NOPRESS;         // key currently pressed
    char last_key = NOPRESS;        // last key pressed
    int time = 0;                   // time debounce delay verify
    bool timed_out = false;         // checks if key pressed lasts longer than debounce time
    
    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        loop_count++;

        // Read ADC values
        adc_oneshot_read(adc1_handle, MODE_SELECTOR, &modeSel_adc_bits);            // Read ADC bits (mode potentiometer)
    
        // Read GPIO inputs
        dseat = gpio_get_level(DSEAT_PIN) == 0;
        pseat = gpio_get_level(PSEAT_PIN) == 0;
        dbelt = gpio_get_level(DBELT_PIN) == 0;
        pbelt = gpio_get_level(PBELT_PIN) == 0;
        ignition = gpio_get_level(IGNITION_BUTTON) == 0;
        
        // Determine wiper mode selection
        off_selected = (modeSel_adc_bits < OFF);
        int_selected = (modeSel_adc_bits >= OFF && modeSel_adc_bits < INT);
        low_selected = (modeSel_adc_bits >= INT && modeSel_adc_bits < LOW);
        high_selected = (modeSel_adc_bits >= LOW);

        //** FSM FOR KEYPAD **//
         // update FSM inputs
        new_key = scan_keypad();                    // assign each new key press as the key that is returned from scan_keypad function 
        timed_out = (time >= DEBOUNCE_TIME);        // checks if key pressed lasts longer than debounce time 

        switch (state) {
            case Wait_for_press:
                if (new_key != NOPRESS) {           // if a key is pressed, set time = 0 and assign as new key pressed
                    time = 0;                       // reset debounce time delay incrementer
                    last_key = new_key;             // set new key pressed
                    state = Debounce;               // transition to next state
                } else {
                    state = Wait_for_press;         // if there is no new key being pressed, stay in initial state
                }
                break;

            case Debounce:
                if (timed_out) {                                // if the debounce time of 40 ms has passed and key pressed remains same, 
                    if (new_key == last_key) {                  // confirm valid key press
                        printf("Key Pressed: %c\n", last_key);  // print once whichever key is being pressed
                        state = Wait_for_release;               // transition to next state
                    } else {                                    // or else, if the key changed during debounce delay (glitch), 
                        state = Wait_for_press;                 // go back to initial state
                    }
                } else {                                        // else, keep incrementing counter until at 40 ms (debounce time), stay in state
                    time += LOOP_DELAY_MS;                      // add 10 ms to time
                }
                break;

            case Wait_for_release:
                if (new_key == NOPRESS) {      // if key released,
                    state = Wait_for_press;    // go back to waiting for press (initial state)
                } else {                       // else, if key still pressed,
                    state = Wait_for_release;  // stay in state
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS)); // add loop delay

        //** SELECTION PROCESS SUBSYSTEM **//
        // if the ON button is pressed, print the welcome message once
        if (dseat){
            if (executed == 0){     // if executed equals 0, print welcome message
                printf("Welcome to enhanced alarm system model 218-W25\n"); 
                executed = 1;       // set executed = 1 so welcome message only prints once
            }
        }

        // Check if ignition is enabled
        bool ignition_enabled = dseat && pseat && dbelt && pbelt;
        
        // if all of the conditions are met
        if (ignition_enabled){
            //set ready led to ON
            if (executed == 1 && ready_led == 0){
                gpio_set_level(READY_LED, 1);
                ready_led = 1;
            }
            // if ignition button is pressed while all conditions are met
            if (ignition == true && executed == 1){
                // turn on ignition LED and turn off ready LED
                gpio_set_level(SUCCESS_LED, 1);
                gpio_set_level(READY_LED, 0);
                gpio_set_level(ALARM_PIN, 0);
                engine_running = true;
                // print engine started message once
                printf("Engine started!\n");
                executed = 2;       // set executed = 2 so engine started message only prints once
            }
        }

        // otherwise (at least one condition is not satisfied)
        else{
            // set ready LED to OFF and set variable ready_led to 0
            gpio_set_level(READY_LED,0);
            ready_led = 0;
            // if ignition button is pressed while conditions are not satisfied
            if (ignition==true && executed < 2){
                    // turn on alarm buzzer
                    gpio_set_level(ALARM_PIN, 1);
                    printf("Ignition inhibited.\n");
                    // check which conditions are not met, print corresponding message
                    if (!pseat){
                        printf("Passenger seat not occupied.\n");
                    }
                    if (!dseat){
                        printf("Driver seat not occupied.\n");
                    }
                    if (!pbelt){
                        printf("Passenger seatbelt not fastened.\n");
                    }
                    if (!dbelt){
                        printf("Drivers seatbelt not fastened.\n");
                    }
                    executed = 4;   
            }
        }

        // if executed = 4 (failed ignition) and ignition button is released
        if (ignition == false && executed == 4){
            // reset to state after welcome message, testing for conditions
            executed = 1;
        }

        // if ignition is successfully started and then ignition is released, set ignition_off = 1
        if (executed == 2 && ignition == false){
            ignition_off = 1;
        }

        // if ignition_off = 1 and inition is pressed, turn off all LEDs
        if (ignition_off==1 && ignition == true){
            engine_running = false;
            gpio_set_level(SUCCESS_LED,0);          // turn off ignition
            state = 0; // Reset wiper to park
            duty = LEDC_DUTY_MIN;
            stop_servo_motor();
            executed = 3;                           // set executed = 3 to keep LEDs off
        } 
        last_ignition_button = ignition;

        //** WINDSHIELD WIPER SUBSYSTEM **//
        
        if (engine_running) {
            // Determine delay time
            if (delayTimeSel_adc_bits < timeDelaySel1) {
                INTtimeDelay = 1000; // SHORT
            } else if (delayTimeSel_adc_bits < timeDelaySel2) {
                INTtimeDelay = 3000; // MEDIUM
            } else {
                INTtimeDelay = 5000; // LONG
            }

            // Determine mode (for display)
            int old_mode = mode;
            if (off_selected) {
                mode = 0;
            }
            else if (int_selected) {
                mode = 1;
            }
            else if (low_selected) {
                mode = 2;
            }
            else {
                mode = 3;
            }

            // Determine selected speed
            if (high_selected) {
                requested_step = STEP_HIGH_SPEED;
            } else {
                requested_step = STEP_LOW_SPEED;
            }

            // States for wiper control
            int old_state = state;
            bool entering_new_state = false;
            
            // Wipers parked at 0 degrees
            if (state == 0) {
                // Only set position when first entering this state
                if (old_state != 0) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    entering_new_state = true;
                }
                
                // Start moving if mode selected
                if (int_selected) {
                    state = 1; // wait at 0 degrees
                    timeInterval = 0;
                    current_step = requested_step; // set requested speed
                } else if (low_selected || high_selected) {
                    state = 2; // start moving up immediately
                    current_step = requested_step; // set speed
                }
            }
            
            // INT mode (waiting at 0 degrees for the selected delay time)
            else if (state == 1) {
                timeInterval += 20;
                
                // Only set position when first entering wait state
                if (old_state != 1) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    entering_new_state = true;
                }
                
                // Check if we should exit wait
                if (off_selected) {
                    state = 0; // Stay parked
                } else if (timeInterval >= INTtimeDelay) {
                    state = 2; // Start moving up
                }
            }
            
            // Wipers moving up from 0 to 90 degrees
            else if (state == 2) {
                duty += current_step;
                if (duty >= LEDC_DUTY_MAX) {
                    duty = LEDC_DUTY_MAX;
                    state = 3; // Start moving down
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (int)duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }
            
            // Wipers moving down from 90 to 0 degrees
            else if (state == 3) {
                duty -= current_step;
                if (duty <= LEDC_DUTY_MIN) {
                    duty = LEDC_DUTY_MIN;
                    // Update speed for next cycle
                    current_step = requested_step;
                    // Determine next state based on mode
                    if (off_selected) {
                        state = 0; // reset wipers to park
                    } else if (int_selected) {
                        state = 1; // have wipers wait
                        timeInterval = 0;
                    } else {
                        state = 2; // Continuous (LOW/HIGH)
                    }
                } else {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (int)duty);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                }
            }
            
        } else {
            // Engine off, make sure wipers are parked
            if (state != 0) {
                state = 0;
                duty = LEDC_DUTY_MIN;
                stop_servo_motor();
            }
        }
    }
}
