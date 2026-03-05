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
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

// INPUT & OUTPUT variables
#define ON_LED               GPIO_NUM_21      // On LED pin 21
#define REFILL_LED           GPIO_NUM_4       // Refill LED pin 4
#define SELECT_BUTTON        GPIO_NUM_6       // selection button pin 6
#define REFILL_BUTTON        GPIO_NUM_5       // Refill button pin 5
#define ON_BUTTON            GPIO_NUM_2       // On button pin 2
#define MSG_DELAY            2000             // Time delay for display message

// POTENTIOMETER config variables
#define PORTION_SEL ADC_CHANNEL_0
#define ADC_ATTEN ADC_ATTEN_DB_12
#define BITWIDTH ADC_BITWIDTH_12
#define DELAY_MS 25

// SERVO MOTOR config variables
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (7)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 50 Hz for a 20ms period.
#define LEDC_DUTY_STOP           (240) // Set duty to 2.7% (0 deg angle position) (min pulse width)
#define LEDC_DUTY_FWD           (270) // Set duty to spin forward
        // Disk dispenser: 8 slices, each slice = 45 degrees
        // 45 deg duty = LEDC_DUTY_MIN + (LEDC_DUTY_MAX - LEDC_DUTY_MIN) / 2 = 415
#define SLICE_ROTATE_MS         (500) // 
#define SLICE_SETTLE_MS         (300) // 
#define TOTAL_FOOD_SLICES       (7)

// Keypad variables
#define LOOP_DELAY_MS           10      // Loop sampling time (ms)
#define DEBOUNCE_TIME           40      // Debounce time (ms)
#define NROWS                   4       // Number of keypad rows
#define NCOLS                   4       // Number of keypad columns
#define ACTIVE                  0       // Keypad active state (0 = low, 1 = high)
#define NOPRESS                 '\0'    // NOPRESS character

// Feeding schedule/refill defines
#define SMALL_MAX            1365       // ADC < 1365  → SMALL
#define MEDIUM_MAX           2730       // ADC < 2730  → MEDIUM, >= 2730 → LARGE
#define REFILL_THRESHOLD     7         // total rotations before refill LED triggers
#define MAX_DIGITS           2          // max digits for feedings-per-day entry

// Initialize keypad rows, columns, layout
int row_pins[] = {GPIO_NUM_14, GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_11};     // Pin numbers for rows
int col_pins[] = {GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_46, GPIO_NUM_3};   // Pin numbers for columns
char keypad_array[NROWS][NCOLS] = {   // Keypad layout
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Global state variables
bool turn_on = false;
bool select_portion = false;
bool refill = false;
int executed = 0;
int on_led = 0;

// Portion Selection globals
bool small_selected = false;
bool medium_selected = false;
bool large_selected = false;
int mode = 0;           // Current mode: 0=SMALL, 1=MEDIUM, 2=LARGE

// Feeding Schedule globals
int slices = 0; // rotations to perform this dispense (1,2,3)
int slice_count = 0; // total rotations since last refill
int feed_hour = 0;   // feed hour inputted on keypad (0-23)
int feed_minute = 0; // feed minute inputted on keypad (0-59)
int seconds_until_feed = 0; // current countdown value

// Keypad input buffer
char keypad_buf[MAX_DIGITS + 1];
int digit_count = 0;

// LCD global handling
hd44780_t lcd_display;

// Stop the servo motor at 0 degrees
void stop_servo_motor(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_STOP);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Rotate disk 45 degrees to dispense one slice of food, then stop and wait for food to fall using delay
void dispense_one_slice(void) {
    // Move to 45 deg (one slice forward)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FWD);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    vTaskDelay(SLICE_ROTATE_MS / portTICK_PERIOD_MS);   // wait for food to fall through
    stop_servo_motor();
    vTaskDelay(SLICE_SETTLE_MS / portTICK_PERIOD_MS);
}

// After all 7 slices are used, rotate another 45 degrees to return to starting position
void return_to_home(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FWD);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    vTaskDelay(SLICE_ROTATE_MS / portTICK_PERIOD_MS);
    stop_servo_motor();
    printf("Disk returned to starting position. \n"); // print debugger to verify
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

    char line2[17];

    while (1)
    {
        // Clear the display
        hd44780_clear(&lcd_display);
        
        if (turn_on && executed == 0) {
            // Show message when system turned on
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Smart Pet Feeder");
            hd44780_gotoxy(&lcd_display, 0, 1);
            hd44780_puts(&lcd_display, "Activated!");
        }
            
        if (executed == 1) {
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Portion Size:");
            // Show the current mode on second line
            hd44780_gotoxy(&lcd_display, 0, 1);
            if (mode == 0) {
                hd44780_puts(&lcd_display, "SMALL");
            }
            else if (mode == 1) {
                hd44780_puts(&lcd_display, "MEDIUM");
            }
            else {
                hd44780_puts(&lcd_display, "LARGE");
            }
        }

        if (executed == 2) {
            // feeding time entry via keypad (in military time HH:MM)
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Feed time (HHMM)");
            hd44780_gotoxy(&lcd_display, 0, 1);
            // Format typed digits as HH:MM with cursor
            char formatted[17]; // buffer for formatted string
            if (digit_count <= 2) { // only hours entered so far, show as HH_
                snprintf(formatted, sizeof(formatted), "%s_", keypad_buf); // append underscore cursor to end of input
            } else {
                snprintf(formatted, sizeof(formatted), "%.2s:%.2s_", // format first two digits as hours, next two as minutes, append underscore cursor
                         keypad_buf, keypad_buf + 2); // point to same buffer but offset by 2 to get minutes portion
            }
            hd44780_puts(&lcd_display, formatted); // display formatted input on LCD
        }

        if (executed == 3) {
            // Counting down to next feeding — show target time and remaining seconds
            hd44780_gotoxy(&lcd_display, 0, 0);
            snprintf(line2, sizeof(line2), "Feed at %02d:%02d", feed_hour, feed_minute);
            hd44780_puts(&lcd_display, line2);
            hd44780_gotoxy(&lcd_display, 0, 1);
            snprintf(line2, sizeof(line2), "%d sec", seconds_until_feed);
            hd44780_puts(&lcd_display, line2);
        }

        if (executed == 6) {
            // Low food warning
            int slices_remaining = REFILL_THRESHOLD - slice_count;
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Low food!");
            // hd44780_gotoxy(&lcd_display, 0, 1);
            // snprintf(line2, sizeof(line2), "%d left, need %d", slices_remaining, slices);
            // hd44780_puts(&lcd_display, line2);
        }

        if (executed == 4) {
            // Dispensing — show partial warning if not enough slices
            int slices_remaining = REFILL_THRESHOLD - slice_count;
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Dispensing...");
            hd44780_gotoxy(&lcd_display, 0, 1);
            if (slices_remaining < slices && slices_remaining > 0) {
                hd44780_puts(&lcd_display, "Partial portion!");
            } else {
                hd44780_puts(&lcd_display, "");
            }
        }

        if (executed == 5) {
            // Refill needed
            hd44780_gotoxy(&lcd_display, 0, 0);
            hd44780_puts(&lcd_display, "Refill needed!");
            hd44780_gotoxy(&lcd_display, 0, 1);
            hd44780_puts(&lcd_display, "Press REFILL btn");
        }
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
    char new_key = NOPRESS;                     // declare variable to hold key that the function returns

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
    // set on button config to input and internal pullup
    gpio_reset_pin(ON_BUTTON);
    gpio_set_direction(ON_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(ON_BUTTON);

    // set refill button config to input and internal pullup
    gpio_reset_pin(REFILL_BUTTON);
    gpio_set_direction(REFILL_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(REFILL_BUTTON);

    // set select button config to input and internal pullup
    gpio_reset_pin(SELECT_BUTTON);
    gpio_set_direction(SELECT_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(SELECT_BUTTON);

    // set on led pin config to output, level 0
    gpio_reset_pin(ON_LED);
    gpio_set_direction(ON_LED, GPIO_MODE_OUTPUT);

    // set refill led pin config to output, level 0
    gpio_reset_pin(REFILL_LED);
    gpio_set_direction(REFILL_LED, GPIO_MODE_OUTPUT);
    
    // SERVO MOTOR initialization //
    ledc_init(); // Initialize servo motor
    stop_servo_motor(); // Start with motor stopped
    
    // POTENTIOMETER initialization (ADC) //
    // Initialize ADC
    int PORTION_SEL_adc_bits;
    int loop_count = 0;

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;                              // Unit handle
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // Populate unit handle

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                  // Channel configuration

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PORTION_SEL, &chan_config));
    
    // LCD Display Initialization //
    xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 5, NULL); // Create Task for LCD Display

    // Set system clock to current time before flashing
    // NEED TO UPDATE t.tm_hour AND t.tm_min TO THE CURRENT TIME BEFORE FLASHING
    struct timeval demo_time = {0};
    struct tm t = {0};
    t.tm_year = 125;    // years since 1900 (2025)
    t.tm_mon  = 2;      // month (0-indexed, 2 = March)
    t.tm_mday = 5;      // day of month
    t.tm_hour = 15;     // !! SET THIS to current hour (24hr format) !!
    t.tm_min  = 59;     // !! SET THIS to current minute !!
    t.tm_sec  = 0;
    demo_time.tv_sec = mktime(&t);
    settimeofday(&demo_time, NULL);

    // KEYPAD //
    init_keypad();  // initialize the keypad

    // FSM variables
    State_t state = Wait_for_press; // initial state
    char new_key = NOPRESS;         // key currently pressed
    char last_key = NOPRESS;        // last key pressed
    char confirmed_key = NOPRESS;   // set only on the cycle a keypress is confirmed 
    int time = 0;                   // time debounce delay verify
    bool timed_out = false;         // checks if key pressed lasts longer than debounce time
    
    // clear keypad buffer
    memset(keypad_buf, 0, sizeof(keypad_buf));

    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        loop_count++;

        // Read ADC values
        adc_oneshot_read(adc1_handle, PORTION_SEL, &PORTION_SEL_adc_bits);            // Read ADC bits (mode potentiometer)
    
        // Read GPIO inputs
        turn_on = gpio_get_level(ON_BUTTON) == 0;
        select_portion = gpio_get_level(SELECT_BUTTON) == 0;
        refill = gpio_get_level(REFILL_BUTTON) == 0;

        // Determine wiper mode selection
        small_selected = (PORTION_SEL_adc_bits >= 0 && PORTION_SEL_adc_bits < SMALL_MAX);
        medium_selected = (PORTION_SEL_adc_bits >= SMALL_MAX && PORTION_SEL_adc_bits < MEDIUM_MAX);
        large_selected = (PORTION_SEL_adc_bits >= MEDIUM_MAX);

        //** FSM FOR KEYPAD **//
        // update FSM inputs
        confirmed_key = NOPRESS;
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
                        confirmed_key = last_key;               // mark key as confirmed for this cycle
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

        //** SELECTION PROCESS SUBSYSTEM **//

        // if the ON button is pressed, print the welcome message once
        if (turn_on){
            if (executed == 0 && on_led == 0){     // if executed equals 0, print welcome message
                gpio_set_level(ON_LED, 1);
                on_led = 1;
                printf("Smart Pet Feeder Activated!\n");
                vTaskDelay(MSG_DELAY / portTICK_PERIOD_MS);
                executed = 1;       // set executed = 1 so welcome message only prints once
                vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
            }
        }

        // portion selection
        if (executed == 1) {
            // update mode on potentiometer
            if (small_selected) {
                mode = 0; // SHORT
            } else if (medium_selected) {
                mode = 1; // MEDIUM
            } else {
                mode = 2; // LARGE
            }

            // Lock in portion when SELECT button is pressed
            if (select_portion) {
                slices = mode + 1;      // SMALL=1 slice, MEDIUM=2 slices, LARGE=3 slices
                printf("Portion selected: %s (%d slice(s))\n",
                    mode == 0 ? "SMALL" : mode == 1 ? "MEDIUM" : "LARGE", slices);
                memset(keypad_buf, 0, sizeof(keypad_buf));
                digit_count = 0;
                vTaskDelay(300 / portTICK_PERIOD_MS);   // brief delay to avoid button bounce
                executed = 2;
            }
        }

        // enter feeding time in military time (HHMM) on keypad and then confirm with select button
        if (executed == 2) {
            if (confirmed_key != NOPRESS) {
                if (confirmed_key >= '0' && confirmed_key <= '9' && digit_count < MAX_DIGITS) {
                    // append digit to buffer
                    keypad_buf[digit_count] = confirmed_key;
                    digit_count++;
                    keypad_buf[digit_count] = '\0';
                    printf("Time input so far: %s\n", keypad_buf);

                } else if (confirmed_key == '*' && digit_count > 0) {
                    // backspace: remove last digit
                    digit_count--;
                    keypad_buf[digit_count] = '\0';
                    printf("Backspace. Input now: %s\n", keypad_buf);
                }
            }
            
            // press select button to confirm the 4 digit time entry
            if (select_portion && digit_count == 4) {
                // parse HHMM into hours and minutes
                char hh[3] = {keypad_buf[0], keypad_buf[1], '\0'};
                char mm[3] = {keypad_buf[2], keypad_buf[3], '\0'};
                feed_hour   = atoi(hh);
                feed_minute = atoi(mm);

                // validate range
                if (feed_hour > 23) feed_hour = 23;
                if (feed_minute > 59) feed_minute = 59;

                // get current time and calculate seconds until target
                struct timeval tv;
                gettimeofday(&tv, NULL);
                struct tm *now = localtime(&tv.tv_sec);

                int current_seconds = now->tm_hour * 3600 + now->tm_min * 60 + now->tm_sec;
                int target_seconds  = feed_hour * 3600 + feed_minute * 60;

                // if target is earlier in the day than now, schedule for tomorrow
                if (target_seconds <= current_seconds) {
                    target_seconds += 24 * 3600;
                }

                seconds_until_feed = target_seconds - current_seconds;
                printf("Feed scheduled for %02d:%02d. Countdown: %d seconds.\n",
                       feed_hour, feed_minute, seconds_until_feed);
                vTaskDelay(300 / portTICK_PERIOD_MS);   // brief delay to avoid button bounce
                executed = 3;
            }
        }

        // executed == 3: countdown to next feeding (non-blocking, 100 cycles of 10ms = 1 second)
        if (executed == 3) {
            static int cycle_count = 0;
            cycle_count++;
            if (cycle_count >= 100) {
                cycle_count = 0;
                if (seconds_until_feed > 0) {
                    seconds_until_feed--;
                }
                if (seconds_until_feed <= 0) {
                    // check if enough slices before dispensing
                    int slices_remaining = REFILL_THRESHOLD - slice_count;
                    if (slices_remaining < slices) {
                        executed = 6;   // not enough — show warning first
                    } else {
                        executed = 4;   // enough slices — dispense normally
                    }
                }
            }
        }

        // executed == 6: low food warning — not enough slices for the selected portion
        //               SELECT to dispense what's left anyway, REFILL to cancel and refill first
        if (executed == 6) {
            if (select_portion) {
                // user accepts partial portion — proceed to dispense
                vTaskDelay(300 / portTICK_PERIOD_MS);
                executed = 4;
            }
            if (refill) {
                // user wants to refill first — go straight to refill alert
                vTaskDelay(300 / portTICK_PERIOD_MS);
                return_to_home();
                gpio_set_level(REFILL_LED, 1);
                executed = 5;
            }
        }

        // executed == 4: dispense food
        if (executed == 4) {
            int slices_remaining = REFILL_THRESHOLD - slice_count;  // how many slices are left

            if (slices_remaining <= 0) {
                // no slices left at all — skip dispensing, go straight to refill alert
                printf("No slices remaining. Refill needed!\n");
                return_to_home();
                gpio_set_level(REFILL_LED, 1);
                executed = 5;

            } else if (slices_remaining < slices) {
                // not enough slices for the full portion — dispense what is left
                printf("Only %d slice(s) left, dispensing partial portion.\n", slices_remaining);
                for (int i = 0; i < slices_remaining; i++) {
                    dispense_one_slice();
                    slice_count++;
                }
                printf("Partial portion dispensed. Refill needed!\n");
                return_to_home();
                gpio_set_level(REFILL_LED, 1);
                executed = 5;

            } else {
                // enough slices for the full portion — dispense normally
                printf("Dispensing %d slice(s)...\n", slices);
                for (int i = 0; i < slices; i++) {
                    dispense_one_slice();
                    slice_count++;
                }
                printf("Dispensed. Total slices since refill: %d\n", slice_count);

                if (slice_count >= REFILL_THRESHOLD) {
                    // all 7 slices used — return to home and trigger refill
                    return_to_home();
                    gpio_set_level(REFILL_LED, 1);
                    printf("Refill needed!\n");
                    executed = 5;
                } else {
                    // schedule next feeding for same time tomorrow
                    seconds_until_feed = 24 * 3600;
                    printf("Next feed at %02d:%02d tomorrow. Countdown: %d seconds.\n",
                           feed_hour, feed_minute, seconds_until_feed);
                    executed = 3;
                }
            }
        }

        // executed == 5: refill alert — wait for refill button
        if (executed == 5) {
            if (refill) {
                slice_count = 0;
                gpio_set_level(REFILL_LED, 0);
                printf("Refill acknowledged. Resuming schedule.\n");
                vTaskDelay(300 / portTICK_PERIOD_MS);
                seconds_until_feed = 24 * 3600;  // resume — next feed same time tomorrow
                executed = 3;
            }
        }
        vTaskDelay(DELAY_MS/portTICK_PERIOD_MS);
    }
}
        

