#define NO_PORTB_PINCHANGES
//#define NO_PORTC_PINCHANGES
#define NO_PORTD_PINCHANGES
#include <PinChangeInt.h>
#include <TimerOne.h>

#define rcrx_num_channels 7 // 1-indexed, 0 isn't used except as a pad
#define rcrx_pin 14 //analog 0


// TODO: User a timer interrupt to check how long it's been since we saw a pulse, and if it's been a long time we should flag rcrx inactive


volatile bool rcrx_data_flag = false;
volatile uint8_t rcrx_cur_channel = 0; // Channel 1 from the Tx/Rx corresponds to 1, 0 is the buffer pulse between pulse trains
volatile uint16_t rcrx_channel_data[rcrx_num_channels] = {0};
volatile uint16_t since = 0;
volatile uint16_t pulse_train_count = 0;
void rcrx_isr() {
    /* We're measuring the length of the PWM pulses in a 6 channel pulse train. (There is a long seventh "pulse" as the pin is held high between trains")
     * ------    --    ----    ----    -    ------    ----    ----------------
     * (7)   |  |1 |  |2   |  |3   |  |4|  |5     |  |6   |  |(7)
     *       |  |  |  |    |  |    |  | |  |      |  |    |  |
     *       |  |  |  |    |  |    |  | |  |      |  |    |  |
     *       |  |  |  |    |  |    |  | |  |      |  |    |  |
     *        --    --      --      --   --        --      --
     * We restart the timer every time we detect a rising edge,  and record the timer value every time we detect a falling edge.
     *
     * Also, we set a "data available" flag when we finish the pulse train, so that the main loop can process the data outside of the ISR
     */
    since = Timer1.read();

    if (PCintPort::pinState == HIGH) {
        // RISING EDGE DETECTED
        Timer1.restart();

    } else {
        // FALLING EDGE DETECTED
        if (since > 3000) {
            // This is the long pulse between trains, reset the channel counter
            rcrx_cur_channel = 0;
        } else {
            rcrx_cur_channel++;
        }

        rcrx_channel_data[rcrx_cur_channel] = since;

        if (rcrx_cur_channel == rcrx_num_channels-1) {
            // this is the last pulse in the train
            rcrx_data_flag = true;
            pulse_train_count++;
        }
    }
}

#define pwm_l 6
#define pwm_r 11
#define dir_l 9
#define dir_r 10

uint16_t rcrx_channel_max[7];
uint16_t rcrx_channel_min[7];
void setup() {
    Serial.begin(115200);
    Serial.println("BEGIN-------------------------------------");
    pinMode(rcrx_pin, INPUT); digitalWrite(rcrx_pin, HIGH);
    PCintPort::attachInterrupt(rcrx_pin, &rcrx_isr, CHANGE);
    Timer1.initialize(19000);

    pinMode(pwm_l, OUTPUT);
    pinMode(pwm_r, OUTPUT);
    pinMode(dir_l, OUTPUT);
    pinMode(dir_r, OUTPUT);
    for (int channel=0; channel<rcrx_num_channels; channel++) {
        rcrx_channel_min[channel] = 800;
        rcrx_channel_max[channel] = 1200;
    }
    rcrx_channel_min[6] = 2000;
}

uint8_t rcrx_signal[rcrx_num_channels] = {0};
uint16_t pulse_trains_processed = 0;
int forward_speed = 0;
int left_speed = 0;
int right_speed = 0;
bool left_direction = HIGH;
bool right_direction = HIGH;
void loop() {
    if (rcrx_data_flag && pulse_train_count > 3) {
        // The rcrx isr has finished a pulsetrain and set the data available flag
        rcrx_data_flag = false;

        uint8_t oldSREG = SREG; // save status register
        cli(); // prevent interrupts while we read volatile data

        // dynamic bound calibration and signal mapping
        for (int channel=0; channel<rcrx_num_channels; channel++) {
            // Expand bounds as needed
            if (rcrx_channel_data[channel] < rcrx_channel_min[channel]) {
                rcrx_channel_min[channel] = rcrx_channel_data[channel];
            }
            if (rcrx_channel_data[channel] > rcrx_channel_max[channel]) {
                rcrx_channel_max[channel] = rcrx_channel_data[channel];
            }

            // Map the timing data to a signal value using calibration bounds
            rcrx_signal[channel] = map(rcrx_channel_data[channel], rcrx_channel_min[channel], rcrx_channel_max[channel], 0, 255);
        }

        pulse_train_count--;
        Serial.print("pulse trains missed: ");
        Serial.print(pulse_train_count);
        Serial.print("   ");

        SREG = oldSREG; // restore the status register (and allow interrupts)

/* print signal values for each channel
        for (int channel=0; channel<rcrx_num_channels; channel++) {
            Serial.print(channel);
            Serial.print(":");
            Serial.print(rcrx_signal[channel], DEC);
            Serial.print(" ");
        }
        */

        Serial.println();
    /***** DRIVE LOGIC
     * or what there is of it... */
     left_speed = rcrx_signal[3] - 127; // channel 3 is the throttle, or left vertical axis
     right_speed = rcrx_signal[2] - 127; // channel 1 is the elevator, right vertical axis

     left_direction = HIGH; // forward
     if (left_speed < 0) { left_direction = LOW; } // or backward
     right_direction = HIGH; // forward
     if (right_speed < 0) { right_direction = LOW; } // or backward

     left_speed = abs(left_speed * 2);
     right_speed = abs(right_speed * 2);

     Serial.print("L ");
     Serial.print(left_direction, DEC);
     Serial.print(" ");
     Serial.print(left_speed, DEC);
     Serial.print(" ");
     Serial.print(rcrx_signal[3], DEC);
     Serial.print("  -  R ");
     Serial.print(right_direction, DEC);
     Serial.print(" ");
     Serial.print(right_speed, DEC);
     Serial.print(" ");
     Serial.print(rcrx_signal[2], DEC);
     Serial.println();

     digitalWrite(dir_l, left_direction);
     analogWrite(pwm_l, left_speed);
     digitalWrite(dir_r, right_direction);
     analogWrite(pwm_r, right_speed);
     /*
      ***** END DRIVE LOGIC */
    } // end rcrx data processing

}
