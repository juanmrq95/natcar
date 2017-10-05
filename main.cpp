#include "mbed.h"
#include "telemetry.h"
#include "MODSERIAL.h"

/* Pin setup */
DigitalOut clk1(PTD5);
DigitalOut si1(PTA13);
AnalogIn camdata1(PTB0);
// DigitalOut clk2(PTC9);
// DigitalOut si2(PTC8);
// AnalogIn camdata2(A1);
PwmOut servo_motor(PTD4);
PwmOut brushed_motor(PTE20);
Serial pc(USBTX, USBRX);

/* Tickers for interrupts */
// Ticker camera1_ticker;
// Ticker camera2_ticker;
Ticker servo_motor_ticker;
// Ticker brushed_motor_ticker;
// Ticker bluetooth_ticker;

/* Camera Variables */
const int NUM_PIXELS = 128;
int read_start_pixel = 10; // cuts off the first and last section of the camera data
int read_end_pixel = NUM_PIXELS - read_start_pixel;
float camera1data[NUM_PIXELS];
// float camera2data[NUM_PIXELS];
float t_SI = 0.00000002; //20ns
float t_hold = 0.000000005; //5ns
float t_w = 0.00000005; //50ns
float t_qt = 0.00002; //20us
float t_garbage = 0.000000005; //5ns
float max_pixel;
float curr_line_index;
float found_line_thresh = 0.1;
int curr_line_thickness = 0;
int found_line_thickness = 0;
int prev_line_thickness;
int end_of_line_pixel = 0;
float curr_pixel_val;
int max_line_thickness_thresh = 40;
int tunnel_vision_thresh = 40;
int start_tunnel_pixel;
int end_tunnel_pixel;
bool left_side_line_found = false;
bool right_side_line_found = false;
int avg_line_half_thickness = 18;
float crossing_found = 0.0;
int thickness_change_thresh = 15;

/* Servo Motor Controller */
float K_turn = 0.0004;
// float prev_K_turn = 0.0005;
float deviate_from_line_percent = 0;
float servo_pwm = 0.0015;
float servo_pwm_center = 0.0015;
float prev_line_index = 64;
float delta_servo_pwm;
int steer_center = 63.5;
float turn_left_thresh = -0.00040;
float turn_right_thresh = 0.00040;
int line_dist_away_thresh = 29;

/* Brushed Motor Controller */
float slow_turn_threshold;
float brushed_motor_pwm = 0.10;
/* Bluetooth Variables */
Timer bluetooth_timer;
MODSERIAL telemetry_serial(PTE22, PTE23); // (TX, RX)
int tele_baud_rate = 115200;

void control_servo()
{
    servo_motor.pulsewidth(servo_pwm);
}

void control_brushed_motor()
{
    brushed_motor.pulsewidth(brushed_motor_pwm);
}

int main()
{
    /* Camera Setup */
    si1 = 0;
    clk1 = 0;
    // float cam_update_freq = 0.001; //1ms

    /* Servo motor Setup */
    servo_motor.period(0.005f);
    servo_motor.pulsewidth_ms(servo_pwm_center);
    float servo_update_freq = 0.005; //5ms

    /* Brushed Motor Setup */
    brushed_motor.period(0.001f); // 1ms period
    brushed_motor.write(brushed_motor_pwm);
    // float brushed_update_freq = 0.005; //5ms

    /* Telemetry Setup */
    telemetry_serial.baud(tele_baud_rate); // This rate should match with how you start the plotter.
    telemetry::MbedHal telemetry_hal(telemetry_serial);
    telemetry::Telemetry telemetry_obj(telemetry_hal);
    
    // Data objects to be plotted //
    telemetry::Numeric<uint32_t> tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
    //telemetry::NumericArray<uint16_t, 128> tele_linescan(telemetry_obj, "linescan", "Linescan", "ADC", 0);
    // telemetry::Numeric<float> tele_motor_pwm(telemetry_obj, "motor", "Motor PWM", "%DC", 0);
    telemetry::Numeric<float> tele_line_max(telemetry_obj, "line", "Max Index", "Index", 0);
    telemetry::Numeric<float> tele_max_value(telemetry_obj, "max", "Max Value", "Max", 0);
    telemetry::Numeric<float> tele_line_thickness(telemetry_obj, "thickness", "Line Thickness", "Thickness", 0);
    telemetry::Numeric<float> tele_line_crossing_found(telemetry_obj, "crossing found", "Crossing Found", "Found", 0);
    // tele_max_value.set_limits(0.0, 0.4);
    
    // Transmit data definitions // 
    telemetry_obj.transmit_header();

    // Start the timer for bluetooth telemetry plotting //
    bluetooth_timer.start();

    /* Ticker Setup */
    servo_motor_ticker.attach(&control_servo, servo_update_freq);
    // camera1_ticker.attach(&get_camera1_data, cam_update_freq);
    // camera2_ticker.attach(&get_camera2_data, cam_update_freq);
    
    while (true) {
        // Real camera reading //
        si1 = 1;
        wait(t_SI);
        clk1 = 1;
        wait(t_hold);
        si1 = 0;
        wait(t_w - t_hold);
        camera1data[0] = camdata1.read();
        clk1 = 0;
        wait(t_w);
        for(int i = 1; i < NUM_PIXELS; i++) {
            clk1 = 1;
            wait(t_w);
            camera1data[i] = camdata1.read();
            clk1 = 0;
            wait(t_w);
        }
        wait(t_qt);
        
        // Find the line //
        max_pixel = -100;
        curr_line_index = 0;
        curr_line_thickness = 0;
        found_line_thickness = 0;
        end_of_line_pixel = NUM_PIXELS;
        left_side_line_found = false;
        right_side_line_found = false;
        // start_tunnel_pixel = prev_line_index - tunnel_vision_thresh;
        // end_tunnel_pixel = prev_line_index + tunnel_vision_thresh;
        start_tunnel_pixel = read_start_pixel;
        end_tunnel_pixel = read_end_pixel;
        crossing_found = 0.0;
        // if (start_tunnel_pixel < 0) {
            // start_tunnel_pixel = 0;
        // }
        // if (end_tunnel_pixel > NUM_PIXELS) {
            // end_tunnel_pixel = NUM_PIXELS;
        // }

        for (int i = start_tunnel_pixel; i < end_tunnel_pixel; i++) {
        // for (int i = 0; i < NUM_PIXELS; i++) {
            //tele_linescan[i] = camera1data[i]; // for telemetry (this messes up the servo timing)
            curr_pixel_val = camera1data[i];

            // tunnel vision //
            if (curr_pixel_val > found_line_thresh) {
                curr_line_thickness = curr_line_thickness + 1;
                if (i < prev_line_index - avg_line_half_thickness) {
                    left_side_line_found = true;
                }
                if (i > prev_line_index + avg_line_half_thickness) {
                    right_side_line_found = true;
                }
            } else {
                if (curr_line_thickness > found_line_thickness) {
                    found_line_thickness = curr_line_thickness;
                    end_of_line_pixel = i - 1;
                }
                curr_line_thickness = 0;
            }

            // line thickness //
            // if (curr_pixel_val > found_line_thresh) {
            //     curr_line_thickness = curr_line_thickness + 1;
            // } else {
            //     if (curr_line_thickness > found_line_thickness) {
            //         found_line_thickness = curr_line_thickness;
            //         end_of_line_pixel = i - 1;
            //     }
            //     curr_line_thickness = 0;
            // }

            // argmax //
            if (camera1data[i] > max_pixel) {
                max_pixel = camera1data[i];
            //     curr_line_index = i;
            }
        }
        // Check if line was found //
        // argmax //


        // line thickness // 
        if (found_line_thickness != 0) {
            if (left_side_line_found && right_side_line_found) {
                curr_line_index = prev_line_index;
                crossing_found = 1.0;
            } else {
                curr_line_index = end_of_line_pixel - found_line_thickness / 2;
            }
        } else {
            curr_line_index = prev_line_index;
        }
        prev_line_thickness = found_line_thickness;

        // Check if the bright spot was too far away from the previous reading //
        // if (abs(curr_line_index - prev_line_index) > line_dist_away_thresh) {
        //     K_turn = K_turn + 0.00025;
            // curr_line_index = prev_line_index;
        // }
        prev_line_index = curr_line_index;

        // Update servo pwm //
        deviate_from_line_percent = (curr_line_index-steer_center)/steer_center;
        delta_servo_pwm = deviate_from_line_percent*K_turn;
        if (delta_servo_pwm <= turn_left_thresh) {
            delta_servo_pwm = turn_left_thresh;
        }
        if (delta_servo_pwm >= turn_right_thresh) {
            delta_servo_pwm = turn_right_thresh;
        }
        servo_pwm = servo_pwm_center - delta_servo_pwm;
        // K_turn = prev_K_turn;

        // Send telemetry //
        tele_time_ms = bluetooth_timer.read_ms();
        // tele_motor_pwm = servo_pwm;
        tele_line_max = curr_line_index;
        tele_max_value = max_pixel;
        tele_line_thickness = found_line_thickness;
        tele_line_crossing_found = crossing_found;
        telemetry_obj.do_io();
    }
}