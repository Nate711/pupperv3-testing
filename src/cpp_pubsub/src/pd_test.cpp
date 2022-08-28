#include "motor_interface.hpp"
#include "prof_utils.hpp"
#include <atomic>
#include <memory>
#include <algorithm>

#define K_SERVOS_PER_CHANNEL 1

atomic<bool> quit(false); // signal flag

void sigint_handler(sig_atomic_t s)
{
    quit.store(true);
    printf("Caught signal %d\n", s);
}

int main()
{
    signal(SIGINT, sigint_handler);

    // TODO: Replace with make_unique after resolving argument list error
    MotorInterface motor_interface = MotorInterface<K_SERVOS_PER_CHANNEL>({CANChannel::CAN0}, /*bitrate=*/1000000);
    motor_interface.initialize_canbuses();
    motor_interface.initialize_motors(); // not needed if you just want angles
    motor_interface.start_read_threads();
    cout << "Initialized canbuses, motors, threads" << endl;

    auto loop_start = time_now();

    int rotations = 0;
    int16_t previous_encoder_counts = 0;
    float multi_loop_angle = 0;

    int loop_count = 0;
    while (!quit.load())
    {
        // Print time since start of program
        auto loop_now = time_now();
        auto since_start = chrono::duration_cast<chrono::microseconds>(loop_now - loop_start);
        if (loop_count % 2 == 0)
        {
            cout << "\nSince start (us): " << since_start.count() << "\t";
        }

        auto latest_data = motor_interface.latest_data();
        auto motor_data = latest_data.at(0).at(0);

        if (motor_data.encoder_counts - previous_encoder_counts > kEncoderCountsPerRot / 2)
        {
            rotations--;
        }
        if (motor_data.encoder_counts - previous_encoder_counts < -kEncoderCountsPerRot / 2)
        {
            rotations++;
        }
        previous_encoder_counts = motor_data.encoder_counts;

        multi_loop_angle = (float)(rotations * kEncoderCountsPerRot + motor_data.encoder_counts) / kEncoderCountsPerRot * 360.0;

        float velocity_target = 360.0;
        float velocity_error = velocity_target - motor_data.velocity;
        float current_command = velocity_error * 0.01;
        current_command = std::clamp(current_command, /*low=*/-2.0f, /*high=*/2.0f);
        // Print time took to send angle request
        // motor_interface.command_current(/*channel=*/CANChannel::CAN0, /*motor_id=*/1, /*current=*/current_command);


        float position_target = 1000.0;
        float position_error = position_target - multi_loop_angle;
        float velocity_command = position_error * 10;
        velocity_command = std::clamp(velocity_command, -3600.0f, 3600.0f);
        motor_interface.command_velocity(CANChannel::CAN0, 1, velocity_command);
        if (loop_count % 2 == 0)
        {
            // cout << motor_data.temp << "\t" << motor_data.current << "\t" << motor_data.velocity << "\t" << motor_data.encoder_counts << "\t" << rotations << "\t" << multi_loop_angle << "\t";
            cout << motor_data.velocity << "\t" << motor_data.encoder_counts << "\t" << rotations << "\t" << multi_loop_angle << "\t";
        }
        // motor_interface.request_multi_angle(CANChannel::CAN0, 1);
        // motor_interface.command_velocity(CANChannel::CAN0, 1, 0.0);
        usleep(2000); // sending to 3 motors takes 2500us

        // torque control: jiggles with kd=0.01 and usleep(5000)
        // torque control: jiggles with kd=0.01 and usleep(2000)
        // speed-based pos control: good with kp = 10, usleep(1000 or 2000), max speed 3600
        // have to plug in motor before running program
        loop_count++;
    }
    return 0;
}
