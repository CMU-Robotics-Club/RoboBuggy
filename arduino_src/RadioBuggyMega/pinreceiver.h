#ifndef _PINRECEIVER_H_
#define _PINRECEIVER_H_

class PinReceiver {
    int receiver_pin_;
    int leftmost, rightmost, centermost;
    int pwm_time, pwm_thresh, big_pulse, short_pulse;
    int signal_time;
    int signal_diff_time;
    int rc_available;
    int up_switch_time;
    int down_switch_time;
    int rc_value;

    public:
        PinReceiver();
        void Begin(int pin, int int_num, void (*int_wrapper)() );
        void OnInterruptReceiver();
        bool Available();
        int GetAngle();
};

#endif

