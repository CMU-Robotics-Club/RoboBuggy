#ifndef _NEWRECEIVER_H_
#define _NEWRECEIVER_H_

volatile char rc_connected;

class PinReceiver {
    int receiver_pin, receiver_int;
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
        void Begin(int receiverPin, int receiverInt);
        void OnInterruptReceiver();
};

#endif

