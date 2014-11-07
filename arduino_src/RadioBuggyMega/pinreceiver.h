#ifndef _PINRECEIVER_H_
#define _PINRECEIVER_H_

class PinReceiver {
    unsigned int receiver_pin_;
    int leftmost, rightmost, centermost;
    int pwm_time, pwm_thresh, big_pulse, short_pulse;
    volatile unsigned int rc_available;
    volatile unsigned long up_switch_time;
    volatile unsigned long down_switch_time;
    volatile unsigned long rc_value;

    public:
        PinReceiver();
        void Begin(int pin, int int_num, void (*int_wrapper)() );
        void OnInterruptReceiver();
        bool Available();
        int GetAngle();
        void PrintDebugInfo();
};

#endif

