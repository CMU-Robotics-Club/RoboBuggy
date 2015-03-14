#ifndef _PINRECEIVER_H_
#define _PINRECEIVER_H_

class PinReceiver {
    unsigned int receiver_pin_;
    int k_pwm_time_;
    int k_pwm_thresh_;
    int k_big_pulse_;
    int k_short_pulse_;
    volatile unsigned int rc_available_;
    volatile unsigned long up_switch_time_;
    volatile unsigned long down_switch_time_;
    volatile unsigned long rc_value_;

    public:
        PinReceiver();
        void Begin(int pin_num, int int_num, void (*int_wrapper)() );
        void OnInterruptReceiver();
        bool Available();
        int GetAngle();
        void PrintDebugInfo();
};

#endif

