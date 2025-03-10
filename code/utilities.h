#ifndef UTILITIES__H__
#define UTILITIES__H__

enum mode_t {
    IDLE, 
    PWM, 
    ITEST, 
    HOLD, 
    TRACK};

void set_mode(enum mode_t md);
enum mode_t get_mode(void);

#endif