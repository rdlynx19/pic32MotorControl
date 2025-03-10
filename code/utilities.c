#include "utilities.h"

volatile enum mode_t mode;

enum mode_t get_mode(){
    return mode;
}

void set_mode(enum mode_t md){
    mode = md;
}