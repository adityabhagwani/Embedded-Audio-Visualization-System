#ifndef SHARED_STATE_H
#define SHARED_STATE_H

struct SystemState {
    float band_energy[3];
    bool  beat_flag;
    uint8_t mode;
    uint8_t sensitivity;
    uint8_t brightness;
};

extern volatile SystemState g_state;

#endif
