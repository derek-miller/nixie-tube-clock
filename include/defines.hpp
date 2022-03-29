#ifndef NIXIE_TUBE_CLOCK_DEFINES_H
#define NIXIE_TUBE_CLOCK_DEFINES_H

enum class ClockState {
    OFF,
    DISPLAY_TIME,
    DISPLAY_TIME_AND_DATE,
    DEBUG,
};

enum class LedState {
    IDLE,
    FAIL,
    SUCCESS,
};

typedef struct {
    unsigned int sel0;
    unsigned int sel1;
    unsigned int sel2;
    unsigned int sel3;
} arduinixmux_t;

typedef struct {
    int UpperYear;
    int LowerYear;
    int UpperMonth;
    int LowerMonth;
    int UpperDay;
    int LowerDay;
    int UpperHour;
    int LowerHour;
    int UpperMin;
    int LowerMin;
    bool Colon;
    bool Dots;
    bool Initialized;
} nixieDisplay_t;

typedef struct {
    int red;
    int green;
    int blue;
} rgbValues_t;

#endif //NIXIE_TUBE_CLOCK_DEFINES_H
