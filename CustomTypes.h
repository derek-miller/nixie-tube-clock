typedef struct {
    uint8_t UpperYear;
    uint8_t LowerYear;
    uint8_t UpperMonth;
    uint8_t LowerMonth;
    uint8_t UpperDay;
    uint8_t LowerDay;
    uint8_t UpperHour;
    uint8_t LowerHour;
    uint8_t UpperMin;
    uint8_t LowerMin;
    bool Colon;
    bool Dots;
    bool Initialized;
} nixieDisplay_t;

typedef struct {
    uint8_t Red;
    uint8_t Green;
    uint8_t Blue;
} rgbValues_t;
