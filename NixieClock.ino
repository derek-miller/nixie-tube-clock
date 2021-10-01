#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>

// https://github.com/PaulStoffregen/Time
#include <TimeLib.h>
// https://github.com/JChristensen/Timezone
#include <Timezone.h>
// https://github.com/arduino-libraries/NTPClient
#include <NTPClient.h>

#include <CustomTypes.h>


/*******************************************************************************
 * CONFIGURE
 ******************************************************************************/
/*
 * Ethernet
 */
byte mac[] = {0x90, 0xA2, 0xDA, 0x00, 0xF8, 0x1A};
EthernetUDP udp;
NTPClient timeClient(udp);


/*
 * RTC
 */
const uint8_t RTC_CHIP_SEL = 53;
const uint8_t RTC_PWR_PIN = 48;


/*
 * Button
 */
const uint8_t BUTTON_DIO = 2;
const uint8_t BUTTON_PWR_PIN = 4;
const uint8_t BUTTON_GND_PIN = 7;


/*
 * LED
 */
const uint8_t LED_GND_PIN = 8;
const uint8_t LED_RED_PIN = 3;
const uint8_t LED_GREEN_PIN = 6;
const uint8_t LED_BLUE_PIN = 5;


const uint8_t LED_STATE_IDLE = 0;
const uint8_t LED_STATE_FAIL = 1;
const uint8_t LED_STATE_SUCCESS = 2;
const uint8_t LED_BLINK_DEFAULT = 5;
const uint8_t LED_BLINK_INTERVAL = 100;

const uint8_t DISPLAY_STATE_OFF = 0;
const uint8_t DISPLAY_STATE_TIME = 1;
const uint8_t DISPLAY_STATE_TIME_AND_DATE = 2;
const uint8_t DISPLAY_STATE_DEBUG = 3;

const uint8_t IN_18_ON_TIME_DELAY = 2;
const uint8_t IN_12_ON_TIME_DELAY = 1;
const uint8_t INS_1_ON_TIME_DELAY = 2;

/*
 * Arduinix
 */
uint8_t DATE_SEL_PINS_A[] = {23, 25, 27, 29};
uint8_t DATE_SEL_PINS_B[] = {31, 33, 35, 37};
uint8_t TIME_SEL_PINS_A[] = {22, 24, 26, 28};
uint8_t TIME_SEL_PINS_B[] = {30, 32, 34, 36};
uint8_t ANODE_DATE_SEL_PINS[] = {45, 43, 41, 39};
uint8_t ANODE_TIME_SEL_PINS[] = {44, 42, 40, 38};


/*******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************/
void printTimeElements(tmElements_t &tm) {
    Serial.print(tm.Year + 1970);
    Serial.print("-");
    Serial.print(tm.Month);
    Serial.print("-");
    Serial.print(tm.Day);
    Serial.print("T");
    Serial.print(tm.Hour);
    Serial.print(":");
    Serial.print(tm.Minute);
    Serial.print(":");
    Serial.println(tm.Second);
}

void printNixieDisplay(nixieDisplay_t &digits) {
    Serial.print(digits.UpperYear);
    Serial.print(digits.LowerYear);
    Serial.print("-");
    Serial.print(digits.UpperMonth);
    Serial.print(digits.LowerMonth);
    Serial.print("-");
    Serial.print(digits.UpperDay);
    Serial.print(digits.LowerDay);
    Serial.print("T");
    Serial.print(digits.UpperHour);
    Serial.print(digits.LowerHour);
    Serial.print(":");
    Serial.print(digits.UpperMin);
    Serial.println(digits.LowerMin);
}


/*
 * LED Helper Functions
 */
void LEDOn(rgbValues_t &rgb) {
    analogWrite(LED_RED_PIN, rgb.Red);
    analogWrite(LED_GREEN_PIN, rgb.Green);
    analogWrite(LED_BLUE_PIN, rgb.Blue);
}

void LEDOff() {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
}


int _ledState = LOW;
unsigned long _previousMillis = 0;

int LEDBlink(int blinks, rgbValues_t &rgb) {
    unsigned long currentMillis = millis();
    if (currentMillis - _previousMillis > LED_BLINK_INTERVAL) {
        _previousMillis = currentMillis;
        if (_ledState == LOW) {
            _ledState = HIGH;
            LEDOn(rgb);
            blinks--;
        } else {
            _ledState = LOW;
            LEDOff();
        }
    }
    return blinks;
}

rgbValues_t getLEDColor(int displayState) {
    rgbValues_t rgb;
    switch (displayState) {
        default:
        case (DISPLAY_STATE_OFF):
            rgb.Red = 5;
            rgb.Green = 0;
            rgb.Blue = 0;
            break;
        case (DISPLAY_STATE_TIME):
            rgb.Red = 0;
            rgb.Green = 0;
            rgb.Blue = 5;
            break;
        case (DISPLAY_STATE_TIME_AND_DATE):
            rgb.Red = 0;
            rgb.Green = 5;
            rgb.Blue = 0;
            break;
        case (DISPLAY_STATE_DEBUG):  // Rolling Digits
            rgb.Red = 5;
            rgb.Green = 5;
            rgb.Blue = 5;
            break;
    }
    return rgb;
}


/*
 * Nixie Helper Functions
 */
uint8_t rollDigit(uint8_t digit) {
    if (digit == 0) return 9;
    return digit - 1;
}

long _previousColonOnSec = -1;
unsigned long _previousColonOnMillis = 0;

void tmElementsToNixieDisplay(tmElements_t &tm, nixieDisplay_t &digits) {
    uint8_t smallYear = (tm.Year + 1970) % 100;
    digits.LowerYear = smallYear % 10;
    digits.UpperYear = smallYear - digits.LowerYear;
    if (digits.UpperYear >= 10) digits.UpperYear = digits.UpperYear / 10;

    digits.LowerMonth = tm.Month % 10;
    digits.UpperMonth = tm.Month - digits.LowerMonth;
    if (digits.UpperMonth >= 10) digits.UpperMonth = digits.UpperMonth / 10;

    digits.LowerDay = tm.Day % 10;
    digits.UpperDay = tm.Day - digits.LowerDay;
    if (digits.UpperDay >= 10) digits.UpperDay = digits.UpperDay / 10;

    uint8_t hour = tm.Hour > 12 ? tm.Hour - 12 : tm.Hour;
    digits.LowerHour = hour % 10;
    digits.UpperHour = hour - digits.LowerHour;
    if (digits.UpperHour >= 10) digits.UpperHour = digits.UpperHour / 10;

    digits.LowerMin = tm.Minute % 10;
    digits.UpperMin = tm.Minute - digits.LowerMin;
    if (digits.UpperMin >= 10) digits.UpperMin = digits.UpperMin / 10;

    digits.Dots = true;

    unsigned long currentMillis = millis();
    if (_previousColonOnSec != tm.Second) {
        _previousColonOnMillis = currentMillis;
        _previousColonOnSec = tm.Second;
        digits.Colon = true;
    } else if ((currentMillis - _previousColonOnMillis) < 500) {
        digits.Colon = true;
    } else {
        digits.Colon = false;
    }

    digits.Initialized = true;
}

void displayNixieTubeDigitPair(int anodeIndex, uint8_t *anodeSelPins,
                               int num1, uint8_t *num1SelPins,
                               int num2, uint8_t *num2SelPins, int onTimeDelay) {
    bool active = false;
    if (num1 >= 0) {
        // Write to select pins for mux 1
        digitalWrite(num1SelPins[0], (uint8_t) bitRead(num1, 0));
        digitalWrite(num1SelPins[1], (uint8_t) bitRead(num1, 1));
        digitalWrite(num1SelPins[2], (uint8_t) bitRead(num1, 2));
        digitalWrite(num1SelPins[3], (uint8_t) bitRead(num1, 3));
        active = true;
    } else {
        digitalWrite(num1SelPins[0], 1);
        digitalWrite(num1SelPins[1], 1);
        digitalWrite(num1SelPins[2], 1);
        digitalWrite(num1SelPins[3], 1);
    }
    if (num2 >= 0) {
        // Write to select pins for mux 2
        digitalWrite(num2SelPins[0], (uint8_t) bitRead(num2, 0));
        digitalWrite(num2SelPins[1], (uint8_t) bitRead(num2, 1));
        digitalWrite(num2SelPins[2], (uint8_t) bitRead(num2, 2));
        digitalWrite(num2SelPins[3], (uint8_t) bitRead(num2, 3));
        active = true;
    } else {
        digitalWrite(num2SelPins[0], 1);
        digitalWrite(num2SelPins[1], 1);
        digitalWrite(num2SelPins[2], 1);
        digitalWrite(num2SelPins[3], 1);
    }
    if (active) {
        digitalWrite(anodeSelPins[anodeIndex], HIGH);
    }
    delay(onTimeDelay);
    digitalWrite(anodeSelPins[anodeIndex], LOW);
}

void displayNixieTubeTimePair(int anode, int num1, int num2, int onTimeDelay) {
    displayNixieTubeDigitPair(anode, ANODE_TIME_SEL_PINS, num1, TIME_SEL_PINS_A, num2, TIME_SEL_PINS_B, onTimeDelay);
}

void displayNixieTubeDatePair(int anode, int num1, int num2, int onTimeDelay) {
    displayNixieTubeDigitPair(anode, ANODE_DATE_SEL_PINS, num1, DATE_SEL_PINS_A, num2, DATE_SEL_PINS_B, onTimeDelay);
}

void displayNixieTubeTime(nixieDisplay_t &digits) {
    displayNixieTubeTimePair(0, digits.UpperHour, digits.LowerHour, IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(1, digits.UpperMin, digits.LowerMin, IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(2, digits.Colon ? 0 : -1, -1, INS_1_ON_TIME_DELAY);
    // UNUSED displayNixieTubeTimePair(3, -1, -1, IN_18_ON_TIME_DELAY);
}

void turnOffNixieTubeTime() {
    displayNixieTubeTimePair(0, -1, -1, IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(1, -1, -1, IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(2, -1, -1 , INS_1_ON_TIME_DELAY);
    // UNUSED displayNixieTubeTimePair(3, -1, -1, IN_18_ON_TIME_DELAY);
}

void displayNixieTubeDate(nixieDisplay_t &digits) {
    displayNixieTubeDatePair(0, digits.UpperMonth, digits.LowerMonth, IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(1, digits.UpperDay, digits.LowerDay, IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(2, digits.UpperYear, digits.LowerYear, IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(3, digits.Dots ? 0 : -1, digits.Dots ? 0 : -1, INS_1_ON_TIME_DELAY);
}

void turnOffNixieTubeDate() {
    displayNixieTubeDatePair(0, -1, -1, IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(1, -1, -1, IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(2, -1, -1, IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(3, -1, -1, INS_1_ON_TIME_DELAY);
}

void DisplayNixieTubeDateTime(int displayState, nixieDisplay_t &digits) {
    switch (displayState) {
        case (DISPLAY_STATE_DEBUG):
            printNixieDisplay(digits);
        case (DISPLAY_STATE_TIME_AND_DATE):
            displayNixieTubeTime(digits);
            displayNixieTubeDate(digits);
            break;
        case (DISPLAY_STATE_TIME):
            displayNixieTubeTime(digits);
            turnOffNixieTubeDate();
            break;
        case (DISPLAY_STATE_OFF):
        default:
            turnOffNixieTubeTime();
            turnOffNixieTubeDate();
    }
}


/*
 * Real Time Clock Helper Functions
 */

void SetRTCDateTime(tmElements_t &tm) {
    SPI.setDataMode(SPI_MODE3);
    int TimeDate[7] = {tm.Second,
                       tm.Minute,
                       tm.Hour > 12 ? tm.Hour - 12 : tm.Hour,
                       0,
                       tm.Day,
                       tm.Month,
                       (tm.Year + 1970 - 2000)};
    for (int i = 0; i <= 6; i++) {
        if (i == 3)
            i++;
        int b = TimeDate[i] / 10;
        int a = TimeDate[i] - b * 10;
        if (i == 2) {
            if (b == 1)
                if (tm.Hour < 12)
                    b = B0000101;
                else
                    b = B0000111;
            else if (b == 0)
                if (tm.Hour < 12)
                    b = B0000100;
                else
                    b = B0000110;

        }
        TimeDate[i] = a + (b << 4);

        digitalWrite(RTC_CHIP_SEL, LOW);
        SPI.transfer((uint8_t) (i + 0x80));
        SPI.transfer((uint8_t) TimeDate[i]);
        digitalWrite(RTC_CHIP_SEL, HIGH);
    }
    SPI.setDataMode(SPI_MODE0);
}

void GetRTCDateTime(tmElements_t &tm) {
    SPI.setDataMode(SPI_MODE3);
    int dateTime[7]; //second,minute,hour,null,day,month,year
    int pm = 0;
    for (int i = 0; i <= 6; i++) {
        if (i == 3)
            i++;
        digitalWrite(RTC_CHIP_SEL, LOW);
        SPI.transfer((uint8_t) (i + 0x00));
        unsigned int n = SPI.transfer(0x00);
        digitalWrite(RTC_CHIP_SEL, HIGH);
        int a = n & B00001111;
        if (i == 2) {
            int b = (n & B01110000) >> 4;
            if (b == B00000100) {
                pm = 0;
                b = 0;
            } else if (b == B00000101) {
                pm = 0;
                b = 10;
            } else if (b == B00000110) {
                pm = 1;
                b = 0;
            } else {
                pm = 1;
                b = 10;
            }
            dateTime[i] = a + b;
        } else if (i == 4) {
            int b = (n & B00110000) >> 4;
            dateTime[i] = a + b * 10;
        } else if (i == 5) {
            int b = (n & B00010000) >> 4;
            dateTime[i] = a + b * 10;
        } else if (i == 6) {
            int b = (n & B11110000) >> 4;
            dateTime[i] = a + b * 10;
        } else {
            int b = (n & B01110000) >> 4;
            dateTime[i] = a + b * 10;
        }
    }
    SPI.setDataMode(SPI_MODE0);

    tm.Year = (uint8_t) (dateTime[6] + 2000 - 1970); // Year
    tm.Month = (uint8_t) dateTime[5]; // Month
    tm.Day = (uint8_t) dateTime[4]; // Day
    tm.Hour = (uint8_t) (pm == 1 ? dateTime[2] + 12 : dateTime[2]); // Hour
    tm.Minute = (uint8_t) dateTime[1]; // Min
    tm.Second = (uint8_t) dateTime[0]; // Sec
//    printTimeElements(tm);
}


/*
 * High Level Helper Functions
 */

TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300}; // UTC−05:00
TimeChangeRule usCST = {"CST", First, Sun, Nov, 2, -360}; //  UTC−06:00
Timezone usCT(usCDT, usCST);

bool UpdateRTCDateTime() {
    rgbValues_t rgb = {0, 0, 255};
    LEDOn(rgb);

    Serial.print("fetching ntp time...");
    if (!timeClient.update()) {
        Serial.println("failed");
        return false;
    }
    Serial.println("done");

    // Parse utc timestamp into time elements and print it
    time_t unixTime = (time_t) timeClient.getEpochTime();
    tmElements_t utcElements;
    breakTime(unixTime, utcElements);
    Serial.print("UTC: ");
    printTimeElements(utcElements);

    // Parse utc timestamp and convert it to local time
    time_t local = usCT.toLocal(unixTime);
    tmElements_t ptElements;
    breakTime(local, ptElements);
    Serial.print("CT: ");
    printTimeElements(ptElements);

    // Set RTC to updated time
    SetRTCDateTime(ptElements);
    return true;
}


nixieDisplay_t _currentDigits;


uint8_t counter = 0;
uint8_t numTimes = 0;
const uint8_t totalTimes = 40;

void UpdateNixieTubeDateTime(int displayState, tmElements_t &tm) {
    // Load the latest time from the RTC module
    nixieDisplay_t targetDigits;
    if (displayState == DISPLAY_STATE_DEBUG) {
        if (counter > 9) counter = 0;
        targetDigits.UpperYear = counter;
        targetDigits.LowerYear = counter;
        targetDigits.UpperMonth = counter;
        targetDigits.LowerMonth = counter;
        targetDigits.UpperDay = counter;
        targetDigits.LowerDay = counter;
        targetDigits.UpperHour = counter;
        targetDigits.LowerHour = counter;
        targetDigits.UpperMin = counter;
        targetDigits.LowerMin = counter;
        targetDigits.Dots = counter % 2 == 0;
        targetDigits.Colon = counter % 2 != 0;
        targetDigits.Initialized = true;
        if (numTimes == totalTimes) {
            numTimes = 0;
            counter++;
        } else {
            numTimes++;
        }
    } else {
        GetRTCDateTime(tm);
        tmElementsToNixieDisplay(tm, targetDigits);
    }
    if (!_currentDigits.Initialized) {
        memcpy(&_currentDigits, &targetDigits, sizeof(targetDigits));
    }

    if (_currentDigits.UpperYear != targetDigits.UpperYear) {
        _currentDigits.UpperYear = rollDigit(_currentDigits.UpperYear);
    }
    if (_currentDigits.LowerYear != targetDigits.LowerYear) {
        _currentDigits.LowerYear = rollDigit(_currentDigits.LowerYear);
    }
    if (_currentDigits.UpperMonth != targetDigits.UpperMonth) {
        _currentDigits.UpperMonth = rollDigit(_currentDigits.UpperMonth);
    }
    if (_currentDigits.LowerMonth != targetDigits.LowerMonth) {
        _currentDigits.LowerMonth = rollDigit(_currentDigits.LowerMonth);
    }
    if (_currentDigits.UpperDay != targetDigits.UpperDay) {
        _currentDigits.UpperDay = rollDigit(_currentDigits.UpperDay);
    }
    if (_currentDigits.LowerDay != targetDigits.LowerDay) {
        _currentDigits.LowerDay = rollDigit(_currentDigits.LowerDay);
    }
    if (_currentDigits.UpperHour != targetDigits.UpperHour) {
        _currentDigits.UpperHour = rollDigit(_currentDigits.UpperHour);
    }
    if (_currentDigits.LowerHour != targetDigits.LowerHour) {
        _currentDigits.LowerHour = rollDigit(_currentDigits.LowerHour);
    }
    if (_currentDigits.UpperMin != targetDigits.UpperMin) {
        _currentDigits.UpperMin = rollDigit(_currentDigits.UpperMin);
    }
    if (_currentDigits.LowerMin != targetDigits.LowerMin) {
        _currentDigits.LowerMin = rollDigit(_currentDigits.LowerMin);
    }
    if (_currentDigits.Dots != targetDigits.Dots) {
        _currentDigits.Dots = targetDigits.Dots;
    }
    if (_currentDigits.Colon != targetDigits.Colon) {
        _currentDigits.Colon = targetDigits.Colon;
    }

    DisplayNixieTubeDateTime(displayState, _currentDigits);
}


/******************************************************************************/


int _blinksRemaining = LED_BLINK_DEFAULT;
int _successState = LED_STATE_IDLE; // 0: success, 1: fail, 2: idle
int _displayState = DISPLAY_STATE_OFF;
tmElements_t _tm;

int nextDisplay(int displayState) {
    switch (displayState) {
        default:
        case (DISPLAY_STATE_OFF):
            return DISPLAY_STATE_TIME_AND_DATE;
        case (DISPLAY_STATE_TIME_AND_DATE):
            return DISPLAY_STATE_TIME;
        case (DISPLAY_STATE_TIME):
        case (DISPLAY_STATE_DEBUG):
            return DISPLAY_STATE_OFF;
    }
}

bool _lastButtonState = HIGH;
bool _currentButtonState;
unsigned long _lastDebounceTime = 0;
unsigned long _debounceDelay = 25;

bool isButtonPressed() {
    bool newButtonState = (bool) digitalRead(BUTTON_DIO);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (newButtonState != _lastButtonState) {
        // reset the debouncing timer
        _lastDebounceTime = millis();
    }

    if ((millis() - _lastDebounceTime) > _debounceDelay) {
        // whatever the reading is at, it's been there for longer
        // than the debounce delay, so take it as the actual current state:

        // if the button state has changed:
        if (newButtonState != _currentButtonState) {
            _currentButtonState = newButtonState;
        }
    }
    _lastButtonState = newButtonState;
    return _currentButtonState == LOW;
}

void setup() {
    Serial.begin(9600);

    //LED Setup
    pinMode(LED_GND_PIN, OUTPUT);
    digitalWrite(LED_GND_PIN, LOW);

    //RTC Setup
    pinMode(RTC_CHIP_SEL, OUTPUT);

    pinMode(RTC_PWR_PIN, OUTPUT);
    digitalWrite(RTC_PWR_PIN, HIGH);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    digitalWrite(RTC_CHIP_SEL, LOW);
    // Access Control Register
    SPI.transfer(0x8E);
    // Disable:  Osciallator, Battery SQ wave, Alarms
    SPI.transfer(0x60);
    // Enable:  Temperature Compensation
    digitalWrite(RTC_CHIP_SEL, HIGH);

    Ethernet.begin(mac);
    timeClient.begin();

    //Button Setup
    pinMode(BUTTON_PWR_PIN, OUTPUT);
    digitalWrite(BUTTON_PWR_PIN, HIGH);

    pinMode(BUTTON_GND_PIN, OUTPUT);
    digitalWrite(BUTTON_GND_PIN, LOW);


    //Arduinix Setup
    pinMode(DATE_SEL_PINS_A[0], OUTPUT);
    pinMode(DATE_SEL_PINS_A[1], OUTPUT);
    pinMode(DATE_SEL_PINS_A[2], OUTPUT);
    pinMode(DATE_SEL_PINS_A[3], OUTPUT);

    pinMode(DATE_SEL_PINS_B[0], OUTPUT);
    pinMode(DATE_SEL_PINS_B[1], OUTPUT);
    pinMode(DATE_SEL_PINS_B[2], OUTPUT);
    pinMode(DATE_SEL_PINS_B[3], OUTPUT);

    pinMode(TIME_SEL_PINS_A[0], OUTPUT);
    pinMode(TIME_SEL_PINS_A[1], OUTPUT);
    pinMode(TIME_SEL_PINS_A[2], OUTPUT);
    pinMode(TIME_SEL_PINS_A[3], OUTPUT);

    pinMode(TIME_SEL_PINS_B[0], OUTPUT);
    pinMode(TIME_SEL_PINS_B[1], OUTPUT);
    pinMode(TIME_SEL_PINS_B[2], OUTPUT);
    pinMode(TIME_SEL_PINS_B[3], OUTPUT);

    pinMode(ANODE_DATE_SEL_PINS[0], OUTPUT);
    pinMode(ANODE_DATE_SEL_PINS[1], OUTPUT);
    pinMode(ANODE_DATE_SEL_PINS[2], OUTPUT);
    pinMode(ANODE_DATE_SEL_PINS[3], OUTPUT);

    pinMode(ANODE_TIME_SEL_PINS[0], OUTPUT);
    pinMode(ANODE_TIME_SEL_PINS[1], OUTPUT);
    pinMode(ANODE_TIME_SEL_PINS[2], OUTPUT);
    pinMode(ANODE_TIME_SEL_PINS[3], OUTPUT);
}


void loop() {
    beginning:

    //Set LED based on status last time sync or current idle state
    if (_blinksRemaining == 0) {
        // We are done blinking, reset the number of blinks and set success state to idle
        _successState = LED_STATE_IDLE;
        _blinksRemaining = LED_BLINK_DEFAULT;
    } else if (_successState == LED_STATE_SUCCESS) {
        // Bright green; Successful time sync
        rgbValues_t rgb = {0, 255, 0};
        _blinksRemaining = LEDBlink(_blinksRemaining, rgb);
    } else if (_successState == LED_STATE_FAIL) {
        // Bright red; Failed time sync
        rgbValues_t rgb = {255, 0, 0};
        _blinksRemaining = LEDBlink(_blinksRemaining, rgb);
    } else {
        // Standby
        rgbValues_t rgb = getLEDColor(_displayState);
        LEDOn(rgb);
    }

    // Read button;
    if (isButtonPressed()) {
        // Button was pressed
        unsigned long pressed_time = millis();
        while (isButtonPressed()) {
            UpdateNixieTubeDateTime(_displayState, _tm);
            if ((millis() - pressed_time) > 1500) {
                // Sync time from ntp source
                _successState = UpdateRTCDateTime() ? LED_STATE_SUCCESS : LED_STATE_FAIL;

                // Wait for the button to be released
                while (isButtonPressed()) {
                    UpdateNixieTubeDateTime(_displayState, _tm);
                }

                // If the button was held for more than 5s go into debug state.
                if ((millis() - pressed_time) > 5000) {
                    _displayState = DISPLAY_STATE_DEBUG;
                }

                goto beginning;
            }
        }
        _displayState = nextDisplay(_displayState);
    }

    UpdateNixieTubeDateTime(_displayState, _tm);
}
