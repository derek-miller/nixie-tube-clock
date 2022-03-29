#include <Arduino.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>

#define USE_ETHERNET_WRAPPER  true
#define USE_ETHERNET          true

#include <EthernetWebServer.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <ds3234.h>

#include <defines.hpp>


namespace Constants {
    /*
     * LED
     */
    const static unsigned int LED_BLINK_COUNT = 5;
    const static unsigned long LED_BLINK_INTERVAL = 100;

    /*
     * Nixie tube
     */
    const static unsigned long IN_18_ON_TIME_DELAY = 2;
    const static unsigned long IN_12_ON_TIME_DELAY = 1;
    const static unsigned long INS_1_ON_TIME_DELAY = 2;

    /*
     * Ethernet
     */
    static byte MAC_ADDRESS[] = {0x90, 0xA2, 0xDA, 0x00, 0xF8, 0x1A};
    const static unsigned int SERVER_HTTP_PORT = 80;

    const static unsigned int CLOCK_STATE_ADDR = 0;
    const static ClockState DEFAULT_STATE = ClockState::DISPLAY_TIME_AND_DATE;
}

namespace Pins {
    /*
     * Real-time clock module
     */
    const static unsigned int RTC_SEL = 53;
    const static unsigned int RTC_PWR = 48;

    /*
     * Button
     */
    const static unsigned int BUTTON_DIO = 2;
    const static unsigned int BUTTON_PWR = 4;
    const static unsigned int BUTTON_GND = 7;

    /*
     * Status LED
     */
    const static unsigned int LED_GND = 8;
    const static unsigned int LED_RED = 3;
    const static unsigned int LED_GREEN = 6;
    const static unsigned int LED_BLUE = 5;

    /*
     * Arduinix
     */
    const static arduinixmux_t CATHODE_DATE_SEL_A = arduinixmux_t{23, 25, 27, 29};
    const static arduinixmux_t CATHODE_DATE_SEL_B = arduinixmux_t{31, 33, 35, 37};
    const static arduinixmux_t ANODE_DATE_SEL = arduinixmux_t{45, 43, 41, 39};

    const static arduinixmux_t CATHODE_TIME_SEL_A = arduinixmux_t{22, 24, 26, 28};
    const static arduinixmux_t CATHODE_TIME_SEL_B = arduinixmux_t{30, 32, 34, 36};
    const static arduinixmux_t ANODE_TIME_SEL = arduinixmux_t{44, 42, 40, 38};
}

/*******************************************************************************
 * CONFIGURE
 ******************************************************************************/

/*
 * Ethernet
 */
EthernetUDP udp;
NTPClient timeClient(udp);
EthernetWebServer server(Constants::SERVER_HTTP_PORT);

/*******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************/
void printTimeElements(tmElements_t &tm) {
    char s[20];
    snprintf(
            s,
            sizeof(s),
            "%04d-%02d-%02dT%02d:%02d:%02d", tmYearToCalendar(tm.Year + 1970),
            tm.Month,
            tm.Day,
            tm.Hour,
            tm.Minute,
            tm.Second
    );
    Serial.println(s);
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
    analogWrite(Pins::LED_RED, rgb.red);
    analogWrite(Pins::LED_GREEN, rgb.green);
    analogWrite(Pins::LED_BLUE, rgb.blue);
}

void LEDOff() {
    analogWrite(Pins::LED_RED, 0);
    analogWrite(Pins::LED_GREEN, 0);
    analogWrite(Pins::LED_BLUE, 0);
}


int _ledState = LOW;
unsigned long _previousLedBlinkMillis = 0;

int LEDBlink(int blinks, rgbValues_t &rgb) {
    unsigned long currentMillis = millis();
    if (currentMillis - _previousLedBlinkMillis > Constants::LED_BLINK_INTERVAL) {
        _previousLedBlinkMillis = currentMillis;
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

rgbValues_t getLEDColor(ClockState clockState) {
    rgbValues_t rgb;
    switch (clockState) {
        case (ClockState::DISPLAY_TIME):
            rgb.red = 0;
            rgb.green = 0;
            rgb.blue = 5;
            break;
        case (ClockState::DISPLAY_TIME_AND_DATE):
            rgb.red = 0;
            rgb.green = 5;
            rgb.blue = 0;
            break;
        case (ClockState::DEBUG):  // Rolling Digits
            rgb.red = 5;
            rgb.green = 5;
            rgb.blue = 5;
            break;
        case (ClockState::OFF):
        default:
            rgb.red = 5;
            rgb.green = 0;
            rgb.blue = 0;
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

void displayNixieTubeDigitPair(unsigned int anodePin,
                               int num1, arduinixmux_t num1SelPins,
                               int num2, arduinixmux_t num2SelPins, unsigned long onTimeDelay) {
    bool active = false;
    if (num1 >= 0) {
        // Write to select pins for mux 1
        digitalWrite(num1SelPins.sel0, (uint8_t) bitRead(num1, 0));
        digitalWrite(num1SelPins.sel1, (uint8_t) bitRead(num1, 1));
        digitalWrite(num1SelPins.sel2, (uint8_t) bitRead(num1, 2));
        digitalWrite(num1SelPins.sel3, (uint8_t) bitRead(num1, 3));
        active = true;
    } else {
        digitalWrite(num1SelPins.sel0, 1);
        digitalWrite(num1SelPins.sel1, 1);
        digitalWrite(num1SelPins.sel2, 1);
        digitalWrite(num1SelPins.sel3, 1);
    }
    if (num2 >= 0) {
        // Write to select pins for mux 2
        digitalWrite(num2SelPins.sel0, (uint8_t) bitRead(num2, 0));
        digitalWrite(num2SelPins.sel1, (uint8_t) bitRead(num2, 1));
        digitalWrite(num2SelPins.sel2, (uint8_t) bitRead(num2, 2));
        digitalWrite(num2SelPins.sel3, (uint8_t) bitRead(num2, 3));
        active = true;
    } else {
        digitalWrite(num2SelPins.sel0, 1);
        digitalWrite(num2SelPins.sel1, 1);
        digitalWrite(num2SelPins.sel2, 1);
        digitalWrite(num2SelPins.sel3, 1);
    }
    if (active) {
        digitalWrite(anodePin, HIGH);
    }
    delay(onTimeDelay);
    digitalWrite(anodePin, LOW);
}

void displayNixieTubeTimePair(unsigned int anodePin, int num1, int num2, unsigned long onTimeDelay) {
    displayNixieTubeDigitPair(anodePin, num1, Pins::CATHODE_TIME_SEL_A, num2, Pins::CATHODE_TIME_SEL_B, onTimeDelay);
}

void displayNixieTubeDatePair(unsigned int anodePin, int num1, int num2, unsigned long onTimeDelay) {
    displayNixieTubeDigitPair(anodePin, num1, Pins::CATHODE_DATE_SEL_A, num2, Pins::CATHODE_DATE_SEL_B, onTimeDelay);
}

void displayNixieTubeTime(nixieDisplay_t digits) {
    displayNixieTubeTimePair(Pins::ANODE_TIME_SEL.sel0, digits.UpperHour, digits.LowerHour,
                             Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(Pins::ANODE_TIME_SEL.sel1, digits.UpperMin, digits.LowerMin,
                             Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(Pins::ANODE_TIME_SEL.sel2, digits.Colon ? 0 : -1, -1, Constants::INS_1_ON_TIME_DELAY);
    // UNUSED displayNixieTubeTimePair(Pins::ANODE_TIME_SEL_PINS.sel3, -1, -1, IN_18_ON_TIME_DELAY);
}

void turnOffNixieTubeTime() {
    displayNixieTubeTimePair(Pins::ANODE_TIME_SEL.sel0, -1, -1, Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(Pins::ANODE_TIME_SEL.sel1, -1, -1, Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeTimePair(Pins::ANODE_TIME_SEL.sel2, -1, -1, Constants::INS_1_ON_TIME_DELAY);
    // UNUSED displayNixieTubeTimePair(Pins::ANODE_TIME_SEL_PINS.sel3, -1, -1, IN_18_ON_TIME_DELAY);
}

void displayNixieTubeDate(nixieDisplay_t &digits) {
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel0, digits.UpperMonth, digits.LowerMonth,
                             Constants::IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel1, digits.UpperDay, digits.LowerDay,
                             Constants::IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel2, digits.UpperYear, digits.LowerYear,
                             Constants::IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel3, digits.Dots ? 0 : -1, digits.Dots ? 0 : -1,
                             Constants::INS_1_ON_TIME_DELAY);
}

void turnOffNixieTubeDate() {
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel0, -1, -1, Constants::IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel1, -1, -1, Constants::IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel2, -1, -1, Constants::IN_12_ON_TIME_DELAY);
    displayNixieTubeDatePair(Pins::ANODE_DATE_SEL.sel3, -1, -1, Constants::INS_1_ON_TIME_DELAY);
}

void DisplayNixieTubeDateTime(ClockState clockState, nixieDisplay_t &digits) {
    switch (clockState) {
        case (ClockState::DEBUG):
            printNixieDisplay(digits);
        case (ClockState::DISPLAY_TIME_AND_DATE):
            displayNixieTubeTime(digits);
            displayNixieTubeDate(digits);
            break;
        case (ClockState::DISPLAY_TIME):
            displayNixieTubeTime(digits);
            turnOffNixieTubeDate();
            break;
        case (ClockState::OFF):
        default:
            turnOffNixieTubeTime();
            turnOffNixieTubeDate();
    }
}


/*
 * Real Time Clock Helper Functions
 */

void SetRTCDateTime(tmElements_t &tm) {
    struct ts time = {
            tm.Second,
            tm.Minute,
            tm.Hour,
            tm.Day,
            tm.Month,
            tmYearToCalendar(tm.Year),
            tm.Wday,
    };
    // printTimeElements(tm);

    SPI.setDataMode(SPI_MODE3);
    DS3234_set(Pins::RTC_SEL, time);
    SPI.setDataMode(SPI_MODE0);
}

void GetRTCDateTime(tmElements_t &tm) {
    struct ts t;
    SPI.setDataMode(SPI_MODE3);
    DS3234_get(Pins::RTC_SEL, &t);
    SPI.setDataMode(SPI_MODE0);

    tm.Second = t.sec;
    tm.Minute = t.min;
    tm.Hour = t.hour;
    tm.Wday = t.wday;
    tm.Day = t.mday;
    tm.Month = t.mon;
    tm.Year = CalendarYrToTm(t.year);
    // printTimeElements(tm);
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
    const unsigned long unixTime = timeClient.getEpochTime();
    tmElements_t utcElements;
    breakTime(unixTime, utcElements);
    Serial.print("UTC: ");
    printTimeElements(utcElements);

    // Parse utc timestamp and convert it to local time
    const unsigned long local = usCT.toLocal(unixTime);
    tmElements_t localElements;
    breakTime(local, localElements);
    Serial.print("CT: ");
    printTimeElements(localElements);

    // Set RTC to updated time
    SetRTCDateTime(localElements);
    return true;
}


nixieDisplay_t _currentDigits;
uint8_t counter = 0;
uint8_t debugCycleCount = 0;
const uint8_t debugDisplayCycles = 40;

void UpdateNixieTubeDateTime(ClockState clockState, tmElements_t &tm) {
    // Load the latest time from the RTC module
    nixieDisplay_t targetDigits;
    if (clockState == ClockState::DEBUG) {
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
        if (debugCycleCount == debugDisplayCycles) {
            debugCycleCount = 0;
            counter++;
        } else {
            debugCycleCount++;
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

    DisplayNixieTubeDateTime(clockState, _currentDigits);
}


/******************************************************************************/


int _blinksRemaining = Constants::LED_BLINK_COUNT;
LedState _successState = LedState::IDLE;
tmElements_t _tm;

ClockState setClockState(ClockState clockState) {
    EEPROM.update(Constants::CLOCK_STATE_ADDR, (int) clockState);
    return clockState;
}

ClockState getClockState() {
    return static_cast<ClockState>(EEPROM.read(Constants::CLOCK_STATE_ADDR));
}

ClockState cycleClockState() {
    ClockState clockState;
    switch (getClockState()) {
        default:
        case (ClockState::OFF):
            clockState = ClockState::DISPLAY_TIME_AND_DATE;
            break;
        case (ClockState::DISPLAY_TIME_AND_DATE):
            clockState = ClockState::DISPLAY_TIME;
            break;
        case (ClockState::DISPLAY_TIME):
        case (ClockState::DEBUG):
            clockState = ClockState::OFF;
            break;
    }
    return setClockState(clockState);
}


bool _lastButtonState = HIGH;
bool _currentButtonState;
unsigned long _lastDebounceTime = 0;
unsigned long _debounceDelay = 25;

bool isButtonPressed() {
    bool newButtonState = (bool) digitalRead(Pins::BUTTON_DIO);

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

    // LED Setup
    pinMode(Pins::LED_GND, OUTPUT);
    digitalWrite(Pins::LED_GND, LOW);

    // RTC Setup
    pinMode(Pins::RTC_PWR, OUTPUT);
    digitalWrite(Pins::RTC_PWR, HIGH);
    // Enable: Temperature Compensation
    // Disable: Oscillator, Battery SQ wave, Alarms
    DS3234_init(Pins::RTC_SEL, 0x60);

    // Button Setup
    pinMode(Pins::BUTTON_PWR, OUTPUT);
    digitalWrite(Pins::BUTTON_PWR, HIGH);
    pinMode(Pins::BUTTON_GND, OUTPUT);
    digitalWrite(Pins::BUTTON_GND, LOW);

    // Arduinix Setup
    const arduinixmux_t pinGroups[] = {
            Pins::CATHODE_DATE_SEL_A,
            Pins::CATHODE_DATE_SEL_B,
            Pins::CATHODE_TIME_SEL_A,
            Pins::CATHODE_TIME_SEL_B,
            Pins::ANODE_DATE_SEL,
            Pins::ANODE_TIME_SEL
    };
    for (arduinixmux_t pins: pinGroups) {
        pinMode(pins.sel0, OUTPUT);
        pinMode(pins.sel1, OUTPUT);
        pinMode(pins.sel2, OUTPUT);
        pinMode(pins.sel3, OUTPUT);
    }

    // Ethernet Setup
    EthernetInit();
    Ethernet.begin(Constants::MAC_ADDRESS);
    Serial.print("Clock has IP address: ");
    Serial.println(Ethernet.localIP());

    // NTP Setup
    timeClient.begin();

    // REST Server Setup
    server.on("/display/status", HTTP_GET, []() {
        String content = R"({"state": ")";
        switch (getClockState()) {
            case ClockState::DISPLAY_TIME_AND_DATE:
                content += "time-and-date";
                break;
            case ClockState::DISPLAY_TIME:
                content += "time";
                break;
            case ClockState::DEBUG:
                content += "debug";
                break;
            case ClockState::OFF:
            default:
                content += "off";
                break;
        }
        content += R"("})";
        server.send(200, "application/json", content);
    });
    server.on("/display/time-and-date", HTTP_POST, []() {
        setClockState(ClockState::DISPLAY_TIME_AND_DATE);
        server.send(200, "application/json", "{}");
    });
    server.on("/display/time", HTTP_POST, []() {
        setClockState(ClockState::DISPLAY_TIME);
        server.send(200, "application/json", "{}");
    });
    server.on("/display/debug", HTTP_POST, []() {
        setClockState(ClockState::DEBUG);
        server.send(200, "application/json", "{}");
    });
    server.on("/display/off", HTTP_POST, []() {
        setClockState(ClockState::OFF);
        server.send(200, "application/json", "{}");
    });
    server.on("/time/ntp-sync", HTTP_POST, []() {
        bool success = UpdateRTCDateTime();
        _successState = UpdateRTCDateTime() ? LedState::SUCCESS : LedState::FAIL;
        String content = R"({"success": )";
        content += success ? "true" : "false";
        content += "}";
        server.send(200, "application/json", content);
    });
    server.onNotFound([]() {
        server.send(404, "application/json", "{}");
    });
    server.begin();
}

void loop() {
    ClockState clockState = getClockState();

    // Run the RestServer
    server.handleClient();

    //Set LED based on status last time sync or current idle state
    if (_blinksRemaining == 0) {
        // We are done blinking, reset the number of blinks and set success state to idle
        _successState = LedState::IDLE;
        _blinksRemaining = Constants::LED_BLINK_COUNT;
    } else if (_successState == LedState::SUCCESS) {
        // Bright green; Successful time sync
        rgbValues_t rgb = {0, 255, 0};
        _blinksRemaining = LEDBlink(_blinksRemaining, rgb);
    } else if (_successState == LedState::FAIL) {
        // Bright red; Failed time sync
        rgbValues_t rgb = {255, 0, 0};
        _blinksRemaining = LEDBlink(_blinksRemaining, rgb);
    } else {
        // Standby
        rgbValues_t rgb = getLEDColor(clockState);
        LEDOn(rgb);
    }

    UpdateNixieTubeDateTime(clockState, _tm);

    // Read button;
    if (isButtonPressed()) {
        // Button was pressed
        unsigned long pressedTime = millis();
        unsigned long pressedDuration = millis() - pressedTime;

        while (isButtonPressed()) {
            pressedDuration = millis() - pressedTime;

            // Continue to update the nixie tubes
            UpdateNixieTubeDateTime(clockState, _tm);

            // Change the LED color indicating the action
            if (pressedDuration > 6000) {
                break;
            } else if (pressedDuration > 5000) {
                rgbValues_t rgb = {255, 255, 255};
                LEDOn(rgb);
            } else if (pressedDuration > 1500) {
                rgbValues_t rgb = {0, 0, 255};
                LEDOn(rgb);
            }
        }

        if (pressedDuration > 5000) {
            // If the button was held for more than 5s go into debug state.
            setClockState(ClockState::DEBUG);
        } else if (pressedDuration > 1500) {
            // Sync time from ntp source
            _successState = UpdateRTCDateTime() ? LedState::SUCCESS : LedState::FAIL;
        } else {
            cycleClockState();
        }
    }
}
