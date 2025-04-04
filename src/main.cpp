#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>

#define USE_ETHERNET_WRAPPER            true
#define USE_ETHERNET                    true
#include <NTPClient.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <ds3234.h>
#include <PubSubClient.h>

#include <defines.hpp>
#include <configuration.h>


namespace Constants {
    /*
     * LED
     */
    static constexpr unsigned int LED_BLINK_COUNT = 5;
    static constexpr unsigned long LED_BLINK_INTERVAL = 100;

    static constexpr rgbValues_t LED_COLOR_RED_BRIGHT = {255, 0, 0};
    static constexpr rgbValues_t LED_COLOR_RED_DULL = {5, 0, 0};
    static constexpr rgbValues_t LED_COLOR_GREEN_BRIGHT = {0, 255, 0};
    static constexpr rgbValues_t LED_COLOR_GREEN_DULL = {0, 5, 0};
    static constexpr rgbValues_t LED_COLOR_BLUE_BRIGHT = {0, 0, 255};
    static constexpr rgbValues_t LED_COLOR_BLUE_DULL = {0, 0, 5};
    // static constexpr rgbValues_t LED_COLOR_YELLOW_BRIGHT = {255, 255, 0};
    // static constexpr rgbValues_t LED_COLOR_YELLOW_DULL = {5, 5, 0};
    // static constexpr rgbValues_t LED_COLOR_CYAN_BRIGHT = {0, 255, 255};
    static constexpr rgbValues_t LED_COLOR_CYAN_DULL = {0, 5, 5};
    // static constexpr rgbValues_t LED_COLOR_PINK_BRIGHT = {255, 0, 255};
    // static constexpr rgbValues_t LED_COLOR_PINK_DULL = {5, 0, 5};
    static constexpr rgbValues_t LED_COLOR_OFF = {0, 0, 0};

    /*
     * Nixie tube
     */
    static constexpr unsigned long IN_18_ON_TIME_DELAY = 2;
    static constexpr unsigned long IN_12_ON_TIME_DELAY = 1;
    static constexpr unsigned long INS_1_ON_TIME_DELAY = 2;

    /*
     * Ethernet
     */
    static byte MAC_ADDRESS[] = {0x90, 0xA2, 0xDA, 0x00, 0xF8, 0x1A};

    static constexpr unsigned int CLOCK_STATE_ADDR = 0;
    static constexpr unsigned int TIME_SYNC_ADDR = 1;
}

namespace Pins {
    /*
     * Real-time clock module
     */
    static constexpr unsigned int RTC_SEL = 53;
    static constexpr unsigned int RTC_PWR = 48;

    /*
     * Button
     */
    static constexpr unsigned int BUTTON_DIO = 2;
    static constexpr unsigned int BUTTON_PWR = 4;
    static constexpr unsigned int BUTTON_GND = 7;

    /*
     * Status LED
     */
    static constexpr unsigned int LED_GND = 8;
    static constexpr unsigned int LED_RED = 3;
    static constexpr unsigned int LED_GREEN = 6;
    static constexpr unsigned int LED_BLUE = 5;

    /*
     * Arduinix
     */
    static auto CATHODE_DATE_SEL_A = arduinixmux_t{23, 25, 27, 29};
    static auto CATHODE_DATE_SEL_B = arduinixmux_t{31, 33, 35, 37};
    static auto ANODE_DATE_SEL = arduinixmux_t{45, 43, 41, 39};

    static auto CATHODE_TIME_SEL_A = arduinixmux_t{22, 24, 26, 28};
    static auto CATHODE_TIME_SEL_B = arduinixmux_t{30, 32, 34, 36};
    static auto ANODE_TIME_SEL = arduinixmux_t{44, 42, 40, 38};
}

/*******************************************************************************
 * CONFIGURE
 ******************************************************************************/

/*
 * Ethernet
 */
EthernetUDP udp;
NTPClient timeClient(udp, NTP_POOL_SERVER_NAME);
EthernetClient ethClient;

/*******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************/
void printTimeElements(const tmElements_t &tm) {
    char s[20];
    snprintf(
        s,
        sizeof(s),
        "%04d-%02d-%02dT%02d:%02d:%02d", tmYearToCalendar(tm.Year),
        tm.Month,
        tm.Day,
        tm.Hour,
        tm.Minute,
        tm.Second
    );
    Serial.println(s);
}

void printNixieDisplay(const nixieDisplay_t &digits) {
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

ClockState getClockState(const boolean ignoreTimeSync) {
    if (!ignoreTimeSync && EEPROM.read(Constants::TIME_SYNC_ADDR) == 1) {
        return ClockState::TIME_SYNC;
    }
    return static_cast<ClockState>(EEPROM.read(Constants::CLOCK_STATE_ADDR));
}

ClockState getClockState() {
    return getClockState(false);
}

const char *convertClockState(const ClockState clockState) {
    switch (clockState) {
        case ClockState::ON:
            return "on";
        case ClockState::DEBUG:
            return "debug";
        case ClockState::TIME_SYNC:
            return "time-sync";
        case ClockState::OFF:
        default:
            return "off";
    }
}

ClockState convertClockState(const char *clockState) {
    if (strcmp(clockState, "on") == 0) {
        return ClockState::ON;
    }
    if (strcmp(clockState, "debug") == 0) {
        return ClockState::DEBUG;
    }
    if (strcmp(clockState, "time-sync") == 0) {
        return ClockState::TIME_SYNC;
    }
    if (strcmp(clockState, "off") == 0) {
        return ClockState::OFF;
    }
    return getClockState();
}

void setClockState(ClockState clockState);

// ReSharper disable twice CppParameterMayBeConstPtrOrRef
void mqttCallback(char *topic, byte *payload, const unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: '");

    char state[length + 1]{}; // Reserve one position for the termination character.
    memcpy(state, payload, length);
    state[length] = '\0';

    Serial.print(state);
    Serial.println("'");

    if (strcmp(topic, MQTT_COMMANDS_TOPIC) == 0) {
        setClockState(convertClockState(state));
    }
}

PubSubClient mqttClient(MQTT_BROKER_HOST, MQTT_BROKER_PORT, mqttCallback, ethClient);

void sendMqttClockStateUpdate(const ClockState clockState) {
    if (!mqttClient.connected()) {
        return;
    }
    Serial.print("Sending state [");
    const char *state = convertClockState(clockState);
    Serial.print(state);
    Serial.print("] to MQTT...");
    if (mqttClient.publish(MQTT_STATE_TOPIC, state, true)) {
        Serial.println("done");
    } else {
        Serial.println("failed");
    }
}

void setClockState(const ClockState clockState) {
    const ClockState currentClockState = getClockState();
    if (clockState != currentClockState) {
        Serial.print("Changing clock state from ");
        Serial.print(convertClockState(currentClockState));
        Serial.print(" -> ");
        Serial.println(convertClockState(clockState));
        if (clockState == ClockState::TIME_SYNC) {
            if (EEPROM.read(Constants::TIME_SYNC_ADDR) == 0) {
                EEPROM.update(Constants::TIME_SYNC_ADDR, 1);
            }
        } else {
            if (EEPROM.read(Constants::TIME_SYNC_ADDR) == 1) {
                EEPROM.update(Constants::TIME_SYNC_ADDR, 0);
            }
            EEPROM.update(Constants::CLOCK_STATE_ADDR, static_cast<int>(clockState));
        }
        sendMqttClockStateUpdate(clockState);
    }
}

ClockState cycleClockState(const ClockState clockState) {
    switch (clockState) {
        default:
        case ClockState::DEBUG:
        case ClockState::OFF:
        case ClockState::TIME_SYNC:
            return ClockState::ON;
        case ClockState::ON:
            return ClockState::OFF;
    }
}


/*
 * LED Helper Functions
 */
void setLedColor(const rgbValues_t &rgb) {
    analogWrite(Pins::LED_RED, rgb.red);
    analogWrite(Pins::LED_GREEN, rgb.green);
    analogWrite(Pins::LED_BLUE, rgb.blue);
}

void setLedOff() {
    setLedColor(Constants::LED_COLOR_OFF);
}

boolean ledBlinkState = false;
unsigned long previousLedBlinkMillis = 0;

int LEDBlink(int blinks, const rgbValues_t &rgb) {
    const unsigned long currentMillis = millis();
    if (currentMillis - previousLedBlinkMillis > Constants::LED_BLINK_INTERVAL) {
        previousLedBlinkMillis = currentMillis;
        if (!ledBlinkState) {
            setLedColor(rgb);
            blinks--;
        } else {
            setLedOff();
        }
        ledBlinkState = !ledBlinkState;
    }
    return blinks;
}

rgbValues_t getLEDColor(const ClockState clockState) {
    switch (clockState) {
        case ClockState::ON:
            return Constants::LED_COLOR_GREEN_DULL;
        case ClockState::DEBUG: // Rolling Digits
            return Constants::LED_COLOR_CYAN_DULL;
        case ClockState::TIME_SYNC:
            return Constants::LED_COLOR_BLUE_DULL;
        case ClockState::OFF:
        default:
            return Constants::LED_COLOR_RED_DULL;
    }
}

rgbValues_t getLEDColor(const LedState ledState) {
    switch (ledState) {
        case LedState::PENDING:
            return Constants::LED_COLOR_BLUE_BRIGHT;
        case LedState::SUCCESS:
            return Constants::LED_COLOR_GREEN_BRIGHT;
        case LedState::FAIL:
            return Constants::LED_COLOR_RED_BRIGHT;
        case LedState::IDLE:
        default:
            return getLEDColor(getClockState());
    }
}

void setLedColor(const ClockState clockState) {
    setLedColor(getLEDColor(clockState));
}

void setLedColor(const LedState ledState) {
    setLedColor(getLEDColor(ledState));
}

/*
 * Real Time Clock Helper Functions
 */

void SetRTCDateTime(const tmElements_t &tm) {
    const ts time = {
        tm.Second,
        tm.Minute,
        tm.Hour,
        tm.Day,
        tm.Month,
        static_cast<int>(tmYearToCalendar(tm.Year)),
        tm.Wday,
    };
    // printTimeElements(tm);

    SPI.setDataMode(SPI_MODE3);
    DS3234_set(Pins::RTC_SEL, time);
    SPI.setDataMode(SPI_MODE0);
}

void GetRTCDateTime(tmElements_t &tm) {
    ts t{};
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
}


/*
 * High Level Helper Functions
 */
TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -300}; // UTC−07:00
TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -360};  // UTC−08:00
TimeChangeRule usMDT = {"MDT", Second, Sun, Mar, 2, -300}; // UTC−06:00
TimeChangeRule usMST = {"MST", First, Sun, Nov, 2, -360};  // UTC−07:00
TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300}; // UTC−05:00
TimeChangeRule usCST = {"CST", First, Sun, Nov, 2, -360};  // UTC−06:00
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240}; // UTC−04:00
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};  // UTC−05:00

Timezone usPT(usPDT, usPST);
Timezone usMT(usMDT, usMST);
Timezone usMT_AZ( usMST);
Timezone usCT(usCDT, usCST);
Timezone usET(usEDT, usEST);

LedState UpdateRTCDateTime() {
    Serial.print("Syncing time with ");
    Serial.print(NTP_POOL_SERVER_NAME);
    Serial.print("... ");
    if (!timeClient.update()) {
        Serial.println("failed");
        return LedState::FAIL;
    }
    Serial.println("done");

    // Parse utc timestamp into time elements and print it
    const unsigned long unixTime = timeClient.getEpochTime();
    tmElements_t utcElements;
    breakTime(unixTime, utcElements);
    Serial.print("UTC:   ");
    printTimeElements(utcElements);

    // Parse utc timestamp and convert it to local time
    tmElements_t localElements;
    unsigned long local;
    // ReSharper disable All
    if (strcmp(TIMEZONE, "PT") == 0) {
        local = usPT.toLocal(unixTime);
    } else if (strcmp(TIMEZONE, "MT") == 0) {
        local = usMT.toLocal(unixTime);
    } else if (strcmp(TIMEZONE, "MT_AZ") == 0) {
        local = usMT_AZ.toLocal(unixTime);
    } else if (strcmp(TIMEZONE, "CT") == 0) {
        local = usCT.toLocal(unixTime);
    } else if (strcmp(TIMEZONE, "ET") == 0) {
        local = usET.toLocal(unixTime);
    } else {
        local = unixTime;
    }
    // ReSharper restore All
    breakTime(local, localElements);
    Serial.print("LOCAL: ");
    printTimeElements(localElements);

    // Set RTC to updated time
    SetRTCDateTime(localElements);
    return LedState::SUCCESS;
}


/*
 * Nixie Helper Functions
 */
uint8_t rollDigit(const uint8_t digit) {
    if (digit == 0) return 9;
    return digit - 1;
}

long previousColonOnSec = -1;
unsigned long previousColonOnMillis = 0;

void tmElementsToNixieDisplay(const tmElements_t &tm, nixieDisplay_t &digits) {
    const uint8_t smallYear = tmYearToCalendar(tm.Year) % 100;
    digits.LowerYear = smallYear % 10;
    digits.UpperYear = smallYear - digits.LowerYear;
    if (digits.UpperYear >= 10) digits.UpperYear = digits.UpperYear / 10;

    digits.LowerMonth = tm.Month % 10;
    digits.UpperMonth = tm.Month - digits.LowerMonth;
    if (digits.UpperMonth >= 10) digits.UpperMonth = digits.UpperMonth / 10;

    digits.LowerDay = tm.Day % 10;
    digits.UpperDay = tm.Day - digits.LowerDay;
    if (digits.UpperDay >= 10) digits.UpperDay = digits.UpperDay / 10;

    const uint8_t hour = tm.Hour > 12 ? tm.Hour - 12 : tm.Hour;
    digits.LowerHour = hour % 10;
    digits.UpperHour = hour - digits.LowerHour;
    if (digits.UpperHour >= 10) digits.UpperHour = digits.UpperHour / 10;

    digits.LowerMin = tm.Minute % 10;
    digits.UpperMin = tm.Minute - digits.LowerMin;
    if (digits.UpperMin >= 10) digits.UpperMin = digits.UpperMin / 10;

    digits.Dots = true;

    const unsigned long currentMillis = millis();
    if (previousColonOnSec != tm.Second) {
        previousColonOnMillis = currentMillis;
        previousColonOnSec = tm.Second;
        digits.Colon = true;
    } else if (currentMillis - previousColonOnMillis < 500) {
        digits.Colon = true;
    } else {
        digits.Colon = false;
    }

    digits.Initialized = true;
}

void displayNixieTubeDigitPair(const unsigned int anodePin,
                               const int digit1, const arduinixmux_t &digit1SelPins,
                               const int digit2, const arduinixmux_t &digit2SelPins, const unsigned long onTimeDelay) {
    if (digit1 >= 0) {
        // Write to select pins for mux 1
        digitalWrite(digit1SelPins.sel0, static_cast<uint8_t>(bitRead(digit1, 0)));
        digitalWrite(digit1SelPins.sel1, static_cast<uint8_t>(bitRead(digit1, 1)));
        digitalWrite(digit1SelPins.sel2, static_cast<uint8_t>(bitRead(digit1, 2)));
        digitalWrite(digit1SelPins.sel3, static_cast<uint8_t>(bitRead(digit1, 3)));
    } else {
        digitalWrite(digit1SelPins.sel0, 1);
        digitalWrite(digit1SelPins.sel1, 1);
        digitalWrite(digit1SelPins.sel2, 1);
        digitalWrite(digit1SelPins.sel3, 1);
    }
    if (digit2 >= 0) {
        // Write to select pins for mux 2
        digitalWrite(digit2SelPins.sel0, static_cast<uint8_t>(bitRead(digit2, 0)));
        digitalWrite(digit2SelPins.sel1, static_cast<uint8_t>(bitRead(digit2, 1)));
        digitalWrite(digit2SelPins.sel2, static_cast<uint8_t>(bitRead(digit2, 2)));
        digitalWrite(digit2SelPins.sel3, static_cast<uint8_t>(bitRead(digit2, 3)));
    } else {
        digitalWrite(digit2SelPins.sel0, 1);
        digitalWrite(digit2SelPins.sel1, 1);
        digitalWrite(digit2SelPins.sel2, 1);
        digitalWrite(digit2SelPins.sel3, 1);
    }
    if (digit1 >= 0 || digit2 >= 0) {
        digitalWrite(anodePin, HIGH);
        delay(onTimeDelay);
    }
    digitalWrite(anodePin, LOW);
}

void displayNixieTubeDigitPair(const unsigned int anodePin, const int digit1, const int digit2,
                              const unsigned long onTimeDelay) {
    displayNixieTubeDigitPair(anodePin, digit1, Pins::CATHODE_TIME_SEL_A, digit2, Pins::CATHODE_TIME_SEL_B, onTimeDelay);
}

void displayNixieTubeDatePair(const unsigned int anodePin, const int digit1, const int digit2,
                              const unsigned long onTimeDelay) {
    displayNixieTubeDigitPair(anodePin, digit1, Pins::CATHODE_DATE_SEL_A, digit2, Pins::CATHODE_DATE_SEL_B, onTimeDelay);
}

void displayNixieTubeTime(const nixieDisplay_t &digits) {
    displayNixieTubeDigitPair(Pins::ANODE_TIME_SEL.sel0, digits.UpperHour, digits.LowerHour,
                             Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeDigitPair(Pins::ANODE_TIME_SEL.sel1, digits.UpperMin, digits.LowerMin,
                             Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeDigitPair(Pins::ANODE_TIME_SEL.sel2, digits.Colon ? 0 : -1, -1, Constants::INS_1_ON_TIME_DELAY);
    // UNUSED displayNixieTubeTimePair(Pins::ANODE_TIME_SEL_PINS.sel3, -1, -1, IN_18_ON_TIME_DELAY);
}

void turnOffNixieTubeTime() {
    displayNixieTubeDigitPair(Pins::ANODE_TIME_SEL.sel0, -1, -1, Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeDigitPair(Pins::ANODE_TIME_SEL.sel1, -1, -1, Constants::IN_18_ON_TIME_DELAY);
    displayNixieTubeDigitPair(Pins::ANODE_TIME_SEL.sel2, -1, -1, Constants::INS_1_ON_TIME_DELAY);
    // UNUSED displayNixieTubeTimePair(Pins::ANODE_TIME_SEL_PINS.sel3, -1, -1, IN_18_ON_TIME_DELAY);
}

void displayNixieTubeDate(const nixieDisplay_t &digits) {
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

nixieDisplay_t currentDigits;
uint8_t counter = 0;
uint8_t debugCycleCount = 0;
constexpr uint8_t debugDisplayCycles = 40;

void DisplayNixieTubeDateTime(const ClockState clockState, const nixieDisplay_t &digits) {
    // Reset the watchdog timer in the display path.
    wdt_reset();

    switch (clockState) {
        case ClockState::DEBUG:
            if (debugCycleCount == debugDisplayCycles) {
                printNixieDisplay(digits);
            }
        case ClockState::ON:
            displayNixieTubeTime(digits);
            displayNixieTubeDate(digits);
            break;
        case ClockState::TIME_SYNC:
        case ClockState::OFF:
        default:
            turnOffNixieTubeTime();
            turnOffNixieTubeDate();
    }
}

void DisplayNixieTubeDateTime(const ClockState clockState, tmElements_t &tm) {
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
    if (!currentDigits.Initialized) {
        memcpy(&currentDigits, &targetDigits, sizeof(targetDigits));
    }

    if (currentDigits.UpperYear != targetDigits.UpperYear) {
        currentDigits.UpperYear = rollDigit(currentDigits.UpperYear);
    }
    if (currentDigits.LowerYear != targetDigits.LowerYear) {
        currentDigits.LowerYear = rollDigit(currentDigits.LowerYear);
    }
    if (currentDigits.UpperMonth != targetDigits.UpperMonth) {
        currentDigits.UpperMonth = rollDigit(currentDigits.UpperMonth);
    }
    if (currentDigits.LowerMonth != targetDigits.LowerMonth) {
        currentDigits.LowerMonth = rollDigit(currentDigits.LowerMonth);
    }
    if (currentDigits.UpperDay != targetDigits.UpperDay) {
        currentDigits.UpperDay = rollDigit(currentDigits.UpperDay);
    }
    if (currentDigits.LowerDay != targetDigits.LowerDay) {
        currentDigits.LowerDay = rollDigit(currentDigits.LowerDay);
    }
    if (currentDigits.UpperHour != targetDigits.UpperHour) {
        currentDigits.UpperHour = rollDigit(currentDigits.UpperHour);
    }
    if (currentDigits.LowerHour != targetDigits.LowerHour) {
        currentDigits.LowerHour = rollDigit(currentDigits.LowerHour);
    }
    if (currentDigits.UpperMin != targetDigits.UpperMin) {
        currentDigits.UpperMin = rollDigit(currentDigits.UpperMin);
    }
    if (currentDigits.LowerMin != targetDigits.LowerMin) {
        currentDigits.LowerMin = rollDigit(currentDigits.LowerMin);
    }
    if (currentDigits.Dots != targetDigits.Dots) {
        currentDigits.Dots = targetDigits.Dots;
    }
    if (currentDigits.Colon != targetDigits.Colon) {
        currentDigits.Colon = targetDigits.Colon;
    }

    DisplayNixieTubeDateTime(clockState, currentDigits);
}


/******************************************************************************/


int blinksRemaining = Constants::LED_BLINK_COUNT;
auto ledState = LedState::IDLE;
tmElements_t tm;

bool lastButtonState = HIGH;
bool currentButtonState;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 25;

bool isButtonPressed() {
    const bool newButtonState = static_cast<bool>(digitalRead(Pins::BUTTON_DIO));

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (newButtonState != lastButtonState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if (millis() - lastDebounceTime > debounceDelay) {
        // whatever the reading is at, it's been there for longer
        // than the debounce delay, so take it as the actual current state:

        // if the button state has changed:
        if (newButtonState != currentButtonState) {
            currentButtonState = newButtonState;
        }
    }
    lastButtonState = newButtonState;
    return currentButtonState == LOW;
}

unsigned long lastReconnectAttempt = -60000;

void mqttLoop() {
    if (!mqttClient.connected()) {
        const unsigned long now = millis();
        if (now - lastReconnectAttempt >= 60000) {
            lastReconnectAttempt = now;
            Serial.print("Connecting to MQTT... ");
            if (
                mqttClient.connect(
                    MQTT_ID,
                    MQTT_USER,
                    MQTT_PASSWORD,
                    MQTT_WILL_TOPIC,
                    MQTTQOS0,
                    true,
                    MQTT_WILL_MESSAGE
                )
            ) {
                Serial.println("done");
                Serial.print("Subscribing to MQTT topics... ");
                if (mqttClient.subscribe(MQTT_COMMANDS_TOPIC)) {
                    Serial.println("done");
                    Serial.print("Publishing MQTT birth message... ");
                    if (mqttClient.publish(MQTT_WILL_TOPIC, MQTT_BIRTH_MESSAGE, true)) {
                        Serial.println("done");
                    } else {
                        Serial.println("failed!");
                    }

                } else {
                    Serial.println("failed!");
                }
            } else {
                Serial.println("failed!");
            }
        }
    }

    if (mqttClient.connected()) {
        lastReconnectAttempt = 0;
        mqttClient.loop();
    }
}

void setup() {
    // Enable the watchdog timer with a 8s timeout
    wdt_enable(WDTO_8S);

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
    for (const arduinixmux_t pins: pinGroups) {
        pinMode(pins.sel0, OUTPUT);
        pinMode(pins.sel1, OUTPUT);
        pinMode(pins.sel2, OUTPUT);
        pinMode(pins.sel3, OUTPUT);
    }

    // Ethernet Setup
    Serial.print("Connecting to network... ");
    if (Ethernet.begin(Constants::MAC_ADDRESS) == 1) {
        Serial.println(Ethernet.localIP());
    } else {
        Serial.println("failed!");
    }

    // Ethernet Web Server Setup
    mqttLoop();

    // NTP Setup
    timeClient.begin();
}

void loop() {
    ClockState clockState = getClockState();

    // Check if we need to sync the time
    if (clockState == ClockState::TIME_SYNC) {
        setLedColor(LedState::PENDING);
        ledState = UpdateRTCDateTime();
        clockState = getClockState(true);
        setClockState(clockState);
    }

    if (blinksRemaining == 0) {
        // We are done blinking, reset the number of blinks and set success state to idle
        ledState = LedState::IDLE;
        blinksRemaining = Constants::LED_BLINK_COUNT;
    } else if (ledState == LedState::IDLE || ledState == LedState::PENDING) {
        setLedColor(ledState);
    } else {
        blinksRemaining = LEDBlink(blinksRemaining, getLEDColor(ledState));
    }

    DisplayNixieTubeDateTime(clockState, tm);

    // Read button;
    if (isButtonPressed()) {
        // Button was pressed
        const unsigned long pressedTime = millis();
        unsigned long pressedDuration = 0;

        setLedOff();

        ClockState nextClockState = cycleClockState(clockState);
        while (isButtonPressed()) {
            pressedDuration = millis() - pressedTime;

            // Continue to update the nixie tubes
            DisplayNixieTubeDateTime(clockState, tm);

            // Change the LED color indicating the action
            if (pressedDuration > 5000) {
                nextClockState = ClockState::DEBUG;
                setLedColor(nextClockState);
                break;
            }
            if (pressedDuration > 1500) {
                nextClockState = ClockState::TIME_SYNC;
                setLedColor(nextClockState);
            }
        }

        setClockState(nextClockState);
    }

    // Run the MQTT client
    mqttLoop();
}
