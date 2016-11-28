#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>

// https://github.com/PaulStoffregen/Time
#include <Time.h>
// https://github.com/JChristensen/Timezone
#include <Timezone.h>

#include <CustomTypes.h>


/*******************************************************************************
 * CONFIGURE
 ******************************************************************************/
/*
 * Ethernet
 */
byte mac[] = {0x90, 0xA2, 0xDA, 0x00, 0xF8, 0x1A};
EthernetClient client;


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
const uint8_t LED_BLINK_DEFAULT = 5;
const uint8_t LED_BLINK_INTERVAL = 100;


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
/*
 * LED Helper Functions
 */
void LEDOn(int red, int green, int blue) {
    analogWrite(LED_RED_PIN, red);
    analogWrite(LED_GREEN_PIN, green);
    analogWrite(LED_BLUE_PIN, blue);
}

void LEDOff() {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
}


int _ledState = LOW;
long _previousMillis = 0;

int LEDBlink(int blinks, int red, int green, int blue) {
    unsigned long currentMillis = millis();
    if (currentMillis - _previousMillis > LED_BLINK_INTERVAL) {
        _previousMillis = currentMillis;
        if (_ledState == LOW) {
            _ledState = HIGH;
            LEDOn(red, green, blue);
            blinks--;
        } else {
            _ledState = LOW;
            LEDOff();
        }
    }
    return blinks;
}


/*
 * Nixie Helper Functions
 */
uint8_t rollDigit(uint8_t digit) {
    if (digit == 0) return 9;
    return digit - 1;
}

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
    digits.Dots = 0;
    digits.Colon = 0;
    digits.Initialized = true;
}

void displayNixieTubeDigitPair(int anodeIndex, uint8_t *anodeSelPins,
                               int num1, uint8_t *num1SelPins,
                               int num2, uint8_t *num2SelPins) {
    // Write to select pins for mux 1
    digitalWrite(num1SelPins[0], (uint8_t) bitRead(num1, 0));
    digitalWrite(num1SelPins[1], (uint8_t) bitRead(num1, 1));
    digitalWrite(num1SelPins[2], (uint8_t) bitRead(num1, 2));
    digitalWrite(num1SelPins[3], (uint8_t) bitRead(num1, 3));

    // Write to select pins for mux 2
    digitalWrite(num2SelPins[0], (uint8_t) bitRead(num2, 0));
    digitalWrite(num2SelPins[1], (uint8_t) bitRead(num2, 1));
    digitalWrite(num2SelPins[2], (uint8_t) bitRead(num2, 2));
    digitalWrite(num2SelPins[3], (uint8_t) bitRead(num2, 3));

    digitalWrite(anodeSelPins[anodeIndex], HIGH);
    delay(2);
    digitalWrite(anodeSelPins[anodeIndex], LOW);
}

void displayNixieTubeTimePair(int anode, int num1, int num2) {
    displayNixieTubeDigitPair(anode, ANODE_TIME_SEL_PINS, num1, TIME_SEL_PINS_A, num2, TIME_SEL_PINS_B);
}

void displayNixieTubeDatePair(int anode, int num1, int num2) {
    displayNixieTubeDigitPair(anode, ANODE_DATE_SEL_PINS, num1, DATE_SEL_PINS_A, num2, DATE_SEL_PINS_B);
}

void DisplayNixieTubeDateTime(int displayState, nixieDisplay_t &digits) {
    switch (displayState) {
        case (2):
            // Display time
            displayNixieTubeTimePair(3, digits.UpperHour, 0);
            displayNixieTubeTimePair(0, 0, digits.LowerHour);
            displayNixieTubeTimePair(1, digits.UpperMin, digits.LowerMin);
            displayNixieTubeTimePair(2, digits.Colon, digits.Colon);
            // Display date
            displayNixieTubeDatePair(0, digits.UpperMonth, digits.LowerMonth);
            displayNixieTubeDatePair(1, digits.UpperDay, digits.LowerDay);
            displayNixieTubeDatePair(2, digits.UpperYear, digits.LowerYear);
            displayNixieTubeDatePair(3, digits.Dots, digits.Dots);
            break;
        case (1):
            // Display time
            displayNixieTubeTimePair(3, digits.UpperHour, 0);
            displayNixieTubeTimePair(0, 0, digits.LowerHour);
            displayNixieTubeTimePair(1, digits.UpperMin, digits.LowerMin);
            displayNixieTubeTimePair(2, digits.Colon, digits.Colon);
            // Disable date display
            digitalWrite(ANODE_DATE_SEL_PINS[0], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[1], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[2], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[3], LOW);
            break;
        case (0):
        default:
            // Disable date and dime displays
            digitalWrite(ANODE_TIME_SEL_PINS[0], LOW);
            digitalWrite(ANODE_TIME_SEL_PINS[1], LOW);
            digitalWrite(ANODE_TIME_SEL_PINS[2], LOW);
            digitalWrite(ANODE_TIME_SEL_PINS[3], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[0], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[1], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[2], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[3], LOW);

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
}


/*
 * High Level Helper Functions
 */

const char _server[] = "www.timeapi.org";
const char _location[] = "/utc/now?format=^%25Y-%25m-%25dT%25I:%25M:%25S%25p$";
TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420};
TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480};
Timezone usPT(usPDT, usPST);

boolean UpdateRTCDateTime() {
    LEDOn(0, 0, 255);

    Serial.print("initializing...");
    if (Ethernet.begin(mac) == 0) {
        Serial.println("failed");
        return false;
    }
    Serial.println("done");


    Serial.print("connecting...");
    if (client.connect(_server, 80)) {
        Serial.println("done");

        client.println("GET " + String(_location) + " HTTP/1.1");
        client.println("Host: " + String(_server));
        client.println("User-Agent: Arduino/NixieClock");
        client.println("Accept: */*");
        client.println("Connection: close");
        client.println();

        Serial.print("reading...");
        String dateTimeStr = "";
        boolean startReading = false;
        while (client.connected()) {
            if (client.available()) {
                String c = String((char) client.read());
                if (c == "^") {
                    startReading = true;
                } else if (c == "$") {
                    break;
                } else if (startReading) {
                    dateTimeStr = dateTimeStr + c;
                }
            } else {
                delay(5);
            }
        }
        Serial.println("done");

        Serial.print("disconnecting...");
        client.stop();
        client.flush();
        Serial.println("done");

        // Convert UTC time to the local timezone correcting for DST
        tmElements_t utcElements;
        utcElements.Year = (uint8_t) (dateTimeStr.substring(0, 4).toInt() - 1970);
        utcElements.Month = (uint8_t) dateTimeStr.substring(5, 7).toInt();
        utcElements.Day = (uint8_t) dateTimeStr.substring(8, 10).toInt();
        utcElements.Hour = (uint8_t) dateTimeStr.substring(11, 13).toInt();
        utcElements.Minute = (uint8_t) dateTimeStr.substring(14, 16).toInt();
        utcElements.Second = (uint8_t) dateTimeStr.substring(17, 19).toInt();
        time_t utc = makeTime(utcElements);
        time_t local = usPT.toLocal(utc);
        tmElements_t ptElements;
        breakTime(local, ptElements);

        // Set RTC to updated time
        SetRTCDateTime(ptElements);
        return true;
    }
    Serial.println("failed");
    return false;
}


nixieDisplay_t _currentDigits;

void UpdateNixieTubeDateTime(int displayState, tmElements_t &tm) {
    // Load the latest time from the RTC module
    GetRTCDateTime(tm);

    nixieDisplay_t targetDigits;
    tmElementsToNixieDisplay(tm, targetDigits);

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

    DisplayNixieTubeDateTime(displayState, _currentDigits);
}


/******************************************************************************/


int _blinks = LED_BLINK_DEFAULT;
int _displayState = 2; // 0: off, 1: time only, 2: time & date
int _successState = 2; // 0: success, 1: fail, 2: idle
tmElements_t _tm;
int _ledStandby[] = {0, 1, 0}; // RGB values (0-255)


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
    if (_blinks == 0 && (_successState == 0 || _successState == 1)) {
        // We are done blinking, reset the number of blinks and set success state to idle
        _successState = 2;
        _blinks = LED_BLINK_DEFAULT;
    } else if (_successState == 0) {
        // Bright green; Successful time sync
        _blinks = LEDBlink(_blinks, 0, 255, 0);
    } else if (_successState == 1) {
        // Bright red; Failed time sync
        _blinks = LEDBlink(_blinks, 255, 0, 0);
    } else {
        // Standby
        LEDOn(_ledStandby[0], _ledStandby[1], _ledStandby[2]);
    }

    // Read button;
    boolean sync = (boolean) digitalRead(BUTTON_DIO);
    if (sync == LOW) {
        // Button was pressed
        unsigned long pressed = millis();
        UpdateNixieTubeDateTime(_displayState, _tm);

        while (sync == LOW) {
            UpdateNixieTubeDateTime(_displayState, _tm);
            if ((millis() - pressed) > 1500) {
                // Sync time to internet
                _successState = UpdateRTCDateTime() ? 0 : 1;
                goto beginning;
            }
            sync = (boolean) digitalRead(BUTTON_DIO);
        }

        switch (_displayState) {
            case (0): // Currently Off
                _ledStandby[0] = 0;
                _ledStandby[1] = 1;
                _ledStandby[2] = 0;
                _displayState = 2;
                break;
            case (1): // Currently Time Only
                _ledStandby[0] = 1;
                _ledStandby[1] = 0;
                _ledStandby[2] = 0;
                _displayState = 0;
                break;
            case (2):  // Currently Time & Date
            default:
                _ledStandby[0] = 0;
                _ledStandby[1] = 0;
                _ledStandby[2] = 1;
                _displayState = 1;
                break;
        }
    }
    UpdateNixieTubeDateTime(_displayState, _tm);
}
