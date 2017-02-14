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
EthernetUDP udp;
unsigned int localUdpPort = 8888;  // local port to listen for UDP packets


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
void printTimeElements(tmElements_t &tm){
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

void printNixieDisplay(nixieDisplay_t &digits){
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
void LEDOn(ledDisplay_t &rgb) {
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
long _previousMillis = 0;

int LEDBlink(int blinks, ledDisplay_t &rgb) {
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

ledDisplay_t getLEDColor(int displayState) {
    ledDisplay_t rgb;
    switch (displayState) {
        default:
        case (0): // Off
            rgb.Red = 50;
            rgb.Green = 0;
            rgb.Blue = 0;
            break;
        case (1): // Time Only
            rgb.Red = 0;
            rgb.Green = 0;
            rgb.Blue = 50;
            break;
        case (2):  // Time & Date
            rgb.Red = 0;
            rgb.Green = 50;
            rgb.Blue = 0;
            break;
        case (3):  // Rolling Digits
            rgb.Red = 50;
            rgb.Green = 50;
            rgb.Blue = 50;
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
    if(num1 >= 0) {
        // Write to select pins for mux 1
        digitalWrite(num1SelPins[0], (uint8_t) bitRead(num1, 0));
        digitalWrite(num1SelPins[1], (uint8_t) bitRead(num1, 1));
        digitalWrite(num1SelPins[2], (uint8_t) bitRead(num1, 2));
        digitalWrite(num1SelPins[3], (uint8_t) bitRead(num1, 3));
    } else {
        digitalWrite(num1SelPins[0], 0);
        digitalWrite(num1SelPins[1], 0);
        digitalWrite(num1SelPins[2], 0);
        digitalWrite(num1SelPins[3], 0);
    }
    if(num2 >= 0) {
        // Write to select pins for mux 2
        digitalWrite(num2SelPins[0], (uint8_t) bitRead(num2, 0));
        digitalWrite(num2SelPins[1], (uint8_t) bitRead(num2, 1));
        digitalWrite(num2SelPins[2], (uint8_t) bitRead(num2, 2));
        digitalWrite(num2SelPins[3], (uint8_t) bitRead(num2, 3));
    } else {
        digitalWrite(num2SelPins[0], 0);
        digitalWrite(num2SelPins[1], 0);
        digitalWrite(num2SelPins[2], 0);
        digitalWrite(num2SelPins[3], 0);
    }
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
//    printNixieDisplay(digits);
    switch (displayState) {
        case (2):
        case (3):
            // Display time
            // displayNixieTubeTimePair(3, digits.UpperHour, 0);  old broken code
            // displayNixieTubeTimePair(0, 0, digits.LowerHour);
            displayNixieTubeTimePair(3, digits.UpperHour, digits.LowerHour);
            displayNixieTubeTimePair(0, digits.UpperHour, digits.LowerHour);
            displayNixieTubeTimePair(1, digits.UpperMin, digits.LowerMin);
            displayNixieTubeTimePair(2, digits.Colon, digits.Colon);
            // Display date
            displayNixieTubeDatePair(0, digits.UpperMonth, digits.LowerMonth);
            displayNixieTubeDatePair(1, digits.UpperDay, digits.LowerDay);
            displayNixieTubeDatePair(2, digits.UpperYear, digits.LowerYear);
            displayNixieTubeDatePair(3, digits.Dots, digits.Dots);
            break;
        case (1):
            // Disable date display
            digitalWrite(ANODE_DATE_SEL_PINS[0], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[1], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[2], LOW);
            digitalWrite(ANODE_DATE_SEL_PINS[3], LOW);
            // Display time
            displayNixieTubeTimePair(3, digits.UpperHour, 0);
            displayNixieTubeTimePair(0, 0, digits.LowerHour);
            displayNixieTubeTimePair(1, digits.UpperMin, digits.LowerMin);
            displayNixieTubeTimePair(2, digits.Colon, digits.Colon);
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
//    printTimeElements(tm);
}


/*
 * High Level Helper Functions
 */

/*
 * © Francesco Potortì 2013 - GPLv3 - Revision: 1.13
 *
 * Send an NTP packet and wait for the response, return the Unix time
 *
 * To lower the memory footprint, no buffers are allocated for sending
 * and receiving the NTP packets.  Four bytes of memory are allocated
 * for transmision, the rest is random garbage collected from the data
 * memory segment, and the received packet is read one byte at a time.
 * The Unix time is returned, that is, seconds from 1970-01-01T00:00.
 */
unsigned long inline ntpUnixTime(UDP &udp) {
    static int udpInited = udp.begin(localUdpPort); // open socket on arbitrary port

    const char timeServer[] = "pool.ntp.org";  // NTP server

    // Only the first four bytes of an outgoing NTP packet need to be set
    // appropriately, the rest can be whatever.
    const long ntpFirstFourBytes = 0xEC0600E3; // NTP request header

    // Fail if WiFiUdp.begin() could not init a socket
    if (!udpInited) {
        Serial.println("failed; could not init ntp socket");
        return 0;
    }

    // Clear received data from possible stray received packets
    udp.flush();

    // Send an NTP request
    if (!(udp.beginPacket(timeServer, 123) // 123 is the NTP port
          && udp.write((byte *) &ntpFirstFourBytes, 48) == 48
          && udp.endPacket())) {
        Serial.println("failed; could not send ntp packet");
        return 0;                // sending request failed
    }

    // Wait for response; check every pollIntv ms up to maxPoll times
    const int pollIntv = 150;        // poll every this many ms
    const byte maxPoll = 15;        // poll up to this many times
    int pktLen;                // received packet length
    for (byte i = 0; i < maxPoll; i++) {
        if ((pktLen = udp.parsePacket()) == 48)
            break;
        delay(pollIntv);
    }
    if (pktLen != 48) {
        Serial.println("failed; invalid ntp packet received");
        return 0;                // no correct packet received
    }
    // Read and discard the first useless bytes
    // Set useless to 32 for speed; set to 40 for accuracy.
    const byte useless = 40;
    for (byte i = 0; i < useless; ++i)
        udp.read();

    // Read the integer part of sending time
    unsigned long time = udp.read();    // NTP time
    for (byte i = 1; i < 4; i++)
        time = time << 8 | udp.read();

    // Round to the nearest second if we want accuracy
    // The fractionary part is the next byte divided by 256: if it is
    // greater than 500ms we round to the next second; we also account
    // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
    // additionally, we account for how much we delayed reading the packet
    // since its arrival, which we assume on average to be pollIntv/2.
    time += (udp.read() > 115 - pollIntv / 8);

    // Discard the rest of the packet
    udp.flush();

    return time - 2208988800ul;        // convert NTP time to Unix time
}


TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420};
TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480};
Timezone usPT(usPDT, usPST);

boolean UpdateRTCDateTime() {
    ledDisplay_t rgb = {0, 0, 255};
    LEDOn(rgb);

    Serial.print("fetching ntp time...");
    unsigned long unixTime = ntpUnixTime(udp);
    if (unixTime == 0) {
        return false;
    }
    Serial.println("done");

    // Parse utc timestamp into time elements and print it
    tmElements_t utcElements;
    breakTime(unixTime, utcElements);
    Serial.print("UTC: ");
    printTimeElements(utcElements);

    // Parse utc timestamp and convert it to local time
    time_t local = usPT.toLocal(unixTime);
    tmElements_t ptElements;
    breakTime(local, ptElements);
    Serial.print("PT: ");
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
    if (displayState == 3) {
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
        targetDigits.Dots = 0;
        targetDigits.Colon = 0;
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

    DisplayNixieTubeDateTime(displayState, _currentDigits);
}


/******************************************************************************/


int _blinks = LED_BLINK_DEFAULT;
int _displayState = 2; // 0: off, 1: time only, 2: time & date
int _successState = 2; // 0: success, 1: fail, 2: idle
tmElements_t _tm;

int nextDisplay(int displayState) {
    switch (displayState) {
        default:
        case (0): // Currently Off
            return 2;
        case (2):  // Currently Time & Date
            return  1;
        case (1): // Currently Time Only
            return 0;  // returning 3 here enables the rolling digit
        case (3):  // Currently Rolling
            return  0;
    }
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
        ledDisplay_t rgb = {0, 255, 0};
        _blinks = LEDBlink(_blinks, rgb);
    } else if (_successState == 1) {
        // Bright red; Failed time sync
        ledDisplay_t rgb = {255, 0, 0};
        _blinks = LEDBlink(_blinks, rgb);
    } else {
        // Standby
        ledDisplay_t rgb = getLEDColor(_displayState);
        LEDOn(rgb);
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
                // Sync time from ntp source
                _successState = UpdateRTCDateTime() ? 0 : 1;
                goto beginning;
            }
            sync = (boolean) digitalRead(BUTTON_DIO);
        }
        _displayState = nextDisplay(_displayState);
    }
    UpdateNixieTubeDateTime(_displayState, _tm);
}
