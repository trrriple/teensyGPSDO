#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>


#include "extIntFreqCount.h"

// #include <DAC855X.h>

// DAC855X dac;

#include "DAC8550.h"


DAC8550 dac(10);


#define GOAL_FREQ 10000000.0
#define PPS_PIN 7
#define GpsSerial Serial1
#define DAC_VREF 5.0
#define DAC_COUNTS 65536
#define DAC_ZERO_VAL (DAC_VREF / 2)


static const float K_DAC_LSB = DAC_VREF / DAC_COUNTS;

static uint32_t freqTenBuf[10];
static uint32_t freqHunBuf[100];
static uint32_t freqThouBuf[1000];

static uint8_t curRollTenPos  = 0;
static uint8_t curRollHunBuf  = 0;
static uint8_t curRollThouBuf = 0;

static bool freqTenBufFull  = 0;
static bool freqTenHunFull  = 0;
static bool freqTenThouFull = 0;

static double freqTenBufAvg = 0.0;


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
SFE_UBLOX_GNSS gps;


static int _getDacVal(float desiredV)
{
    return (desiredV - DAC_ZERO_VAL) / K_DAC_LSB;
}

void setup()
{
    Serial.begin(115200);

    lcd.init(); // initialize the lcd
    Wire.setClock(1000000);
    lcd.backlight();
    lcd.print("Waiting for PPS");

    // dac.setPins(11, 13, 10);
    // dac.initializeDAC855X();
    dac.begin();

    
    ExtIntFreqCount.begin(PPS_PIN, FALLING); /* pin to trigger cycle capture */

    // Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
    // Loop until we're in sync and then ensure it's at 38400 baud.
    do
    {
        Serial.println("gps: trying 460800 baud");
        GpsSerial.begin(460800);
        if (gps.begin(GpsSerial) == true)
            break;

        delay(100);
        Serial.println("gps: trying 9600 baud");
        GpsSerial.begin(9600);
        if (gps.begin(GpsSerial) == true)
        {
            Serial.println("gps: connected at 9600 baud, switching to 38400");
            gps.setSerialRate(460800);
            delay(100);
        }
        else
        {
            // gps.factoryReset();
            delay(2000); // Wait a bit before trying again to limit the Serial output
        }
    } while (1);

    Serial.println("gps serial connected");

}

static void _updateLCD()
{
    lcd.clear();
    lcd.print("f(10): ");
    lcd.print(freqTenBufAvg, 4);
    Serial.println(freqTenBufAvg, 4);

    uint32_t tow_ms = gps.getTimeOfWeek();
    float tow_s = (float)tow_ms / 1000.0;
    lcd.setCursor(0, 1);
    lcd.print(F("TOW: "));
    lcd.print(tow_s);

    Serial.print(F("TOW: "));
    Serial.print(tow_s);

    byte siv = gps.getSIV();
    lcd.setCursor(0, 2);
    lcd.print(F("SIV: "));
    lcd.print(siv);

    Serial.print(F(" SIV: "));
    Serial.print(siv);

    Serial.println();
}

static void _calcAvgs()
{
    if(freqTenBufFull)
    {   
        freqTenBufAvg = 0;
        for(int i = 0; i < 10; i++)
        {
            freqTenBufAvg += freqTenBuf[i];
        }
        freqTenBufAvg /= 10.0;
    }
}


static void _tuneOsc()
{
    static float targetVoltage = 2.0; /* Middle of OXCO 4.0 V tuning range */
    /* Manipulate the DAC to hone in on correct frequency */

    double curError = GOAL_FREQ - freqTenBufAvg;


    float adjVal = 0;
    /* precision steering */
    if(abs(curError) < .2)
    {
        adjVal = .001;
    }
    /* fine steering */
    else if(abs(curError) < 1)
    {
        adjVal = .01;
    }
    /* coarse steering */
    else
    {
        adjVal = .1;
    }

    /* increase case */
    if(curError > 0)
    { 
        targetVoltage += adjVal;
        Serial.print("DAC target voltage: ");
        Serial.print(targetVoltage, 5);
        Serial.print(" (");
        Serial.print(adjVal, 5);
        Serial.println(")");
    }
    /* decrease case */
    else
    {
        targetVoltage -= adjVal;
        Serial.print("DAC target voltage: ");
        Serial.print(targetVoltage, 5);
        Serial.print(" (");
        Serial.print(-adjVal, 5);
        Serial.println(")");
    }

    int targetVoltageRaw = _getDacVal(targetVoltage);
    dac.setValue(targetVoltageRaw);
}

/* The PPS will set a flag which latches the current frequncy count, we 
   essentially loop on this value and check the count when it is valid */
static void _ppsHandler()
{
    if(ExtIntFreqCount.available())
    {
        uint32_t count = ExtIntFreqCount.read();

        freqTenBuf[curRollTenPos] = count;

        if(curRollTenPos < 10)
        {
            curRollTenPos++;
        }
        else
        {
            freqTenBufFull = true; /* After the first 10, always true */
            curRollTenPos = 0;
            _tuneOsc();           
        }


        /* Update LCD */
        if(freqTenBufFull)
        {
            _calcAvgs();
            _updateLCD();
        }
    }

}

void loop()
{
    _ppsHandler();
}