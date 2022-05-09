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

#define GpsSerial Serial1
#define CountOutSerial Serial5

#define GOAL_FREQ 10000000.000
#define PPS_PIN 7
#define DAC_VREF 5.0
#define DAC_COUNTS 65536
#define DAC_ZERO_VAL (DAC_VREF / 2)

#define LCD_UPD_PERIOD_MS 1000 /* approx 60 hz */

static const float K_DAC_LSB = DAC_VREF / DAC_COUNTS;

static uint32_t g_freqTenBuf[10];
static uint32_t g_freqHunBuf[100];
static uint32_t g_freqThouBuf[1000];

static uint8_t g_curRollTenPos  = 0;
static uint8_t g_curRollHunPos  = 0;
static uint8_t g_curRollThouPos = 0;

static bool g_freqTenBufFull  = 0;
static bool g_freqHunBufFull  = 0;
static bool g_freqThouBufFull = 0;

static double g_freqTenBufAvg = 0.0;
static double g_freqHunBufAvg = 0.0;

static uint32_t g_gpsTow_ms = 0;
static uint8_t  g_gpsSiv    = 0;


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
SFE_UBLOX_GNSS gps;


static int _getDacVal(float desiredV)
{
    return (desiredV - DAC_ZERO_VAL) / K_DAC_LSB;
}

void setup()
{
    Serial.begin(115200);
    CountOutSerial.begin(115200);

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
    lcd.print(g_freqTenBufAvg, 1);

    lcd.setCursor(0, 1);
    lcd.print("f(100):");
    lcd.print(g_freqHunBufAvg, 2);

    float tow_s = (float)g_gpsTow_ms / 1000.0;
    lcd.setCursor(0, 2);
    lcd.print(F("TOW:   "));
    lcd.print(tow_s);

    lcd.setCursor(0, 3);
    lcd.print(F("SIV:   "));
    lcd.print(g_gpsSiv);

}

static void _printLog()
{
    char buf[256];
    
    float tow_s = (float)g_gpsTow_ms / 1000.0;
    uint32_t count = ExtIntFreqCount.read();

    snprintf(buf, 256, "TOW: %f, SIV: %u, Cnt: %lu, Cnt(10): %.01f, " 
             "Cnt(100): %.02f\n",
             tow_s, g_gpsSiv, count, g_freqTenBufAvg, g_freqHunBufAvg);
    
    Serial.print(buf);
}

static void _calcAvgs()
{
    if(g_freqTenBufFull)
    {   
        g_freqTenBufAvg = 0;
        for(int i = 0; i < 10; i++)
        {
            g_freqTenBufAvg += g_freqTenBuf[i];
        }
        g_freqTenBufAvg /= 10.0;
    }
    if(g_freqHunBufFull)
    {
        g_freqHunBufAvg = 0;
        for (int i = 0; i < 100; i++)
        {
            g_freqHunBufAvg += g_freqHunBuf[i];
        }
        g_freqHunBufAvg /= 100.0;
    }
}

static double _tuneOscShort(double error)
{

    double adjVal = 0;
    /* precision steering */
    if (abs(error) <= .2)
    {
        adjVal = .001;
    }
    /* fine steering */
    else if (abs(error) <= 1)
    {
        adjVal = .01;
    }
    /* coarse steering */
    else
    {
        adjVal = .1;
    }

    return adjVal;
}

static double _tuneOscLong(double error)
{

    double adjVal = 0;
    /* ultra precision steering */
    if(abs(error) == 0)
    {
        adjVal = 0;
    }
    else if (abs(error) <= .02)
    {
        adjVal = .0001;
    }
    /* precision steering */
    else if (abs(error) <= .05)
    {
        adjVal = .0005;
    }
    /* fine steering */
    else if (abs(error) <= .1)
    {
        adjVal = .001;
    }
    /* coarse steering */
    else
    {
        adjVal = .01;
    }

    return adjVal;
}


static void _tuneOsc()
{
    static float targetVoltage = 2.0; /* Middle of OXCO 4.0 V tuning range */
    static int cycleCount = 0;

    /* Manipulate the DAC to hone in on correct frequency */
    double adjVal = 0;
    double curError = GOAL_FREQ - g_freqTenBufAvg;
    if (g_freqHunBufFull && abs(curError) < 0.5)
    {
        if (cycleCount >= 99)
        {
            curError = GOAL_FREQ - g_freqHunBufAvg;
            Serial.print("Long Tune. Error: ");
            Serial.println(curError);
            adjVal = _tuneOscLong(curError);
            cycleCount = 0;
        }
    }
    else if (g_freqTenBufFull)
    {
        if (cycleCount >= 10)
        {

            Serial.print("Short Tune. Error: ");
            Serial.println(curError);

            adjVal = _tuneOscShort(curError);
            cycleCount = 0;
        }
    }
    cycleCount++;

    /* increase case */
    if (adjVal != 0)
    {

        if (curError > 0)
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

}

/* The PPS will set a flag which latches the current frequncy count, we 
   essentially loop on this value and check the count when it is valid */
static void _ppsHandler()
{
    static bool skipFirst = true;

    if(ExtIntFreqCount.available())
    {
        /* First pps will not have an accurate amount of pulse counts */
        if (skipFirst)
        {
            skipFirst = false;
            return;
        }

        uint32_t count = ExtIntFreqCount.read();

        if(count > GOAL_FREQ + 20)
        {
            Serial.print("Got Bad count: ");
            Serial.println(count);
            count = GOAL_FREQ;
        }
        
        CountOutSerial.println(count);

        g_freqTenBuf[g_curRollTenPos] = count;
        g_freqHunBuf[g_curRollHunPos] = count;

        if(g_curRollTenPos < 10)
        {
            g_curRollTenPos++;
        }
        else
        {
            g_freqTenBufFull = true; /* After the first 10, always true */
            g_curRollTenPos = 0;
        }

        if(g_curRollHunPos < 99)
        {
            g_curRollHunPos++;
        }
        else
        {
            g_freqHunBufFull = true; /* after first 100, always true */
            g_curRollHunPos = 0;
        }

        /* Update averages */
        if(g_freqTenBufFull)
        {
            _calcAvgs();
        }

        _tuneOsc();  

        g_gpsTow_ms = gps.getTimeOfWeek();
        g_gpsSiv = gps.getSIV();
        _printLog();
    }

}

void loop()
{
    static uint32_t tLastLcdUpd_ms = 0;
    uint32_t tNow_ms = millis();
    _ppsHandler();
    
    if(tNow_ms - tLastLcdUpd_ms > LCD_UPD_PERIOD_MS)
    {
        _updateLCD();
        tLastLcdUpd_ms = tNow_ms;
    }

}