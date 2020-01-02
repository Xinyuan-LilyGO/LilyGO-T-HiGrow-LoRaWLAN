/*
This example shows how to use LilyGO T-HiGrow- Lora SHIELD to connect to TTN and display the data in Cayenne.
2019/12/31 lewis he
*/

#include <BH1750.h>
#include <Adafruit_BME280.h>
#include <OneButton.h>
#include <DHT12.h>
#include <CayenneLPP.h>
#include "lmic.h"
#include <hal/hal.h>

#define I2C_SDA             25
#define I2C_SCL             26
#define DHT12_PIN           16
#define BAT_ADC             33
#define SALT_PIN            34
#define SOIL_PIN            32
#define BOOT_PIN            0
#define POWER_CTRL          4
#define USER_BUTTON         35
#define DS18B20_PIN         21                  //18b20 data pin

#define RADIO_RESET         23
#define RADIO_DIO_0         14
#define RADIO_DIO_1         13
#define RADIO_DIO_2         15
#define RADIO_CS            18
#define RADIO_MISO          19
#define RADIO_MOSI          27
#define RADIO_CLK           5

// Chose LSB mode on the console and then copy it here.
static const u1_t PROGMEM APPEUI[8] = { 0x31, 0x7A, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
// LSB mode
static const u1_t PROGMEM DEVEUI[8] = { 0x06, 0xEB, 0x81, 0xE4, 0xF0, 0xAA, 0xD0, 0x00 };
// MSB mode
static const u1_t PROGMEM APPKEY[16] = { 0xD4, 0xDF, 0x61, 0xFA, 0x66, 0x77, 0xB5, 0x86, 0x79, 0x52, 0xF7, 0x34, 0x07, 0x76, 0x67, 0x22 };




BH1750 lightMeter(0x23); //0x23
Adafruit_BME280 bme;     //0x77
DHT12 dht12((int)DHT12_PIN, true);
OneButton button(USER_BUTTON, true);
CayenneLPP lpp(160);

static osjob_t sendjob;
String lora_msg = "";
int spreadFactor = DR_SF7;
int joinStatus = EV_JOINING;

bool lightSensorDetected = true;
bool bmeSensorDetected = true;
bool buttonClick = false;

const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RADIO_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RADIO_RESET,
    .dio = {RADIO_DIO_0, RADIO_DIO_1, RADIO_DIO_2}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void os_getArtEui (u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t *buf)
{
    memcpy_P(buf, DEVEUI, 8);
}

void os_getDevKey (u1_t *buf)
{
    memcpy_P(buf, APPKEY, 16);
}


uint32_t readSalt()
{
    uint8_t samples = 120;
    uint32_t humi = 0;
    uint16_t array[120];

    for (int i = 0; i < samples; i++) {
        array[i] = analogRead(SALT_PIN);
        delay(2);
    }
    std::sort(array, array + samples);
    for (int i = 0; i < samples; i++) {
        if (i == 0 || i == samples - 1)continue;
        humi += array[i];
    }
    humi /= samples - 2;
    return humi;
}

uint16_t readSoil()
{
    uint16_t soil = analogRead(SOIL_PIN);
    return map(soil, 0, 4095, 100, 0);
}

float readBattery()
{
    int vref = 1100;
    uint16_t volt = analogRead(BAT_ADC);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}

void do_send(osjob_t *j)
{
    if (joinStatus == EV_JOINING) {
        Serial.println(F("Not joined yet"));
        // Check if there is not a current TX/RX job running
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

    } else if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        lpp.reset();

        float t12 = dht12.readTemperature();
        if (!isnan(t12)) {
            lpp.addTemperature(1, t12);
        }

        float h12 = dht12.readHumidity();
        if (!isnan(h12)) {
            lpp.addRelativeHumidity(2, h12);
        }

        if (lightSensorDetected) {
            float lux = lightMeter.readLightLevel();
            lpp.addLuminosity(3, lux);
        }

        float mv = readBattery();
        lpp.addAnalogInput(4, mv / 1000.0);


        uint16_t soil = readSoil();
        lpp.addAnalogInput(5, soil);


        uint16_t salt = readSalt();
        lpp.addAnalogInput(6, salt);


        // if (bmeSensorDetected) {
        //     float bmeTemperature = bme.readTemperature();
        //     float bmePressure = (bme.readPressure() / 100.0F);
        //     float bmeHumidity = bme.readHumidity();
        //     lpp.addTemperature(7, bmeTemperature);
        //     lpp.addBarometricPressure(8,  bmePressure);
        //     lpp.addRelativeHumidity(9, bmeHumidity);
        // }
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    }
}


void onEvent (ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println(F("Received ack"));
            lora_msg =  "Received ACK.";
        }

        lora_msg = "rssi:" + String(LMIC.rssi) + " snr: " + String(LMIC.snr);

        if (LMIC.dataLen) {
            // char TTN_response[256];
            // int i = 0;
            // data received in rx slot after tx
            Serial.print(F("Data Received: "));
            // Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            // Serial.println();
            // Serial.println(LMIC.rssi);
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING: -> Joining..."));
        lora_msg = "OTAA joining....";
        joinStatus = EV_JOINING;
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
        lora_msg = "OTAA Joining failed";
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        lora_msg = "Joined!";
        joinStatus = EV_JOINED;
        delay(3);
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);

        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}


void loarSetup()
{
    SPI.begin(RADIO_CLK, RADIO_MISO, RADIO_MOSI, RADIO_CS);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(spreadFactor, 14);

    // Start job
    LMIC_startJoining();


    do_send(&sendjob);     // Will fire up also the join
}


void setup()
{
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);

    //! Sensor power control pin , use deteced must set high
    pinMode(POWER_CTRL, OUTPUT);
    digitalWrite(POWER_CTRL, HIGH);
    delay(200);

    dht12.begin();

    if (!bme.begin()) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        bmeSensorDetected = false;
    }

    if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println(F("Error initialising BH1750"));
        lightSensorDetected = false;
    }

    loarSetup();

    button.attachLongPressStart([] {
        // no use
    });
    button.attachLongPressStop([] {
        // no use
    });
}


void loop()
{
    os_runloop_once();
}
