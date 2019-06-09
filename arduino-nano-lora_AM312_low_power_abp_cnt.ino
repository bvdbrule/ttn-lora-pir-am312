/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example will send Battery voltage and movement
 * using frequency and encryption settings matching those of
 * the The Things Network. Application will 'sleep' 1x300 seconds (300 seconds)
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

/*******************************************************************************
 * Add som code for reading PIR AM312 senser
 * Bjorn van den Brule (Code73) 
 * Device : Arduino Nano (5V)
 * Program with Old Bootloader

 * Network : TTN
 * Device registration : ABP
 * Sensor  : AM312
 * Port interface code(s) : 
 *   20 - Battery
 *   25 - PIR activated
 *   32 - PIR armed
 *   
 *******************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <Arduino.h>

#define C_TIMEOUT 5  // timeout, dont look for AM312, for interference from lora module
#define C_SLEEPCYCLES 300
int sleepcycles = C_SLEEPCYCLES;  // every sleepcycle will last 1 secs, total sleeptime will be sleepcycles * 1 sec
bool joined = false;
bool sleeping = false;

int LED = 13;  // Works only at startup 

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000; // <-- Change this address for every node!
// arduino-nano-rfm95_6

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping for custom print CO_AM312
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {7, 8, 9},
};


int send_msg = 20;
// 20 = battery status
// 21 = temperature
// 22 = humidity
// 23 = co

bool intr_motion = false;
bool intr_state = LOW;
bool pir_armed = true;
uint8_t intr_cnt = 0;
uint8_t intr_cnt_to = 0;


#define PinAM312    2     // Pin D2  PIR sensor


long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADcdMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
 
}

float getBatVoltage() {
  float measuredvbat = readVcc()/1000.0f;
 
//Serial.print("VBat: " ); Serial.println(measuredvbat);
  return measuredvbat;
}

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
       Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        pir_armed = LMIC.frame[LMIC.dataBeg];
//        if (!pir_armed)
//        { 
//          sleepcycles = C_SLEEPCYCLES; // every sleepcycle will last 1 secs, total sleeptime will be sleepcycles * 1 sec
//        }
//        else
//        {
//          sleepcycles = C_SLEEPCYCLES * 2;  // every sleepcycle will last 1 secs, total sleeptime will be sleepcycles * 1 sec
//        }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
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


void do_send(osjob_t* j, int intr_level){
  byte buffer[22];
  float fval;
  int ival;
  // It is a bad idea to send ASCII for real, only use this when developing!!
  buffer[0]='H';
  buffer[1]='i';
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.

    if (intr_level == 1) // interrupt 
    {
      buffer[0] = pir_armed;
      buffer[1] = intr_cnt;
      LMIC_setTxData2(25, (uint8_t*) buffer, 2 , 0);
//    Serial.print(F("Sending: ")); Serial.println(25);
    }
    else {      // no interrupt, normal sensors, time based

      switch (send_msg) {
        case 20:
           fval = getBatVoltage();
           ival = (int)(fval*100);
           buffer[0] = (uint8_t)highByte(ival);
           buffer[1] = (uint8_t)lowByte(ival);
        break;
        case 21:
           send_msg = send_msg+11;        
        case 32:
           buffer[0] = pir_armed;
           buffer[1] = intr_cnt;
        break;
        
        default:
        break;
      } // switch

      LMIC_setTxData2(send_msg, (uint8_t*) buffer, 2 , 0);
//    Serial.print(F("Sending: ")); Serial.println(send_msg);    
      send_msg++;
      if (send_msg >= 33) { send_msg = 20; }
   
    } // else intr_level 
  } // else
} // end do_send function




// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}


void setup()
  {
  // if LED is connected to pin 10, it has to be defined before any SPI initialization else
  // it will be used as SS (Slave Select) and controlled by the SPI module
    pinMode(LED, OUTPUT);

    Serial.begin(115200);
    delay(250);
    Serial.println(F("Starting"));

    digitalWrite(LED, HIGH);  // LED 
    delay(100);
    digitalWrite(LED, LOW);  
    
    pinMode(PinAM312, INPUT);
     
    // LMIC init
    
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
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
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // relax clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);

}


//unsigned long time;
void loop() {
  

   if ((intr_motion) && (intr_state == LOW)) // motion detected
   {
     intr_state = HIGH;
     intr_motion = false;
     intr_cnt_to = C_TIMEOUT;   // start timeout
     Serial.println(F("Sensor Up - Send to TTN"));
     do_send(&sendjob, 1);   // First time Sent intr directly
   }
   else if ((intr_motion) && (intr_state == HIGH)) 
   {
    intr_motion = false;
    Serial.println(F("Sensor Up - Still High"));
    do_send(&sendjob, 0);    // Sent sensor values
   }
   else
   {
     Serial.println(F("Send values to TTN, after timeout"));
     do_send(&sendjob, 0);    // Sent sensor values
   }
 
   //do_send(&sendjob, 0);    // Sent sensor values
   while(sleeping == false)
   {
     os_runloop_once();  // always use do_send() for os_runloop_once()
   }
   sleeping = false;


   // Sleepy time loop
   // Break out only for a first time interrupt or timeout
   for (int i=0;i<sleepcycles;i++)
   {
      
       LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);    //sleep 1 second

       if ((digitalRead(PinAM312) == HIGH) && (intr_cnt_to == 0))// if first motion is detected then stop sleeping
       { 
         intr_motion = true;
         digitalWrite(LED, HIGH);
         Serial.println(F("Sensor Up, break loop"));
         Serial.flush();
         break;
       } //fi

       if (intr_state == HIGH)  // Interrupt High
       {
         if (digitalRead(PinAM312) == LOW) // wait for input signal is low
         {
           intr_state = LOW;   // no motion anymore
           intr_cnt = 0;       // reset counter
           digitalWrite(LED, LOW);
           Serial.println(F("Sensor Down")); 
           Serial.flush();
//           break;
         } //fi
      } //fi
      Serial.print(F("Cnt:"));
      Serial.println(i);
      Serial.flush();

      if (intr_cnt_to > 0) intr_cnt_to--;
            
   } // for
   Serial.println(F("End loop"));
   Serial.flush();
}

