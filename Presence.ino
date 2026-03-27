//
// This ESP32 ARDUINO program is a Zibgee end device that will interact with a solenoid chime door bell. It allows the door bell
// button presses to become Zibgee binary sensors and allows playing of the bells via relays using inputs from zigbee.
// It uses the 12v-24v AC power of the normal door bell.
//
// HARDWARE:
//
// On an ESP32-C6 we have a factory reset button, inputs for two door bell buttons, and outputs for two relays which drive the
// door bell chimes. So you need a 12-24AC to 5v DC converter to drive it. The output of the AC to DC converter powers the 5V
// input and ground of the Esp board. The raw AC is however is fed into to one side of the bell solenoids while the other 
// side is switched by the normally open side of the switches. The switches are driven by the 3.3v output pins from the ESP32.
// As a result the ESP32 can detect when door bells buttons are pressed and can also trigger the bells in any patter in wants by
// simply setting the proper output pins that drive the switches. The exact timeing and sequence of course depends on the physical
// characterastics of the solenoids and experimentation is required to get the proper duraction of the 'true' output to get a good
// hard strike without undue buzzing.
//
// BUILD NOTES:
//          I built this on a Mac and had problems with the USB driver. Waveshare has a nice page describing how to put a new
//          driver on your Mac which worked perfectly. Without it one of my boards refused to load the code via Arduino but 
//          surprise a bunch of other boards worked just fine. Anyway if you get CRC errors downlaoding, try lower speeds and if that
//          fails pop over to Waveshare and lookup the USB driver problem.
//
//          ARDUINO IDE TOOLS SETTINGS:
//          You need to set a number of settings in the Arduino IDE/Tools menu for this to work properly.
//              1 - Tools/USB CDC on boot - enabled (allows serial IO for debugging).
//              2 - Tools/Core debug level (set as desired useful for debugging zibbee attach etc.) start verbose.
//              3 - Tools/Erase all flash before upload - this means each download its a brand new Zibgee end point.
//                  Id erase for first few downloads and always delete/reattach but after its working don't erease the
//                  flash each time. Once its working you can erase the flash and start scratch with a long press on the
//                  reset button anyway.
//              3 - Tools/partition scheme: 4MB with spiffs - seems to be what the Zibgee library wants.
//              4 - Tools/zibgee mode ED (end device) - you can also use the end mode with debug enabled for more tracing.
//
// SOFTWARE:
//
// The software has three interrupt handlers. The first handles the factory reset button which erases all the zibbee data so
// that rebinding is required. The second and third handlers will fire when one of the door bell buttons is pressed. 
// These handlers are a bit special because if the zigee connection is not up we don't want to ignore a door bell press so we
// simply pass the state of the button through to the switch. This in insures that if zibgee goes down or does not connect the
// door bells function with a single strike per button press each. 
//
// The setup() function of course configures zigbee clusters and sets all the attributes correctly then attaches to the 
// zigbee network. Once the network is up we enter the main loop().
//
// The main loop listens either for signs of button presses by the interrupt handlers, or from the HA binary switches. In 
// Either case it looks up the proper tones and repetitions and ask the relays to play that pattern. After they are finished
// it resets all the zibgee attributes. As a result zibgee will show when an external button is pressed so that can be used as
// a trigger for other things, and it also allows playing from zibgee. In addition two the two manual buttons I also provide a
// Z button which only can be triggered by Zibgee. This allows automations/notifications etc. to set a tone, repetition and 
// then request it be played. 
//
// There is a watch dog timers that is fed in the main loop and a simple blue flashing led when trying to bind to zibeee and
// a green flashing led when its fully bound.
//
// Since the solenoids/mechanical buttons can be quite noisy we need do quite a bit of debouncing. A few tricks are employed 
// here. First we look for enough '1's so we read a bit of the signal to ensure its steady enough to warrant an event.
// Next we completely ignore the interrupts if the solenoids are busy. Some external hardware would be useful here with some
// capacitors/diods and perhaps Schmidt triggers but this is simpler albeit a bit ugly.
//
// For debugging purposes we store a number of attributes in non volatile store (such as reboot reasons etc) and display them
// as clusters for debugging.
//
#include <Arduino.h>
#include <s3km1110.h>
#include <esp_task_wdt.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
#include "Zigbee.h"
#include "esp_log.h"

//
// This is the object that interfaces with the radar1 via Serial1 and
// radar2 via Serial2.
//
s3km1110 radar1;
s3km1110 radar2;

//
// Decide which or both radars to use.
//
const bool RADAR1_ENABLED = true;
const bool RADAR2_ENABLED = true;
//
// Hardware Pin configurations.
//
const int isr_resetButtonPin = 18;                      // Causes a factory reset by erasing all NVS

//
// Output unitless count app type missing so define it.
//
#define ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT  ESP_ZB_ZCL_AO_SET_APP_TYPE_WITH_ID( ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS, 0x0000)
//
// Debugging stuff, simple macro to log debug for us.
//
static const char *TAG = "zPres"; 
#define DPRINTF(format, ...)  ESP_LOGD(TAG, format, ##__VA_ARGS__) 

//
// Set 1 and you'll get lots of useful info as it runs. For debugging the lower layer Zibgee see the tools settings
// in the Arduino menu for use with the debug enabled library and debug levels in that core. We can also compile in/out 
// the watch dog timers. 
//
const bool debug_g = true;
const bool wdt_g   = true;

// 
// Non volatile storage for debugging. When we restart etc. we will write the reasons 
// and track last uptime etc. for display via a Zigbee debug cluster sensor.
//
const char       *ha_nvs_name = "_ZPRES_";                 // Unique name for our partition
const char       *ha_nvs_vname= "_vars_";                  // name for our packeed variables
nvs_handle_t      ha_nvs_handle = 0;                       // Once open this is read/write to NVS
uint32_t          ha_nvs_last_uptime = 0;                  // minutes we were up last time before reboot
uint32_t          ha_nvs_last_reboot_reason = 0;           // why we rebooted last time. (0 factory reset)
uint32_t          ha_nvs_last_reboot_count = 0;            // increase each reboot except factory reset

//
// We are looking for persistant values of the last reboot reason and last uptime. We store these two packed
// into a single Uint32 which we depack after reading from the NVS.
//
void ha_nvs_read()
{    
     ha_nvs_last_reboot_reason = 0;
     ha_nvs_last_uptime        = 0;
     ha_nvs_last_reboot_count  = 0;
     //
     esp_err_t err = nvs_flash_init();
     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        nvs_flash_erase();
        nvs_flash_init();
        if (debug_g) DPRINTF("ha_nvs_read - nvs_flash_init\n", esp_err_to_name(err));
     }
     err = nvs_open(ha_nvs_name, NVS_READWRITE, &ha_nvs_handle);
     if (err != ESP_OK) {
        if (debug_g) DPRINTF("ha_nvs_read - Error (%s) opening NVS name %s!\n", esp_err_to_name(err), ha_nvs_name);
        return;
     }  
     //
     uint32_t vars;
     err = nvs_get_u32(ha_nvs_handle, ha_nvs_vname, &vars);
     if (err != ESP_OK) {
          if (debug_g) DPRINTF("ha_nvs_read - cant get variable name %s\n", ha_nvs_name);
          return;
     }
     ha_nvs_last_reboot_reason  = vars         & 0xff;
     ha_nvs_last_reboot_count   = (vars >> 8)  & 0xff;
     ha_nvs_last_uptime         = (vars >> 16) & 0xffff;
     ha_nvs_last_reboot_reason += esp_reset_reason() * 1000;  // See below for why * 1000
     /* 
      * For convenient reference. We multiply these by 1000 to we can see the 
      * ESPs idea why it rebooted together with our own reboot reason as a single
      * number displayed as a Zigbee cluster. This is taken from the enum so they 
      * start at 0,1,2... 
      *
      * ESP_RST_UNKNOWN,    //!< Reset reason can not be determined
      * ESP_RST_POWERON,    //!< Reset due to power-on event
      * ESP_RST_EXT,        //!< Reset by external pin (not applicable for ESP32)
      * ESP_RST_SW,         //!< Software reset via esp_restart
      * ESP_RST_PANIC,      //!< Software reset due to exception/panic
      * ESP_RST_INT_WDT,    //!< Reset (software or hardware) due to interrupt watchdog
      * ESP_RST_TASK_WDT,   //!< Reset due to task watchdog
      * ESP_RST_WDT,        //!< Reset due to other watchdogs
      * ESP_RST_DEEPSLEEP,  //!< Reset after exiting deep sleep mode
      * ESP_RST_BROWNOUT,   //!< Brownout reset (software or hardware)
      * ESP_RST_SDIO,       //!< Reset over SDIO
      * ESP_RST_USB,        //!< Reset by USB peripheral
      * ESP_RST_JTAG,       //!< Reset by JTAG
      * ESP_RST_EFUSE,      //!< Reset due to efuse error
      * ESP_RST_PWR_GLITCH, //!< Reset due to power glitch detected
      * ESP_RST_CPU_LOCKUP, //!< Reset due to CPU lock up (double exception)
      */
     if (debug_g) {
          DPRINTF("ha_nvs_read got vars=%x, reason %d, count %d, uptime=%d\n", vars,
               ha_nvs_last_reboot_reason, ha_nvs_last_reboot_count, ha_nvs_last_uptime);
     }
}

//
// And here is the write to NVS of the attributes after they have been changed and sent to the Heat Pump
//
void ha_nvs_write(uint32_t reason = 0, uint32_t uptime = 0)
{
     ha_nvs_last_reboot_count = (ha_nvs_last_reboot_count + 1) & 0xff;
     reason &= 0xff;
     uptime &= 0x0000fffff;
     uint32_t vars  = reason | (ha_nvs_last_reboot_count << 8) | (uptime << 16);
     if (debug_g) {
          DPRINTF("ha_nvs_write got vars=%x, reason %d, count %d, uptime=%d\n", vars, reason, ha_nvs_last_reboot_count, uptime);
     }
     esp_err_t err = nvs_set_u32(ha_nvs_handle, ha_nvs_vname, vars);
     if (err != ESP_OK) {
         if (debug_g) DPRINTF("ha_nvs_write  %s can't write, because %s\n", ha_nvs_vname, esp_err_to_name(err));
         return;
     }
     err = nvs_commit(ha_nvs_handle);
     if (err != ESP_OK) {
         if (debug_g) DPRINTF("ha_nvs_write %s can't commit, because %s\n", ha_nvs_vname, esp_err_to_name(err));
     }  
}
// 
// Function complete shutdown and restart. Forward declared also a flash sequence for factory reset.
//
extern void ha_restart(uint32_t reason, uint32_t uptime); 
extern void rgb_led_set_factory_reset();

//
// Interrupt handler for Reset button. If its pressed we do full factory reset. Normal reset is just done with a power off/on.
// We just look to see if we are getting a bunch of lows on the reset pin and if so we reset otherwise just ignore it as we can
// get spurious interrupts when the solenoids activate.
//
void isr_resetButtonPress()      
{    
     int n = 1;
     for(int i = 0; i < 50; i++) {
         n += (digitalRead(isr_resetButtonPin) == 0) ? 1 : 0;
     }
     if (n < 40) return;                                // if its too much like noise ignore it. 
     //
     rgb_led_set_factory_reset();                       // Go white so its obvious
     Zigbee.factoryReset(false);                        // This should do the same but not sure it does anyway ...
     ha_restart(0, 0);                                  // And stop all the Zigbee stuff and just restart the ESP
}

//
// Control of the color LED for various purposes. We can set a few different colors by RGB and also
// can vary the brightness.
//
const uint8_t RGB_LED_OFF    = 0;        // Enums are causing compiler problems when passed as first argument.
const uint8_t RGB_LED_WHITE  = 1;        // so back to old school.
const uint8_t RGB_LED_RED    = 2;
const uint8_t RGB_LED_GREEN  = 3;
const uint8_t RGB_LED_BLUE   = 4;
const uint8_t RGB_LED_ORANGE = 5;
const uint8_t rbg_max        = RGB_BRIGHTNESS;        // Fairly dim or they keep people awake
const uint8_t RGB_MIN        = 0;
#define       RGB_ORDER        LED_COLOR_ORDER_RGB    // Compiler problems passing enums, have to be explicit
//
void rgb_led_set(uint32_t color, uint32_t brightness = 10) {
     if (brightness > 10) brightness = 10;
     int rbg_max = (RGB_BRIGHTNESS * brightness) / 10;
     switch(color) {                                                      //RED     GREEN     BLUE
         case RGB_LED_GREEN : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, rbg_max,  RGB_MIN); break;
         case RGB_LED_WHITE : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, rbg_max, rbg_max,  rbg_max); break;
         case RGB_LED_RED   : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, rbg_max, RGB_MIN,  RGB_MIN); break;          
         case RGB_LED_BLUE  : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MIN,  rbg_max); break;
         case RGB_LED_ORANGE: rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, rbg_max, rbg_max/2,RGB_MIN); break;
         case RGB_LED_OFF   : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MIN,  RGB_MIN); break;
     }
}

//
//   Simple LED flash routine, should really use a background task to do this.. tbd. Flash the chosen color and
//   then return to the restore color after.
//
void rgb_led_flash(int color, int restore_color)
{
     for(int i = 0; i < 5; i++) {
        rgb_led_set(color);
        delay(50);
        rgb_led_set(RGB_LED_OFF);
        delay(50);
     }
     rgb_led_set(restore_color);
}

//
// We are in an interrupt handler for the reset button and will indicate a factory reset.
// Just use white for now.
//
void rgb_led_set_factory_reset()
{
     rgb_led_set(RGB_LED_WHITE);
}

//
// Setup the Input and Output Interrupt service routine for the reset button. Just call the isr_resetButtonPress routing when the pin goes LOW.
// This triggers a factory reset.
//
void hw_setup()
{   
     pinMode(isr_resetButtonPin, INPUT_PULLUP); 
     attachInterrupt(digitalPinToInterrupt(isr_resetButtonPin), isr_resetButtonPress,   FALLING);  
}

//
// Complete restart for some reason and we've been up for some amount of time. Write this to the 
// NVS for display after reboot.
//
void ha_restart(uint32_t reason, uint32_t uptime)
{  
     ha_nvs_write(reason, uptime);        // remember why we are restarting so it can be shown in HA next time
     rgb_led_set(RGB_LED_OFF);            // Sometimes gets stuck on, don't know why perhaps timing.      
     delay(100);
     rgb_led_set(RGB_LED_OFF);            // So do it twice .
     delay(100);
     if (debug_g) DPRINTF("Restarting...\n"); 
     Zigbee.closeNetwork();
     Zigbee.stop();
     delay(100);
     ESP.restart();
}

//
// If the task watch dog times out, rather than use the system handler it will come here and we do a nice
// controlled reboot and keep track of the reason and how long we were up for better debugging.
//
void esp_task_wdt_isr_user_handler(void)
{
     ha_restart(1, millis()/1000); 
}

//
// Zibgee clusters
//
// Debug clusters
//
ZigbeeAnalog      zbRebootReason  = ZigbeeAnalog(10);      // reason for last reboot
ZigbeeAnalog      zbLastUptime    = ZigbeeAnalog(11);      // How long it was up last time before reboot
ZigbeeAnalog      zbRebootCount   = ZigbeeAnalog(12);      // How many reboots since factory reset
ZigbeeAnalog      zbUptime        = ZigbeeAnalog(13);      // Seconds since last reboot.
//
// Working clusters
//
ZigbeeBinary      zbPresence      = ZigbeeBinary(14);      // Presence yes/no
ZigbeeAnalog      zbRange         = ZigbeeAnalog(15);      // Trigger presence to zibgee if range < this. 0 means no trigger
ZigbeeAnalog      zbBrightness    = ZigbeeAnalog(16);      // How bright is night light Presence indicator
ZigbeeAnalog      zbFrequency     = ZigbeeAnalog(17);      // min number of seconds between zibgee updates.
//
// These are the local variables that mirror the zibgee clusters above.
//
uint16_t      ha_Presence   = 0xFF; // This is a sensor "INPUT" from this device to Zibgee 0 false, 1 true, FF unset
uint16_t      ha_Range      = 5;    // This is an output from Zibgee to this device to control range in meters 0..10
uint16_t      ha_Brightness = 5;    //                              ..... to control brighness of night LED 0..10
uint16_t      ha_Frequency  = 0;    //                              ..... to control max frequency of updates to Zibgee 0..10 min

//
// For detecting dead radar1.
//
uint32_t      radar1_last_reads = 0;    // When was radar last read (in seconds since startup)
uint32_t      radar2_last_reads = 0;    // and for 2nd radar.
uint16_t      radar1_presence   = 0xFF; // Unknown
uint16_t      radar2_presence   = 0xFF; // Unknown

//
// These are just useful debugging functions to display the attributes that HA has given us.
// One for each attributes.
// 
void ha_displayPresence()      { DPRINTF("Presence    = %d\n", ha_Presence);   }
void ha_displayRange()         { DPRINTF("Range       = %d\n", ha_Range);      }
void ha_displayBrightness()    { DPRINTF("Brightness  = %d\n", ha_Brightness); }
void ha_displayFrequency()     { DPRINTF("Frequency   = %d\n", ha_Frequency);  }
//
// Callbacks to set the attributes when updated by zibgee
//
void ha_setRange     (float v) { ha_Range = v;      if (debug_g) ha_displayRange();      }
void ha_setBrightness(float v) { ha_Brightness = v; if (debug_g) ha_displayBrightness(); }
void ha_setFrequency (float v) { ha_Frequency = v;  if (debug_g) ha_displayFrequency();  }
// 
// Keep HA up to date with any changes that happen.
//
void ha_sync_status()
{
     if (debug_g) DPRINTF("HA sync %d\n", ha_Presence);
     //
     // Sync the control clusters
     //
     zbRange.setAnalogOutput(ha_Range);
     zbRange.reportAnalogOutput();
     zbBrightness.setAnalogOutput(ha_Brightness);
     zbBrightness.reportAnalogOutput();
     zbFrequency.setAnalogOutput(ha_Frequency);
     zbFrequency.reportAnalogOutput();
     // Sync the presence cluster
     zbPresence.setBinaryInput(ha_Presence == 1);  // careful because 0xff means unset
     zbPresence.reportBinaryInput();
     // Sync the debug clusters
     zbRebootReason.setAnalogInput(ha_nvs_last_reboot_reason);
     zbLastUptime.setAnalogInput(ha_nvs_last_uptime);
     zbRebootCount.setAnalogInput(ha_nvs_last_reboot_count);
     zbUptime.setAnalogInput(millis()/1000);
     zbRebootReason.reportAnalogInput();
     zbLastUptime.reportAnalogInput();
     zbRebootCount.reportAnalogInput();
     zbUptime.reportAnalogInput();
}

//
// Called when device is asked to identify itself. We will just flash alternating white and green for 1/2 second or so.
// We get called 5 or 6 times with x = 5, 4, ... down to 0. We only triggers the flashing on the 0 call.
//
void ha_identify(uint16_t x)
{
     if (debug_g) DPRINTF("******** HA => IDENTIFY(%d) ******\n", (int) x);
     //
     if (x == 0) {
        rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
        delay(500);
        rgb_led_flash(RGB_LED_GREEN, RGB_LED_GREEN);
        delay(500);
        rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
        delay(500); 
        rgb_led_set(RGB_LED_GREEN);
     }
}

//
// The setup - we configure watch dogs, congfigure the hardware for interrupts, configure the 
// UARTS for the radar and try to establish connection to the radar, configure zibgee clusters
// and endpoints and if all goes well we will enter main loop(), otherwise we will reset with
// reason and try again.
//
void setup(void)
{   
     //
     // Watch dog timer on this task to panic if we don't get to main loop regulary.
     //
     if (wdt_g) {
         static const esp_task_wdt_config_t wdt_config = {                  // MUST BE CONST!!
              .timeout_ms = 10 * 60 * 1000,                                 // 10 minutes max to get back to main loop()
              .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Bitmask of all cores
              .trigger_panic = true };                                      // no panic, just restart
         esp_task_wdt_reconfigure(&wdt_config);
         esp_task_wdt_add(NULL);
         esp_task_wdt_status(NULL);
     }
     hw_setup();
     rgb_led_flash(RGB_LED_RED, RGB_LED_RED);

     //
     // Debug stuff
     //
     if (debug_g) {
         Serial.begin(115200);
         esp_log_level_set(TAG, ESP_LOG_DEBUG);
         DPRINTF("RiverView S/W Zibgee 3.0 Presence sensor\n");
         DPRINTF("Waiting for radar to boot...\n");
         delay(2000); // and wait for debug serial
     }
    
     //
     // Configure the serial port used by the each radar and bring them
     // up if they are configured.
     //
     if (RADAR1_ENABLED) {
         Serial1.begin(115200, SERIAL_8N1,  11,  10);
         bool isRadar1Enabled = false;
         for(int i=0; i<3; i++) {
             if(radar1.begin(&Serial1, debug_g ? &Serial : NULL)) {
                rgb_led_flash(RGB_LED_GREEN, RGB_LED_OFF);
                isRadar1Enabled = true;
                break;
             }
             if (debug_g) { DPRINTF("Retrying radar1 connection...\n"); };
             delay(1000);
         }
         if (isRadar1Enabled) {
             if (radar1.readFirmwareVersion()) {
                 if (debug_g) { DPRINTF("Radar1 Firmware: %s\n", radar1.firmwareVersion); }
             }
             if (radar1.readSerialNumber()) {
                if (debug_g) { DPRINTF("[Radar1 Serial number: %s\n", radar1.serialNumber); }
             }
             auto config = radar1.radarConfiguration;
             if (debug_g) {
                 DPRINTF("[Info] Radar1 config:\n");
                 DPRINTF("|- Gates min: %u\n", config.detectionGatesMin);
                 DPRINTF("|- Gates max %u\n", config.detectionGatesMax);
                 DPRINTF("|- Disappearance delay: %u sec\n", config.targetDisappearanceDelay);
             }
             rgb_led_flash(RGB_LED_GREEN, RGB_LED_OFF);
         } else {
             delay(5000);
             rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
             ha_restart(2, millis()/1000); 
         }
     }
   
     if (RADAR2_ENABLED) {
         Serial2.begin(115200, SERIAL_8N1,  4,   5);
         bool isRadar2Enabled = false;
         for(int i=0; i<3; i++) {
             if(radar2.begin(&Serial2, debug_g ? &Serial : NULL)) {
                rgb_led_flash(RGB_LED_GREEN, RGB_LED_OFF);
                isRadar2Enabled = true;
                break;
             }
             if (debug_g) { DPRINTF("Retrying radar2 connection...\n"); };
             delay(1000);
         }
         if (isRadar2Enabled) {
             if (radar2.readFirmwareVersion()) {
                 if (debug_g) { DPRINTF("Radar2 Firmware: %s\n", radar2.firmwareVersion); }
             }
             if (radar2.readSerialNumber()) {
                if (debug_g) { DPRINTF("[Radar2 Serial number: %s\n", radar2.serialNumber); }
             }
             auto config = radar2.radarConfiguration;
             if (debug_g) {
                 DPRINTF("[Info] Radar2 config:\n");
                 DPRINTF("|- Gates min: %u\n", config.detectionGatesMin);
                 DPRINTF("|- Gates max %u\n", config.detectionGatesMax);
                 DPRINTF("|- Disappearance delay: %u sec\n", config.targetDisappearanceDelay);
             }
             rgb_led_flash(RGB_LED_GREEN, RGB_LED_OFF);
         } else {
             delay(5000);
             rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
             ha_restart(3, millis()/1000); 
         }
     }

     //
     // We get debug information from last reboot (uptime and reboot reason etc.)
     ha_nvs_read();

     //
     // Initialize all the major variables that are sent to/from zigbee/HA
     //
     radar1_presence   = 0xFF;    // Unknown
     radar2_presence   = 0xFF;    // Unknown
     ha_Presence       = 0xFF;    //  This is a sensor "INPUT" from this device to Zibgee 0 false, 1 true, FF unset
     ha_Range      = 5;   // This is an output from Zibgee to this device to control range in meters 0..10
     ha_Brightness = 5;   //      ..... to control brighness of night LED 0..10
     ha_Frequency  = 2;   //      ..... to control max frequency of updates to Zibgee 0s, 1=10s, 2=20s,.. 10=100s.
     //
     // Add the zibgee clusters (buttons/sliders etc.)
     //
     const char *MFGR = "RiverView";      // Because my home office looks out over the ottwawa river ;)
     const char *MODL = "zPres001";       // Presence Sensor 001

     if (debug_g) DPRINTF("Range Cluster\n");
     zbRange.setManufacturerAndModel(MFGR,MODL);
     zbRange.addAnalogOutput();
     zbRange.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbRange.setAnalogOutputDescription("Range(M)");
     zbRange.setAnalogOutputResolution(1);
     zbRange.setAnalogOutputMinMax(0, 10);  
     zbRange.onAnalogOutputChange(ha_setRange);
     //
     if (debug_g) DPRINTF("Brightness Cluster\n");
     zbBrightness.setManufacturerAndModel(MFGR,MODL);
     zbBrightness.addAnalogOutput();
     zbBrightness.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbBrightness.setAnalogOutputDescription("Brightness");
     zbBrightness.setAnalogOutputResolution(1);
     zbBrightness.setAnalogOutputMinMax(0, 10);  
     zbBrightness.onAnalogOutputChange(ha_setBrightness);
     //
     if (debug_g) DPRINTF("Frequency Cluster\n");
     zbFrequency.setManufacturerAndModel(MFGR,MODL);
     zbFrequency.addAnalogOutput();
     zbFrequency.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbFrequency.setAnalogOutputDescription("Freqx10s");
     zbFrequency.setAnalogOutputResolution(1);
     zbFrequency.setAnalogOutputMinMax(0, 10);  
     zbFrequency.onAnalogOutputChange(ha_setFrequency);
     //
     // Debug clusters
     //
     if (debug_g) DPRINTF("RebootReason cluster\n");
     zbRebootReason.setManufacturerAndModel(MFGR,MODL);
     zbRebootReason.addAnalogInput();
     zbRebootReason.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbRebootReason.setAnalogInputDescription("Last Reboot Reason");
     zbRebootReason.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("LastUptime cluster\n");
     zbLastUptime.setManufacturerAndModel(MFGR,MODL);
     zbLastUptime.addAnalogInput();
     zbLastUptime.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbLastUptime.setAnalogInputDescription("Last Uptime s");
     zbLastUptime.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("RebootCount cluster\n");
     zbRebootCount.setManufacturerAndModel(MFGR,MODL);
     zbRebootCount.addAnalogInput();
     zbRebootCount.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbRebootCount.setAnalogInputDescription("Reboot Count");
     zbRebootCount.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("This Uptime\n");
     zbUptime.setManufacturerAndModel(MFGR,MODL);
     zbUptime.addAnalogInput();
     zbUptime.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbUptime.setAnalogInputDescription("This Uptime s");
     zbUptime.setAnalogInputResolution(1.0);
     //
     //
     if (debug_g) DPRINTF("Set mains power & identify callback\n");
     zbPresence.setManufacturerAndModel(MFGR,MODL);
     zbPresence.addBinaryInput();
     zbPresence.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbPresence.setBinaryInputDescription("Presence");
     zbPresence.setPowerSource(ZB_POWER_SOURCE_MAINS); 
     zbPresence.onIdentify(ha_identify);
     //
     // add all clusters to the end point.
     //
     Zigbee.addEndpoint(&zbPresence);
     Zigbee.addEndpoint(&zbRange);
     Zigbee.addEndpoint(&zbBrightness);
     Zigbee.addEndpoint(&zbFrequency);
     Zigbee.addEndpoint(&zbRebootReason);
     Zigbee.addEndpoint(&zbLastUptime);
     Zigbee.addEndpoint(&zbRebootCount);
     Zigbee.addEndpoint(&zbUptime);
     //
     // Create a custom Zigbee configuration for End Device with longer timeouts/keepalive
     //
     esp_zb_cfg_t zigbeeConfig =                             \
       {  .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,              \
          .install_code_policy = false,                      \
          .nwk_cfg = {                                       \
            .zed_cfg =  {                                    \                                                          
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN, \
                .keep_alive = 3000,                          \
              },                                             \
          },                                                 \
       };
     //
     if (debug_g) DPRINTF("Starting Zigbee\n");
     rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);
     //
     // When all EPs are registered, start Zigbee in End Device mode
     //
     if (!Zigbee.begin(&zigbeeConfig, false)) { 
        if (debug_g) {
            DPRINTF("Zigbee failed to start!\n");
            DPRINTF("Rebooting ESP32!\n");
        }
        for(int i = 0; i < 10; i++) {
           rgb_led_flash(RGB_LED_RED, RGB_LED_WHITE);
           rgb_led_flash(RGB_LED_WHITE, RGB_LED_RED);
        }
        ha_restart(4, millis()/1000);             // restart and remember why
     }
     //
     // Now connect to network.
     //
     if (debug_g) DPRINTF("Connecting to network\n");   
     int tries = 0;      
     while (!Zigbee.connected()) {
        rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);         // the led sets have delays built in
        delay(5000);
        if (debug_g) DPRINTF("connecting..\n");
        if (tries ++ > 360) {                              // Maximum 30 minutes trying    
           if (debug_g) {
               DPRINTF("Zigbee failed to connect!\n");
               DPRINTF("Rebooting ESP32!\n");
           }
           rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);  // We tried for 30 minutes, restart.
           rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
           ha_restart(5, millis()/1000);   
        }
     }
     rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);   
     if (debug_g) DPRINTF("Successfully connected to Zigbee network\n");
     //
     // Update the debug related information to HA.
     //
     ha_sync_status();

     //
     // Radar is working so report last read time for continuous failure checking in loop().
     //
     radar1_last_reads = millis()/1000;
     radar2_last_reads = radar1_last_reads;
}

//
// Master loop - look for changes in radar or zibgee status. If something moves into detection range notify zigbee
// but not too frequently. If radar goes down we won't refresh watch dog, so it will reset eventually. If zibgee goes 
// disconnected we restart immediately. 
//
void loop(void)
{ 
     //
     // We will swap the radars to avoid interference between them. We operate for 500ms on one antena, then 
     // put it in command mode (which stops it transmitting). Then we put take the second antenna out of 
     // command mode which start it functioning again. This is repeated continuously. While we can operate with 
     // both radars at the same time, we seem to get a lot of spurious results.
     //
     delay(500);    // HERE WE SHOULD BE SWAPPING THE RADARS!!

     //
     // Any Zigbee problems we reset.
     //
     if (!Zigbee.connected()) {
         if (debug_g) DPRINTF("zigbee disconnected while in loop()- restarting\n");
         ha_restart(6, millis()/1000);   
     }

     //
     // Any Radar Serial problems we will reset. Just check to see if its been more than 10 seconds since
     // we saw any radar data, if so a full reboot will occur and we remember the reason and time.
     // Actually we continue to operate even if one of the sensors is dead.
     //
     uint32_t nows = millis()/1000;                     // Current time in seconds since reboot
     uint32_t delta1 = nows - radar1_last_reads;        // time since last success full radar read
     uint32_t delta2 = nows - radar2_last_reads;        // time since last success full radar read
     //
     if (RADAR1_ENABLED && (delta1 > 60)) {                           
         if (debug_g) DPRINTF("A radar 1 disconnected while in loop()- restarting\n");
         ha_restart(7, nows);   
     }
     if (RADAR2_ENABLED && (delta2 > 60)) {                           
         if (debug_g) DPRINTF("A radar 2 disconnected while in loop()- restarting\n");
         ha_restart(8, nows);   
     }
     //
     // Initial conditions we don't know what color to set of if any presene has been
     // detected.
     //
     static int status_color = RGB_LED_OFF;     
     //
     // Read as much data as we can from both radars, but guard against spending too much time here.
     // If we get too much data then we have a problem and will reboot. We stop when both radars have
     // fed us everything they know. 
     //
     int r1_count = 0;                                   // How many packets we got from radar1
     int r2_count = 0;                                   // How many packets we got from radar2
     while(true) {                                       // yes , we break out in the middle
           bool r1_read = RADAR1_ENABLED? radar1.read() : false;  // see if radar1 has a packet
           bool r2_read = RADAR2_ENABLED? radar2.read() : false;  // see if radar2 has a packet
           if (r1_read) r1_count += 1;                   // accumulate tally for radar1
           if (r2_read) r2_count += 1;                   // tally for radar2
           if ((!r1_read) && (!r2_read)) break;          // if nothing new get out and process
           if ((r1_count > 100) || (r2_count > 100)) {   // guard against infinite loop, reboot if so
               if (debug_g) DPRINTF("abnormal amount of radar data - can't keep up\n");
               ha_restart(9, millis()/1000);   
           }
     } 
     //
     // Look at the last radar packets we got and use them to decide if either radar has a target
     // within the desire range.
     //
     if (r1_count > 0) { 
         radar1_last_reads = millis()/1000;
         radar1_presence = 0;
         if (radar1.isTargetDetected) {
             uint16_t distance = radar1.distanceToTarget;
             if (debug_g) { DPRINTF("radar 1 at: %ucm, range=%u\n", distance, ha_Range); }
             radar1_presence = (distance < ha_Range*100) ? 1 : 0;
         }
     } 
     if (r2_count > 0) { 
         radar2_last_reads = millis()/1000;
         radar2_presence = 0;
         if (radar2.isTargetDetected) {
             uint16_t distance = radar2.distanceToTarget;
             if (debug_g) { DPRINTF("radar 2 at: %ucm, range=%u\n", distance, ha_Range); }
             radar2_presence = (distance < ha_Range*100) ? 1 : 0;
         }
     } 
     //
     // If either detector has presence then we consider this presence for zigbee.
     // If both detectors have no presence then this is no presence for zigbee.
     // There are FF values for unitialized so those must be ignored.
     //
     if ((radar1_presence == 1) || (radar2_presence == 1))
         ha_Presence = 1;
     if ((radar1_presence == 0) && (radar2_presence == 0))
         ha_Presence = 0;
     //
     // And feed the watch dog because radar and zibgee are both ok and above
     // loop was not infinite.
     // 
     if (wdt_g) esp_task_wdt_reset();  
     //
     // Update zibgee if sensor changes from last state but 
     // may need to throttle a bit to avoid too many updates to Zibgee. 
     // We track last time we send a notification to zibgee about presence gone and
     // will suppress it for the required number of seconds to meet the desired
     // maximum frequency as set in the ha_Frequency cluster.
     //
     static uint16_t last_ha_Presence = 0xFF;
     static uint32_t zigbee_last_notification = 0; 
     if (ha_Presence != last_ha_Presence) {
         uint32_t nows = millis()/1000;
         uint32_t deltas = (ha_Presence == 0) ? nows - zigbee_last_notification : 0;
         if ((deltas > ha_Frequency*10)||(deltas == 0)) {
             zbPresence.setBinaryInput(ha_Presence == 0 ? false : true);       
             zbPresence.reportBinaryInput();
             status_color = (ha_Presence == 1) ? RGB_LED_WHITE : RGB_LED_OFF;
             zigbee_last_notification = nows;
             last_ha_Presence = ha_Presence;
             if (debug_g) { DPRINTF("ha report sensor Presence=%d\n", ha_Presence); }
         } else {
             if (debug_g) { DPRINTF("ha report sensor Presence=0 SUPPRESSED %us WAIT=%us\n", deltas, ha_Frequency*10); }
         }
     }
     //
     // Every so often (5 mins) we update the HA with all all attributes.
     //
     {  const unsigned long  MAX_TIME              = 60*5;
        static unsigned long last_update_time      = 0;
        unsigned long        now_time              = millis() / 1000;
        //
        if ((last_update_time + MAX_TIME) <= now_time) {
            last_update_time = now_time;
            ha_sync_status();                 
        }
     }
     //
     // Led is on for 4.5 seconds, then briefly on, repeat. Color depends on
     // what happend above. White (night light) is the default). Note that the
     // brightness comes from Zigbee and is set by callbacks.
     //
     rgb_led_set(status_color, ha_Brightness);
}
