/**************************************************************
 *    ESP32 VBAN Audio Source Demo
 *    R. Kinnett, 2020
 *    https://github.com/rkinnett/ESP32-VBAN-Audio-Source
 **************************************************************/
 
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include "vban.h"


//SSID and Password of your WiFi router
#include "my_wifi.h"  
///// my_wifi.h contains the following lines with my own wifi info.
///// You can remove the line above and uncomment the lines below with your info,
///// or make your own my_wifi.h containing the following lines with your info.
///// I had to do it this way in order to safely upload to github.
/*
const char* ssid = "my_wifi_ssid";
const char* password = "my_wifi_pw";
*/


/**   UDP SETUP  **/
// Per various discussions online, and confirmed by my own experimentation,
// UDP broadcast and multicast modes do not seem to support the data rates
// needed for robust audio streaming, presumably due to practial routing 
// limitations.  Use unicast (addressed) UDP mdoe instead.
// Hard-coding the destination address is not ideal.  How to make this dynamic?
// As far as I can tell, VBAN does not solicit connections, so can't wait for
// an exlicit subscription request from the host PC, at least not from VBAN.
// One could use YAT or similar UDP-capable terminal emulator to send request
// to the ESP32, and then have the ESP32 use that address for unicasting.
// For my own applications, I plan to have the ESP32 serve a configuration
// webpage with an html form, and I will probably just use that to specify
// the address of the VBAN host PC.  Hardcoded for now.
WiFiUDP udp;
uint16_t udpPort = 6980;
IPAddress destIP(192,168,1,19);


VBan vban;


/** ADC CONFIG **/
i2s_port_t i2s_port = I2S_NUM_0;   // I2S Port 
adc1_channel_t adc_channel = ADC1_CHANNEL_5; //GPIO33  (SET THIS FOR YOUR HARDWARE)
const uint16_t adc_sample_freq = 12000;
const uint16_t dma_buffer_len = 256;
const uint16_t i2s_buffer_len = dma_buffer_len;
uint16_t* i2s_read_buff = (uint16_t*)calloc(i2s_buffer_len, sizeof(uint16_t));


boolean streaming = true;
boolean sampling  = true;
unsigned long prev_micros;
size_t bytes_read;
TaskHandle_t samplingTaskHandle;

// Prototypes:
static void getDataLoopTask(void * pvParameters);
static const inline void sendSamples();



//////////////////////////   CONFIGURE VBAN PACKET   ////////////////////////////////
void configure_vban() {
  // Set vban packet header, counter, and data frame pointers to respective parts of packet:
  vban.hdr            = (VBanHeader*) &vban.packet[0];
  vban.packet_counter = (uint32_t*)   &vban.packet[VBPACKET_HEADER_BYTES];
  vban.data_frame     = (uint8_t*)    &vban.packet[VBPACKET_HEADER_BYTES + VBPACKET_COUNTER_BYTES];
  
  // Setup the packet header:
  strncpy(vban.hdr->preamble, "VBAN", 4);
  vban.hdr->sample_rate      = VBAN_PROTOCOL_AUDIO | SAMPLE_RATE_12000_HZ;   // 11025 Hz, which matches default sample rate for soundmodem
  vban.hdr->num_samples      = VBPACKET_NUM_SAMPLES-1;                       // 255 = 256 samples
  vban.hdr->num_channels     = 0;                                            // 1 channel
  vban.hdr->sample_format    = VBAN_BITFMT_16_INT | VBAN_CODEC_PCM;          // int16 PCM
  strncpy(vban.hdr->stream_name, "ESP32_AUDIO_STRM", 16);

  *vban.packet_counter = 0;  // initialize packet counter

  vban.packet_data_bytes = (vban.hdr->num_samples+1) * (vban.hdr->num_channels+1) * ((vban.hdr->sample_format & VBAN_BIT_RESOLUTION_MASK)+1);
  vban.packet_total_bytes = vban.packet_data_bytes + VBPACKET_HEADER_BYTES + VBPACKET_COUNTER_BYTES;
}


//////////////////////////   CONFIGURE I2S SAMPLING   ////////////////////////////////
void configure_i2s(){
  i2s_config_t i2s_config = 
    {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),  // I2S receive mode with ADC
    .sample_rate = adc_sample_freq,                                               // set I2S ADC sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                                 // 16 bit I2S (even though ADC is 12 bit)
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,                                 // handle adc data as single channel (right)
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S,               // I2S format
    .intr_alloc_flags = 0,                                                        // 
    .dma_buf_count = 16,                                                           // number of DMA buffers >=2 for fastness
    .dma_buf_len = dma_buffer_len,                                                // number of samples per buffer
    .use_apll = 0,                                                                // no Audio PLL - buggy and not well documented
  };
  adc1_config_channel_atten(adc_channel, ADC_ATTEN_11db);
  adc1_config_width(ADC_WIDTH_12Bit);
  i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
 
  i2s_set_adc_mode(ADC_UNIT_1, adc_channel);
  i2s_adc_enable(i2s_port);
  vTaskDelay(1000);
}



//////////////////////////   SETUP   ////////////////////////////////////////
void setup() {
  Serial.begin(500000);

  // Setup Wifi:
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {  // Wait for connection
    delay(500);
    Serial.print(".");
  }
  IPAddress myIP = WiFi.localIP();

  Serial.print("Wifi connected. IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(";  Subnet Mask: ");
  Serial.println(WiFi.subnetMask());

  configure_i2s();
  configure_vban();
  udp_connected = udp.begin(myIP, udpPort);

  xTaskCreatePinnedToCore(getDataLoopTask, "getDataLoopTask", 80000, NULL, 0, &samplingTaskHandle, 0 );
}



//////////////////////////   MAIN LOOP   /////////////////////////////////
void loop() {
  // Handle infrastructural things in main loop.
  // Sampling is handled in separate tasks outside of this loop.
}



/////////////////////   SAMPLING TASK AND FUNCTIONS  ////////////////////
static void getDataLoopTask(void * pvParameters){
  for( ;; ){
    if(sampling){
      i2s_read(i2s_port, (void*)i2s_read_buff, i2s_buffer_len*sizeof(uint16_t), &bytes_read, portMAX_DELAY);
      if(streaming && I2S_EVENT_RX_DONE && bytes_read>0){
        sendSamples();
      }
    }
    vTaskDelay(1);  // resets watchdog with 1-tick (1ms) delay
    // To-do:  consider calculating longer delay on-the-fly
  }
}


static const inline void sendSamples(){
  int16_t adc_data[VBPACKET_NUM_SAMPLES];

  //// Per esp32.com forum topic 11023, esp32 swaps even/odd samples,
  ////   i.g. samples 0 1 2 3 4 5 are stored as 1 0 3 2 5 4 ..
  ////   Have to deinterleave manually...
  //// Also need to mask upper 4 bits which contain channel info (see gitter chat between me-no-dev and bzeeman)  
  for(int i=0; i<VBPACKET_NUM_SAMPLES; i+=2){  // caution: this is not robust to odd buffer lens
    adc_data[i]   = (int16_t)(i2s_read_buff[i+1] & 0x0FFF) - 2048;
    adc_data[i+1] = (int16_t)(i2s_read_buff[i]   & 0x0FFF) - 2048;
  }
  memcpy(vban.data_frame, adc_data, vban.packet_data_bytes);

  // Send packet
  udp.beginPacket(destIP, udpPort);
  udp.write((uint8_t*)&vban.packet, vban.packet_total_bytes);
  udp.endPacket();  
  
  (*vban.packet_counter)++;   // increment packet counter
}
