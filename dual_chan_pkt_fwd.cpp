 /******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * HBM:
 * Changes for creating a dual channel gateway with the Raspberry Pi+ LoRa(TM) Expansion Board
 * of Uputronics, see also: store.uputronics.com/index.php?route=product/product&product_id=68
 * It now also supports downlink, config file, separate thread for downlink and separate functions
 * in the LoraModem.c file
 *******************************************************************************/

// Raspberry PI pin mapping
// Pin number in this global_conf.json are Wiring Pi number (wPi colunm)
// issue a `gpio readall` on PI command line to see mapping
// +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
// |   2 |   8 |   SDA.1 |   IN | 1 |  3 || 4  |   |      | 5V      |     |     |
// |   3 |   9 |   SCL.1 |   IN | 1 |  5 || 6  |   |      | 0v      |     |     |
// |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
// |     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
// |  17 |   0 | GPIO. 0 |  OUT | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
// |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
// |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
// |     |     |    3.3v |      |   | 17 || 18 | 1 | IN   | GPIO. 5 | 5   | 24  |
// |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
// |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
// |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
// |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
// |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
// |   5 |  21 | GPIO.21 |  OUT | 0 | 29 || 30 |   |      | 0v      |     |     |
// |   6 |  22 | GPIO.22 |  OUT | 0 | 31 || 32 | 1 | IN   | GPIO.26 | 26  | 12  |
// |  13 |  23 | GPIO.23 |  OUT | 0 | 33 || 34 |   |      | 0v      |     |     |
// |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
// |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
// |     |     |      0v |      |   | 39 || 40 | 0 | OUT  | GPIO.29 | 29  | 21  |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
// For Uputronics Raspberry Pi+ LoRa(TM) Expansion Board
// pins configuration in file global_conf.json
//
//
//  "pin_nss": 10,
//  "pin_dio0": 6,
//  "pin_nss2": 11,
//  "pin_dio0_2": 27,
//  "pin_rst": 0,
//  "pin_NetworkLED": 22,
//  "pin_InternetLED": 23,
//  "pin_ActivityLED_0": 21,
//  "pin_ActivityLED_1": 29,
//


#include "base64.h"
#include "parson.h"

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netdb.h>

#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <pthread.h>
#include <netdb.h>              // gai_strerror
#include "LoraModem.h"		// include Lora stuff

// Debug messages
//int debug = 2;

static const int CHANNEL = 0;

using namespace std;

using namespace rapidjson;

#define BASE64_MAX_LENGTH 341

typedef unsigned char byte;

uint32_t cp_nb_pull;

// Set location in global_conf.json
float lat =  0.0;
float lon =  0.0;
int   alt =  0;

/* Informal status fields */
char platform[24] ;    /* platform definition */
char email[40] ;       /* used for contact email */
char description[64] ; /* used for free form description */

// internet interface
char interface[6];     // Used to set the interface to communicate to the internet either eth0 or wlan0

static int keepalive_time = DEFAULT_KEEPALIVE; // send a PULL_DATA request every X seconds
/* network protocol variables */
//static struct timeval push_timeout_half = {0, (PUSH_TIMEOUT_MS * 500)}; /* cut in half, critical for throughput */
static struct timeval pull_timeout = {0, (PULL_TIMEOUT_MS * 1000)}; /* non critical for throughput */

#define TX_BUFF_SIZE    2048
#define STATUS_SIZE     1024

//Downlink stuff
#define MAX_SERVERS                 4 /* Support up to 4 servers, more does not seem realistic */
#define RX_BUFF_SIZE  1024	// Downstream received from MQTT
uint8_t buff_down[RX_BUFF_SIZE];	// Buffer for downstream
uint8_t buff_up[RX_BUFF_SIZE];	// Buffer for upstream
/* network sockets */
//static int sock_up[MAX_SERVERS]; /* sockets for upstream traffic */
static int sock_down[MAX_SERVERS]; /* sockets for downstream traffic */
pthread_t thrid_down[MAX_SERVERS];

void thread_down(void* pic);

volatile bool exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool quit_sig = false; /* 1 -> application terminates without shutting down the hardware */


void LoadConfiguration(string filename);
void PrintConfiguration();

void Die(const char *s) {
  perror(s);
  exit(1);
}

void SetupLoRa(byte CE) {
  char buff[16];
  if (CE == 0) {
    printf("Trying to detect module CE0 with ");
    printf("NSS=%s "  , PinName(ssPin, buff));
    printf("DIO0=%s " , PinName(dio0 , buff));
    printf("Reset=%s ", PinName(RST  , buff));
    printf("Led1=%s\n", PinName(Led1 , buff));
  } else {
    printf("Trying to detect module CE1 with ");
    printf("NSS=%s "  , PinName(ssPin_2, buff));
    printf("DIO0=%s " , PinName(dio0_2 , buff));
    printf("Reset=%s ", PinName(RST  , buff));
    printf("Led1=%s\n", PinName(Led1 , buff));
  }
  // check basic
  if (ssPin == 0xff || dio0 == 0xff) {
    Die("Bad pin configuration ssPin and dio0 need at least to be defined");
  }
  uint8_t version = ReadRegister(REG_VERSION, CE);
  if (version == 0x22) {
    // sx1272
    printf("SX1272 detected, starting.\n");
    sx1272 = true;
  } else {
    // sx1276?
    version = ReadRegister(REG_VERSION, CE);
    if (version == 0x12) {
      // sx1276
      if (CE == 0)
      {
        printf("SX1276 detected on CE0, starting.\n");
      } else {
        printf("SX1276 detected on CE1, starting.\n");
      }
      sx1272 = false;
    } else {
      printf("Transceiver version 0x%02X\n", version);
      Die("Unrecognized transceiver");
    }
  }
  WriteRegister(REG_OPMODE, SX72_MODE_SLEEP, CE);
  // set frequency
  uint64_t frf;
  if (CE == 0)
  {
    frf = ((uint64_t)freq << 19) / 32000000;
  } else {
    frf = ((uint64_t)freq_2 << 19) / 32000000;
  }
  WriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16), CE );
  WriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8), CE );
  WriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0), CE );
  WriteRegister(REG_SYNC_WORD, 0x34, CE); // LoRaWAN public sync word
  if (sx1272) {
    if (sf == SF11 || sf == SF12) {
      WriteRegister(REG_MODEM_CONFIG1, 0x0B, CE);
    } else {
      WriteRegister(REG_MODEM_CONFIG1, 0x0A, CE);
    }
    WriteRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04, CE);
  } else {
    if (sf == SF11 || sf == SF12) {
      WriteRegister(REG_MODEM_CONFIG3, 0x0C, CE);
    } else {
      WriteRegister(REG_MODEM_CONFIG3, 0x04, CE);
    }
    WriteRegister(REG_MODEM_CONFIG1, 0x72, CE);
    WriteRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04, CE);
  }
  if (sf == SF10 || sf == SF11 || sf == SF12) {
    WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x05, CE);
  } else {
    WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x08, CE);
  }
  WriteRegister(REG_MAX_PAYLOAD_LENGTH, 0x80, CE);
  WriteRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH, CE);
  WriteRegister(REG_HOP_PERIOD, 0xFF, CE);
  WriteRegister(REG_FIFO_ADDR_PTR, ReadRegister(REG_FIFO_RX_BASE_AD, CE), CE);
  // Set Continous Receive Mode
  WriteRegister(REG_LNA, LNA_MAX_GAIN, CE);  // max lna gain
  WriteRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS, CE);
}

void SendStat() {
  static char status_report[STATUS_SIZE]; /* status report as a JSON object */
  char stat_timestamp[24];
  int stat_index = 0;
  digitalWrite(InternetLED, HIGH);
  /* pre-fill the data buffer with fixed fields */
  status_report[0] = PROTOCOL_VERSION;
  status_report[3] = PKT_PUSH_DATA;
  status_report[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
  status_report[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
  status_report[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
  status_report[7] = 0xFF;
  status_report[8] = 0xFF;
  status_report[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
  status_report[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
  status_report[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];
  /* start composing datagram with the header */
  uint8_t token_h = (uint8_t)rand(); /* random token */
  uint8_t token_l = (uint8_t)rand(); /* random token */
  status_report[1] = token_h;
  status_report[2] = token_l;
  stat_index = 12; /* 12-byte header */
  /* get timestamp for statistics */
  time_t t = time(NULL);
  strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));
  // Build JSON object.
  StringBuffer sb;
  Writer<StringBuffer> writer(sb);
  writer.StartObject();
  writer.String("stat");
  writer.StartObject();
  writer.String("time");
  writer.String(stat_timestamp);
  writer.String("lati");
  writer.Double(lat);
  writer.String("long");
  writer.Double(lon);
  writer.String("alti");
  writer.Int(alt);
  writer.String("rxnb");
  writer.Uint(cp_nb_rx_rcv);
  writer.String("rxok");
  writer.Uint(cp_nb_rx_ok);
  writer.String("rxfw");
  writer.Uint(cp_up_pkt_fwd);
  writer.String("ackr");
  writer.Double(0);
  writer.String("dwnb");
  writer.Uint(0);
  writer.String("txnb");
  writer.Uint(0);
  writer.String("pfrm");
  writer.String(platform);
  writer.String("mail");
  writer.String(email);
  writer.String("desc");
  writer.String(description);
  writer.EndObject();
  writer.EndObject();
  string json = sb.GetString();
  //printf("stat update: %s\n", json.c_str());
  printf("stat update: %s", stat_timestamp);
  if (cp_nb_rx_ok_tot==0) {
    printf(" no packet received yet\n");
  } else {
    printf(" %u packet%sreceived, %u packet%ssent...\n", cp_nb_rx_ok_tot, cp_nb_rx_ok_tot>1?"s ":" ", cp_nb_pull, cp_nb_pull>1?"s ":" ");
  }
  // Build and send message.
  memcpy(status_report + 12, json.c_str(), json.size());
  //SendUdp(status_report, stat_index + json.size());
  printf("DEBUG: save gateway status file here...\n");
  digitalWrite(InternetLED, LOW);
}

int main() {
  struct timeval nowtime;
  uint32_t lasttime;
  unsigned int led0_timer,led1_timer;
  LoadConfiguration("global_conf.json");
  PrintConfiguration();
  // Init WiringPI
  wiringPiSetup() ;
  pinMode(ssPin, OUTPUT);
  pinMode(ssPin_2, OUTPUT);
  pinMode(dio0, INPUT);
  pinMode(dio0_2, INPUT);
  pinMode(RST, OUTPUT);
  pinMode(NetworkLED, OUTPUT);
  pinMode(ActivityLED_0, OUTPUT);
  pinMode(ActivityLED_1, OUTPUT);
  pinMode(InternetLED, OUTPUT);
  // Init SPI
  wiringPiSPISetup(SPI_CHANNEL, 500000);
  wiringPiSPISetup(SPI_CHANNEL_2, 500000);
  // Setup LORA
  digitalWrite(RST, HIGH);
  delay(100);
  digitalWrite(RST, LOW);
  delay(100);
  initLoraModem(0);
  initLoraModem(1);
  // Prepare Socket connection
  while ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
     digitalWrite(NetworkLED, 1);
     printf("No socket connection possible yet. Retrying in 10 seconds...\n");
     delay(10000);
    digitalWrite(NetworkLED, 0);
  }
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, interface, IFNAMSIZ-1);  // use configured network interface eth0 or wlan0
  ioctl(s, SIOCGIFHWADDR, &ifr);
  // ID based on MAC Adddress of interface
  printf( "Gateway ID: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
    (uint8_t)ifr.ifr_hwaddr.sa_data[0],
    (uint8_t)ifr.ifr_hwaddr.sa_data[1],
    (uint8_t)ifr.ifr_hwaddr.sa_data[2],
    (uint8_t)ifr.ifr_hwaddr.sa_data[3],
    (uint8_t)ifr.ifr_hwaddr.sa_data[4],
    (uint8_t)ifr.ifr_hwaddr.sa_data[5]
  );
  printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
  printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq_2/1000000);
  printf("-----------------------------------\n");
  while(1) {
    // Packet received ?
    if (ReceivePacket(0)) {
      // Led ON
      if (ActivityLED_0 != 0xff) {
        digitalWrite(ActivityLED_0, 1);
      }
      // start our Led blink timer, LED as been lit in Receivepacket
      led0_timer=millis();
    }
    if (ReceivePacket(1)) {
      // Led ON
      if (ActivityLED_1 != 0xff) {
        digitalWrite(ActivityLED_1, 1);
      }
      // start our Led blink timer, LED as been lit in Receivepacket
      led1_timer=millis();
    }
   // Receive UDP PUSH_ACK messages from server. (*2, par. 3.3)
   // This is important since the TTN broker will return confirmation
   // messages on UDP for every message sent by the gateway. So we have to consume them..
   // As we do not know when the server will respond, we test in every loop.
   //
   //int received_bytes = recvfrom( handle, packet_data, sizeof(packet_data),0, (struct sockaddr*)&cliaddr, &len );
   //start the thread
    gettimeofday(&nowtime, NULL);
    uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
    if (nowseconds - lasttime >= 30) {
      lasttime = nowseconds;
      SendStat();
      cp_nb_rx_rcv = 0;
      cp_nb_rx_ok = 0;
      cp_up_pkt_fwd = 0;
    }
    // Led timer in progress ?
    if (led0_timer) {
      // Led timer expiration, Blink duration is 250ms
      if (millis() - led0_timer >= 250) {
        // Stop Led timer
        led0_timer = 0;

        // Led OFF
        if (ActivityLED_0 != 0xff) {
          digitalWrite(ActivityLED_0, 0);
        }
      }
    }
    if (led1_timer) {
      // Led timer expiration, Blink duration is 250ms
      if (millis() - led1_timer >= 250) {
        // Stop Led timer
        led1_timer = 0;

        // Led OFF
        if (ActivityLED_1 != 0xff) {
          digitalWrite(ActivityLED_1, 0);
        }
      }
    }
    // Let some time to the OS
    delay(1);
  }
  return (0);
}

void LoadConfiguration(string configurationFile) {
  FILE* p_file = fopen(configurationFile.c_str(), "r");
  char buffer[65536];
  FileReadStream fs(p_file, buffer, sizeof(buffer));
  Document document;
  document.ParseStream(fs);
  for (Value::ConstMemberIterator fileIt = document.MemberBegin(); fileIt != document.MemberEnd(); ++fileIt) {
    string objectType(fileIt->name.GetString());
    if (objectType.compare("SX127x_conf") == 0) {
      const Value& sx127x_conf = fileIt->value;
      if (sx127x_conf.IsObject()) {
        for (Value::ConstMemberIterator confIt = sx127x_conf.MemberBegin(); confIt != sx127x_conf.MemberEnd(); ++confIt) {
          string key(confIt->name.GetString());
          if (key.compare("freq") == 0) {
            freq = confIt->value.GetUint();
          } else if (key.compare("freq_2") == 0) {
            freq_2 = (SpreadingFactor_t)confIt->value.GetUint();
          } else if (key.compare("spread_factor") == 0) {
            sf = (SpreadingFactor_t)confIt->value.GetUint();
          } else if (key.compare("pin_nss") == 0) {
            ssPin = confIt->value.GetUint();
          } else if (key.compare("pin_nss_2") == 0) {
            ssPin_2 = confIt->value.GetUint();
          } else if (key.compare("pin_dio0") == 0) {
            dio0 = confIt->value.GetUint();
          } else if (key.compare("pin_dio0_2") == 0) {
            dio0_2 = confIt->value.GetUint();
          } else if (key.compare("pin_rst") == 0) {
            RST = confIt->value.GetUint();
          } else if (key.compare("pin_NetworkLED") == 0) {
            NetworkLED = confIt->value.GetUint();
          } else if (key.compare("pin_InternetLED") == 0) {
            InternetLED = confIt->value.GetUint();
          } else if (key.compare("pin_ActivityLED_0") == 0) {
            ActivityLED_0 = confIt->value.GetUint();
          } else if (key.compare("pin_ActivityLED_1") == 0) {
            ActivityLED_1 = confIt->value.GetUint();
          }
        }
      }
    } else if (objectType.compare("gateway_conf") == 0) {
      const Value& gateway_conf = fileIt->value;
      if (gateway_conf.IsObject()) {
        for (Value::ConstMemberIterator confIt = gateway_conf.MemberBegin(); confIt != gateway_conf.MemberEnd(); ++confIt) {
          string memberType(confIt->name.GetString());
          if (memberType.compare("ref_latitude") == 0) {
            lat = confIt->value.GetDouble();
          } else if (memberType.compare("ref_longitude") == 0) {
            lon = confIt->value.GetDouble();
          } else if (memberType.compare("ref_altitude") == 0) {
            alt = confIt->value.GetUint();
          } else if (memberType.compare("name") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(platform, str.length()<=24 ? str.c_str() : "name too long");
          } else if (memberType.compare("email") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(email, str.length()<=40 ? str.c_str() : "email too long");
          } else if (memberType.compare("desc") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(description, str.length()<=64 ? str.c_str() : "description is too long");
          } else if (memberType.compare("interface") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(interface, str.length()<=6 ? str.c_str() : "interface too long");
          } else if (memberType.compare("servers") == 0) {
            const Value& serverConf = confIt->value;
            if (serverConf.IsObject()) {
              const Value& serverValue = serverConf;
              Server_t server;
              for (Value::ConstMemberIterator srvIt = serverValue.MemberBegin(); srvIt != serverValue.MemberEnd(); ++srvIt) {
                string key(srvIt->name.GetString());
                if (key.compare("address") == 0 && srvIt->value.IsString()) {
                  server.address = srvIt->value.GetString();
                } else if (key.compare("port") == 0 && srvIt->value.IsUint()) {
                  server.port = srvIt->value.GetUint();
                } else if (key.compare("enabled") == 0 && srvIt->value.IsBool()) {
                  server.enabled = srvIt->value.GetBool();
                }
              }
              servers.push_back(server);
            }
            else if (serverConf.IsArray()) {
              for (SizeType i = 0; i < serverConf.Size(); i++) {
                const Value& serverValue = serverConf[i];
                Server_t server;
                for (Value::ConstMemberIterator srvIt = serverValue.MemberBegin(); srvIt != serverValue.MemberEnd(); ++srvIt) {
                  string key(srvIt->name.GetString());
                  if (key.compare("address") == 0 && srvIt->value.IsString()) {
                    server.address = srvIt->value.GetString();
                  } else if (key.compare("port") == 0 && srvIt->value.IsUint()) {
                    server.port = srvIt->value.GetUint();
                  } else if (key.compare("enabled") == 0 && srvIt->value.IsBool()) {
                    server.enabled = srvIt->value.GetBool();
                  }
                }
                servers.push_back(server);
              }
            }
          }
        }
      }
    }
  }
}

void PrintConfiguration() {
  for (vector<Server_t>::iterator it = servers.begin(); it != servers.end(); ++it) {
    printf("server: .address = %s; .port = %hu; .enable = %d\n", it->address.c_str(), it->port, it->enabled);
  }
  printf("Gateway Configuration\n");
  printf("  %s (%s)\n  %s\n", platform, email, description);
  printf("  Latitude=%.8f\n  Longitude=%.8f\n  Altitude=%d\n", lat,lon,alt);
  printf("  Interface: %s\n", interface);
}
