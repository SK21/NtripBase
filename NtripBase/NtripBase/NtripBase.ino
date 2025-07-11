
const char* ntripMountPoint = "MountPoint";
const char* casterPassword = "Password";
uint8_t ReceiverSerialPort = 8;	// gps receiver
const uint32_t ReceiverBaud = 460800;


#define InoDescription "NtripBase   11-Jul-2025"
const uint16_t InoID = 11075;	// change to send defaults to eeprom, ddmmy, no leading 0
const uint8_t InoType = 7;		// 0 - Teensy AutoSteer, 1 - Teensy Rate, 2 - Nano Rate, 3 - Nano SwitchBox, 4 - ESP Rate, 7 NtripBase

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "FXUtil.h"
extern "C" {
#include "FlashTxx.h"
}

#define ReceiveBufferSize 1024

EthernetClient ntripClient;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

const char* ntripServer = "rtkdata.online";
const int ntripPort = 2101;

HardwareSerial* SerialReceiver;

uint32_t BytesSent = 0;
uint32_t LastSentAfterConnection = 0;
uint32_t lastReport_ms = 0;
uint32_t lastConnectionAttempt = 0;
uint32_t LastSent = 0;

uint16_t maxTimeBeforeHangup_ms = 10000;

const uint32_t RestartInterval = 300000;
const uint32_t connectionInterval = 5000;

uint8_t HangUpCount = 0;
uint8_t BadPacket = 0;

char ReceiveBufferData[ReceiveBufferSize];
uint16_t ReceiveBufferIndex = 0;
bool inRtcmMessage = false;
uint16_t expectedRtcmLength = 0;


// ethernet comm
EthernetUDP UDPcomm;
uint16_t ListeningPort = 5710;
uint16_t DestinationPort = 5350;
IPAddress DestinationIP(192, 168, 1, 255);

const uint16_t SendTime = 1000;
uint32_t SendLast = SendTime;

// firmware update
EthernetUDP UpdateComm;
uint16_t UpdateReceivePort = 29100;
uint16_t UpdateSendPort = 29000;
uint32_t buffer_addr, buffer_size;
bool UpdateMode = false;

//******************************************************************************
// hex_info_t struct for hex record and hex file info
//******************************************************************************
typedef struct {  //
    char* data;   // pointer to array allocated elsewhere
    unsigned int addr;  // address in intel hex record
    unsigned int code;  // intel hex record type (0=data, etc.)
    unsigned int num; // number of data bytes in intel hex record

    uint32_t base;  // base address to be added to intel hex 16-bit addr
    uint32_t min;   // min address in hex file
    uint32_t max;   // max address in hex file

    int eof;    // set true on intel hex EOF (code = 1)
    int lines;    // number of hex records received
} hex_info_t;

static char data[16];// buffer for hex data

hex_info_t hex =
{ // intel hex info struct
  data, 0, 0, 0,        //   data,addr,num,code
  0, 0xFFFFFFFF, 0,     //   base,min,max,
  0, 0					//   eof,lines
};

uint32_t lastRtcmByteReceived = 0;
const uint16_t maxRtcmMessageTime = 1000; // 100ms max to complete a RTCM message

void ConnectCaster()
{
    if (!ntripClient.connected())
    {
        Serial.printf("Opening socket to %s\n", ntripServer);

        if (ntripClient.connect(ntripServer, ntripPort))
        {
            Serial.printf("Connected to %s:%d\n", ntripServer, ntripPort);

            char serverRequest[512];
            snprintf(serverRequest, sizeof(serverRequest),
                "SOURCE %s /%s\r\nSource-Agent: Teensy NTRIP\r\n\r\n",
                casterPassword, ntripMountPoint);

            Serial.println(F("Sending server request:"));
            Serial.println(serverRequest);
            ntripClient.write(serverRequest, strlen(serverRequest));

            unsigned long timeout = millis();
            while (!ntripClient.available())
            {
                if (millis() - timeout > 5000)
                {
                    Serial.println("Caster timed out!");
                    ntripClient.stop();
                    return;
                }
                delay(10);
            }

            char response[512] = { 0 };
            int responseSpot = 0;
            bool connectionSuccess = false;

            while (ntripClient.available() && responseSpot < (int)(sizeof(response) - 1))
            {
                response[responseSpot++] = ntripClient.read();
                if (strstr(response, "200"))
                    connectionSuccess = true;
            }

            if (connectionSuccess)
            {
                Serial.println("Connected to Caster");
            }
            else
            {
                Serial.printf("Failed to connect to Caster: %s\n", response);
                ntripClient.stop();
            }
        }
        else
        {
            Serial.println("Connection to host failed");
            delay(2000);
        }

        lastConnectionAttempt = millis();
    }
}

void setup()
{
    Serial.begin(38400);
    delay(5000);
    Serial.println();
    Serial.println(InoDescription);
    Serial.println();

    Serial.println("Initializing Ethernet...");
    if (Ethernet.begin(mac) == 0)
    {
        Serial.println("DHCP failed.");
    }
    delay(1000);

    Serial.print("IP Address: ");
    Serial.println(Ethernet.localIP());

    DestinationIP = IPAddress(Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], 255);

    UDPcomm.begin(ListeningPort);
    UpdateComm.begin(UpdateReceivePort);

    // receive data from gps receiver
    switch (ReceiverSerialPort)
    {
    case 1:
        SerialReceiver = &Serial1;
        break;
    case 2:
        SerialReceiver = &Serial2;
        break;
    case 3:
        SerialReceiver = &Serial3;
        break;
    case 4:
        SerialReceiver = &Serial4;
        break;
    case 5:
        SerialReceiver = &Serial5;
        break;
    case 6:
        SerialReceiver = &Serial6;
        break;
    case 7:
        SerialReceiver = &Serial7;
        break;
    default:
        SerialReceiver = &Serial8;
        break;
    }

    SerialReceiver->begin(ReceiverBaud);

    Serial.println("Finished Setup.");
}

void loop()
{
    if (!UpdateMode)
    {
        if (ntripClient.connected())
        {
            while (SerialReceiver->available())
            {
                char incomingByte = SerialReceiver->read();

                if (!inRtcmMessage)
                {
                    if (incomingByte == 0xD3)
                    {
                        ReceiveBufferIndex = 0;
                        ReceiveBufferData[ReceiveBufferIndex++] = incomingByte;
                        inRtcmMessage = true;
                        lastRtcmByteReceived = millis();
                    }
                }
                else
                {
                    if (ReceiveBufferIndex < ReceiveBufferSize)
                    {
                        ReceiveBufferData[ReceiveBufferIndex++] = incomingByte;
                        lastRtcmByteReceived = millis();

                        if (ReceiveBufferIndex == 3)
                        {
                            expectedRtcmLength = ((ReceiveBufferData[1] & 0x03) << 8) | ReceiveBufferData[2];
                            expectedRtcmLength += 6;
                        }

                        if (ReceiveBufferIndex == expectedRtcmLength)
                        {
                            ntripClient.write((const uint8_t*)ReceiveBufferData, ReceiveBufferIndex);
                            BytesSent += ReceiveBufferIndex;
                            ReceiveBufferIndex = 0;
                            inRtcmMessage = false;
                            LastSentAfterConnection = millis();
                            LastSent = millis();
                        }
                    }
                    else
                    {
                        ReceiveBufferIndex = 0;
                        inRtcmMessage = false;
                    }
                }
            }

            if (inRtcmMessage && (millis() - lastRtcmByteReceived > maxRtcmMessageTime))
            {
                Serial.println("Incomplete RTCM message, resetting buffer.");
                ReceiveBufferIndex = 0;
                inRtcmMessage = false;
                BadPacket++;
            }

            if (millis() - lastReport_ms > 1000)
            {
                lastReport_ms = millis();
                Serial.printf("Total sent: %d\n", BytesSent);
            }

            if (millis() - LastSentAfterConnection > maxTimeBeforeHangup_ms)
            {
                Serial.println("RTCM timeout. Disconnecting...");
                ntripClient.stop();
                HangUpCount++;
            }
        }
        else
        {
            if (millis() - lastConnectionAttempt > connectionInterval)
                ConnectCaster();

            LastSentAfterConnection = millis();

            if (millis() - LastSent > RestartInterval)
            {
                SCB_AIRCR = 0x05FA0004;
            }
        }
        SendComm();
    }
    ReceiveUpdate();
}

bool GoodCRC(byte Data[], byte Length)
{
    byte ck = CRC(Data, Length - 1, 0);
    bool Result = (ck == Data[Length - 1]);
    return Result;
}

byte CRC(byte Chk[], byte Length, byte Start)
{
    byte Result = 0;
    int CK = 0;
    for (int i = Start; i < Length; i++)
    {
        CK += Chk[i];
    }
    Result = (byte)CK;
    return Result;
}

