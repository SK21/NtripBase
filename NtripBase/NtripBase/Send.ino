void SendComm()
{
    if (millis() - SendLast > SendTime)
    {
        SendLast = millis();

        //PGN5000, RTKbase info
        //0     136
        //1     19
        //2     bytes sent 0
        //3     sent 1
        //4     sent 2
        //5     sent 3
        //6     Status
        //      bit 0 - connected
        //      bit 1 - RTCM message found
        //7     hangup count 
        //8     bad packet
        //9     InoID lo
        //10    InoID hi
        //11    IP0
        //12    IP1
        //13    IP2
        //14    IP3
        //15    CRC

        const uint16_t PGNlength = 16;
        byte data[PGNlength];

        data[0] = 136;
        data[1] = 19;
        data[2] = BytesSent;
        data[3] = BytesSent >> 8;
        data[4] = BytesSent >> 16;
        data[5] = BytesSent >> 24;

        data[6] = 0;
        if (ntripClient.connected()) data[6] |= 0b00000001;
        if (inRtcmMessage) data[6] |= 0b00000010;

        data[7] = HangUpCount;
        data[8] = BadPacket;

        data[9] = (byte)InoID;
        data[10] = InoID >> 8;

        data[11] = Ethernet.localIP()[0];
        data[12] = Ethernet.localIP()[1];
        data[13] = Ethernet.localIP()[2];
        data[14] = Ethernet.localIP()[3];

        data[15] = CRC(data, PGNlength - 1, 0);

        if (Ethernet.linkStatus() == LinkON)
        {
            UDPcomm.beginPacket(DestinationIP, DestinationPort);
            UDPcomm.write(data, PGNlength);
            UDPcomm.endPacket();
        }
    }
}
