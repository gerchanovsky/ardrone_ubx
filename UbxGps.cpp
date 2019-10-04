#include "UbxGps.h"

UbxGps::UbxGps(SoftwareSerial &serial, u8 c, u8 i, u16 len)
  : serial(serial), position(0), hdr{{0xB5,0x62},c,i,len} {
    hdr.magic[0] = 0xB5;
    hdr.magic[1] = 0x62;
    hdr.cls = c;
    hdr.id = i;
    hdr.length = sizeof(UbxHdr) - sizeof(UbxHdr::magic) + len;
}

void UbxGps::calculateChecksum()
{
    checksum[0] = 0;
    checksum[1] = 0;
    u8 i;
    for (i = 0; i < sizeof(hdr)-sizeof(hdr.magic); i++) {
        checksum[0] += ((u8*)&hdr.cls)[i];
        checksum[1] += checksum[0];
    }
    for (i = 0; i < hdr.length - (sizeof(UbxHdr) - sizeof(UbxHdr::magic)); i++) {
        checksum[0] += ((u8*)&payload)[i];
        checksum[1] += checksum[0];
    }
}

boolean UbxGps::ready()
{
    int pos = position;
    while (serial.available() > 0) {
        u8 c = serial.read();
        if (pos < sizeof(hdr)) {
            // Carriage is at the first or the second sync byte, should be equals
            u8 *p = ((u8*)&hdr)+pos;
            if (c != *p) {
                if ((pos==3 && valid_id(c)) ||
                    (pos==4 && valid_len(c-(sizeof(UbxHdr)-sizeof(UbxHdr::magic))))) {
                    *p = c;      
                } else {
#if 0
                    if (pos>1) Serial.printf("HDR id=%02x pos=%d %02xh(%d) %02xh(%d)\n", hdr.id, pos, c, c, *p, *p);
#endif
                    pos = 0; // Reset if not
                    continue;
                }
            }
        } else if (pos < hdr.length + sizeof(hdr.magic)) {
            ((u8*)&payload)[pos-sizeof(hdr)] = c;
        } else if (pos == hdr.length + sizeof(hdr.magic)) {
            calculateChecksum();
#ifndef IGNORE_CHKSUM
            if (c != checksum[0]) {// Reset if not
                pos = 0;
                continue;
            }
#endif
        } else {//if (pos > hdr.length + sizeof(hdr) + 1) {
//Serial.printf("HDR pos=%d class=%d id=%d size=%d-%d chksum=0x%02x %02x-%02x\n", pos, c(int)hdr.Class, (int)hdr.Id, (int)hdr.Length, hdr.Length,checksum[1], checksum[0], c);
#ifndef IGNORE_CHKSUM
            if (c != checksum[1]) {
                pos = 0;
                continue;
            }
#endif
            position = 0;// Reset the carriage
            // The readings are correct and filled the object, return true
            return true;
        }
        pos++;// Move the carriage forward
    }
    position = pos;
    return false;
}
