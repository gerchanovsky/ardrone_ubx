#pragma once
#include "Arduino.h"
#include "SoftwareSerial.h"

#define IGNORE_CHKSUM // My GPS always send 0 checksum

typedef struct {//__attribute__((packed, aligned(1))) {
    u8   magic[2];
    u8   cls;
    u8   id;
    u16  length;
} UbxHdr;

class UbxGps
{
  protected:
    UbxGps(SoftwareSerial &serial, u8 c, u8 i, u16 len);
    virtual boolean valid_id(u8 id) = 0;
    virtual boolean valid_len(u8 len) = 0;

  public:
    boolean ready();
    u8 id() {return hdr.id;}

  private:
    void calculateChecksum();
    // Class properties
    SoftwareSerial &serial;
    u8 position;
    u8 checksum[2];
    // Headers (common)
  protected:
    UbxHdr hdr;
    u32 payload[0];  // mind dword allignment (don't glue with hdr - you'll ruin allignment!)
};
