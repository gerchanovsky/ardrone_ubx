#pragma once
#include "UbxGps.h"

//u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221).pdf
// valid mask
enum validMask {
    validDate     =  (1<<0), //1 = valid UTC Date (see Time Validity section for details)
    validTime     =  (1<<1), //1 = valid UTC Time of Day (see Time Validity section for details)
    fullyResolved =  (1<<2), //1 = UTC Time of Day has been fully resolved (no seconds uncertainty)
    validMag      =  (1<<3)  //1 = valid Magnetic declination
};

enum FixType {
    no_fix,    //0: no fix
    dead,      //1: dead reckoning only
    fix_2d,    //2: 2D-fix
    fix_3d,    //3: 3D-fix
    gnss,      //4: GNSS + dead reckoning combined
    time_only  //5: time only fix
};

//flags
enum flagsMask {
    gnssFixOK    = (1<<0),//1 = valid fix (i.e within DOP & accuracy masks)
    diffSoln     = (1<<1),//1 = differential corrections were applied
    headVehValid = (1<<5) //1 = heading of vehicle is valid
};

//flags2
enum flags2Mask {
    confirmedAvai = (1<<5),//1 = information about UTC Date and Time of Day validity confirmation is available (see Time Validity section for details)
    confirmedDate = (1<<6),//1 = UTC Date validity could be confirmed (see Time Validity section for details)
    confirmedTime = (1<<7) //1 = UTC Time of Day could be confirmed (see Time Validity section for details)
};

typedef struct {
    //Type Name        Offs Unit  Description (scaling)
    u32  iTOW;       // 0:  ms    GPS time of week of the navigation epoch. See the description of iTOW for details
    u16  year;       // 4:  y     Year UTC
    u8   month;      // 6:  month Month, range 1..12 UTC
    u8   day;        // 7:  d     Day of month, range 1..31 UTC
    u8   hour;       // 8:  h     Hour of day, range 0..23 UTC
    u8   min;        // 9:  min   Minute of hour, range 0..59 UTC
    u8   sec;        //10:  s     Seconds of minute, range 0..60 UTC
    s8   valid;      //11:  -     Validity Flags (see graphic below)
    u32  tAcc;       //12:  ns    Time accuracy estimate UTC
    s32  nano;       //16:  ns    Fraction of second, range -1e9..1e9 UTC
    u8   fixType;    //20:  -     GNSSfix Type, range 0..5
    s8   flags;      //21:  -     Fix Status Flags (see graphic below)
    u8   flags2;     //22:  -     Reserved
    u8   numSV;      //23:  -     Number of satellites used in Nav Solution
    s32  lon;        //24:  deg   Longitude (1e-7)
    s32  lat;        //28:  deg   Latitude (1e-7)
    s32  height;     //32:  mm    Height above Ellipsoid
    s32  hMSL;       //36:  mm    Height above mean sea level
    u32  hAcc;       //40:  mm    Horizontal Accuracy Estimate
    u32  vAcc;       //44:  mm    Vertical Accuracy Estimate
    s32  velN;       //48:  mm/s  NED north velocity
    s32  velE;       //52:  mm/s  NED east velocity
    s32  velD;       //56:  mm/s  NED down velocity
    s32  gSpeed;     //60:  mm/s  Ground Speed (2-D)
    s32  heading;    //64:  deg   Heading of motion 2-D (1e-5)
    u32  sAcc;       //68:  mm/s  Speed Accuracy Estimate
    u32  headingAcc; //72:  deg   Heading Accuracy Estimate (1e-5)
    u16  pDOP;       //76:  -     Position DOP (0.01)
    u8   reserved[6];//78:  -     Reserved
    s32  headVeh;    //84: deg   Heading of vehicle (2-D)  (1e-5)
    //s16  magDec;   //88: deg   Magnetic declination (1e-2)
    //u16  magAcc;   //90: deg   Magnetic declination accuracy (1e-2)
} UbxNavPvtProp;// __attribute__ ((packed));
#define UBX_NAV_PVT 0x07

typedef struct {
    u32 iTOW;
    s32 clkbias;   // clock bias in nanoseconds
    s32 clkdrift;  // clock drift in ns/s
    u32 tacc;      // time accuracy estimate (ns)
    //u32 facc;      // frequency accuracy estimate (ps/s)
} UbxNavClockProp;// __attribute__ ((packed));
#define UBX_NAV_CLOCK 0x22

class UbxGpsNav : public UbxGps {
public:
    union {
      UbxNavPvtProp pvt;
      UbxNavClockProp clk;
    };
    UbxGpsNav(SoftwareSerial &serial) : UbxGps(serial, 0x01, UBX_NAV_PVT, sizeof(UbxNavPvtProp)) {}
    virtual boolean valid_id(u8 id) {return (id==UBX_NAV_PVT)||(id==UBX_NAV_CLOCK);}
    virtual boolean valid_len(u8 len) {return len==((hdr.id==UBX_NAV_PVT)?sizeof(UbxNavPvtProp):sizeof(UbxNavClockProp));}
};
