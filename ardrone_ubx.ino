// Copyright 2018 (c) Alex Gerchanovsky
// Pawelsky 20130805
// Note that this code is experimental, it may behave unexpectedly and cause potential damage!
// USE AT YOUR OWN RISK!!!
#include "SoftwareSerial.h"
#include "UbxGpsNav.h"
#include <ESP8266WiFi.h>
extern "C" {
#include "user_interface.h"
}
ADC_MODE(ADC_VCC);    //Required for getVcc()
RF_MODE(RF_DISABLED); //Disable RF permanently
RF_PRE_INIT() {system_phy_set_powerup_option(2);}//only calibrate VDD33 which will take about 2 ms

SoftwareSerial gpsSerial(2, SW_SERIAL_UNUSED_PIN, false, 256);
UbxGpsNav gps_ubx(gpsSerial);

#define USE_SIRF_CHECKSUM  // uncomment to calculate checksum for SiRF messages (e.g. for testing using SiRFDemo) - ignored by AR.Drone
//#define NMEA_GPS_DEBUG     // to see commands to/from NMEA GPS - for debug only, will not work with AR.Drne in this mode (disables SiRF messages)

void send_sirf(const u8 *payload, u16 len)
{
    static u8 begin[4] = {0xA0,0xA2,0,0};// start sequence
    begin[2] = len >> 8;
    begin[3] = len & 0xFF;
    Serial.write(begin, sizeof(begin));

    Serial.write(payload, len);
    u16 chksum = 0;
#ifdef USE_SIRF_CHECKSUM
    for(int i = 0; i < len; i++) {
      chksum = (chksum + payload[i]) & 0x7FFF;                // checksum calculation
    }
#endif
    static u8 end[4] = {0,0,0xB0,0xB3};
    end[0] = chksum >> 8;
    end[1] = chksum & 0xFF;
    Serial.write(end, sizeof(end));
}

static inline void send_sirf_ack(u8 id)
{
    // Cmmand Acknowledgement - ID 11
    static u8 SIRF_MSG_0B[] = {
        0x0B,    //[0] Message ID
        0x00 };  //[1] Ack ID
    SIRF_MSG_0B[1] = id;
    send_sirf(SIRF_MSG_0B, sizeof(SIRF_MSG_0B));
}

static inline void send_sirf_version()
{
    // Software Version String - ID 06
    //http://www.drone-forum.com/forum/viewtopic.php?f=67&t=6747&sid=699becac59da24e90bbd154e4c004665&start=135
    static const u8 SIRF_MSG_06[] = "\x06\x21\x01GSD4e_4.1.2-P6 F+ 08/14/2013 226\x00\x00";//"\x06XXX-XXX-4.1.1-P1";
    send_sirf(SIRF_MSG_06, sizeof(SIRF_MSG_06));
}

                                    //Offs Unit  Descripion
#define sirf_message_id                00//u8        Message ID 41 (0x29)
#define sirf_fix_invalid               01//u16       Nav Valid
#define sirf_fix_type                  03//u16       Nav Type
#define sirf_week                /**/  05//u16       Extended Week Number
#define sirf_tow                       07//u32 sec   Time of Week (1e3)
#define sirf_year                      11//u16 year  UTC Year
#define sirf_month                     13//u8  month UTC Month
#define sirf_day                       14//u8  day   UTC Day
#define sirf_hour                      15//u8  hour  UTC Hour
#define sirf_minute                    16//u8  min   UTC Minute
#define sirf_msecond                   17//u16 msec  UTC Millisecond
#define sirf_satellites_used     /**/  19//u32 bits  Satellite ID list
#define sirf_latitude                  23//u32 deg   Latitude (1e7)
#define sirf_longitude                 27//u32 deg   Longitude (1e7)
#define sirf_altitude_ellipsoid        31//u32 cm    Altitude from Ellipsoid
#define sirf_altitude_msl              35//u32 cm    Altitude from MSL
#define sirf_map_datum                 39//u8        Map Datum
#define sirf_sog                       40//u16 cm/s  Speed Over Ground - SOG
#define sirf_cog                       42//u16 deg   Course Over Ground - COG, True (deg cw from true N x100)
#define sirf_magnetic_variation  /**/  44//u16       Magnetic Variation (not implemented)
#define sirf_climb_rate                46//u16 m/s   Climb Rate (1e2)
#define sirf_heading_rate        /**/  48//u16 deg/s Heading Rate (1e2)
#define sirf_horizontal_position_error 50//u32 cm    Estimated Horizontal Position Error
#define sirf_vertical_position_error   54//u32 cm    Estimated Vertical Position Error
#define sirf_time_error                58//u32 sec   Estimated Time Error (1e2)
#define sirf_horizontal_velocity_error 62//u16 cm/s  Estimated Horizontal Velocity Error
#define sirf_clock_bias                64//u32 cm    Clock Bias
#define sirf_clock_bias_error    /**/  68//u32 cm    Clock Bias Error
#define sirf_clock_drift               72//u32 cm/s  Clock Drift
#define sirf_clock_drift_error   /**/  76//u32 cm/s  Clock Drift Error
#define sirf_distance            /**/  80//u32 m     Distance - travelled since reset
#define sirf_distance_error      /**/  84//u16 m     Distance Error
#define sirf_heading_error             86//u16 deg   Heading Error (1e2)
#define sirf_satellites                88//u8        Number of SVs in fix
#define sirf_hdop                      89//u8        HDOP ( x5)
#define sirf_mode_info           /**/  90//u8        Additional Mode Info
#define sirf_msg_41_length 91

static UbxNavPvtProp gps;
static UbxNavClockProp clk;
static u8 sirf_geonav[sirf_msg_41_length];
static inline void set8(u8 offs,u8 b)
{
    sirf_geonav[offs] = b;
}
static inline void set16(u8 offs, u16 w)
{
    sirf_geonav[offs++] = (u8)(w>>8);
    sirf_geonav[offs]   = (u8)w;
}
static inline void set32(u8 offs, u32 dw)
{
    sirf_geonav[offs++] = (u8)(dw>>24);
    sirf_geonav[offs++] = (u8)(dw>>16);
    sirf_geonav[offs++] = (u8)(dw>>8);
    sirf_geonav[offs]   = (u8)dw;
}

// AR.Drone requested messages 4,41,66
//http://dev.gateworks.com/datasheets/w2sg0008i/One_Socket_Protocol_Interface_Control_Document_(OSP_ICD)_(CS-129291-DC-15).pdf
//>>> Enabling message 4 at rate 1            Measured Tracker Data Out
//>>> Enabling message 41(0x29) at rate 1     Geodetic Navigation Data
//>>> Enabling message 66(0x42) at rate 1     DOP Values Output ??? http://www.mt-system.ru/sites/default/files/docs/documents/sim18%20module%20osp%20manual%20%28cs-129291-dc-8%29%5B1%5D.pdf
static inline void send_sirf_gps()
{
    //Geodetic Navigation Data - Message ID 41
    //SiRF_Binary_Reference_Manual_2.3.pdf
    u16 fix_invalid = 0;
    if (gps.numSV < 5)
        fix_invalid |= (1<<0);// no fix
    if (gps.fixType == no_fix)
        fix_invalid |= (1<<6);// invalid position fix
    //if ((gps.flags & headVehValid)==0)
    //    fix_invalid |= (1<<7);// invalid heading
    
    u16 fix_type = 0;
    if (gps.numSV<4) {
        fix_type |= gps.numSV;
    } else {
        if (gps.numSV >= 5)
            fix_type |= (1<<9);
        switch (gps.fixType) {
        case fix_2d: fix_type |= 5; break;
        case fix_3d: fix_type |= 6; break;
        //case gnss: fix_type |= 7; break;
        default:     fix_type |= 4; break;
        }
    }
    memset(&sirf_geonav, 0, sizeof(sirf_geonav));
    // Prepare and send message 41 (0x29)
    set8(sirf_message_id,                 41); // Message ID 41 //00
    set8(sirf_map_datum,                  21); // WGS-84        //39
    set8(sirf_satellites,                 gps.numSV);           //88
    set16(sirf_fix_invalid,               fix_invalid);         //01
    set16(sirf_fix_type,                  fix_type);            //03
    if (gps.valid & (validDate|fullyResolved)) {
    set16(sirf_year,                      gps.year);            //11
    set8(sirf_month,                      gps.month);           //13
    set8(sirf_day,                        gps.day);             //14
    }
    if (gps.valid & (validTime|fullyResolved)) {
    set32(sirf_tow,                       gps.iTOW);            //07
    set8(sirf_hour,                       gps.hour);            //15
    set8(sirf_minute,                     gps.min);             //16
    set16(sirf_msecond,                   (int)gps.sec*1000+gps.nano/1000000);
    }
    if (gps.fixType>no_fix || (gps.flags & gnssFixOK)) {
    set32(sirf_latitude,                  gps.lat);             //23
    set32(sirf_longitude,                 gps.lon);             //27
    set32(sirf_altitude_ellipsoid,        gps.height/10);       //31
    set32(sirf_altitude_msl,              gps.hMSL/10);         //35
    set16(sirf_sog,                       gps.gSpeed/10);       //40
    set16(sirf_cog,                       gps.heading/1000);    //42
    set16(sirf_climb_rate,                gps.velD/10); //? -   //46
    set32(sirf_horizontal_position_error, gps.hAcc/10);         //50
    set32(sirf_vertical_position_error,   gps.vAcc/10);         //54
    set32(sirf_time_error,                gps.tAcc/10000000);   //58
    set16(sirf_horizontal_velocity_error, gps.sAcc/10);         //62
    set16(sirf_heading_error,             gps.headingAcc/1000); //86
    set8(sirf_hdop,                       gps.pDOP/20); //100/5 //89
    }
    set32(sirf_clock_bias,                clk.clkbias/10000000);//64
    set32(sirf_clock_drift,               clk.clkdrift/10000000);//72
    send_sirf((const u8 *)&sirf_geonav, sizeof(sirf_geonav));
}

void setup()
{
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    Serial.begin(115200);//38400);
    gpsSerial.begin(38400);
    // Set Static nav: OFF, SBAS: ON, WAAS: ON, SBAS TEST MODE: ON
#ifdef NMEA_GPS_DEBUG
    Serial.printf("id:%02x size:%d\n",UBX_NAV_PVT,sizeof(UbxNavPvtProp));
    Serial.printf("id:%02x size:%d\n",UBX_NAV_CLOCK,sizeof(UbxNavClockProp));
#endif
}

static u8 msgId = 0, msgLen = 0, seq = 0;
static u32 lastTime = 0;

void loop()
{
#if 1//def NMEA_GPS_DEBUG
    while (gps_ubx.ready()) {
#define S(a) String(a).c_str()
        switch (gps_ubx.id()) {
        case UBX_NAV_PVT:
            gps = gps_ubx.pvt;
#ifdef NMEA_GPS_DEBUG
            //Serial.printf("%04d.%02d.%02d %02d:%02d:%02d.%d acc=%dms ", gps.year, gps.month, gps.day, gps.hour, gps.min, gps.sec, gps.nano/1000000, gps.tAcc/1000000);
            Serial.printf("valid=%02X flags=%02x flags2=%02x fixType=%d SVs=%d pDOP=%d/100 ", gps.valid, gps.flags, gps.flags2, gps.fixType, gps.numSV, gps.pDOP);
            Serial.printf("lat/lon=%s,%s height/msl=%s/%s ",S(gps.lat*1e-7),S(gps.lon*1e-7),S(gps.height/1000.0),S(gps.hMSL/1000.0));
            //Serial.printf("speed=%s climb=%s heading=%s headVeh=%s ",S(gps.gSpeed/1000),S(-gps.velD/1000.0),S(gps.heading*1e-5),S(gps.headVeh*1e-5));
            //Serial.printf("Accuracy:H/V/S=%s/%s/%sm/s coarse=%sdeg/s ",S(gps.hAcc/1000.0),S(gps.vAcc/1000.0),S(gps.sAcc/1000.0),S(gps.headingAcc*1e-5));
            Serial.println();
#endif
            break;
        case UBX_NAV_CLOCK:
            clk = gps_ubx.clk;
#ifdef NMEA_GPS_DEBUG
            Serial.printf("clkbias:%s clkdrift=%s\n", S(clk.clkbias/10000000), S(clk.clkdrift/10000000));
#endif
            break;
        default:
#ifdef NMEA_GPS_DEBUG
            Serial.printf("Unknown id=%02x\n", gps_ubx.id());
#endif
            ;
        }
    }
#else
    int i = 0;
    while (gpsSerial.available() > 0) {
        Serial.printf("%02X ",gpsSerial.read());
        i++;
        if ((i%32)==0)
            Serial.println();
    }
    if (i>0)
        Serial.println();
#endif
    while (Serial.available() > 0) {
#ifdef NMEA_GPS_DEBUG
        gpsSerial.write(Serial.read());
#else
        int c = Serial.read();
        if (c>=0)
        switch (seq) {
        case 0: if (c == 0xA0) { seq++; }; break;// start sequence 2 bytes
        case 1: if (c == 0xA2) { seq++; }; break;
        case 2: seq++; break;// length 2 bytes
        case 3: seq++; msgLen = (u8)c; break;
        case 4: seq++; msgId = (u8)c; break;//Message ID
        //skip: Control + 2 byte checksum
        case 5: if (c == 0xB0) { seq++; }; break;// end sequence 2 bytes
        case 6: if (c == 0xB3) { seq = 0; }; break;
        //default:
            //if (seq<(6+msgLen) || ((seq==(6+msgLen) && c==0xB0)) seq++; else
            //if (seq==(7+msgLen) && c==0xB3) seq = 0; else
            //seq = 0;
        }
#endif
    }

#ifndef NMEA_GPS_DEBUG
    if ((seq == 0) && (msgId > 0)) {
        if (msgId == 0x84)
            send_sirf_version();
        send_sirf_ack(msgId);
        msgId = 0;
    } else if ((millis() - lastTime) >= 200) {
        lastTime = millis();
        send_sirf_gps();
    }
#endif
}

