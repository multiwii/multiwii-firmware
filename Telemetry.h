#ifndef TELEMETRY_H_
#define TELEMETRY_H_
// ****************************************************************
// FrSky telemetry
// Version: 0.4.0
// Changes: V0.4.0: - supports 2.4
// Version: 0.3.0
// Cahnges: V0.3.0: - new structure with cpp/h-files
// Date 20/09/2012
// Changes: V0.2.1: - make it work with 2.1 (shared dev)
// Date: 14/08/2012
// Changes: V0.2: - Byte stuffing added
//                - vBat will be send, if "#define FAS_100" is comment out
//                V0.1: - First release
// ****************************************************************

#if defined(FRSKY_TELEMETRY)
    // Definitions
    #define TELEMETRY_BAUD     9600  

    // Frame protocol
    #define Protocol_Header    0x5E
    #define Protocol_Tail      0x5E

    // Data Ids  (bp = before point; af = after point)
    // Official data IDs
    #define ID_GPS_Altitude_bp    0x01
    #define ID_GPS_Altitude_ap    0x09
    #define ID_Temperature1       0x02
    #define ID_RPM                0x03
    #define ID_Fuel_level         0x04
    #define ID_Temperature2       0x05
    #define ID_Volt               0x06
    #define ID_Altitude_bp        0x10
    #define ID_Altitude_ap        0x21
    #define ID_GPS_speed_bp       0x11
    #define ID_GPS_speed_ap       0x19
    #define ID_Longitude_bp       0x12
    #define ID_Longitude_ap       0x1A
    #define ID_E_W                0x22
    #define ID_Latitude_bp        0x13
    #define ID_Latitude_ap        0x1B
    #define ID_N_S                0x23
    #define ID_Course_bp          0x14
    #define ID_Course_ap          0x1C
    #define ID_Date_Month         0x15
    #define ID_Year               0x16
    #define ID_Hour_Minute        0x17
    #define ID_Second             0x18
    #define ID_Acc_X              0x24
    #define ID_Acc_Y              0x25
    #define ID_Acc_Z              0x26
    #define ID_Voltage_Amp_bp     0x3A
    #define ID_Voltage_Amp_ap     0x3B
    #define ID_Current            0x28
    #define ID_VFAS               0x39
#endif // FRSKY_TELEMETRY

#if defined(SPORT_TELEMETRY)

    #define TELEMETRY_BAUD   57600  
    #define FRSKY_START_STOP  0x7e
    #define FRSKY_BYTESTUFF   0x7d
    #define FRSKY_STUFF_MASK  0x20

    // FrSky data IDs (2 bytes)
    #define FRSKY_SPORT_RSSI_ID           0xf101
    #define FRSKY_SPORT_ADC1_ID           0xf102 // A1
    #define FRSKY_SPORT_ADC2_ID           0xf103 // A2
    #define FRSKY_SPORT_BATT_ID           0xf104
    #define FRSKY_SPORT_SWR_ID            0xf105
    #define FRSKY_SPORT_T1_ID             0x0400
    #define FRSKY_SPORT_T2_ID             0x0410
    #define FRSKY_SPORT_RPM_ID            0x0500
    #define FRSKY_SPORT_FUEL_ID           0x0600
    #define FRSKY_SPORT_ALT_ID            0x0100
    #define FRSKY_SPORT_VARIO_ID          0x0110
    #define FRSKY_SPORT_ACCX_ID           0x0700
    #define FRSKY_SPORT_ACCY_ID           0x0710
    #define FRSKY_SPORT_ACCZ_ID           0x0720
    #define FRSKY_SPORT_CURR_ID           0x0200
    #define FRSKY_SPORT_VFAS_ID           0x0210
    #define FRSKY_SPORT_CELLS_ID          0x0300
    #define FRSKY_SPORT_GPS_LONG_LATI_ID  0x0800
    #define FRSKY_SPORT_GPS_ALT_ID        0x0820
    #define FRSKY_SPORT_GPS_SPEED_ID      0x0830
    #define FRSKY_SPORT_GPS_COURS_ID      0x0840
    #define FRSKY_SPORT_GPS_TIME_DATE_ID  0x0850

    // FrSky sensor IDs (this also happens to be the order in which they're broadcast from an X8R)
    // NOTE: As FrSky puts out more sensors let's try to add comments here indicating which is which
    #define FRSKY_SPORT_DEVICE_1   0xa1
    #define FRSKY_SPORT_DEVICE_2   0x22
    #define FRSKY_SPORT_DEVICE_3   0x83
    #define FRSKY_SPORT_DEVICE_4   0xe4
    #define FRSKY_SPORT_DEVICE_5   0x45
    #define FRSKY_SPORT_DEVICE_6   0xc6
    #define FRSKY_SPORT_DEVICE_7   0x67
    #define FRSKY_SPORT_DEVICE_8   0x48
    #define FRSKY_SPORT_DEVICE_9   0xe9
    #define FRSKY_SPORT_DEVICE_10  0x6a
    #define FRSKY_SPORT_DEVICE_11  0xcb
    #define FRSKY_SPORT_DEVICE_12  0xac
    #define FRSKY_SPORT_DEVICE_13  0xd
    #define FRSKY_SPORT_DEVICE_14  0x8e
    #define FRSKY_SPORT_DEVICE_15  0x2f
    #define FRSKY_SPORT_DEVICE_16  0xd0
    #define FRSKY_SPORT_DEVICE_17  0x71
    #define FRSKY_SPORT_DEVICE_18  0xf2
    #define FRSKY_SPORT_DEVICE_19  0x53
    #define FRSKY_SPORT_DEVICE_20  0x34
    #define FRSKY_SPORT_DEVICE_21  0x95
    #define FRSKY_SPORT_DEVICE_22  0x16
    #define FRSKY_SPORT_DEVICE_23  0xb7
    #define FRSKY_SPORT_DEVICE_24  0x98
    #define FRSKY_SPORT_DEVICE_25  0x39
    #define FRSKY_SPORT_DEVICE_26  0xba
    #define FRSKY_SPORT_DEVICE_27  0x1b

#endif // S.PORT telemetry

// Variables
extern uint32_t armedTime;

// Functions
void init_telemetry(void);
void run_telemetry(void);
#endif /* TELEMETRY_H_ */

