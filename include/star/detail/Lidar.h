/**
 * @author   lucb
 * @date     2020/3/2
 */

#ifndef __STAR_SDK_DETAIL_LIDAR_H
#define __STAR_SDK_DETAIL_LIDAR_H

namespace ss {
namespace detail {

//GM��������
const uint8_t RFANS_PRODUCT_MODEL_V6G_X16_0X32 = 0X32;
const uint8_t RFANS_PRODUCT_MODEL_V6G_X32_0X33 = 0X33;
const uint8_t RFANS_PRODUCT_MODEL_V6_X32_0X40 = 0X40;
const uint8_t RFANS_PRODUCT_MODEL_V6_X16A_0X41 = 0X41;
const uint8_t RFANS_PRODUCT_MODEL_V6_X16B_0X42 = 0X42;
const uint8_t RFANS_PRODUCT_MODEL_V6_X16Even_0X43 = 0X43;
const uint8_t RFANS_PRODUCT_MODEL_V6_X16Odd_0X44 = 0X44;
const uint8_t RFANS_PRODUCT_MODEL_V6P_X32_0X45 = 0X45;
const uint8_t RFANS_PRODUCT_MODEL_V6P_X16A_0X46 = 0X46;
const uint8_t RFANS_PRODUCT_MODEL_V6P_X16B_0X47 = 0X47;
const uint8_t RFANS_PRODUCT_MODEL_V6P_X16Even_0X48 = 0X48;
const uint8_t RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49 = 0X49;
const uint8_t RFANS_PRODUCT_MODEL_V6A_X32_0X4A = 0X4A;
const uint8_t RFANS_PRODUCT_MODEL_V6A_X16A_0X4B = 0X4B;
const uint8_t RFANS_PRODUCT_MODEL_V6A_X16B_0X4C = 0X4C;
const uint8_t RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D = 0X4D;
const uint8_t RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E = 0X4E;
const uint8_t RFANS_PRODUCT_MODEL_V6A_X16M_0X4F = 0X4F;
const uint8_t RFANS_PRODUCT_MODEL_V6B_X32_0X50=0X50 ;
const uint8_t RFANS_PRODUCT_MODEL_CFANS_X32_0X80=0X80 ;
const uint8_t RFANS_PRODUCT_MODEL_CFANS_X8_0X81=0X81;
const uint8_t RFANS_PRODUCT_MODEL_CFANS_X32_0X83 = 0X83;//CFans128 V2.0 �汾
const uint8_t RFANS_PRODUCT_MODEL_CFANS_X8_0X84 = 0X84;
const uint8_t RFANS_PRODUCT_MODEL_CFANS_X64_0X86 = 0X86; //CFans256
const uint8_t RFANS_PRODUCT_MODEL_WFANS_X16_0X90 = 0X90;
const uint8_t RFANS_PRODUCT_MODEL_V6A_E1_0X55 = 0X55;
const uint8_t RFANS_PRODUCT_MODEL_V6A_E2_0X56 = 0X56;
const uint8_t RFANS_PRODUCT_MODEL_V6BC_16G_0X57 = 0X57;
const uint8_t RFANS_PRODUCT_MODEL_V6BC_16M_0X58 = 0X58;
const uint8_t RFANS_PRODUCT_MODEL_V6C_Z_X32_0X59 = 0X59;

const uint8_t RFANS_PRODUCT_MODEL_V6K_32M_0X5A = 0X5A;
const uint8_t RFANS_PRODUCT_MODEL_V6K_16M_0X5B = 0X5B;
const uint8_t RFANS_PRODUCT_MODEL_V6K_16M_0X24 = 0X24;
const uint8_t RFANS_PRODUCT_ZG_75W = 0x5C;
const uint8_t RFANS_PRODUCT_MODEL_V6H_32_0X5D = 0x5D;

const uint8_t RFANS_PRODUCT_MODEL_V6K_32MP = 0x60;

const uint8_t RFANS_PRODUCT_MODEL_V6K_32 = 0x5E;
const uint8_t RFANS_PRODUCT_MODEL_V6K_32P = 0x5F;

const uint8_t RFANS_PRODUCT_MODEL_V7M_32_0X29 = 0x29;
const uint8_t RFANS_PRODUCT_MODEL_V7_32_0X2A = 0x2A;



//BK��������
const unsigned char ID_RFANSBLOCK=0xAA;
const unsigned char ID_RFANSBLOCKV2=0x96;
const unsigned char SYNC_DEBUG_BLOCKV16 = 0xAA;
const unsigned char SYNC_RELEASE_BLOCKV16 = 0x96;
const unsigned char SYNC_DEBUG_BLOCKV32_0_15 = 0xAB;
const unsigned char SYNC_DEBUG_BLOCKV32_16_31 = 0xAC;
const unsigned char SYNC_DEBUG_BLOCKV32_0_15_AD = 0xAD;
const unsigned char SYNC_DEBUG_BLOCKV32_0_15_AE = 0xAE;
const unsigned char SYNC_RELEASE_BLOCKV32_0_15  = 0x97;
const unsigned char SYNC_RELEASE_BLOCKV32_16_31 = 0x98;
const unsigned char SYNC_RELEASE_BLOCKV32_V6G_0_15 = 0x99;
const unsigned char SYNC_RELEASE_BLOCKV32_V6G_16_31 = 0x9A;
const unsigned char SYNC_RELEASE_BLOCKV32_V6A_0_15 = 0x9B;
const unsigned char SYNC_RELEASE_BLOCKV32_V6A_16_31 = 0x9C;
const unsigned char SYNC_RELEASE_BLOCKV32_CFANS128_0_15 = 0x9D;
const unsigned char SYNC_RELEASE_BLOCKV32_CFANS128_16_31 = 0x9E;
const unsigned char SYNC_RELEASE_BLOCKV32_V6A_16E1_16 = 0x9F;
const unsigned char SYNC_RELEASE_BLOCKV32_V6A_16E2_16 = 0x8F;
const unsigned char SYNC_V6GBLOCKV32 = 0xFF;
const unsigned char SYNC_V6GBLOCKV32_EE = 0xEE;

const unsigned char SYNC_RELEASE_BLOCKV16_V6BC_16G_16 = 0x8D;
const unsigned char SYNC_RELEASE_BLOCKV16_V6BC_16M_16 = 0x8C;
const unsigned char SYNC_RELEASE_BLOCKV32_V6B3_16_32 = 0x8E;

const unsigned char SYNC_RELEASE_BLOCKV16_V6C_Z_0_15 = 0x8A;
const unsigned char SYNC_RELEASE_BLOCKV16_V6C_Z_16_31 = 0x8B;

const double UTC_TI_DELTA[32] = {
	0,0,3.13,3.13,6.26,9.39,9.39,12.52,12.52,15.65,15.65,18.78,21.91,21.91,25.04,25.04,
	28.17,28.17,31.3,31.3,34.43,37.56,37.56,40.69,40.69,43.82,43.82,46.95,50.08,50.08,53.21,53.21
};

const double UTC_TI83_DELTA[32] = {
	0,0,25.04,25.04,3.13,3.13,28.17,28.17,6.26,6.26,31.3,31.3,9.39,9.39,34.43,34.43,
	12.52,12.52,37.56,37.56,15.65,15.65,40.69,40.69,18.78,18.78,43.82,43.82,21.91,21.91,46.95,46.95
};

const double UTC_TI85_DELTA[32] = {
	0,0.78,25.04,25.82,3.13,3.91,28.17,28.95,6.26,7.04,31.3,32.08,9.39,10.17,34.43,35.21,
	12.52,13.3,37.56,38.34,15.65,16.43,40.69,41.47,18.78,19.56,43.82,44.6,21.91,22.69,46.95,47.73
};

//RFans��ֱ�Ƕ�
static double VAngle[16] = {    -15.0, -13.0, -11.0, -9.0,  -7.0, -5.0,  -4.0, -3.0,  -2.0, -1.0,    0,  1.0, 3.0,  5.0, 7.0,  9.0};
static double VAngle_16E1[16] = { -19.5, -17.5, -15.5, -13.5, -11.5, -9.5, -7.5, -5.5, -3.5, -1.5, 0.5, 2.5, 4.5, 6.5, 8.5, 10.5 };
static double VAngle_16E2[16] = { -19, -17, -15, -13,-11, -9, -7, -5,-3, -1, 1, 3,5, 7, 9, 11};
static double VAngle_V6B_16M[16] = { -15, -13, -11, -9, -7, -5, -4, -3, -2, -1, 0, 1, 3, 5, 7, 9 };
static double VAngle_V6B_16G[16]={-15,-13,-11,-9,-7,-5,-3,-1,1,3,5,7,9,11,13,15};
static double  VAngle_V6C_Z_32[32] = {
        -15.5, -14.5, -13.5, -12.5
                             - 11.5, -10.5, -9.5, -8.5,
        -7.5, -6.5, -5.5, -4.5,
        -3.5, -2.5, -1.5, -0.5,
        +0.5, +1.5, +2.5, +3.5,
        +4.5, +5.5, +6.5, +7.5,
        +8.5, +9.5, +10.5, +11.5,
        +12.5, +13.5, +14.5, +15.5
};

const double VANGLE_V6_X32_0X40[] = {
        -20.5, -19.5, -18.5, -17.5,
        -16.5, -15.5, -14.5, -13.5,
        -12.5, -11.5, -10.5, -9.5,
        -8.5,   -7.5,  -6.5, -5.5,
        -4.5,   -3.5,  -2.5, -1.5,
        -0.5,    0.5,   1.5,  2.5,
        3.5,     4.5,   5.5,  6.5,
        7.5,     8.5,   9.5, 10.5,
};
const double VANGLE_V6A_X32[] = {
        -20, -19, -18, -17,
        -16, -15, -14, -13,
        -12, -11, -10,  -9,
        -8,  -7,  -6,  -5,
        -4,  -3,  -2,  -1,
        0,   1,   2,   3,
        4,   5,   6,   7,
        8,   9,  10,  11,
};
const double VANGLE_V6G_X32_0X33[] = {
        -25,    -22,   -19, -16,
        -13,    -11,    -9,  -7,
        -5.5,   -4.5,  -3.5, -2.9,
        -2.45,   -2.1, -1.75, -1.4,
        -1.05,   -0.7, -0.35,    0,
        0.35,    0.7,  1.05,  1.4,
        2.5,    3.5,   4.5,    6,
        8,     10,    12,   15,
};

//RFansˮƽ�Ƕȣ�����GM��ʽ����
const double HAngle[] = {
        -2.5, 2.5, -2.5, 2.5, -2.5,
        2.5, -2.5, 2.5, -2.5, 2.5,
        -2.5, 2.5, -2.5, 2.5, -2.5, 2.5
};
//const double HANGLE_V6G_X32_0X33[] = {
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
//       5.52, -3.48, 3.48, -5.52,
// };
const double HANGLE_V6G_X32_0X33[] = {
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
        5.21, -3.48, 3.04, -5.62,
};
const double HANGLE_V6B_X32_0x40[] = {
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
        6.01, -4.068, 3.377, -6.713,
};
const double HANGLE_V6_X32_0x40[] = {
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
        6.35, -3.85, 3.85, -6.35,
};

const double HANGLE_V6K_32M[] = {
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7,
        3.7, -6.35, 6.35, -3.7
};

const double HANGLE_V6K_16M[] = {
        1.325, -1.325, 1.325, -1.325,
        1.325, -1.325, 1.325, -1.325,
        1.325, -1.325, 1.325, -1.325,
        1.325, -1.325, 1.325, -1.325
};

const double HANGLE_V6K_16M_0X24[] = {
        -1.325, 1.325, -1.325, 1.325,
        -1.325, 1.325, -1.325, 1.325,
        -1.325, 1.325, -1.325, 1.325,
        -1.325, 1.325, -1.325, 1.325
};

const double HANGLE_V6A_E1_0x55[] = {
        -4.068, -6.713,-4.068, -6.713,
        -4.068, -6.713,-4.068, -6.713,
        -4.068, -6.713,-4.068, -6.713,
        -4.068, -6.713,-4.068, -6.713,
};

const double HANGLE_V6BC_16G_0x57[]={
        6.01, 3.377, 6.01, 3.377,
        6.01, 3.377, 6.01, 3.377,
        6.01, 3.377, 6.01, 3.377,
        6.01, 3.377, 6.01, 3.377,
};

const double HANGLE_V6BC_16M_0x58[]={
        2.1889,-2.8544,2.1889,-2.8544,
        2.1889,-2.8544,2.1889,-2.8544,
        2.1889,-2.8544,2.1889,-2.8544,
        2.1889,-2.8544,2.1889,-2.8544,
};


const double HANGLE_16ZG_75W[] = {
        6.01, 3.377, 6.01, 3.377,
        6.01, 3.377, 6.01, 3.377,
        6.01, 3.377, 6.01, 3.377,
        6.01, 3.377, 6.01, 3.377,
};

const double VANGLE_16ZG_75W[16] ={
        -15,-13,-11,-9,-7,-5,-3,-1,
        1,3,5,7,9,11,13,15};

static double VAngle_V6K_32M[32] = {
        -16.9, -15, -13.68, -12, -10.5, -9.5, -8.5, -7.5,
        -6.5, -6, -5.5, -5, -4.5, -4, -3.5, -3,
        -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1,
        2, 3, 4, 5, 6.37, 8, 9.26, 11
};

static double VAngle_V6K_16M[16] = {
        -12, -15, -7.5, -9.5, -5, -6, -3, -4,
        -1, -2, 1, 0, 5, 3, 11, 8
};

static const double VAngle_V6K_16M_0X24[16] = { -15, -12, -9.5, -7.5, -6, -5, -4, -3, -2, -1, 0, 1, 3, 5, 8, 11 };

static const double HANGLE_V6H_32_0x5D[] = {
        0.25, -0.25, 0.25, -0.25,
        0.25, -0.25,   -6,    -3,
        3,     6, 0.25,    -6,
        -3,     3,    6, -0.25,
        -6,    -3,    3,     6,
        0.25,    -6,   -3,     3,
        6, -0.25, 0.25, -0.25,
        0.25, -0.25, 0.25, -0.25
};

static const double VANGLE_V6H_32_0x5D[] = {
        -16,    -14,    -12,    -10,    -8,     -6,     -5.6,   -5.2,
        -4.8,   -4.4,   -4,     -3.6,   -3.2,   -2.8,   -2.4,   -2,
        -1.6,   -1.2,   -0.8,   -0.4,   0,      0.4,    0.8,    1.2,
        1.6,    2,      4,      6,      8,      10,     12,     14
};

static const double VANGLE_V7_32_0x2A[] = {
   -16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
};

//32�߱�Ϊ16��
const int RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[] = {
        5, 7, 9, 11, 13, 15, 16, 18, 17, 19, 20, 21, 23, 25, 27, 29
};
const int RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[] = {
        6,8,10,12,14,16,18,17,19,20,22,21,24,26,28,30
};
const int RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[] = {
        1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31
};
const int RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[] = {
        0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30
};
const int RFANS_PRODUCT_MODEL_V6A_X16M_0X4F_LASER_ID[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
};

}
}

#endif //__STAR_SDK_DETAIL_LIDAR_H
