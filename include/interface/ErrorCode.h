/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/

#ifndef ERROR_CODE_H
#define ERROR_CODE_H
/*******************************Error Code ***********************************/

const int ERR_NEW_LIDAR          = -1;
const int ERR_NEW_DATAC          = -2;
const int ERR_NEW_SCADA          = -3;
const int ERR_NEW_ACCPT          = -4;
const int ERR_NEW_CONCT          = -5;
const int ERR_NEW_OISTUB         = -6;
const int ERR_NEW_LASER          = -7;
const int ERR_NEW_POS            = -8;
const int ERR_NEW_PDU            = -9;
const int ERR_NEW_MCU            = -10;
const int ERR_NEW_SCDSTUB        = -11;
const int ERR_NEW_MB             = -12;//申请一个新的Message_Block出错
const int ERR_NEW_TIMER          = -13;//申请一个新的定时器出错
const int ERR_NEW_RCV_EQUIP_INFO = -14;//申请一个新的rcv_equip_info线程出错
const int ERR_NEW_TTYTHRD        = -15;//开始串口通信的线程时出错
const int ERR_NEW_SBC6713E       = -16;  

const int ERR_INIT_LIDAR    = -30;
const int ERR_INIT_DATAC    = -31;
const int ERR_INIT_SCADA    = -32;
const int ERR_INIT_LASER    = -33;
const int ERR_INIT_POS      = -34;
const int ERR_INIT_PDU      = -35;
const int ERR_INIT_MCU      = -36;
const int ERR_INIT_DATAC_A  = -37; /* can't open data file one */
const int ERR_INIT_DATAC_B  = -38; /* can't open data file two */
const int ERR_INIT_DATAC_AB = -39;

const int ERR_OPEN_ACCPT = -51;
const int ERR_OPEN_CONCT = -52;
const int ERR_OPEN_LASER = -56;
const int ERR_OPEN_POS   = -57;
const int ERR_OPEN_PDU   = -58;
const int ERR_OPEN_MCU   = -59;

const int ERR_ACTV_TIMER    = -69;
const int ERR_ACTV_LIDAR    = -70;
const int ERR_ACTV_DATAC    = -71;
const int ERR_ACTV_SCADAR   = -72;
const int ERR_ACTV_TTYTHRD  = -73;
const int ERR_ACTV_GTMR     = -74; // Failed to activate global timer
const int ERR_ACTV_POLLTMR  = -75;
const int ERR_ACTV_RPRTTMR  = -76;
// In this timer Lidar sends cmr FLASH command to MCU 
const int ERR_ACTV_MCUTMR   = -77;
const int ERR_ACTV_SMPLTMR  = -78;
// Failed to activate SCADA net inbound thread port 1009
const int ERR_ACTV_INBOUND  = -79; 
// Failed to activate SCADA net outbound thread port 1008
const int ERR_ACTV_OUTBOUND = -80; 

const int ERR_SCD_6713A = -101;
const int ERR_SCD_6713B = -102;
const int ERR_SCD_6713C = -103;
const int ERR_SCD_6713D = -104;
const int ERR_SCD_6713E = -105;
const int ERR_SCD_6713F = -106;
const int ERR_SCD_6713G = -107;
const int ERR_SCD_6713H = -108;
const int ERR_SCD_6713I = -109;

const int ERR_SCD_NONE      = -201;
const int ERR_SCD_UNINIT    = -202;
const int ERR_SCD_NOTSURVEY = -203;
const int ERR_SCD_SURVEY    = -204;
const int ERR_SCD_REJECT    = -205;
const int ERR_SCD_UNPEND    = -206;


const int ERR_DISK_MOUNT_FAILED = -300; // failed to mount disk A 

// failed to enqueue mesaage to DC queue 
const int ERR_DATAC_ENQ = -400; 
// failed to dequeue message to DC queue 
const int ERR_DATAC_DEQ = -401;
// failed to enqueue mesaage to LDR queue 
const int ERR_LDR_ENQ   = -402;
// failed to dequeue message to LDR queue   
const int ERR_LDR_DEQ   = -403; 

////////////////////    OI error
// report MCU abnormal warning info to OI timeout 
const int ERR_RPT_MCU_WRN_TIMEOUT  = -501;
// report PDU abnormal warning info to OI timeout       
const int ERR_RPT_PDU_WRN_TIMEOUT  = -502; 
// report shot sample info to OI timeout    
const int ERR_RPT_SSAMPLE_TIMEOUT  = -503;  
// lidar failed to report MCU info back to OI    
const int ERR_RPT_MCU_INFO_TIMEOUT = -504;     
const int ERR_RPT_PDU_INFO_TIMEOUT = -505;
const int ERR_RPT_LSR_INFO_TIMEOUT = -506;
// lidar failed to report SCD_MSGDEEP to OI
const int ERR_RPT_SCD_DEEP         = -507;     
// lidar failed to report LDRSTAT to OI
const int ERR_RPT_SCDSTAT          = -508;     
// send command or msg to OI timeout
const int ERR_SND_OI_TIMEOUT       = -511; 
// recv command or msg to OI timeout     
const int ERR_RCV_OI_TIMEOUT       = -512;
// send OI_MSG_TESTSCD command to OI timeout     
const int ERR_SND_OI_MSG_TESTSCD   = -513;
// lidar failed to send OI_RPT_LDRSTAT to OI    
const int ERR_SND_OI_RPT_LDRSTAT   = -514; 
// lidar failed to send OI_MSG_PDU command to OI   
const int ERR_SND_OI_MSG_PDU       = -515; 
// lidar received wrong SCADA param    
const int ERR_WRG_SCD_PARAM        = -516;      
// failed to recv right=-sized OI_LOG_FLIGHT cmd
const int ERR_OI_LOG_FLIGHT_SIZE   = -521;
// failed to recv right=-sized OI_LOG_Environ cmd     
const int ERR_OI_LOG_ENVIRON_SIZE  = -522;     
// failed to recv right=-sized OI_LOG_TEXT cmd
const int ERR_OI_LOG_TEXT_SIZE     = -523;    
// failed to recv right=-sized OI_MSG_S cmd
const int ERR_OI_MSG_SIZE          = -524;     
// lidar received an unknown OI cmd
const int ERR_UNKWN_OI_CMD         = -530;    

////////////////////    TTY error
// TTY thread recved a wrong=-sized TTY_CMD_S
const int ERR_TTY_CMD_SIZE  = -601;
// TTY thread recved an unknown TTY_CMD_S     
const int ERR_UNKWN_TTY_CMD = -602;     

////////////////////    MCU error
// send command or msg to MCU timeout
const int ERR_SND_MCU_TIMEOUT = -701;
// recv command or msg to MCU timeout   
const int ERR_RCV_MCU_TIMEOUT = -702; 
// failed to put MCU cmd into tty thread's message queue    
const int ERR_ENQ_MCU_CMD     = -703; 
// received an unknown MCU cmd from OI     
const int ERR_UNKWN_MCU_CMD   = -704;    

////////////////////    PDU error
// send command or msg to PDU timeout 
const int ERR_SND_PDU_TIMEOUT = -801; 
// recv command or msg to PDU timeout   
const int ERR_RCV_PDU_TIMEOUT = -802;    

////////////////////    Laser error
// send command or msg to LSR timeout
const int ERR_SND_LSR_TIMEOUT = -901;
// recv command or msg to LSR timeout     
const int ERR_RCV_LSR_TIMEOUT = -902;   

////////////////////    SCD error
// send command or msg to SCD timeout
const int ERR_SND_SCD_TIMEOUT       = -1001;
// recv command or msg to SCD timeout    
const int ERR_RCV_SCD_TIMEOUT       = -1002;
// send SCD_TEST_START command to SCADA timeout    
const int ERR_SND_SCD_TEST_START    = -1003;  
// send SCD_TEST_START command to SCADA timeout  
const int ERR_SND_SCD_TEST_STOP     = -1004; 
// send SCD_SCNPARAM command to SCADA timeout    
const int ERR_SND_SCD_SCNPARAM      = -1005; 
// send look up table to SCADA timeout  
const int ERR_SND_SCD_LOOKUP_TBL    = -1006;  
// send SCD_RCVWIN command to SCADA timeout   
const int ERR_SND_SCD_RCVWIN        = -1007;   
// send SCD_SCAN_START command to SCADA timeout 
const int ERR_SND_SCD_SCAN_START    = -1008;   
// send SCD_SCAN_STOP command to SCADA timeout
const int ERR_SND_SCD_SCAN_STOP     = -1009;   
// send SCD_TEST_SCANNER command to SCADA timeout
const int ERR_SND_SCD_TEST_SCANNER  = -1010; 
// send SCD_TEST_PEAKTUNE command to SCADA timeout  
const int ERR_SND_SCD_TEST_PEAKTUNE = -1011;   
// send SCD_ECHO_DEEP command to SCADA timeout
const int ERR_SND_SCD_PING_DEEP     = -1012;   
const int ERR_SND_SCD_POS_UTC       = -1013;
const int ERR_SND_SCD_POS_NAV       = -1014;   

// lidar received unknown packet ID 
const int ERR_UNKWN_SCDATA  = -1051;   
// lidar received unknown SCADA command ID
const int ERR_UNKWN_SCD_CMD = -1052;

// SCADA failed to perform command because of self glitch
const int ERR_SCD_GLITCH   = -1080;
// SCADA failed to perform command because of state confiction 
const int ERR_SCD_STATE    = -1081;
// SCADA deep echo arg unequal
const int ERR_SCD_DEEP_ARG = -1082;

/////////////////////   DataC error
// lidar failed to put SCADA data into datac
const int ERR_DC_SCDATA     = -1100; 
const int ERR_DC_LOG_AFAIL  = -1101; //fileA failed
const int ERR_DC_LOG_BFAIL  = -1102; //fileB failed
const int ERR_DC_LOG_IDERR  = -1103; //the ID is unknown
const int ERR_DC_DSK_AFAIL  = -1104; //diskA failed
const int ERR_DC_DSK_BFAIL  = -1105; //diskB failed
const int ERR_DC_WRT_AFAIL  = -1106; //write fileA failed
const int ERR_DC_WRT_BFAIL  = -1107; //write fileB failed

#endif//ERROR_CODE_H
