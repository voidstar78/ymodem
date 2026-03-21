// YMODEM FOR CMDR X16 - XIPHOD MARCH 2026
// RECV ONLY, EACH BLOCK TO RAM BANK
#include <stdio.h>   // printf
#include <string.h>  // strlen
#include <stdlib.h>  // atoi

// DEBUG_OUTPUT is using slow printf.  Only enable if remaining in text-mode,
// do not use if dumping to VRAM1 (the output will skew the VIDEO RAM display).
//#define DEBUG_OUTPUT

// Enabling testing of "failed CRC" case (without actually waiting for any
// line noise.  This forces a CRC failure on the "5th block" of received files.
//#define TEST_CRC

// Enable if want to use the more accurate actual-CRC checksum check.
// Otherwise, only checks the Block Count and Reverse.
#define USE_ACTUAL_CRC

// Enable this to direct received Block Data straight to the next VRAM
// (offset start at 0 for each received file).
//#define DUMP_TO_VRAM1
#define DUMP_TO_FILE

#ifdef TEST_CRC
// Used as indicator on how many CRC errors have been induced (0 means none yet).
unsigned int INDUCED_CRC_ERROR = 0;
#endif

#define TIMEOUT 240  // 14 seconds  (14*60)

#define ASCII_NULL  0
#define ASCII_SOH   1   // Start of Header
#define ASCII_STX   2   // Start of Text
#define ASCII_EOT   4   // End of Transmission
#define ASCII_ACK   6   // Acknowledge
#define ASCII_SPACE 32
#define ASCII_NAK   21  // Negative Acknowledge
#define ASCII_C     67  // Upper case C

#define TRUE 1
#define FALSE 0

#define POKE(addr,val)     (*(unsigned char*) (addr) = (val))
#define PEEK(addr)         (*(unsigned char*) (addr))

unsigned char SERIAL_IN;
unsigned char SERIAL_OUT;
unsigned char LSR;  
unsigned int BLOCK_SIZE;

char temp_ch;  // used to "drain" UART if needed during a SERIAL_OUT SEND_ONE

unsigned int CRC_RESULT;

// Baud rate dividers for X16 TexElec serial card
//                                50,  110,  300,  450,  600,  900, 1200, 1800, 2400, 3600, 4800, 7200, 9600,14400,19200,28800,38400,57600,115200,230400,460800,921600
unsigned char BAUD_DIV_HI[] = { 0x48, 0x20, 0x0C, 0x08, 0x06, 0x04, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00,  0x00,  0x00,  0x00 };
unsigned char BAUD_DIV_LO[] = { 0x00, 0xBA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0xC0, 0x80, 0x60, 0x40, 0x30, 0x20, 0x18, 0x10,  0x08,  0x04,  0x02,  0x01 };
//                                 0     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17     18     19     20     21

unsigned char BAUD_CURR = 20;  // : REM DEFAULT TO X Kbps BAUD (index above)
unsigned char DEVICE_MODE = 1; // 0 == WiFi, 1 == serial
unsigned char PORT_IDX = 4;
unsigned char PORT_STATE = 0;  // 0 == low, 1 == high

unsigned char DEVICE_NUMBER = 8;

#ifdef USE_ACTUAL_CRC
unsigned int CRC_LUT[256] = {
0x0000, // 00 0
0x1021, // 01 1
0x2042, // 02 2
0x3063, // 03 3
0x4084, // 04 4
0x50A5, // 05 5
0x60C6, // 06 6
0x70E7, // 07 7
0x8108, // 08 8
0x9129, // 09 9
0xA14A, // 0A 10
0xB16B, // 0B 11
0xC18C, // 0C 12
0xD1AD, // 0D 13
0xE1CE, // 0E 14
0xF1EF, // 0F 15
0x1231, // 10 16
0x0210, // 11 17
0x3273, // 12 18
0x2252, // 13 19
0x52B5, // 14 20
0x4294, // 15 21
0x72F7, // 16 22
0x62D6, // 17 23
0x9339, // 18 24
0x8318, // 19 25
0xB37B, // 1A 26
0xA35A, // 1B 27
0xD3BD, // 1C 28
0xC39C, // 1D 29
0xF3FF, // 1E 30
0xE3DE, // 1F 31
0x2462, // 20 32
0x3443, // 21 33
0x0420, // 22 34
0x1401, // 23 35
0x64E6, // 24 36
0x74C7, // 25 37
0x44A4, // 26 38
0x5485, // 27 39
0xA56A, // 28 40
0xB54B, // 29 41
0x8528, // 2A 42
0x9509, // 2B 43
0xE5EE, // 2C 44
0xF5CF, // 2D 45
0xC5AC, // 2E 46
0xD58D, // 2F 47
0x3653, // 30 48
0x2672, // 31 49
0x1611, // 32 50
0x0630, // 33 51
0x76D7, // 34 52
0x66F6, // 35 53
0x5695, // 36 54
0x46B4, // 37 55
0xB75B, // 38 56
0xA77A, // 39 57
0x9719, // 3A 58
0x8738, // 3B 59
0xF7DF, // 3C 60
0xE7FE, // 3D 61
0xD79D, // 3E 62
0xC7BC, // 3F 63
0x48C4, // 40 64
0x58E5, // 41 65
0x6886, // 42 66
0x78A7, // 43 67
0x0840, // 44 68
0x1861, // 45 69
0x2802, // 46 70
0x3823, // 47 71
0xC9CC, // 48 72
0xD9ED, // 49 73
0xE98E, // 4A 74
0xF9AF, // 4B 75
0x8948, // 4C 76
0x9969, // 4D 77
0xA90A, // 4E 78
0xB92B, // 4F 79
0x5AF5, // 50 80
0x4AD4, // 51 81
0x7AB7, // 52 82
0x6A96, // 53 83
0x1A71, // 54 84
0x0A50, // 55 85
0x3A33, // 56 86
0x2A12, // 57 87
0xDBFD, // 58 88
0xCBDC, // 59 89
0xFBBF, // 5A 90
0xEB9E, // 5B 91
0x9B79, // 5C 92
0x8B58, // 5D 93
0xBB3B, // 5E 94
0xAB1A, // 5F 95
0x6CA6, // 60 96
0x7C87, // 61 97
0x4CE4, // 62 98
0x5CC5, // 63 99
0x2C22, // 64 100
0x3C03, // 65 101
0x0C60, // 66 102
0x1C41, // 67 103
0xEDAE, // 68 104
0xFD8F, // 69 105
0xCDEC, // 6A 106
0xDDCD, // 6B 107
0xAD2A, // 6C 108
0xBD0B, // 6D 109
0x8D68, // 6E 110
0x9D49, // 6F 111
0x7E97, // 70 112
0x6EB6, // 71 113
0x5ED5, // 72 114
0x4EF4, // 73 115
0x3E13, // 74 116
0x2E32, // 75 117
0x1E51, // 76 118
0x0E70, // 77 119
0xFF9F, // 78 120
0xEFBE, // 79 121
0xDFDD, // 7A 122
0xCFFC, // 7B 123
0xBF1B, // 7C 124
0xAF3A, // 7D 125
0x9F59, // 7E 126
0x8F78, // 7F 127
0x9188, // 80 128
0x81A9, // 81 129
0xB1CA, // 82 130
0xA1EB, // 83 131
0xD10C, // 84 132
0xC12D, // 85 133
0xF14E, // 86 134
0xE16F, // 87 135
0x1080, // 88 136
0x00A1, // 89 137
0x30C2, // 8A 138
0x20E3, // 8B 139
0x5004, // 8C 140
0x4025, // 8D 141
0x7046, // 8E 142
0x6067, // 8F 143
0x83B9, // 90 144
0x9398, // 91 145
0xA3FB, // 92 146
0xB3DA, // 93 147
0xC33D, // 94 148
0xD31C, // 95 149
0xE37F, // 96 150
0xF35E, // 97 151
0x02B1, // 98 152
0x1290, // 99 153
0x22F3, // 9A 154
0x32D2, // 9B 155
0x4235, // 9C 156
0x5214, // 9D 157
0x6277, // 9E 158
0x7256, // 9F 159
0xB5EA, // A0 160
0xA5CB, // A1 161
0x95A8, // A2 162
0x8589, // A3 163
0xF56E, // A4 164
0xE54F, // A5 165
0xD52C, // A6 166
0xC50D, // A7 167
0x34E2, // A8 168
0x24C3, // A9 169
0x14A0, // AA 170
0x0481, // AB 171
0x7466, // AC 172
0x6447, // AD 173
0x5424, // AE 174
0x4405, // AF 175
0xA7DB, // B0 176
0xB7FA, // B1 177
0x8799, // B2 178
0x97B8, // B3 179
0xE75F, // B4 180
0xF77E, // B5 181
0xC71D, // B6 182
0xD73C, // B7 183
0x26D3, // B8 184
0x36F2, // B9 185
0x0691, // BA 186
0x16B0, // BB 187
0x6657, // BC 188
0x7676, // BD 189
0x4615, // BE 190
0x5634, // BF 191
0xD94C, // C0 192
0xC96D, // C1 193
0xF90E, // C2 194
0xE92F, // C3 195
0x99C8, // C4 196
0x89E9, // C5 197
0xB98A, // C6 198
0xA9AB, // C7 199
0x5844, // C8 200
0x4865, // C9 201
0x7806, // CA 202
0x6827, // CB 203
0x18C0, // CC 204
0x08E1, // CD 205
0x3882, // CE 206
0x28A3, // CF 207
0xCB7D, // D0 208
0xDB5C, // D1 209
0xEB3F, // D2 210
0xFB1E, // D3 211
0x8BF9, // D4 212
0x9BD8, // D5 213
0xABBB, // D6 214
0xBB9A, // D7 215
0x4A75, // D8 216
0x5A54, // D9 217
0x6A37, // DA 218
0x7A16, // DB 219
0x0AF1, // DC 220
0x1AD0, // DD 221
0x2AB3, // DE 222
0x3A92, // DF 223
0xFD2E, // E0 224
0xED0F, // E1 225
0xDD6C, // E2 226
0xCD4D, // E3 227
0xBDAA, // E4 228
0xAD8B, // E5 229
0x9DE8, // E6 230
0x8DC9, // E7 231
0x7C26, // E8 232
0x6C07, // E9 233
0x5C64, // EA 234
0x4C45, // EB 235
0x3CA2, // EC 236
0x2C83, // ED 237
0x1CE0, // EE 238
0x0CC1, // EF 239
0xEF1F, // F0 240
0xFF3E, // F1 241
0xCF5D, // F2 242
0xDF7C, // F3 243
0xAF9B, // F4 244
0xBFBA, // F5 245
0x8FD9, // F6 246
0x9FF8, // F7 247
0x6E17, // F8 248
0x7E36, // F9 249
0x4E55, // FA 250
0x5E74, // FB 251
0x2E93, // FC 252
0x3EB2, // FD 253
0x0ED1, // FE 254
0x1EF0  // FF 255
};
/*
void init_CRC_table() {
    for (uint16_t j = 0; j < 256; j++) {
        uint16_t crc = j << 8; 
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // XModem CRC polynomial "magic number"
            } else {
                crc <<= 1;
            }
        }
        CRC_LUT[j] = crc;
    }
}
*/
#endif

unsigned int serial_addr[15];

unsigned int SERIAL_REG_BASE;

unsigned int REG_READ_RBR;
unsigned int REG_READ_IER;
unsigned int REG_READ_IIR;
unsigned int REG_READ_LCR;
unsigned int REG_READ_MCR;
unsigned int REG_READ_LSR;
unsigned int REG_READ_MSR;
unsigned int REG_READ_SCR;

unsigned int REG_WRITE_THR;
unsigned int REG_WRITE_FCR;
unsigned int REG_WRITE_LCR;
unsigned int REG_WRITE_MCR;
unsigned int REG_WRITE_NA1;
unsigned int REG_WRITE_NA2;
unsigned int REG_WRITE_SCR;

// WHEN DLAB==1 (BIT 7 OF LCR, SERIAL.REG.BASE+3, DIVISOR LATCH ACCESS BIT)  
unsigned int REG_READ_DLL;
unsigned int REG_READ_DLM;

unsigned int REG_WRITE_DLL;
unsigned int REG_WRITE_DLM;

unsigned int IN_BUFFER_START_ADDRESS = 0xB000;  // IN_BUFFER == INPUT_BUFFER
unsigned int IN_BUFFER_FINISH_ADDRESS = 0xC000;
unsigned int IN_BUFFER_POS;  // Current "write position" into the above INPUT_BUFFER

void init_serial_addr()
{
  // default to SERIAL addresses first
  // REM LOW ADDRESSES
  serial_addr[3] = 0x9F68;
  serial_addr[4] = 0x9F88;
  serial_addr[5] = 0x9FA8;
  serial_addr[6] = 0x9FC8;
  serial_addr[7] = 0x9FE8;
  // REM HIGH ADDRESSES
  serial_addr[10] = 0x9F78;
  serial_addr[11] = 0x9F98;
  serial_addr[12] = 0x9FB8;
  serial_addr[13] = 0x9FD8;
  serial_addr[14] = 0x9FF8;
  if (DEVICE_MODE == 1) return;

  // If not using SERIAL addresses, then override with the "other" WiFi/ESP32 connected port
  // NOTE: When using these ports, the user will have to init the WiFi connection first (using ROMTERM/BASTERM)
  // And use ATD or similar means to establish a connection first (again using a different terminal program).
  // As long as the WiFi remains powered, the connection remains established.  So can power cycle the X16 and 
  // use YMODEM to complete the data transfer.
  serial_addr[3] = 0x9F60;
  serial_addr[4] = 0x9F80;
  serial_addr[5] = 0x9FA0;
  serial_addr[6] = 0x9FC0;
  serial_addr[7] = 0x9FE0;
  // REM HIGH ADDRESSES
  serial_addr[10] = 0x9F70;
  serial_addr[11] = 0x9F90;
  serial_addr[12] = 0x9FB0;
  serial_addr[13] = 0x9FD0;
  serial_addr[14] = 0x9FF0;  
}

void init_serial_regs()
{
  // WHEN DLAB==0 (BIT 7 OF LCR, REG.BASE+3)  
  REG_READ_RBR = SERIAL_REG_BASE+0;  //: REM RECEIVER BUFFER REGISTER (READ-ONLY)
  REG_READ_IER = SERIAL_REG_BASE+1;  //: REM INTERRUPT ENABLE REGISTER
  REG_READ_IIR = SERIAL_REG_BASE+2;  //: REM INTERRUPT IDENT REGISTER (READ-ONLY)
  REG_READ_LCR = SERIAL_REG_BASE+3;  //: REM LINE CONTROL REGISTER
  REG_READ_MCR = SERIAL_REG_BASE+4;  //: REM MODEM CONTROL REGISTER
  REG_READ_LSR = SERIAL_REG_BASE+5;  //: REM LINE STATUS REGISTER
  REG_READ_MSR = SERIAL_REG_BASE+6;  //: REM MODEM STATUS REGISTER
  REG_READ_SCR = SERIAL_REG_BASE+7;  //: REM SCRATCH REGISTER  

  REG_WRITE_THR = SERIAL_REG_BASE+0;  //: REM TRANSMIT HOLDING REGISTER (WRITE-ONLY)
  REG_WRITE_FCR = SERIAL_REG_BASE+2;  //: REM FIFO CONTROL REGISTER (WRITE-ONLY)
  REG_WRITE_LCR = SERIAL_REG_BASE+3;  //: REM LINE CONTROL REGISTER
  REG_WRITE_MCR = SERIAL_REG_BASE+4;  //: REM MODEM CONTROL REGISTER
  REG_WRITE_NA1 = SERIAL_REG_BASE+5;  //: REM LINE STATUS REGISTER
  REG_WRITE_NA2 = SERIAL_REG_BASE+6;  //: REM MODEM STATUS REGISTER
  REG_WRITE_SCR = SERIAL_REG_BASE+7;  //: REM SCRATCH REGISTER

  // WHEN DLAB==1 (BIT 7 OF LCR, SERIAL.REG.BASE+3, DIVISOR LATCH ACCESS BIT)  
  REG_READ_DLL = SERIAL_REG_BASE+0;  //: REM DIVISOR LATCH LSB
  REG_READ_DLM = SERIAL_REG_BASE+1;  //: REM DIVISOR LATCH MSB
  //  IIR,LCR,MCR SAME
  //  LSR,MSR SAME

  REG_WRITE_DLL = SERIAL_REG_BASE+0; // REM DIVISOR LATCH LSB
  REG_WRITE_DLM = SERIAL_REG_BASE+1; // REM DIVISOR LATCH MSB
  //  FCR,LCR,MCR SAME
}  

void serial_init()
{
  unsigned char X;
  
  POKE(REG_WRITE_LCR, 0b10000011);  // SET DLAB==1 (AND SET 8 DATA BIT, 1 STOP BIT)
  //REM                %10000010 : REM 7 DATA BIT, 1 STOP BIT
  //REM                %10000101 : REM 6 DATA BIT, 2 STOP BIT
  //REM                 STOP-^ (0 = 1 STOP BIT, 1 = 2 STOP BIT)
  
  POKE(REG_WRITE_DLL, BAUD_DIV_LO[BAUD_CURR]); // REM LSB (LO)
  POKE(REG_WRITE_DLM, BAUD_DIV_HI[BAUD_CURR]); // REM MSB (HI)  

  X = PEEK(REG_READ_LCR);  // REM STORE CURRENT BIT MASK
  X = X & 0b01111111;   // REM SET LCR.DLAB BIT == 0
  POKE(REG_WRITE_LCR, X);

  if (BAUD_CURR == 21)  // 921600
  {
    POKE(REG_WRITE_FCR, 0b01000111);  // REM TRIGGER AT 4 BYTE FIFO, CLEAR TX/RX FIFO BUFFERS, ENABLE FIFO
  }
  else if (BAUD_CURR <= 18)  // 115200 or below
  {
#ifdef DEBUG_OUTPUT
    POKE(REG_WRITE_FCR, 0b10000111);  // 8 BYTE FIFO
#else
    POKE(REG_WRITE_FCR, 0b11000111);  // REM TRIGGER AT 14 BYTE FIFO, CLEAR TX/RX FIFO BUFFERS, ENABLE FIFO
#endif
  }
  else  // 230Kbps, 460Kbps
  {
    POKE(REG_WRITE_FCR, 0b10000111);  // REM TRIGGER AT 8 BYTE FIFO, CLEAR TX/RX FIFO BUFFERS, ENABLE FIFO
  }

  POKE(REG_WRITE_MCR, 0b00100011);  // SET AUTO-CTS AND AUTO-RTS AND DTR
}

unsigned char CURR_RECEIVED_MARKER1;  // BLOCK_COUNT (RECEIVED)
unsigned char CURR_RECEIVED_MARKER2;  // BLOCK_INVERSE (RECEIVED)
unsigned char CURR_EXPECTED_MARKER1 = 0x00; // REM BLOCK COUNT
unsigned char CURR_EXPECTED_MARKER2 = 0xFF; // REM INVERSE COUNT  (MARKER1+MARKR2=255)
unsigned char LAST_GOOD_BLOCK = 0xFF;
unsigned long TOTAL_BLOCK = 0;  // OF CURRENT FILE (NOT TOTAL FOR ALL THE FILES)

char BLOCK_HEADER_FN[1024];  // Limited to 128 or 1024, depending on header blocksize
char BLOCK_HEADER_FN_LEN = 0;
char BLOCK_HEADER_LEN[24];  // "2123123123" is 10 bytes.  We'll just supported signed 4-byte transfers (2.1GB)
unsigned long BLOCK_HEADER_LEN_bytes = 0;  // EXPECTED TOTAL NUMBER OF BYTES FOR CURRENT FILE
unsigned long TOTAL_RECEIVED_BYTES = 0;  // : REM BYTES THAT APPLY TO THE CURRENT FILE BEING SENT
unsigned long TEMP_BYTES;
unsigned long DELTA_BYTES;

volatile unsigned long* TIMER = (volatile unsigned long*)(0x0400);
// FFDE == RDTIM, note probably faster to use VIA2 as timer (don't need full 3 byte timer)
#define UPDATE_TIMER \
	__asm__("jsr $FFDE"); \
	__asm__("sta $0400"); \
	__asm__("stx $0401"); \
	__asm__("sty $0402");

unsigned long start_time;
unsigned long delta_time;

// We'll just use the slow 3-byte timer for this.  Not critical.
// This is just draining any "left over" content in the FIFO, and also
// possibly any "wrapper buffer" that is beyond the UART (maybe an application
// buffer or in some USB UARTs with 128+ byte buffers).
void wait_line_quiet(unsigned long wait_threshold)
{  
  UPDATE_TIMER;  start_time = *TIMER;
//#ifdef DEBUG_OUTPUT
  printf("WAIT RECV DRAIN [");
//#endif  
  while (TRUE)
  { 
    UPDATE_TIMER;  delta_time = (*TIMER - start_time);
    SERIAL_IN = PEEK(REG_READ_RBR);
    if ((SERIAL_IN == ASCII_NULL) && (delta_time > wait_threshold))  // 60*5 = 300 jiffies (5 seconds)
	{
//#ifdef DEBUG_OUTPUT
      printf("DONE]\n");
//#endif
      return;
    }
    if (SERIAL_IN != 0)
    {
//#ifdef DEBUG_OUTPUT
      printf("%02X ", SERIAL_IN);
//#endif
      UPDATE_TIMER;  start_time = *TIMER;
    }
  }
}

// This is a reset used in-between files, or if the header parsing fails
// and we need to restart the current file.
void total_block_reset()
{
  TOTAL_BLOCK = 0;
  IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
  BLOCK_HEADER_FN[0] = 0;
  BLOCK_HEADER_LEN[0] = 0;
  TOTAL_RECEIVED_BYTES = 0;
}

void SEND_ONE()
{
check_cts_again:  
  temp_ch = PEEK(REG_READ_MSR);
  if (temp_ch & 0x10) 
  { 
    POKE(REG_WRITE_THR, SERIAL_OUT); 
#ifdef DEBUG_OUTPUT
    printf("[%02X]", SERIAL_OUT);  
#endif
    return; 
  };  // REM STILL CLEAR TO SEND
  
WAIT_LSR_DRAIN:
  LSR = PEEK(REG_READ_LSR); 
  if ((LSR & 1) != 0) { SERIAL_IN = PEEK(REG_READ_RBR); goto WAIT_LSR_DRAIN; }  

  goto check_cts_again;  // REM NOT CLEAR TO SEND, WAIT...      
}

//REM HANDLE "END OF TRANSMISSION" BY ACKNOWLEDGING RECEIPT,
//REM AND ISSUE A "CONTINUE" FOR ANY NEXT BATCHED FILE.
unsigned char HANDLE_EOT()
{
  // This is not a "real" NAK -- This is the expected response of <EOT> from the receiver
  // After this NAK, the sender sends another second EOT.
  SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK 
  
  // WAIT FOR SECOND EOT
  UPDATE_TIMER;  start_time = *TIMER;
EOT_WAIT_LSR:
  LSR = PEEK(REG_READ_LSR);
#ifdef DEBUG_OUTPUT	
  if ((LSR & 2) != 0) { printf("E"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1) == 0) 
  {
    UPDATE_TIMER;  delta_time = (*TIMER - start_time);
    if (delta_time > TIMEOUT)
    {
      // NOTE: Maybe they missed our NAK, or we miss their EOT...
      // Go ahead and re-send the last block.  Even if it is a repeat and was already
      // processed, this gets us back in sync.  (unless they miss our NAK)
      // Returning FALSE will induce xmit of the NAK.
      return FALSE;
    }
    goto EOT_WAIT_LSR;
  }
  SERIAL_IN = PEEK(REG_READ_RBR);

  if (SERIAL_IN != ASCII_EOT)
  {
    return FALSE;
  }

  //REM RESET BLOCK MANAGEMENT VARIABLES
  CURR_EXPECTED_MARKER1 = 0x00;
  CURR_EXPECTED_MARKER2 = 0xFF;
  LAST_GOOD_BLOCK = 0xFF;
  
  //BLOCK_HEADER_FN = "";
  //BLOCK_HEADER_LEN = "";
  total_block_reset();

  SERIAL_OUT = ASCII_ACK;  SEND_ONE();  //   A$ = CHR$(6) : GOSUB SEND.ONE : REM "ACK"
  SERIAL_OUT = ASCII_C;  SEND_ONE();  //   A$ = "C" 'C'    : GOSUB SEND.ONE : REM "CONTINUE"
  
  return TRUE;
}

void main(void)
{
  unsigned int CRC_RECV;
  register unsigned char i8_index;
  register unsigned long i32_poll;
  register unsigned int i16_2;  // block-data receive counter  
  char* endptr;
  
#ifdef DUMP_TO_FILE
  // TBD: Update Device Number to last known LFS ?
  
  // These are used for scratching/erasing a file later.
  //POKE(0xAFFD, 's');
  POKE(0xAFFE, 's');
  POKE(0xAFFF, ':');
#endif

  i8_index = PEEK(0x0600);  // Y <21> <0> <4> <0>
  if (i8_index == 89)  // upper case Y
  {
    BAUD_CURR     = PEEK(0x0601);
    DEVICE_MODE   = PEEK(0x0602);
    PORT_IDX      = PEEK(0x0603);
    PORT_STATE    = PEEK(0x0604);
    DEVICE_NUMBER = PEEK(0x0605);
  }
  printf("BAUD [%u]  ", BAUD_CURR);
  printf("MODE [%u]  ", DEVICE_MODE);
  printf("PORT [%u (%u)]\n", PORT_IDX, PORT_STATE);
  printf("DEVC [%u] ", DEVICE_NUMBER);
#ifdef USE_ACTUAL_CRC
  printf("CRC-ON\n", DEVICE_NUMBER);
#else
  printf("CRC-OFF\n", DEVICE_NUMBER);
#endif
  
  // CLEAR LOCAL TIMER COPY
  POKE(0x400, 0);
  POKE(0x401, 0);
  POKE(0x402, 0);
  POKE(0x403, 0);   
  
  IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;

  init_serial_addr();
 
  SERIAL_REG_BASE = serial_addr[7*PORT_STATE+PORT_IDX];
  printf("PORT ADDR [%04X]\n", SERIAL_REG_BASE);
  
  init_serial_regs();
  serial_init();
  
  wait_line_quiet(300);
  
  POKE(0, 2);  // set RAM bank 2 -- this is optional (just don't use BANK 0)

#ifdef DUMP_TO_VRAM1
  // VERA
  __asm__("lda #$80");
  __asm__("clc");  // SET CARRY, specify using "set screen mode" instead of "get screen mode"
  __asm__("jsr $FF5F");  // SCREEN_MODE
  
  POKE(0x0500, 'p');  // p.bin (VERA palette) [lower case because of CC65-ism]
  POKE(0x0501, '.');
  POKE(0x0502, 'b');
  POKE(0x0503, 'i');
  POKE(0x0504, 'n');  
  __asm__("lda #$05");  // filename length
  __asm__("ldx #$00");  // LO
  __asm__("ldy #$05");  // HI
  __asm__("jsr $FFBD");  // SETNAM
  
  __asm__("lda #$01");  // file num
  __asm__("ldx #$08");  // device
  __asm__("ldy #$02");  // LOAD: 2 == use x,y reg headerless
  __asm__("jsr $FFBA");  // SETLFS
    
  __asm__("lda #$03");  // use VERA $10000
  __asm__("ldx #$00");
  __asm__("ldy #$FA");  // offset into VERA palette at $1:FA00
  __asm__("jsr $FFD5");  // LOAD (no need to CLOSE)
  
  // VERA  
  POKE(0x9F22,0x10);  // : REM HI. (SET AUTO-INCREMENT TO 1-BYTE)
  POKE(0x9F21,0x00);  // : REM MED.
  POKE(0x9F20,0x00);  // : REM LO.  
#endif

  SERIAL_OUT = ASCII_C;  SEND_ONE();  // ASCII HEX $43 "C" 'C'  SIGNAL SENDER TO START

  CRC_RESULT = 0;
  
MAIN_LOOP_STAGE1:
  LSR = PEEK(REG_READ_LSR); 
#ifdef DEBUG_OUTPUT
  if ((LSR & 2) != 0) { printf("1"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1) == 0) goto MAIN_LOOP_STAGE1;
  SERIAL_IN = PEEK(REG_READ_RBR);  
#ifdef DEBUG_OUTPUT  
  printf("  1:%02X ", SERIAL_IN);
#endif
  if (SERIAL_IN == ASCII_SOH) { BLOCK_SIZE = 128;  goto MAIN_LOOP_STAGE2; }
  if (SERIAL_IN == ASCII_STX) { BLOCK_SIZE = 1024; goto MAIN_LOOP_STAGE2; }
  if (SERIAL_IN == ASCII_EOT) 
  {
#ifdef DEBUG_OUTPUT	
    printf("RECV EOT: ");
#endif
    if (TOTAL_BLOCK == 0)  // Shouldn't get EOT during first block.  Something is not right.
    {
      printf("NAK1\n");

      SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
      goto MAIN_LOOP_STAGE1;  // START OVER...
    }
	// Else...
    i8_index = HANDLE_EOT(); 
	if (i8_index == FALSE)
    {
      printf(" NAK2\n");

      // WARNING, reminder: recall the last good received block was already processed.
      SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK      
      CRC_RESULT = 0;
      IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
    }
    else
    {
#ifdef DEBUG_OUTPUT	
      printf(" OK\n");
#endif
#ifdef DUMP_TO_FILE
      __asm__("lda #$01");   // file index
      __asm__("jsr $FFC3");  // CLOSE

      __asm__("jsr $FF4A");  // CLOSE_ALL
      __asm__("jsr $FFCC");  // CLRCHN
#endif
    }
	goto MAIN_LOOP_STAGE1;  // START OVER...
  }  
  goto MAIN_LOOP_STAGE1;
  
// REM STAGE2 - WAIT FOR "EXPECTED BLOCK MARKER #1" (BLOCK COUNTER)
MAIN_LOOP_STAGE2:
  LSR = PEEK(REG_READ_LSR);
#ifdef DEBUG_OUTPUT
  if ((LSR & 2) != 0) { printf("2"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1) == 0) goto MAIN_LOOP_STAGE2;
  CURR_RECEIVED_MARKER1 = PEEK(REG_READ_RBR);

#ifdef TEST_CRC  
  if ((INDUCED_CRC_ERROR == 0) && (TOTAL_BLOCK == 5))
  {
    CRC_RESULT ^= 0x1234;  // Randomly manipulate the local CRC to force it to mismatch
    ++INDUCED_CRC_ERROR;
  }  
#endif

//REM STAGE3 - WAIT FOR "EXPECTED BLOCK MARKER #2" (BLOCK INVERSE)
MAIN_LOOP_STAGE3:
  LSR = PEEK(REG_READ_LSR);
#ifdef DEBUG_OUTPUT
  if ((LSR & 2) != 0) { printf("3"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1) == 0) goto MAIN_LOOP_STAGE3;
  CURR_RECEIVED_MARKER2 = PEEK(REG_READ_RBR);
  
// REM STAGE4 - BUFFER DOWN THE DATA PAYLOAD
// REM ALSO ACCUMULATE/CALCULATE CRC AS WE GO (NECESSARY TO AVOID A "STALL"
// REM LATER IN TRYING TO POST-CALCULATE THE CRC ON THE BLOCK)
//MAIN_LOOP_STAGE4_PREP:  
#ifdef DEBUG_OUTPUT
  printf("  2:%02X ", CURR_RECEIVED_MARKER1);
  printf("  3:%02X ", CURR_RECEIVED_MARKER2); 
  
  printf("4:");
  if (TOTAL_BLOCK == 0) printf("HEADER");
  else printf("DATA");
  printf(" BLOCK %02X/%02X SIZE %u B\n", CURR_EXPECTED_MARKER1, CURR_EXPECTED_MARKER2, BLOCK_SIZE);
#endif
  i16_2 = 0;  // how much received into the current block (ends at BLOCK_SIZE, either 128 or 1024, hence 16-bit)
#ifdef DEBUG_OUTPUT
  i32_poll = 0xFFFFFF;  // Poll Budget (for the data block and two-byte CRC)  
#else
  i32_poll = 0x7FFF;  // Poll Budget (for the data block and two-byte CRC)  
#endif
  __asm__("sei");  // Stop Interrupts
MAIN_LOOP_STAGE4:
  // REM THIS COULD BE A HEADER BLOCK OR A DATA BLOCK, EITHER WAY SAME ACTION...  
  if (i16_2 == BLOCK_SIZE) 
  {
#ifdef DEBUG_OUTPUT
    printf("[ BLOCK ");
#endif
    __asm__("cli");  // Clear to enable Interrupts
    goto MAIN_LOOP_STAGE5;
  }
  
MAIN_LOOP_STAGE4_WAIT_LSR:
  
  LSR = PEEK(REG_READ_LSR);
#ifdef DEBUG_OUTPUT
  if ((LSR & 2) != 0) { printf("4"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1) == 0) 
  {
    --i32_poll;
    if (i32_poll == 0)
    {
      {
        printf("!4");
        SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
        CRC_RESULT = 0;
        IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
        __asm__("cli");  // Clear to enable Interrupts
        goto MAIN_LOOP_STAGE1;
      }      
    }
    goto MAIN_LOOP_STAGE4_WAIT_LSR;
  }  
  SERIAL_IN = PEEK(REG_READ_RBR);
  ++i16_2;  // Count offset into the current Received Data Block
#ifdef DEBUG_OUTPUT
  printf("%02X\r", SERIAL_IN);
#endif

#ifdef DUMP_TO_VRAM1
  if (TOTAL_BLOCK == 0)
  {
    // Header Block, store result into BANK RAM (since we need to parse the header after block is completely received)
    POKE(IN_BUFFER_POS, SERIAL_IN);
    ++IN_BUFFER_POS;
  }
  else
  {
    // Data Block, just put the result straight into VERA VRAM
    POKE(0x9F23, SERIAL_IN);  
  }
#else
  // Always store into INPUT_BUFFER for later processing in STAGE6 (append to file)
  POKE(IN_BUFFER_POS, SERIAL_IN);
  ++IN_BUFFER_POS;
#endif
  
  // ACCUMULATE BYTE INTO 16-BIT CRC RESULT...  
#ifdef USE_ACTUAL_CRC
  CRC_RESULT = (CRC_RESULT << 8) ^ CRC_LUT[ ((CRC_RESULT >> 8) ^ SERIAL_IN) & 0xFF ];  
#endif

  goto MAIN_LOOP_STAGE4;  // : REM REPEAT FOR THE NEXT DATA BYTE BEING SENT

// REM STAGE5 - OBTAIN 16-BIT (2 BYTE) CRC FROM SENDER, SEE IF MATCHES LOCAL CRC.RESULT
MAIN_LOOP_STAGE5:
#ifdef DEBUG_OUTPUT
  printf(" ]\n");
#endif
  
  IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;  //: REM RESET WORKING BUFFER POSITION INDEX BACK TO BEGINNING
  
MAIN_LOOP_STAGE5_A:  
  LSR = PEEK(REG_READ_LSR);
#ifdef DEBUG_OUTPUT
  if ((LSR & 2) != 0) { printf("5A"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1)==0) 
  {
    --i32_poll;
    if (i32_poll == 0)
    {
      {
        printf("!5A");
        SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
        CRC_RESULT = 0;
        IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
        goto MAIN_LOOP_STAGE1;
      }
    }    
    goto MAIN_LOOP_STAGE5_A;
  }
  CRC_RECV = PEEK(REG_READ_RBR) << 8;  // : REM HIGH BYTE

MAIN_LOOP_STAGE5_B:
  LSR = PEEK(REG_READ_LSR);
#ifdef DEBUG_OUTPUT
  if ((LSR & 2) != 0) { printf("5B"); }  // LSR BIT 1 OVERFLOW INDICATOR
#endif
  if ((LSR & 1)==0)
  {
    --i32_poll;
    if (i32_poll == 0)
    {
      {
        printf("!5B");
        SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
        CRC_RESULT = 0;
        IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
        goto MAIN_LOOP_STAGE1;
      }
    }
    goto MAIN_LOOP_STAGE5_B;
  }
  CRC_RECV = CRC_RECV + PEEK(REG_READ_RBR);  // : REM ADD IN LOW BYTE

  if ((unsigned char)(CURR_RECEIVED_MARKER1 + CURR_RECEIVED_MARKER2) != 0xFF) 
  {
    printf("UNMATCHED MARKER - NAK\n");
    SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
    CRC_RESULT = 0;
    IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
    goto MAIN_LOOP_STAGE1;
  }
  
  // Calculate CRC Result (this a post-CRC evaluation, if we don't do it during the receive loop)
  // We either do it here, or do while actively receiving the data (not both; either-or)
  /*
  i32_poll = IN_BUFFER_START_ADDRESS;
  i16_2 = BLOCK_SIZE;
  CRC_RESULT = 0;
  do
  {
    CRC_RESULT = (CRC_RESULT << 8) ^ CRC_LUT[ ((CRC_RESULT >> 8) ^ PEEK(i32_poll)) & 0xFF ];  
    ++i32_poll;
  } while (--i16_2);
  */
  
#ifdef DEBUG_OUTPUT
  printf("5:CRC LOCAL [%04X] / REMOTE [", CRC_RESULT);
  printf("%04X]\n", CRC_RECV);
#endif

#ifdef USE_ACTUAL_CRC
  if (CRC_RESULT != CRC_RECV) 
  {
    printf("MISMATCH CRC [L %04X : R %04X : %lu] - SENDING NAK\n", CRC_RESULT, CRC_RECV, TOTAL_RECEIVED_BYTES);
    SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
    CRC_RESULT = 0;
    IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
    goto MAIN_LOOP_STAGE1;
  }
#endif
  
  if (CURR_RECEIVED_MARKER1 == CURR_EXPECTED_MARKER1)  // TBD: Check MARKER2 inverse also? Kind of optional.
  {
    LAST_GOOD_BLOCK = CURR_RECEIVED_MARKER1;
    goto MAIN_LOOP_STAGE6;  // All good... Now decide what to do with the buffered block data.
  }
  
  if (CURR_RECEIVED_MARKER1 == LAST_GOOD_BLOCK) 
  {
    // We already received this block.  In this case, something went wrong with
    // end of block ACK and a re-transmission happened.  In the attempt to re-sync,
    // the sender re-sends the last successful good block.
    SERIAL_OUT = ASCII_ACK;  SEND_ONE();  // ACK (see if the ACK works this time)
    CRC_RESULT = 0;
    IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
    goto MAIN_LOOP_STAGE1;
  }

  printf("INCORRECT MARKERS [ %02X %02X ] - SENDING NAK\n", CURR_RECEIVED_MARKER1, CURR_RECEIVED_MARKER2);

  SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK
  CRC_RESULT = 0;  
  IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
  
  goto MAIN_LOOP_STAGE1;  
  
//REM STAGE6 - PROCESS RECEIVED BLOCK OF DATA
//REM IF FIRST BLOCK, PARSE HEADER (GET FILENAME, LENGTH, ETC.)
//REM ELSE, APPEND BLOCK CONTENT TO THE CURRENT FILE
MAIN_LOOP_STAGE6:
  IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;  // : REM RESET WORKING BUFFER POSITION INDEX BACK TO BEGINNING
  if (TOTAL_BLOCK == 0) 
  { 
    i8_index = 0; 
    BLOCK_HEADER_FN_LEN = 0;

    // : REM TBD - OPEN THE FILE, ESTABLISH HANDLE
    // If we can't create the specified filename, send a CAN CAN
    // e.g. if the filename is invalid or some issue with the filesystem

    goto MAIN_LOOP_STAGE6_A; 
  } 
  // REM ELSE...
  //REM TBD -- FILE MANAGEMENT STUFF (APPEND BLOCK TO CURRENT FILE,
  //REM UP TO THE NUMBER OF EXPECTED BYTES IN THIS FILE)
  TEMP_BYTES = TOTAL_RECEIVED_BYTES + (unsigned long)BLOCK_SIZE;
  if (TEMP_BYTES > BLOCK_HEADER_LEN_bytes)
  {
    TEMP_BYTES = BLOCK_HEADER_LEN_bytes;  // : REM THE REMAINING BYTES ARE IN THIS BLOCK...    
  }
  DELTA_BYTES = TEMP_BYTES - TOTAL_RECEIVED_BYTES;
  TOTAL_RECEIVED_BYTES = TEMP_BYTES;

#ifdef DEBUG_OUTPUT
  printf("6:(APPEND TO [%s], %lu BYTES) ", BLOCK_HEADER_FN, TOTAL_RECEIVED_BYTES);
#endif
  
#ifdef DUMP_TO_FILE
#ifdef DEBUG_OUTPUT
  printf("WRITE_D [%lu] ", DELTA_BYTES);
#endif
  if (BLOCK_SIZE == 128)
  {
    __asm__("lda %v", DELTA_BYTES);   // number of bytes (128 or less)
    __asm__("ldx #$00");     // LO
    __asm__("ldy #$B0");     // HI  IN_BUFFER_START_ADDRESS
	__asm__("clc");          // clear carry, enable auto increment of address
    __asm__("jsr $FEB1");    // MCIOUT
  }
  else  // 1024
  {
    IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
    while (DELTA_BYTES > 0)
	{
      if (DELTA_BYTES > 127)
      {
        __asm__("lda %v", IN_BUFFER_POS);
		__asm__("tax");
        __asm__("lda %v+1", IN_BUFFER_POS);
		__asm__("tay");
		__asm__("lda #$80");   // number of bytes
        __asm__("clc");          // clear carry, enable auto increment of address
        __asm__("jsr $FEB1");    // MCIOUT
        DELTA_BYTES -= 128;
        IN_BUFFER_POS += 128;
      }
      else
      {
        __asm__("lda %v", IN_BUFFER_POS);
		__asm__("tax");
        __asm__("lda %v+1", IN_BUFFER_POS);
		__asm__("tay");
		__asm__("lda %v", DELTA_BYTES);   // number of bytes
        __asm__("clc");          // clear carry, enable auto increment of address
        __asm__("jsr $FEB1");    // MCIOUT
        DELTA_BYTES = 0;
      }
    }

    IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
    while (DELTA_BYTES > 0)
    {
      SERIAL_IN = PEEK(IN_BUFFER_POS);  //< Not really "SERIAL_IN", just re-using that same variable.

      __asm__("lda %v", SERIAL_IN);   // The byte to output
      __asm__("jsr $FFA8");  // CIOUT

      --DELTA_BYTES;
      ++IN_BUFFER_POS;
    }
  }
#endif
  
  goto MAIN_LOOP_STAGE7;

//REM WARNING: POST-PARSING BLOCK MAY CAUSE A "STALL" HERE.
//REM COULD BE ISSUE WITH SINGLE BYTE UART.  WE HAVE TO PARSE ALL THIS
//REM BEFORE THE FIFO BECOMES FULL (THOUGH CTS/RTS FLOW CONTROL SHOULD PROVIDE
//REM SOME PROTECTION).
MAIN_LOOP_STAGE6_A:
  //REM PARSE HEADER IN THE STORED BLOCK IN RAM
  //REM PART 1: <FILENAME> <NUL>
  
  //REM NOTE, WE ARE RE-USING SERIAL.IN% EVEN THOUGH THIS IS NOT REALLY A SERIAL INPUT.  IT'S COPY OF RAM BUFFER.
  //REM NOTE, THE FOLLOWING ASSUMES THE FILENAME IS AT LEAST 1 CHARACTER.
  SERIAL_IN = PEEK(IN_BUFFER_POS);  // REMINDER - THIS IS READING FROM RAM BUFFER, NOT SERIAL IO
  ++IN_BUFFER_POS;
  ++BLOCK_HEADER_FN_LEN;
  BLOCK_HEADER_FN[i8_index] = SERIAL_IN;
  if (SERIAL_IN == ASCII_NULL)
  {
#ifdef DEBUG_OUTPUT
    printf("6A:HEADER FN [%s] ", BLOCK_HEADER_FN);
#endif
	i8_index = 0;
    goto MAIN_LOOP_STAGE6_B;  // : REM FOUND STRING NUL TERMINATOR
  }
  ++i8_index;
  if (i8_index >= 120)  //BLOCK_SIZE)  TBD: This is header block, which technically could be 128 or 1024...
  {
    // NOTE: For special case of 0-byte file, this case always happens. Because the senders
    // don't send the " " (SPACE) for 0 length file, it's like that header field doesn't exist.

    // For now just going to consider this invalid.  In a more robust version,
    // we could see if this is an absolute vs relative path prefixed to the filename.
    // But not sure if any Sender YModem actually supports that.
    // So picking 250 just as a failsafe in case we happen to be getting a Data Block,
    // and never get the expected NULL after end of the filename.
    SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK

    total_block_reset();  // This is header block, let's reset everything.
    goto MAIN_LOOP_STAGE1;  // start over
  }
  goto MAIN_LOOP_STAGE6_A;  // : REM GET NEXT FILENAME CHARACTER

MAIN_LOOP_STAGE6_B:
  // REM PART 2: <SIZE IN ASCII> <SPACE>
  
  // REM NOTE, WE ARE RE-USING SERIAL.IN% EVEN THOUGH THIS IS NOT REALLY A SERIAL INPUT.  IT'S COPY OF RAM BUFFER.
  // REM NOTE, FOR 0-BYTE, THE BYTE LENGTH AND OTHER HEADER CONTENT MIGHT BE NUL.  
  SERIAL_IN = PEEK(IN_BUFFER_POS);
  ++IN_BUFFER_POS; 
  if ((i8_index == 0) && (SERIAL_IN  == 0))
  {
    // special case of 0-byte length file, the filelength portion
    // of header may actually be omitted (so no SPACE appears).
    BLOCK_HEADER_LEN[0] = '0';
    BLOCK_HEADER_LEN[1] = '\0';
    SERIAL_IN = ASCII_SPACE;
  }
  if (i8_index > 10)     // 2,123,123,123  (will only support 2GB senders anyway)
  {
#ifdef DEBUG_OUTPUT
    printf("6B:BAD_LEN [%u] - NAK\n", i8_index);
#else
	printf("BAD FILE LENGTH - NAK");
#endif
    SERIAL_OUT = ASCII_NAK;  SEND_ONE();  // NAK

    total_block_reset();  // This is header block, let's reset everything.
    goto MAIN_LOOP_STAGE1;  // start over
  }
  if ((SERIAL_IN == ASCII_SPACE) || (SERIAL_IN == ASCII_NULL))
  { 
    BLOCK_HEADER_LEN[i8_index] = '\0';  
#ifdef DEBUG_OUTPUT
	printf("6B:LEN [%s]  ", BLOCK_HEADER_LEN);
#endif
    // Done parsing for the FILE LENGTH
    goto MAIN_LOOP_STAGE6_C; 
  }
  BLOCK_HEADER_LEN[i8_index] = SERIAL_IN;
  ++i8_index;
  goto MAIN_LOOP_STAGE6_B;  // : REM GET NEXT FILE LENGTH CHARACTER
  
MAIN_LOOP_STAGE6_C:  
  BLOCK_HEADER_LEN_bytes = strtoul(BLOCK_HEADER_LEN, &endptr, 10);
  
  //REM TBD - HANDLE THESE LATER...
  //REM PART 3: <TIME MODIFIED IN ASCII/OCTAL> <SPACE>
  //REM PART 4: <MODE> <SPACE>
  //REM PART 5: <SERIAL NUMBER> <SPACE>
  //REM PART 6: <OTHER> <SPACE>
  //REM PART 7: <OPTIONAL FIELDS>
  //REM PART 8: END IS NUMBER OF 128 BYTE BLOCKS
  
  if (
    (strlen(BLOCK_HEADER_FN) == 0) 
    && (BLOCK_HEADER_LEN_bytes == 0)
  ) goto PRESUME_FINAL_EOT;
  
#ifdef DEBUG_OUTPUT
  // Filename and length will be output in other debug printf's.
#else
  printf("FILE [%s] LEN [%lu]\n", BLOCK_HEADER_FN, BLOCK_HEADER_LEN_bytes);
#endif
  
#ifdef DUMP_TO_FILE
  // DELETE THE FILE

  // Filename prefixed with "s:" starting at $AFFE (actual header block starts at $B000)
  BLOCK_HEADER_FN_LEN += 2;
  __asm__("lda %v", BLOCK_HEADER_FN_LEN);  // filename length
  __asm__("ldx #$FE");     // LO
  __asm__("ldy #$AF");     // HI
  __asm__("jsr $FFBD");    // SETNAM

  __asm__("lda #1");      // file num  
  __asm__("ldx %v", DEVICE_NUMBER);  // device
  __asm__("ldy #15");      // OPEN: 1 == SAVE (open for write)
  __asm__("jsr $FFBA");    // SETLFS

  __asm__("jsr $FFC0");    // OPEN   (LOAD == $FFD5)

  __asm__("lda #1");      // file num  
  __asm__("jsr $FFC3");    // CLOSE 
  
  __asm__("jsr $FFCC");    // CLRCHN 
  BLOCK_HEADER_FN_LEN -= 2;
  
  // ------------------------------

  __asm__("lda %v", BLOCK_HEADER_FN_LEN);  // filename length
  __asm__("ldx #$00");     // LO
  __asm__("ldy #$B0");     // HI  IN_BUFFER_START_ADDRESS
  __asm__("jsr $FFBD");    // SETNAM

  __asm__("lda #$01");     // file num  
  __asm__("ldx %v", DEVICE_NUMBER);  // device
  __asm__("ldy #$01");     // OPEN: 1 == SAVE (open for write)
  __asm__("jsr $FFBA");    // SETLFS
    
  __asm__("jsr $FFC0");    // OPEN
  
  if (BLOCK_HEADER_LEN_bytes == 0)
  {
    __asm__("lda #$01");   // file index
    __asm__("jsr $FFC3");  // CLOSE
	
	__asm__("jsr $FF4A");  // CLOSE_ALL

    //__asm__("jsr $FFCC");  // CLRCHN 
  }
  else
  {
    __asm__("ldx #$01");   // file num  
    __asm__("jsr $FFC9");  // CHKOUT
  }
#endif

#ifdef DUMP_TO_VRAM1
  // VERA RESET (this was a header block, so we are starting a new file-receive now)
  POKE(0x9F22,0x10);  // : REM HI. (SET AUTO-INCREMENT TO 1-BYTE)
  POKE(0x9F21,0x00);  // : REM MED.
  POKE(0x9F20,0x00);  // : REM LO. 
#endif

// REM STAGE7 - SEND ACK AND "C" (CONTINUE) RESPONSE
MAIN_LOOP_STAGE7:
#ifdef DEBUG_OUTPUT
  printf("7:ACK  ");
#endif
  SERIAL_OUT = ASCII_ACK;  SEND_ONE();  // WARNING: What if the sender misses this ACK?  (it will timeout and resend the last known good block)
  if (TOTAL_BLOCK == 0)
  {
    SERIAL_OUT = ASCII_C;  SEND_ONE();  // "C" 'C' ONLY ON HEADER BLOCK
  }
  //REM IF RECEIVED.BYTES < BLOCK.HEADER.LEN THEN A$ = "C" : GOSUB SEND.ONE
  //REM A$ = "C" : GOSUB SEND.ONE

//REM STAGE8 - WAIT FOR EOT RESPONSE
//REM EOT = ASCII 4 (NOTHING LEFT FOR THIS FILE STREAM)
//REM OR - ANOTHER DATA BLOCK (#01 OR #02, +1, -1)
//MAIN_LOOP_STAGE8:
#ifdef DEBUG_OUTPUT
  printf("\n");
#endif

  // As 8-bit types, these will "naturally" roll-over from FF to 00, or 00 back to FF
  ++CURR_EXPECTED_MARKER1;  //  = CURR_EXPECTED_MARKER1%+1
  --CURR_EXPECTED_MARKER2;  //  = CURR_EXPECTED_MARKER2%-1

//SKIP_INCREMENT:
  ++TOTAL_BLOCK;  // = CURR.BLOCK + 1   (this is "total" xfer blocks for this file)
  CRC_RESULT = 0;
  IN_BUFFER_POS = IN_BUFFER_START_ADDRESS;
  
  goto MAIN_LOOP_STAGE1;  // do it all again!
  
PRESUME_FINAL_EOT:
#ifdef DEBUG_OUTPUT
  printf("[ BLANK FILE, 0 LEN == END OF BATCH ]\n");
#else
  printf("[ END OF BATCH ]\n");
#endif
  //REM SEND FINAL ACK
  SERIAL_OUT = ASCII_ACK;  SEND_ONE();
  //SERIAL_OUT = ASCII_ACK;  SEND_ONE();  // Could do double ACK, but is non-standard to do so.
  
#ifdef DUMP_TO_FILE
  //__asm__("jsr $FF4A");  // CLOSE_ALL
  //__asm__("jsr $FFCC");  // CLRCHN
  
  // Delete file [Y] if it already exists
  POKE(0x05FE, 's');
  POKE(0x05FF, ':');  
  __asm__("lda #$01");     // filename length
  __asm__("ldx #$FE");     // LO
  __asm__("ldy #$05");     // HI
  __asm__("jsr $FFBD");    // SETNAM

  __asm__("lda #1");       // file num  
  __asm__("ldx %v", DEVICE_NUMBER);  // device
  __asm__("ldy #15");      // OPEN: 15 == command
  __asm__("jsr $FFBA");    // SETLFS

  __asm__("jsr $FFC0");    // OPEN

  __asm__("lda #1");       // file num  
  __asm__("jsr $FFC3");    // CLOSE 
  
  __asm__("jsr $FFCC");    // CLRCHN 
  // --------------------------------------------
  
  // Recreate file [Y]
  __asm__("lda #$01");     // filename length
  __asm__("ldx #$00");     // LO
  __asm__("ldy #$60");     // HI  IN_BUFFER_START_ADDRESS
  __asm__("jsr $FFBD");    // SETNAM

  __asm__("lda #$01");     // file num  
  __asm__("ldx %v", DEVICE_NUMBER);  // device
  __asm__("ldy #$01");     // OPEN: 1 == SAVE (open for write)
  __asm__("jsr $FFBA");    // SETLFS
    
  __asm__("jsr $FFC0");    // OPEN
  
  __asm__("lda #$01");     // file index
  __asm__("jsr $FFC3");    // CLOSE

  __asm__("jsr $FFCC");    // CLRCHN 
  
  // --------------------------------------------

  // Delete temporary file [Y]
  __asm__("lda #$01");     // filename length
  __asm__("ldx #$FE");     // LO
  __asm__("ldy #$05");     // HI
  __asm__("jsr $FFBD");    // SETNAM

  __asm__("lda #1");      // file num  
  __asm__("ldx %v", DEVICE_NUMBER);  // device
  __asm__("ldy #15");      // OPEN: 15 == command
  __asm__("jsr $FFBA");    // SETLFS

  __asm__("jsr $FFC0");    // OPEN

  __asm__("lda #1");      // file num  
  __asm__("jsr $FFC3");    // CLOSE 
  
  __asm__("jsr $FFCC");    // CLRCHN 
  
#endif
  
  while (TRUE)
  {
    // All batched files have been received.
    wait_line_quiet(108000);  // 30min ==> 30min *60 seconds/min * 60 jiffies/second == 108,000
  }
}
