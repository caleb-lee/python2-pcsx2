#pragma once

#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <cstring>
#include <array>
#include <mutex>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <netinet/udp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#endif

#include <lz4.h>
#include <lz4hc.h>

#define CMD_CLOSE 1
#define CMD_INIT 2
#define CMD_SWITCHRES 3
#define CMD_AUDIO 4
#define CMD_GET_STATUS 5
#define CMD_BLIT_VSYNC 6

#define MAX_BUFFER_WIDTH 1024
#define MAX_BUFFER_HEIGHT 768
#define MAX_LZ4_BLOCK   61440

typedef union
{
  struct
  {
    unsigned char bit0 : 1;
    unsigned char bit1 : 1;
    unsigned char bit2 : 1;
    unsigned char bit3 : 1;
    unsigned char bit4 : 1;
    unsigned char bit5 : 1;
    unsigned char bit6 : 1;
    unsigned char bit7 : 1;
  }u;
   uint8_t byte;
} bitByte;

class MiSTer
{
public:
  MiSTer();
  ~MiSTer();

  void CmdClose(void);
  void CmdInit(const char* mister_host, short mister_port, bool lz4_frames, uint32_t sound_rate, uint8_t sound_chan);
  void CmdSwitchres240p();
  void CmdSwitchres480i();
  void CmdSwitchres480p();
  void CmdBlit(char *bufferFrame, uint16_t vsync);

  void SetStartEmulate(void);
  void SetEndEmulate(void);
  void SetStartBlit(void);
  void SetEndBlit(void);

  void Sync(void);
  int  GetVSyncDif(void);

  int GetField(void);
  bool isInterlaced(void);
  bool is480p(void);

private:
  bool lz4_compress = false;
  uint32_t frame = 0;
  uint8_t  frameField = 0;
  uint16_t width = 0;
  uint16_t height = 0;
  uint16_t lines = 0;
  uint8_t  interlaced = 0;
  uint32_t widthTime = 0;
  uint32_t frameTime = 0;

  uint32_t emulationTime = 0;
  uint32_t avgEmulationTime = 0;
  uint32_t blitTime = 0;

  uint16_t vsync_auto = 120;

  uint32_t frameEcho = 0;
  uint16_t vcountEcho = 0;
  uint32_t frameGPU = 0;
  uint16_t vcountGPU = 0;

  uint8_t fpga_debug_bits = 0;
  uint8_t fpga_vram_end_frame = 0;
  uint8_t fpga_vram_synced = 0;
  uint8_t fpga_vga_frameskip = 0;
  uint8_t fpga_vga_vblank = 0;
  uint8_t fpga_vga_f1 = 0;
  uint8_t fpga_vram_queue = 0;

#ifdef _WIN32
  SOCKET sockfd;
  LARGE_INTEGER tickStartEmulate, tickEndEmulate;
  LARGE_INTEGER tickStartBlit, tickEndBlit;
  LARGE_INTEGER tickLastSync;
#else
  int sockfd;
  timespec tickStartEmulate, tickEndEmulate;
  timespec tickStartBlit, tickEndBlit;
  timespec tickLastSync;
#endif

  struct sockaddr_in ServerAddr;
  char bufferRecv[256];

  char inp_buf[2][MAX_LZ4_BLOCK];
  char m_fb_compressed[MAX_LZ4_BLOCK + 2];

  void Send(void *cmd, int cmdSize);
  void SendMTU(char *buffer, int bytes_to_send, int chunk_max_size);
  void SendLZ4(char *buffer, int bytes_to_send, int block_size);
  void ReceiveBlitACK(void);

#ifndef _WIN32
  uint32_t DiffTimespec(timespec start, timespec end);
#endif
};

extern MiSTer g_mister;