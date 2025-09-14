#include "PrecompiledHeader.h"
#include "Mister.h"
#include "Host.h"
#include "Config.h"
#include "GS/Renderers/Common/GSDevice.h"

MiSTer::MiSTer()
{
}

MiSTer::~MiSTer()
{
}

void MiSTer::CmdClose(void)
{
   char buffer[1];
   buffer[0] = CMD_CLOSE;
   Send((char*)&buffer[0], 1);
#ifdef _WIN32
   closesocket(sockfd);
   WSACleanup();
#else
   close(sockfd);
#endif
}

void MiSTer::CmdInit(void)
{
	char buffer[4];

#ifdef _WIN32
	WSADATA wsd;
	int rc;

	Console.WriteLn("MiSTer: Initialising Winsock...");
	rc = WSAStartup(MAKEWORD(2, 2), &wsd);
	if (rc != 0)
	{
		Console.Error("MiSTer: Unable to load Winsock: %d", rc);
		return;
	}

	Console.WriteLn("MiSTer: Initialising socket...");
	sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sockfd == INVALID_SOCKET)
	{
		Console.Error("MiSTer: socket error: %d", WSAGetLastError());
		return;
	}

	memset(&ServerAddr, 0, sizeof(ServerAddr));
	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_port = htons(32100);
	ServerAddr.sin_addr.s_addr = inet_addr(EmuConfig.GS.MisterIP.c_str());

	Console.WriteLn("MiSTer: Setting socket async...");
	u_long iMode = 1;
	rc = ioctlsocket(sockfd, FIONBIO, &iMode);
	if (rc == SOCKET_ERROR)
	{
		Console.Error("MiSTer: set nonblock fail: %d", WSAGetLastError());
		return;
	}

	Console.WriteLn("MiSTer: Setting send buffer to 2097152 bytes...");
	int optVal = 2097152;
	int optLen = sizeof(int);
	rc = setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (char*)&optVal, optLen);
	if (rc == SOCKET_ERROR)
	{
		Console.Error("MiSTer: set so_sndbuff fail: %d", WSAGetLastError());
		return;
	}

#else

	Console.WriteLn("MiSTer: Initialising socket...");
	sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sockfd < 0)
	{
		Console.Error("MiSTer: socket error");
		return;
	}

	memset(&ServerAddr, 0, sizeof(ServerAddr));
	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_port = htons(32100);
	ServerAddr.sin_addr.s_addr = inet_addr(EmuConfig.GS.MisterIP.c_str());

	Console.WriteLn("MiSTer: Setting socket async...");
	int flags;
	flags = fcntl(sockfd, F_GETFL, 0);
	if (flags < 0)
	{
		Console.Error("MiSTer: get flag error");
		return;
	}
	flags |= O_NONBLOCK;
	if (fcntl(sockfd, F_SETFL, flags) < 0)
	{
		Console.Error("MiSTer: set nonblock fail");
		return;
	}

	Console.WriteLn("MiSTer: Setting send buffer to 2097152 bytes...");
	int size = 2 * 1024 * 1024;
	if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (void*)&size, sizeof(size)) < 0)
	{
		Console.Error("MiSTer: Error so_sndbuff");
		return;
	}

#endif

   Console.WriteLn("MiSTer: Sending CMD_INIT...");

   buffer[0] = CMD_INIT;
   buffer[1] = (lz4_frames) ? 1 : 0;
   buffer[2] = (sound_rate == 22050) ? 1 : (sound_rate == 44100) ? 2 : (sound_rate == 48000) ? 3 : 0;
   buffer[3] = sound_chan;

   Send(&buffer[0], 4);

   lz4_compress = lz4_frames;
   width = 0;
   height = 0;
   lines = 0;
   widthTime = 0;
   frame = 0;
   frameTime = 0;
   emulationTime = 0;
   avgEmulationTime = 0;
   vsync_auto = 120;
   blitTime = 0;
   frameEcho = 0;
   vcountEcho = 0;
   frameGPU = 0;
   vcountGPU = 0;
   interlaced = 0;
}

void MiSTer::CmdSwitchres240p()
{
    char buffer[26];

    double px = 12.123704;
    uint16_t udp_hactive = 640;
    uint16_t udp_hbegin = 658;
    uint16_t udp_hend = 715;
    uint16_t udp_htotal = 775;
    uint16_t udp_vactive = 240;
    uint16_t udp_vbegin = 243;
    uint16_t udp_vend = 246;
    uint16_t udp_vtotal = 262;
    uint8_t  udp_interlace = 0;

    width = udp_hactive;
    height = udp_vactive;
    lines = udp_vtotal;
    interlaced = udp_interlace;

    widthTime = round((double) udp_htotal * (1 / px));
    frameTime = widthTime * udp_vtotal;

    if (interlaced)
    {
        frameField = 0;
        frameTime = frameTime >> 1;
    }

    buffer[0] = CMD_SWITCHRES;
    memcpy(&buffer[1],&px,sizeof(px));
    memcpy(&buffer[9],&udp_hactive,sizeof(udp_hactive));
    memcpy(&buffer[11],&udp_hbegin,sizeof(udp_hbegin));
    memcpy(&buffer[13],&udp_hend,sizeof(udp_hend));
    memcpy(&buffer[15],&udp_htotal,sizeof(udp_htotal));
    memcpy(&buffer[17],&udp_vactive,sizeof(udp_vactive));
    memcpy(&buffer[19],&udp_vbegin,sizeof(udp_vbegin));
    memcpy(&buffer[21],&udp_vend,sizeof(udp_vend));
    memcpy(&buffer[23],&udp_vtotal,sizeof(udp_vtotal));
    memcpy(&buffer[25],&udp_interlace,sizeof(udp_interlace));
    Send(&buffer[0], 26);
}

void MiSTer::CmdSwitchres480i()
{
    char buffer[26];

    double px = 12.146841;
    uint16_t udp_hactive = 640;
    uint16_t udp_hbegin = 658;
    uint16_t udp_hend = 715;
    uint16_t udp_htotal = 772;
    uint16_t udp_vactive = 480;
    uint16_t udp_vbegin = 487;
    uint16_t udp_vend = 493;
    uint16_t udp_vtotal = 525;
    uint8_t  udp_interlace = 1;

    width = udp_hactive;
    height = udp_vactive;
    lines = udp_vtotal;
    interlaced = udp_interlace;

    widthTime = round((double) udp_htotal * (1 / px));
    frameTime = widthTime * udp_vtotal;

    if (interlaced)
    {
        frameField = 0;
        frameTime = frameTime >> 1;
    }

    buffer[0] = CMD_SWITCHRES;
    memcpy(&buffer[1],&px,sizeof(px));
    memcpy(&buffer[9],&udp_hactive,sizeof(udp_hactive));
    memcpy(&buffer[11],&udp_hbegin,sizeof(udp_hbegin));
    memcpy(&buffer[13],&udp_hend,sizeof(udp_hend));
    memcpy(&buffer[15],&udp_htotal,sizeof(udp_htotal));
    memcpy(&buffer[17],&udp_vactive,sizeof(udp_vactive));
    memcpy(&buffer[19],&udp_vbegin,sizeof(udp_vbegin));
    memcpy(&buffer[21],&udp_vend,sizeof(udp_vend));
    memcpy(&buffer[23],&udp_vtotal,sizeof(udp_vtotal));
    memcpy(&buffer[25],&udp_interlace,sizeof(udp_interlace));
    Send(&buffer[0], 26);
}

void MiSTer::CmdSwitchres480p()
{
    char buffer[26];

    double px = 25.175;
    uint16_t udp_hactive = 640;
    uint16_t udp_hbegin = 656;
    uint16_t udp_hend = 752;
    uint16_t udp_htotal = 800;
    uint16_t udp_vactive = 480;
    uint16_t udp_vbegin = 490;
    uint16_t udp_vend = 492;
    uint16_t udp_vtotal = 525;
    uint8_t  udp_interlace = 0;

    width = udp_hactive;
    height = udp_vactive;
    lines = udp_vtotal;
    interlaced = udp_interlace;

    widthTime = round((double) udp_htotal * (1 / px));
    frameTime = widthTime * udp_vtotal;

    if (interlaced)
    {
        frameField = 0;
        frameTime = frameTime >> 1;
    }

    buffer[0] = CMD_SWITCHRES;
    memcpy(&buffer[1],&px,sizeof(px));
    memcpy(&buffer[9],&udp_hactive,sizeof(udp_hactive));
    memcpy(&buffer[11],&udp_hbegin,sizeof(udp_hbegin));
    memcpy(&buffer[13],&udp_hend,sizeof(udp_hend));
    memcpy(&buffer[15],&udp_htotal,sizeof(udp_htotal));
    memcpy(&buffer[17],&udp_vactive,sizeof(udp_vactive));
    memcpy(&buffer[19],&udp_vbegin,sizeof(udp_vbegin));
    memcpy(&buffer[21],&udp_vend,sizeof(udp_vend));
    memcpy(&buffer[23],&udp_vtotal,sizeof(udp_vtotal));
    memcpy(&buffer[25],&udp_interlace,sizeof(udp_interlace));
    Send(&buffer[0], 26);
}

void MiSTer::CmdBlitTexture(class GSTexture* texture, const class GSVector4& src_uv, const class GSVector4& draw_rect)
{
	if (!texture || !g_gs_device)
		return;

	// Download texture data from GPU
	GSTexture::GSMap map;
	GSVector4i rect(0, 0, texture->GetWidth(), texture->GetHeight());

	if (!g_gs_device->DownloadTexture(texture, rect, map))
	{
		Console.Error("MiSTer: DownloadTexture failed");
		return;
	}

	// Get texture properties
	int tex_width = texture->GetWidth();
	int tex_height = texture->GetHeight();
	GSTexture::Format format = texture->GetFormat();

	// Calculate output dimensions from draw_rect
	int out_width = (int)(draw_rect.z - draw_rect.x);
	int out_height = (int)(draw_rect.w - draw_rect.y);

	// Convert texture to RGB format for MiSTer
	std::vector<char> rgb_buffer;
	int final_width = 640;  // MiSTer standard resolution
	int final_height = 480;

	if (format == GSTexture::Format::Color)
	{
		// RGBA8 -> RGB conversion
		rgb_buffer.resize(final_width * final_height * 3);

		for (int y = 0; y < std::min(tex_height, final_height); y++)
		{
			for (int x = 0; x < std::min(tex_width, final_width); x++)
			{
				const u8* src_pixel = map.bits + (y * map.pitch) + (x * 4); // RGBA = 4 bytes
				char* dst_pixel = rgb_buffer.data() + ((y * final_width) + x) * 3;

				dst_pixel[0] = src_pixel[0]; // R
				dst_pixel[1] = src_pixel[1]; // G
				dst_pixel[2] = src_pixel[2]; // B
			}
		}
	}
	else
	{
		Console.Error("MiSTer: Unsupported texture format %d", (int)format);
		g_gs_device->DownloadTextureComplete();
		return;
	}

	// Send frame header - always use LZ4 compression for 480p
	char header[9];
	frame++;

	uint16_t vsync_setting = EmuConfig.GS.MisterHardcodedVSync ? EmuConfig.GS.MisterVSync : 0;

	uint8_t blockLinesFactor = (final_width > 384) ? 5 : 4;
	uint32_t blockSize = (final_width << blockLinesFactor) * 3;
	if (blockSize > MAX_LZ4_BLOCK)
		blockSize = MAX_LZ4_BLOCK;

	header[0] = CMD_BLIT_VSYNC;
	memcpy(&header[1], &frame, sizeof(frame));
	memcpy(&header[5], &vsync_setting, sizeof(vsync_setting));
	header[7] = (uint16_t) blockSize & 0xff;
	header[8] = (uint16_t) blockSize >> 8;

	Send(&header[0], 9);

	// Send RGB data with LZ4 compression
	uint32_t bufferSize = final_width * final_height * 3;
	SendLZ4(rgb_buffer.data(), bufferSize, blockSize);

	g_gs_device->DownloadTextureComplete();
}

void MiSTer::SetStartEmulate(void)
{
#ifdef _WIN32
  QueryPerformanceCounter(&tickStartEmulate);
#else
  clock_gettime(CLOCK_MONOTONIC, &tickStartEmulate);
#endif
}

void MiSTer::SetEndEmulate(void)
{
#ifdef _WIN32
  QueryPerformanceCounter(&tickEndEmulate);
  emulationTime = (tickEndEmulate.QuadPart - tickStartEmulate.QuadPart) / 10;
#else
  clock_gettime(CLOCK_MONOTONIC, &tickEndEmulate);
  emulationTime = DiffTimespec(tickStartEmulate, tickEndEmulate) / 10;
#endif

  if (frame > 10)
  {
   	avgEmulationTime = (avgEmulationTime == 0) ? emulationTime + blitTime: (avgEmulationTime + emulationTime + blitTime) / 2;
   	vsync_auto = height - round(lines * avgEmulationTime) / frameTime;
   	if (vsync_auto > 480) vsync_auto = 1;
  }
  else
  {
  	avgEmulationTime = 0;
  	vsync_auto = 120;
  }
}

void MiSTer::SetStartBlit(void)
{
#ifdef _WIN32
  QueryPerformanceCounter(&tickStartBlit);
#else
  clock_gettime(CLOCK_MONOTONIC, &tickStartBlit);
#endif
}

void MiSTer::SetEndBlit(void)
{
#ifdef _WIN32
  QueryPerformanceCounter(&tickEndBlit);
  blitTime = (tickEndBlit.QuadPart - tickStartBlit.QuadPart) / 10;
#else
  clock_gettime(CLOCK_MONOTONIC, &tickEndBlit);
  blitTime = DiffTimespec(tickStartBlit, tickEndBlit) / 10;
#endif
}

void MiSTer::Sync()
{
  int prevSleepTime = 0;
  int sleepTime = 0;
  int realSleepTime = 0;
  int elapsedTime = 0;
  int diffTime = GetVSyncDif();

#ifdef _WIN32
  LARGE_INTEGER tick;
  LARGE_INTEGER tick2;

  QueryPerformanceCounter(&tick);
  elapsedTime = (frame == 1) ? (frameTime / 2) : (tick.QuadPart - tickLastSync.QuadPart) / 10;
#else
  timespec tick;
  timespec tick2;

  clock_gettime(CLOCK_MONOTONIC, &tick);
  elapsedTime = (frame == 1) ? (frameTime / 2) : DiffTimespec(tickLastSync, tick) / 10;
#endif

  sleepTime = (frameTime - elapsedTime + diffTime) * 10;
  prevSleepTime = sleepTime;
  tick2 = tick;

  if (sleepTime > 0)
  {
  	do
  	{
  		if (frame != frameEcho)
  		{
  			diffTime = GetVSyncDif();
  			sleepTime = sleepTime + (diffTime * 10);
  		}

  		#ifdef _WIN32
  		 QueryPerformanceCounter(&tick2);
  		 realSleepTime = (tick2.QuadPart - tick.QuadPart);
  		#else
  		 clock_gettime(CLOCK_MONOTONIC, &tick2);
  		 realSleepTime = DiffTimespec(tick, tick2);
  		#endif

  	} while (realSleepTime < sleepTime);
  }

  tickLastSync = tick2;
}

int MiSTer::GetVSyncDif(void)
{
  uint32_t prevFrameEcho = frameEcho;
  int diffTime = 0;

  if (frame != frameEcho)
  {
  	ReceiveBlitACK();
  }

  if (prevFrameEcho != frameEcho)
  {
  	if ((frameEcho + 1) < frameGPU)
  	{
  	 	frame = frameGPU + 1;
  	}

   	uint32_t vcount1 = ((frameEcho - 1) * lines + vcountEcho) >> interlaced;
	uint32_t vcount2 = (frameGPU * lines + vcountGPU) >> interlaced;
	int dif = (int) (vcount1 - vcount2) / 2;

	diffTime = (int) (widthTime * dif);
  }

  return diffTime;
}

int MiSTer::GetField(void)
{
	int field = 0;
	if (interlaced)
	{
		frameField = !frameField;
		field = frameField;
	}

	return field;
}

bool MiSTer::isInterlaced(void)
{
	return interlaced;
}

bool MiSTer::is480p(void)
{
	return ((height > 240 && !interlaced) || (width < height));
}

void MiSTer::Send(void *cmd, int cmdSize)
{
  sendto(sockfd, (char *) cmd, cmdSize, 0, (struct sockaddr *)&ServerAddr, sizeof(ServerAddr));
}

void MiSTer::SendMTU(char *buffer, int bytes_to_send, int chunk_max_size)
{
   int bytes_this_chunk = 0;
   int chunk_size = 0;
   uint32_t offset = 0;

   do
   {
	chunk_size = bytes_to_send > chunk_max_size? chunk_max_size : bytes_to_send;
	bytes_to_send -= chunk_size;
	bytes_this_chunk = chunk_size;

	Send(buffer + offset, bytes_this_chunk);
	offset += chunk_size;

   } while (bytes_to_send > 0);
}

void MiSTer::SendLZ4(char *buffer, int bytes_to_send, int block_size)
{
   LZ4_stream_t lz4_stream_body;
   LZ4_stream_t* lz4_stream = &lz4_stream_body;
   LZ4_initStream(lz4_stream, sizeof(*lz4_stream));

   int inp_buf_index = 0;
   int bytes_this_chunk = 0;
   int chunk_size = 0;
   uint32_t offset = 0;

   do
   {
	chunk_size = bytes_to_send > block_size? block_size : bytes_to_send;
	bytes_to_send -= chunk_size;
	bytes_this_chunk = chunk_size;

	char* const inp_ptr = inp_buf[inp_buf_index];
	memcpy((char *)&inp_ptr[0], buffer + offset, chunk_size);

	const uint16_t c_size = LZ4_compress_fast_continue(lz4_stream, inp_ptr, (char *)&m_fb_compressed[2], bytes_this_chunk, sizeof(m_fb_compressed), 1);
	uint16_t *c_size_ptr = (uint16_t *)&m_fb_compressed[0];
	*c_size_ptr = c_size;

	SendMTU((char *) &m_fb_compressed[0], c_size + 2, 1472);
	offset += chunk_size;
	inp_buf_index ^= 1;

   } while (bytes_to_send > 0);
}

void MiSTer::ReceiveBlitACK(void)
{
   uint32_t frameUDP = frameEcho;

   socklen_t sServerAddr = sizeof(struct sockaddr);
   int len = 0;
   do
   {
   	len = recvfrom(sockfd, bufferRecv, sizeof(bufferRecv), 0, (struct sockaddr *)&ServerAddr, &sServerAddr);
   	if (len >= 13)
   	{
   		memcpy(&frameUDP, &bufferRecv[0],4);
   		if (frameUDP > frameEcho)
   		{
   			frameEcho = frameUDP;
   			memcpy(&vcountEcho, &bufferRecv[4],2);
			memcpy(&frameGPU, &bufferRecv[6],4);
			memcpy(&vcountGPU, &bufferRecv[10],2);
			memcpy(&fpga_debug_bits, &bufferRecv[12],1);

			bitByte bits;
			bits.byte = fpga_debug_bits;
			fpga_vram_end_frame = bits.u.bit1;
			fpga_vram_synced    = bits.u.bit2;
			fpga_vga_frameskip  = bits.u.bit3;
			fpga_vga_vblank     = bits.u.bit4;
			fpga_vga_f1         = bits.u.bit5;
 			fpga_vram_queue     = bits.u.bit7;
   		}
   	}
   } while (len > 0 && frame != frameEcho);
}

#ifndef _WIN32
uint32_t MiSTer::DiffTimespec(timespec start, timespec end)
{
	uint32_t diffTime = 0;
        timespec temp;
        if ((end.tv_nsec - start.tv_nsec) < 0)
        {
                temp.tv_sec = end.tv_sec - start.tv_sec - 1;
                temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
        }
        else
        {
                temp.tv_sec = end.tv_sec - start.tv_sec;
                temp.tv_nsec = end.tv_nsec - start.tv_nsec;
        }
        diffTime = (temp.tv_sec * 1000000000) + temp.tv_nsec;
        return diffTime / 100;
}
#endif

MiSTer g_mister;