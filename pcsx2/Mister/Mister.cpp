#include "PrecompiledHeader.h"
#include "Mister.h"
#include "Host.h"
#include "Config.h"
#include "GS/Renderers/Common/GSDevice.h"
#include <chrono>
#include <thread>

MiSTer::MiSTer()
{
}

MiSTer::~MiSTer()
{
	// Shutdown background worker thread safely
	{
		std::lock_guard<std::mutex> lock(m_queue_mutex);
		m_shutdown = true;
	}
	m_queue_cv.notify_all();

	if (m_worker_thread.joinable())
		m_worker_thread.join();
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
   // Copy config values to avoid EmuConfig access from worker thread
   m_mister_ip = EmuConfig.GS.MisterIP;
   m_hardcoded_vsync = EmuConfig.GS.MisterHardcodedVSync;
   m_vsync_value = EmuConfig.GS.MisterVSync;

   lz4_compress = true;
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

   // Start background worker thread - it will handle all socket init
   m_shutdown = false;
   m_worker_thread = std::thread(&MiSTer::WorkerThreadFunc, this);
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

void MiSTer::CmdBlitFrameBuffer(const u8* framebuffer, int width, int height, int pitch)
{
	if (!framebuffer)
		return;

	// Convert RGBA framebuffer to RGB format for MiSTer
	std::vector<char> rgb_buffer;

	// Use actual PS2 output resolution
	int final_width = width;
	int final_height = height;

	rgb_buffer.resize(final_width * final_height * 3);

	// Convert RGBA to RGB
	for (int y = 0; y < final_height; y++)
	{
		for (int x = 0; x < final_width; x++)
		{
			// Source pixel (RGBA, 4 bytes per pixel)
			const u8* src_pixel = framebuffer + (y * pitch) + (x * 4);

			// Destination pixel (RGB, 3 bytes per pixel)
			int dst_index = (y * final_width + x) * 3;

			rgb_buffer[dst_index + 0] = src_pixel[0]; // R
			rgb_buffer[dst_index + 1] = src_pixel[1]; // G
			rgb_buffer[dst_index + 2] = src_pixel[2]; // B
			// Skip alpha channel (src_pixel[3])
		}
	}

	// Queue frame for background transmission - return immediately from hot path
	{
		std::lock_guard<std::mutex> lock(m_queue_mutex);
		m_frame_queue.push({std::move(rgb_buffer), final_width, final_height});
	}
	m_queue_cv.notify_one();
}

void MiSTer::WorkerThreadFunc()
{
	// Initialize socket on worker thread
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
	ServerAddr.sin_addr.s_addr = inet_addr(m_mister_ip.c_str());

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
	ServerAddr.sin_addr.s_addr = inet_addr(m_mister_ip.c_str());

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
   buffer[1] = 1; // Always enable LZ4 frames for 480p
   buffer[2] = 0; // No audio for now
   buffer[3] = 0; // No audio channels for now

   Send(&buffer[0], 4);

	while (!m_shutdown)
	{
		std::unique_lock<std::mutex> lock(m_queue_mutex);
		m_queue_cv.wait(lock, [this] { return !m_frame_queue.empty() || m_shutdown; });

		if (m_shutdown)
			break;

		if (m_frame_queue.empty())
			continue;

		// Only process the most recent frame, discard older ones
		FrameData frame_data;
		while (!m_frame_queue.empty())
		{
			frame_data = std::move(m_frame_queue.front());
			m_frame_queue.pop();
		}
		lock.unlock();

		// Increment frame counter on worker thread
		frame++;

		// All MiSTer timing and transmission in background thread
		SetEndEmulate();
		SetStartBlit();

		uint16_t vsync_setting = m_hardcoded_vsync ? m_vsync_value : 0;

		uint8_t blockLinesFactor = (frame_data.width > 384) ? 5 : 4;
		uint32_t blockSize = (frame_data.width << blockLinesFactor) * 3;
		if (blockSize > MAX_LZ4_BLOCK)
			blockSize = MAX_LZ4_BLOCK;

		uint32_t bufferSize = frame_data.width * frame_data.height * 3;
		uint32_t compressed_data_size = LZ4_compress_default(frame_data.rgb_data.data(), m_fb_compressed, bufferSize, LZ4_COMPRESSBOUND(MAX_BUFFER_HEIGHT * MAX_BUFFER_WIDTH * 3));

		// Create GroovyMAME-compatible blit command structure
		struct {
			const uint8_t cmd = CMD_BLIT_VSYNC;
			uint32_t frame;
			uint16_t vsync;
			uint32_t compressed_data_size;
		} blit_command;

		blit_command.frame = frame;
		blit_command.vsync = vsync_setting;
		blit_command.compressed_data_size = compressed_data_size;

		Send(&blit_command, sizeof(blit_command));
		SendMTU(m_fb_compressed, compressed_data_size, 1472);

		// Synchronize with MiSTer like Dolphin does
		SetEndBlit();
		Sync();
		SetStartEmulate();
	}
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
   		memcpy(&frameUDP, &bufferRecv[0], 4);
   		if (frameUDP > frameEcho)
   		{
   			frameEcho = frameUDP;
   			memcpy(&vcountEcho, &bufferRecv[4], 2);
   			memcpy(&frameGPU, &bufferRecv[6], 4);
   			memcpy(&vcountGPU, &bufferRecv[10], 2);
   			memcpy(&fpga_debug_bits, &bufferRecv[12], 1);

   			bitByte bits;
   			bits.byte = fpga_debug_bits;
   			fpga_vram_ready     = bits.u.bit0;
   			fpga_vram_end_frame = bits.u.bit1;
   			fpga_vram_synced    = bits.u.bit2;
   			fpga_vga_frameskip  = bits.u.bit3;
   			fpga_vga_vblank     = bits.u.bit4;
   			fpga_vga_f1         = bits.u.bit5;
   			fpga_audio          = bits.u.bit6;
   			fpga_vram_queue     = bits.u.bit7;
   		}
   	}
   } while (len > 0 && frame != frameEcho);
}

bool MiSTer::WaitForACK(double timeout_ms)
{
	const auto start_time = std::chrono::steady_clock::now();

	while (frame != frameEcho)
	{
		ReceiveBlitACK();

		auto elapsed = std::chrono::steady_clock::now() - start_time;
		if (std::chrono::duration<double, std::milli>(elapsed).count() > timeout_ms)
		{
			Console.Warning("MiSTer: ACK timeout after %.1fms", timeout_ms);
			return false;
		}

		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}

	return true;
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