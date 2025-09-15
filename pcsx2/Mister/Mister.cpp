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

// Copied from switchres modeline.cpp
static int round_near(double number)
{
	return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

static int round_near_odd(double number)
{
	return int(ceil(number)) % 2 == 0? floor(number) : ceil(number);
}

static int round_near_even(double number)
{
	return int(ceil(number)) % 2 == 1? floor(number) : ceil(number);
}

struct monitor_range_data {
	double hfreq_min;
	double hfreq_max;
	double vertical_blank;
};

static int total_lines_for_yres(int yres, double vfreq, monitor_range_data *range, double borders, double interlace)
{
	int vvt = max(yres / interlace + round_near(vfreq * yres / (interlace * (1.0 - vfreq * (range->vertical_blank + borders))) * (range->vertical_blank + borders)), 1);
	while ((vfreq * vvt < range->hfreq_min) && (vfreq * (vvt + 1) < range->hfreq_max)) vvt++;
	return vvt;
}

// Copied from switchres modeline.cpp
static int get_line_params(int hactive, double hfreq, double hfront_porch, double hsync_pulse, double hback_porch, int *hbegin, int *hend, int *htotal)
{
	int hhi, hhf, hht;
	int hh, hs, he, ht;
	double line_time, char_time, new_char_time;
	double hfront_porch_min, hsync_pulse_min, hback_porch_min;
	int char_size = 8;

	hfront_porch_min = hfront_porch * .90;
	hsync_pulse_min  = hsync_pulse  * .90;
	hback_porch_min  = hback_porch  * .90;

	line_time = 1 / hfreq * 1000000;

	hh = round(hactive / char_size);
	hs = he = ht = 1;

	do {
		char_time = line_time / (hh + hs + he + ht);
		if (hs * char_time < hfront_porch_min ||
			fabs((hs + 1) * char_time - hfront_porch) < fabs(hs * char_time - hfront_porch))
			hs++;

		if (he * char_time < hsync_pulse_min ||
			fabs((he + 1) * char_time - hsync_pulse) < fabs(he * char_time - hsync_pulse))
			he++;

		if (ht * char_time < hback_porch_min ||
			fabs((ht + 1) * char_time - hback_porch) < fabs(ht * char_time - hback_porch))
			ht++;

		new_char_time = line_time / (hh + hs + he + ht);
	} while (new_char_time != char_time);

	hhi = (hh + hs) * char_size;
	hhf = (hh + hs + he) * char_size;
	hht = (hh + hs + he + ht) * char_size;

	*hbegin = hhi;
	*hend = hhf;
	*htotal = hht;

	return 0;
}

void MiSTer::CmdSwitchresDynamic(int width, int height, bool is_interlaced)
{
	char buffer[26];

	// Use PS2 refresh rate
	double vfreq_real = 59.94;

	// Setup monitor range based on interlace mode from switchres monitor.cpp
	monitor_range_data range_data;
	struct monitor_range {
		double vfront_porch;
		double vsync_pulse;
		double vback_porch;
		double hfront_porch;
		double hsync_pulse;
		double hback_porch;
		int hsync_polarity;
		int vsync_polarity;
	} range;

	if (is_interlaced) {
		// arcade_15: "15625-16200, 49.50-65.00, 2.000, 4.700, 8.000, 0.064, 0.192, 1.024, 0, 0, 192, 288, 448, 576"
		range_data.hfreq_min = 15625.0;
		range_data.hfreq_max = 16200.0;
		range_data.vertical_blank = 1.280; // vfront + vsync + vback = 0.064 + 0.192 + 1.024
		range.vfront_porch = 0.064;
		range.vsync_pulse = 0.192;
		range.vback_porch = 1.024;
		range.hfront_porch = 2.000;
		range.hsync_pulse = 4.700;
		range.hback_porch = 8.000;
		range.hsync_polarity = 0;
		range.vsync_polarity = 0;
	} else {
		// arcade_31: "31400-31500, 49.50-65.00, 0.940, 3.770, 1.890, 0.349, 0.064, 1.017, 0, 0, 400, 512, 0, 0"
		range_data.hfreq_min = 31400.0;
		range_data.hfreq_max = 31500.0;
		range_data.vertical_blank = 1.430; // vfront + vsync + vback = 0.349 + 0.064 + 1.017
		range.vfront_porch = 0.349;
		range.vsync_pulse = 0.064;
		range.vback_porch = 1.017;
		range.hfront_porch = 0.940;
		range.hsync_pulse = 3.770;
		range.hback_porch = 1.890;
		range.hsync_polarity = 0;
		range.vsync_polarity = 0;
	}

	// Mode structure for calculation
	struct modeline {
		int hactive = width;
		int vactive = height;
		double vfreq = vfreq_real;
		double hfreq;
		double pclock;
		int hbegin;
		int hend;
		int htotal;
		int vbegin;
		int vend;
		int vtotal;
		bool interlace = false;
		bool doublescan = false;
		int hsync = 0;
		int vsync = 0;
	} mode;

	// Calculate total vertical lines using switchres formula
	double borders = 0.0;
	double interlace = 1.0;
	double scan_factor = 1.0;
	double interlace_incr = 0.0;

	int vvt_ini = total_lines_for_yres(mode.vactive, mode.vfreq, &range_data, borders, interlace) + interlace_incr;

	// Calculate horizontal frequency
	mode.hfreq = mode.vfreq * vvt_ini;

	// Fill horizontal part of modeline using switchres algorithm
	double hfront_porch_min = range.hfront_porch * 0.90;
	double hsync_pulse_min = range.hsync_pulse * 0.90;
	double hback_porch_min = range.hback_porch * 0.90;

	double line_time = 1.0 / mode.hfreq * 1000000.0;
	int char_size = 8; // Standard character size

	int hh = round_near(mode.hactive / char_size);
	int hs = 1, he = 1, ht = 1;

	double char_time, new_char_time;
	do {
		char_time = line_time / (hh + hs + he + ht);

		if (hs * char_time < hfront_porch_min ||
			fabs((hs + 1) * char_time - range.hfront_porch) < fabs(hs * char_time - range.hfront_porch))
			hs++;

		if (he * char_time < hsync_pulse_min ||
			fabs((he + 1) * char_time - range.hsync_pulse) < fabs(he * char_time - range.hsync_pulse))
			he++;

		if (ht * char_time < hback_porch_min ||
			fabs((ht + 1) * char_time - range.hback_porch) < fabs(ht * char_time - range.hback_porch))
			ht++;

		new_char_time = line_time / (hh + hs + he + ht);
	} while (new_char_time != char_time);

	int hhi = (hh + hs) * char_size;
	int hhf = (hh + hs + he) * char_size;
	int hht = (hh + hs + he + ht) * char_size;

	mode.hbegin = hhi;
	mode.hend = hhf;
	mode.htotal = hht;

	// Calculate pixel clock
	mode.pclock = mode.htotal * mode.hfreq;

	// Vertical blanking calculation
	mode.vtotal = vvt_ini * scan_factor;
	double vblank_lines = round_near(mode.hfreq * (range_data.vertical_blank + borders)) + interlace_incr;
	double margin = (mode.vtotal - mode.vactive - vblank_lines * scan_factor) / 2.0;

	double v_front_porch = margin + mode.hfreq * range.vfront_porch * scan_factor + interlace_incr;

	mode.vbegin = mode.vactive + max((int)round_near(v_front_porch), 1);
	mode.vend = mode.vbegin + max((int)round_near(mode.hfreq * range.vsync_pulse * scan_factor), 1);

	// Recalculate final vfreq
	mode.vfreq = (mode.hfreq / mode.vtotal) * scan_factor;

	mode.hsync = range.hsync_polarity;
	mode.vsync = range.vsync_polarity;
	mode.interlace = false;
	mode.doublescan = false;

	// Use calculated values
	uint16_t udp_hactive = mode.hactive;
	uint16_t udp_vactive = mode.vactive;
	uint16_t udp_htotal = mode.htotal;
	uint16_t udp_vtotal = mode.vtotal;
	double px = mode.pclock / 1000000.0;

	// Use switchres calculated sync positions
	uint16_t udp_hbegin = mode.hbegin;
	uint16_t udp_hend = mode.hend;
	uint16_t udp_vbegin = mode.vbegin;
	uint16_t udp_vend = mode.vend;

	uint8_t udp_interlace = mode.interlace ? 1 : 0;

	// Update internal state
	this->width = udp_hactive;
	this->height = udp_vactive;
	lines = udp_vtotal;
	interlaced = udp_interlace;
	widthTime = (int)((1000000.0 / mode.hfreq) * 100);
	frameTime = widthTime * udp_vtotal / 100;

	Console.WriteLn("MiSTer: Switching to %dx%d @ %.2fMHz (hfreq=%.1fkHz vtotal=%d) [%s]",
		width, height, px, mode.hfreq/1000.0, udp_vtotal, is_interlaced ? "15kHz" : "31kHz");

	buffer[0] = CMD_SWITCHRES;
	memcpy(&buffer[1], &px, sizeof(px));
	memcpy(&buffer[9], &udp_hactive, sizeof(udp_hactive));
	memcpy(&buffer[11], &udp_hbegin, sizeof(udp_hbegin));
	memcpy(&buffer[13], &udp_hend, sizeof(udp_hend));
	memcpy(&buffer[15], &udp_htotal, sizeof(udp_htotal));
	memcpy(&buffer[17], &udp_vactive, sizeof(udp_vactive));
	memcpy(&buffer[19], &udp_vbegin, sizeof(udp_vbegin));
	memcpy(&buffer[21], &udp_vend, sizeof(udp_vend));
	memcpy(&buffer[23], &udp_vtotal, sizeof(udp_vtotal));
	memcpy(&buffer[25], &udp_interlace, sizeof(udp_interlace));
	Send(&buffer[0], 26);
}

void MiSTer::CmdBlitFrameBuffer(const u8* framebuffer, int width, int height, int pitch, bool is_interlaced)
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
		m_frame_queue.push({std::move(rgb_buffer), final_width, final_height, is_interlaced});
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

	g_mister.SetStartEmulate();

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

		// Check for resolution or interlace change and send switchres if needed
		if (frame_data.width != m_current_width || frame_data.height != m_current_height || frame_data.is_interlaced != m_current_is_interlaced)
		{
			Console.WriteLn("MiSTer: Video mode changed from %dx%d (%s) to %dx%d (%s)",
				m_current_width, m_current_height, m_current_is_interlaced ? "interlaced" : "progressive",
				frame_data.width, frame_data.height, frame_data.is_interlaced ? "interlaced" : "progressive");
			CmdSwitchresDynamic(frame_data.width, frame_data.height, frame_data.is_interlaced);
			m_current_width = frame_data.width;
			m_current_height = frame_data.height;
			m_current_is_interlaced = frame_data.is_interlaced;
		}

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