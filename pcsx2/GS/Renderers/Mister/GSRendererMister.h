/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2021 PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "GS/Renderers/Common/GSRenderer.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET socket_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int socket_t;
#define INVALID_SOCKET -1
#define closesocket close
#endif

#include <chrono>
#include <vector>
#include "lz4/lz4.h"
#include "lz4/lz4hc.h"

// Groovy_Mister protocol definitions
#define CMD_CLOSE 1
#define CMD_INIT 2
#define CMD_SWITCHRES 3
#define CMD_BLIT_VSYNC 6

#define VRAM_READY       (1 << 0)
#define VRAM_END_FRAME   (1 << 1)
#define VRAM_SYNCED      (1 << 2)
#define VRAM_FRAMESKIP   (1 << 3)
#define VGA_VBLANK       (1 << 4)
#define VGA_FIELD        (1 << 5)
#define VGA_QUEUE        (1 << 7)

struct nogpu_modeline {
	double pclock;
	int hactive;
	int hbegin;
	int hend;
	int htotal;
	int vactive;
	int vbegin;
	int vend;
	int vtotal;
	int interlace;
};

struct cmd_init {
	uint8_t cmd;
	uint8_t index;
	uint16_t width;
	uint16_t height;
	nogpu_modeline mode;
};

struct cmd_blit_vsync {
	uint8_t cmd;
	uint32_t frame;
	uint16_t vsync;
	uint32_t compressed_data_size;
};

struct cmd_close {
	uint8_t cmd;
	uint8_t index;
};

struct cmd_switchres {
	uint8_t cmd;
	uint8_t index;
	nogpu_modeline mode;
};

struct status_flags {
	union {
		uint8_t bits;
		struct {
			uint8_t vram_ready     : 1;
			uint8_t vram_end_frame : 1;
			uint8_t vram_synced    : 1;
			uint8_t vram_frameskip : 1;
			uint8_t vga_vblank     : 1;
			uint8_t vga_field      : 1;
			uint8_t fpga_audio     : 1;
			uint8_t vga_queue      : 1;
		};
	};
};

class GSRendererMister final : public GSRenderer
{
public:
	GSRendererMister();
	~GSRendererMister();

	bool Initialize() override;
	void Destroy() override;
	void VSync(u32 field, bool registers_written) override;

protected:
	void Draw() override;
	GSTexture* GetOutput(int i, int& y_offset) override;

private:
	// Network communication
	bool InitializeNetwork();
	void ShutdownNetwork();
	bool SendCommand(const void* data, size_t size);
	bool ReceiveStatus(status_flags& status);
	void SendFrameToMister(GSTexture* frame);
	void SendModeline();
	
	// Frame processing
	void ConvertRGBAToRGB(const uint8_t* rgba_data, uint8_t* rgb_data, int width, int height);
	void SendFrameData(const uint8_t* rgb_data, int width, int height);
	
	// Configuration
	std::string GetMisterIP() const;
	
	// Network state
	socket_t m_socket;
	struct sockaddr_in m_server_addr;
	bool m_network_initialized;
	bool m_mister_initialized;
	
	// Frame state
	uint32_t m_frame_counter;
	int m_current_width;
	int m_current_height;
	std::vector<uint8_t> m_rgb_buffer;
	std::vector<uint8_t> m_compressed_buffer;
	
	// Timing
	std::chrono::high_resolution_clock::time_point m_last_frame_time;
	
	// Constants
	static constexpr int MISTER_PORT = 32100;
	static constexpr int MTU_SIZE = 1472;
	static constexpr int MAX_FRAME_WIDTH = 768;
	static constexpr int MAX_FRAME_HEIGHT = 576;
};