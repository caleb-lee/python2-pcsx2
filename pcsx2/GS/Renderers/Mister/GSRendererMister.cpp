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

#include "PrecompiledHeader.h"
#include "GSRendererMister.h"
#include "Host.h"
#include "../Common/GSDevice.h"
#include <cstring>

#include "lz4/lz4.h"
#include "lz4/lz4hc.h"

#ifdef _WIN32
typedef int ssize_t;
#endif

GSRendererMister::GSRendererMister()
	: m_socket(INVALID_SOCKET)
	, m_network_initialized(false)
	, m_mister_initialized(false)
	, m_frame_counter(0)
	, m_current_width(0)
	, m_current_height(0)
{
	if (!InitializeNetwork())
	{
		Console.Error("GSRendererMister: Failed to initialize network");
	}
	else
	{
		Console.WriteLn("GSRendererMister: Successfully initialized");
	}
}

GSRendererMister::~GSRendererMister()
{
	Destroy();
}

void GSRendererMister::Destroy()
{
	ShutdownNetwork();
	GSRenderer::Destroy();
}

bool GSRendererMister::InitializeNetwork()
{
#ifdef _WIN32
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		Console.Error("GSRendererMister: WSAStartup failed");
		return false;
	}
#endif

	m_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (m_socket == INVALID_SOCKET)
	{
		Console.Error("GSRendererMister: Failed to create socket");
#ifdef _WIN32
		WSACleanup();
#endif
		return false;
	}

	// LATENCY OPTIMIZATION: Set socket options for minimal latency
#ifdef _WIN32
	// Set socket to non-blocking for immediate sends
	u_long mode = 1;
	ioctlsocket(m_socket, FIONBIO, &mode);
	
	// Disable Nagle's algorithm for immediate packet transmission
	BOOL nodelay = TRUE;
	setsockopt(m_socket, IPPROTO_TCP, TCP_NODELAY, (char*)&nodelay, sizeof(nodelay));
	
#else
	// Set socket to non-blocking
	int flags = fcntl(m_socket, F_GETFL, 0);
	fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);
	
	// Disable Nagle's algorithm
	int nodelay = 1;
	setsockopt(m_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
	
	// Set high priority for low latency
	int priority = 6;
	setsockopt(m_socket, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));
	
	// Set socket buffer sizes for optimal throughput
	int sndbuf = 65536;
	setsockopt(m_socket, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
#endif

	// Configure server address
	std::memset(&m_server_addr, 0, sizeof(m_server_addr));
	m_server_addr.sin_family = AF_INET;
	m_server_addr.sin_port = htons(MISTER_PORT);
	
	std::string ip = GetMisterIP();
	if (ip.empty() || ip == "0.0.0.0")
	{
		Console.Error("GSRendererMister: No MiSTer IP configured. Using default: %s", ip.c_str());
	}
	
	if (inet_pton(AF_INET, ip.c_str(), &m_server_addr.sin_addr) <= 0)
	{
		Console.Error("GSRendererMister: Invalid MiSTer IP address: %s", ip.c_str());
		return false;
	}

	m_network_initialized = true;
	Console.WriteLn("GSRendererMister: Low-latency network initialized, targeting %s:%d", ip.c_str(), MISTER_PORT);
	
	// Send initial connection command
	cmd_init init_cmd = {};
	init_cmd.cmd = CMD_INIT;
	init_cmd.index = 0;
	init_cmd.width = 640;  // Default PS2 resolution
	init_cmd.height = 480;
	
	// Set up default modeline (640x480@60Hz)
	init_cmd.mode.pclock = 25.175;
	init_cmd.mode.hactive = 640;
	init_cmd.mode.hbegin = 656;
	init_cmd.mode.hend = 752;
	init_cmd.mode.htotal = 800;
	init_cmd.mode.vactive = 480;
	init_cmd.mode.vbegin = 490;
	init_cmd.mode.vend = 492;
	init_cmd.mode.vtotal = 525;
	init_cmd.mode.interlace = 0;
	
	if (SendCommand(&init_cmd, sizeof(init_cmd)))
	{
		m_mister_initialized = true;
		Console.WriteLn("GSRendererMister: MiSTer connection established");
	}
	
	return true;
}

void GSRendererMister::ShutdownNetwork()
{
	if (m_mister_initialized)
	{
		cmd_close close_cmd = {};
		close_cmd.cmd = CMD_CLOSE;
		close_cmd.index = 0;
		SendCommand(&close_cmd, sizeof(close_cmd));
		m_mister_initialized = false;
	}
	
	if (m_socket != INVALID_SOCKET)
	{
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
	}

#ifdef _WIN32
	if (m_network_initialized)
		WSACleanup();
#endif

	m_network_initialized = false;
}

bool GSRendererMister::SendCommand(const void* data, size_t size)
{
	if (!m_network_initialized || m_socket == INVALID_SOCKET)
		return false;
		
	// LATENCY OPTIMIZATION: Send immediately without buffering
	ssize_t sent = sendto(m_socket, static_cast<const char*>(data), size, 0,
		reinterpret_cast<const sockaddr*>(&m_server_addr), sizeof(m_server_addr));
	
	return sent == static_cast<ssize_t>(size);
}

bool GSRendererMister::ReceiveStatus(status_flags& status)
{
	if (!m_network_initialized || m_socket == INVALID_SOCKET)
		return false;
		
	uint8_t buffer;
	ssize_t received = recvfrom(m_socket, reinterpret_cast<char*>(&buffer), 1, 0, nullptr, nullptr);
	
	if (received == 1)
	{
		status.bits = buffer;
		return true;
	}
	
	return false;
}

void GSRendererMister::VSync(u32 field, bool registers_written)
{
	// Call base VSync to process the frame
	GSRenderer::VSync(field, registers_written);
	
	if (!m_mister_initialized)
		return;
	
	// Get the current rendered frame and send immediately
	GSTexture* current_frame = g_gs_device->GetCurrent();
	if (current_frame)
	{
		SendFrameToMister(current_frame);
	}
	
	m_frame_counter++;
}

void GSRendererMister::SendFrameToMister(GSTexture* frame)
{
	if (!frame || !m_mister_initialized)
		return;
	
	// Get frame dimensions
	int width = frame->GetWidth();
	int height = frame->GetHeight();
	
	// Clamp to maximum supported resolution
	width = std::min(width, MAX_FRAME_WIDTH);
	height = std::min(height, MAX_FRAME_HEIGHT);
	
	// Skip if dimensions are invalid
	if (width <= 0 || height <= 0)
		return;
	
	// Map the texture to get pixel data
	GSTexture::GSMap map;
	if (!frame->Map(map))
		return;
	
	// Resize RGB buffer if needed
	size_t rgb_size = width * height * 3;
	if (m_rgb_buffer.size() < rgb_size)
		m_rgb_buffer.resize(rgb_size);
	
	// Convert RGBA to RGB
	ConvertRGBAToRGB(static_cast<const uint8_t*>(map.bits), m_rgb_buffer.data(), width, height);
	
	// Unmap the texture
	frame->Unmap();
	
	// Send frame data to MiSTer
	SendFrameData(m_rgb_buffer.data(), width, height);
	
	// Update current dimensions
	if (m_current_width != width || m_current_height != height)
	{
		m_current_width = width;
		m_current_height = height;
	}
}

void GSRendererMister::ConvertRGBAToRGB(const uint8_t* rgba_data, uint8_t* rgb_data, int width, int height)
{
	const uint8_t* src = rgba_data;
	uint8_t* dst = rgb_data;
	
	int pixel_count = width * height;
	for (int i = 0; i < pixel_count; i++)
	{
		*dst++ = *src++;  // R
		*dst++ = *src++;  // G
		*dst++ = *src++;  // B
		src++;            // Skip A
	}
}

void GSRendererMister::SendFrameData(const uint8_t* rgb_data, int width, int height)
{
	// Compress frame data using LZ4
	size_t uncompressed_size = width * height * 3;
	size_t max_compressed_size = LZ4_compressBound(uncompressed_size);
	
	// Ensure compressed buffer is large enough
	if (m_compressed_buffer.size() < max_compressed_size)
		m_compressed_buffer.resize(max_compressed_size);
	
	// Compress using LZ4 for speed (critical for 31kHz)
	int compressed_size = LZ4_compress_default(
		reinterpret_cast<const char*>(rgb_data),
		reinterpret_cast<char*>(m_compressed_buffer.data()),
		uncompressed_size,
		max_compressed_size
	);
	
	if (compressed_size <= 0)
		return;
	
	// Send blit command with compressed data size
	cmd_blit_vsync blit_cmd = {};
	blit_cmd.cmd = CMD_BLIT_VSYNC;
	blit_cmd.frame = m_frame_counter;
	blit_cmd.vsync = 0;  // No vsync adjustment for now
	blit_cmd.compressed_data_size = compressed_size;
	
	if (!SendCommand(&blit_cmd, sizeof(blit_cmd)))
		return;
	
	// Send compressed frame data in MTU-sized chunks
	size_t bytes_sent = 0;
	
	while (bytes_sent < static_cast<size_t>(compressed_size))
	{
		size_t chunk_size = std::min(static_cast<size_t>(compressed_size) - bytes_sent, static_cast<size_t>(MTU_SIZE));
		
		ssize_t sent = sendto(m_socket, reinterpret_cast<const char*>(m_compressed_buffer.data() + bytes_sent), 
			chunk_size, 0, reinterpret_cast<const sockaddr*>(&m_server_addr), sizeof(m_server_addr));
		
		if (sent <= 0)
			break;
		
		bytes_sent += sent;
	}
}

std::string GSRendererMister::GetMisterIP() const
{
	//TODO: Make configurable
	return "192.168.5.2";
}

void GSRendererMister::Draw()
{
}

GSTexture* GSRendererMister::GetOutput(int i, int& y_offset)
{
	return nullptr;
}