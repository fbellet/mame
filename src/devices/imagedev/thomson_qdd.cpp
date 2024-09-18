// license:BSD-3-Clause
// copyright-holders:Fabrice Bellet
/*********************************************************************

    thomson_qdd.cpp

    MAME interface to the Thomson Quick Disk Drive image abstraction code

*********************************************************************/

#include "emu.h"
#include "thomson_qdd.h"

/***************************************************************************
    CONSTANTS
***************************************************************************/

#define LOG 0

#define QDD_SECTOR_COUNT            400
#define QDD_SECTOR_LENGTH           128
#define QDD_IMAGE_LENGTH            (QDD_SECTOR_COUNT * QDD_SECTOR_LENGTH)
#define QDD_TRACK_LENGTH	    101317

#define QDD_BITRATE                 101265

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// device type definition
DEFINE_DEVICE_TYPE(THOMSON_QDD, thomson_qdd_image_device, "qdd_image", "Thomson Quick Disk Drive")

//-------------------------------------------------
//  thomson_qdd_image_device - constructor
//-------------------------------------------------

thomson_qdd_image_device::thomson_qdd_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	microtape_image_device(mconfig, THOMSON_QDD, tag, owner, clock)
{
}

//-------------------------------------------------
//  thomson_qdd_image_device - destructor
//-------------------------------------------------

thomson_qdd_image_device::~thomson_qdd_image_device()
{
}

void thomson_qdd_image_device::device_start()
{
	m_track_buffer = std::make_unique<uint8_t[]>(QDD_TRACK_LENGTH);

	// allocate timers
	m_bit_timer = timer_alloc(FUNC(thomson_qdd_image_device::bit_timer), this);
	m_bit_timer->adjust(attotime::zero, 0, attotime::from_hz(QDD_BITRATE));
	m_bit_timer->enable(0);
	m_disk_present = false;
	m_index = false;
	m_ssda = nullptr;
}

std::pair<std::error_condition, std::string> thomson_qdd_image_device::call_load()
{
	if (length() != QDD_IMAGE_LENGTH)
		return std::make_pair(image_error::INVALIDLENGTH, std::string());

	uint8_t *dst = &m_track_buffer[0];
	uint8_t *org = dst;
	memset(dst, 0x16, 2796); dst += 2796;
	for (int i = 1; i <= 400; i++)
	{
		dst[0] = 0xa5;
		dst[1] = i >> 8;
		dst[2] = i & 0xff;
		dst[3] = dst[0] + dst[1] + dst[2];
		dst += 4;
		memset(dst, 0x16, 10); dst += 10;
		dst[0] = 0x5a;
		fread(&dst[1], 128);
		uint8_t crc = 0;
		for (int j = 0; j < 129; j++)
		    crc += dst[j];
		dst[129] = crc; dst += 130;
		memset(dst, 0x16, 17); dst += 17;
	}
	memset(dst, 0x16, QDD_TRACK_LENGTH - (int)(dst - org));

	m_bit_offset = 0;
	m_byte_offset = 0;
	m_disk_present = true;
	m_bit_timer->enable(1);

	return std::make_pair(std::error_condition(), std::string());
}

void thomson_qdd_image_device::call_unload()
{
	if (m_track_buffer.get())
		memset(m_track_buffer.get(), 0, QDD_TRACK_LENGTH);
	m_disk_present = false;
}

TIMER_CALLBACK_MEMBER(thomson_qdd_image_device::bit_timer)
{
	m_bit_offset++;

	if (m_bit_offset == 8)
	{
		m_bit_offset = 0;
		m_index = (m_byte_offset < 13); // 1ms
		if (m_ssda) {
			logerror("Sent 0x%02x [%d/%d] to the SSDA\n",
					m_track_buffer[m_byte_offset],
					m_byte_offset, QDD_TRACK_LENGTH);
			m_ssda->receive_byte(m_track_buffer[m_byte_offset]);
		}
		m_byte_offset++;

		if (m_byte_offset == QDD_TRACK_LENGTH)
		{
			m_byte_offset = 0;
			logerror("Track completed, restarting\n");
		}
	}
}
