// license:BSD-3-Clause
// copyright-holders:Fabrice Bellet
/*********************************************************************

    thomson_qdd.cpp

    MAME interface to the Thomson Quick Disk Drive image abstraction code

*********************************************************************/

#include "emu.h"
#include "thomson_qdd.h"

#define VERBOSE 0
#include "logmacro.h"

/***************************************************************************
    CONSTANTS
***************************************************************************/

#define QDD_SECTOR_COUNT	400
#define QDD_SECTOR_LENGTH	128
#define QDD_IMAGE_LENGTH	(QDD_SECTOR_COUNT * QDD_SECTOR_LENGTH)
#define QDD_TRACK_LENGTH	101317

#define QDD_BITRATE		101265

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
	m_write_enable = false;
	m_ssda = nullptr;
}

/* fixed interlacing map for QDDs */
static int thomson_qdd_map[QDD_SECTOR_COUNT];

static void thomson_qdd_compute_map ( void )
{
	/* this map is hardcoded in the QDD BIOS */
	static const int p[6][4] = {
		{20,  2, 14, 8}, {21, 19, 13, 7},
		{22, 18, 12, 6}, {23, 17, 11, 5},
		{24, 16, 10, 4}, { 1, 15,  9, 3}};
	static const int q[4] = {0, 8, 4, 12};
	int t, s;

	for (t = 0; t < 24; t++)
		for (s = 0; s < 16; s++)
			thomson_qdd_map[t*16 + s] = p[t/4][s%4]*16 + (s/4) + 4*(t%4);
	for (s = 0; s < 16; s++)
		thomson_qdd_map[24*16 + s] = q[s%4] + (s/4);
}

std::pair<std::error_condition, std::string> thomson_qdd_image_device::call_load()
{
	if (length() != QDD_IMAGE_LENGTH)
		return std::make_pair(image_error::INVALIDLENGTH, std::string());

	uint8_t *dst = &m_track_buffer[0];
	uint8_t *org = dst;
	memset(dst, 0x16, 2796); dst += 2796;
	thomson_qdd_compute_map();

	for (int i = 1; i <= QDD_SECTOR_COUNT; i++)
	{
		dst[0] = 0xa5;
		dst[1] = i >> 8;
		dst[2] = i & 0xff;
		dst[3] = dst[0] + dst[1] + dst[2];
		dst += 4;
		memset(dst, 0x16, 10); dst += 10;
		dst[0] = 0x5a;
		fseek(QDD_SECTOR_LENGTH * thomson_qdd_map[i - 1], SEEK_SET);
		fread(&dst[1], QDD_SECTOR_LENGTH);
		uint8_t crc = 0;
		for (int j = 0; j < QDD_SECTOR_LENGTH + 1; j++)
		    crc += dst[j];
		dst[QDD_SECTOR_LENGTH + 1] = crc; dst += QDD_SECTOR_LENGTH + 2;
		memset(dst, 0x16, 17); dst += 17;
	}
	memset(dst, 0x16, QDD_TRACK_LENGTH - (int)(dst - org));

	m_bit_offset = 0;
	m_byte_offset = 0;
	m_disk_present = true;
	m_bit_timer->enable(1);
	if (m_ssda) {
		m_ssda->cts_w(1);	// TODO: set it to the
					// write-protect status of the
					// image
	}

	return std::make_pair(std::error_condition(), std::string());
}

void thomson_qdd_image_device::call_unload()
{
	uint8_t *src = &m_track_buffer[0];
	uint8_t *org = src;

	while (src[0] == 0x16) src++;
	LOG("Unload: skipped %d synchro bytes before sector 1\n", src - org);
	for (int i = 1; i <= QDD_SECTOR_COUNT; i++) {
		uint8_t *syn = src;
		uint8_t crc = 0;

		if (src[0] != 0xa5) {
			LOG("Unload: header synchro byte 0xa5 not found for sector %d at pos %d\n", i, src - org);
			break;
		}
		if ((src[1] << 8 | src[2]) != i) {
			LOG("Unload: invalid sector id %d (should be %d) at pos %d\n",
					src[1] << 8 | src[2], i, src - org + 1);
			break;
		}
		crc = src[0] + src[1] + src[2];
		if (crc != src[3]) {
			LOG("Unload: invalid header crc 0x%02x (should be 0x%02x) for sector %d at pos %d\n",
					src[3], crc, i, src - org + 3);
			break;
		}
		src += 4;
		while (src[0] == 0x16) src++;
		LOG("Unload: skipped %d synchro bytes between header and data of sector %d\n", src - syn, i);
		if (src[0] != 0x5a) {
			LOG("Unload: data synchro byte 0x5a not found for sector %d at pos %d\n", i, src - org);
			break;
		}
		fseek(QDD_SECTOR_LENGTH*thomson_qdd_map[i - 1], SEEK_SET);
		fwrite(&src[1], QDD_SECTOR_LENGTH);
		crc = 0;
		for (int j = 0; j < QDD_SECTOR_LENGTH + 1; j++)
			crc += src[j];
		if (src[QDD_SECTOR_LENGTH + 1] != crc) {
			LOG("Unload: invalid data crc 0x%02x (should be 0x%02x) for sector %d at pos %d\n",
					src[QDD_SECTOR_LENGTH + 1], crc, i, src - org + QDD_SECTOR_LENGTH + 1);
			break;
		}
		src += QDD_SECTOR_LENGTH + 2;
		syn = src;
		while (src[0] == 0x16) src++;
		LOG("Unload: skipped %d synchro bytes after sector %d\n", src - syn, i);
	}

	if (m_track_buffer.get())
		memset(m_track_buffer.get(), 0, QDD_TRACK_LENGTH);
	m_disk_present = false;
}

TIMER_CALLBACK_MEMBER(thomson_qdd_image_device::bit_timer)
{
	m_bit_offset++;

	if ((m_bit_offset == 1) && m_write_enable) {
		int tuf;
		uint8_t data = m_ssda->get_tx_byte(&tuf);
		m_track_buffer[m_byte_offset] = data;
		LOG("Sent 0x%02x replace=0x%02x tuf=%d [%d/%d] to the QDD\n",
				data, m_track_buffer[m_byte_offset], tuf,
				m_byte_offset, QDD_TRACK_LENGTH);
	}
	if (m_bit_offset == 8)
	{
		m_bit_offset = 0;
		m_index = (m_byte_offset < 13); // 1ms
		if (m_ssda) {
			LOG("Sent 0x%02x [%d/%d] to the SSDA\n",
					m_track_buffer[m_byte_offset],
					m_byte_offset, QDD_TRACK_LENGTH);
			m_ssda->receive_byte(m_track_buffer[m_byte_offset]);
		}
		m_byte_offset++;

		if (m_byte_offset == QDD_TRACK_LENGTH)
		{
			m_byte_offset = 0;
			LOG("Track completed, restarting\n");
		}
	}
}
