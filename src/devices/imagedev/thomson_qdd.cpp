// license:BSD-3-Clause
// copyright-holders:Fabrice Bellet
/*********************************************************************

    thomson_qdd.cpp

    MAME interface to the Thomson Quick Disk Drive image abstraction code

*********************************************************************/

#include "emu.h"
#include "thomson_qdd.h"

#define LOG_IMG		(1U << 1) // Show image format handling
#define LOG_HW		(1U << 2) // Show hardware pins values
#define LOG_READ	(1U << 3) // Show byte read operations
#define LOG_WRITE	(1U << 4) // Show byte write operations

// #define VERBOSE (LOG_IMG|LOG_HW)

#include "logmacro.h"

#define LOGIMG(...)	LOGMASKED(LOG_IMG, __VA_ARGS__)
#define LOGHW(...)	LOGMASKED(LOG_HW, __VA_ARGS__)
#define LOGREAD(...)	LOGMASKED(LOG_READ, __VA_ARGS__)
#define LOGWRITE(...)	LOGMASKED(LOG_WRITE, __VA_ARGS__)

/***************************************************************************
    CONSTANTS
***************************************************************************/

#define QDD_SECTOR_COUNT	400
#define QDD_SECTOR_LENGTH	128
#define QDD_IMAGE_LENGTH	(QDD_SECTOR_COUNT * QDD_SECTOR_LENGTH)

#define QDD_BITRATE		101265

#define QDD_TRACK_LEN		((8000 * QDD_BITRATE)/8000)
#define QDD_HEAD_READ_SW_POS	((500 * QDD_BITRATE)/8000)
#define QDD_DATA_RDY_POS	(QDD_HEAD_READ_SW_POS + ((160 * QDD_BITRATE)/8000))
// QDD BIOS from CQ90-028 controler waits for 160ms + 220ms
// to skip the initial 2796 sync codes duration when formatting, else
// waits for 160ms + 110ms to start reading in the middle of the
// preamble
//
// QDD BIOS from THMFC1 controler waits for 160ms + 110ms before
// rewritting the full length preamble. This is probably a bug, since
// we loose 110ms compared to CQ90-028
#define QDD_MOTOR_STOP_SW_POS	(QDD_HEAD_READ_SW_POS + ((5600 * QDD_BITRATE)/8000))
// This is a rough estimate that should fit the content, even in the
// worst case scenario (THMFC1 formatting)

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
	m_track_buffer = std::make_unique<uint8_t[]>(QDD_TRACK_LEN);

	// allocate timers
	m_byte_timer = timer_alloc(FUNC(thomson_qdd_image_device::byte_timer), this);

	save_item(NAME(m_byte_offset));
	save_item(NAME(m_motor_cmd));

	save_item(NAME(m_disk_present));
	save_item(NAME(m_motor_on));
	save_item(NAME(m_ready));
	save_item(NAME(m_write_gate));
	save_item(NAME(m_write_protected));
}

void thomson_qdd_image_device::device_reset()
{
	m_motor_on = 0;
	m_motor_cmd = 0;
	m_ready = 0;
	m_write_gate = 0;
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

	for(t = 0; t < 24; t++)
		for(s = 0; s < 16; s++)
			thomson_qdd_map[t*16 + s] = p[t/4][s%4]*16 + (s/4) + 4*(t%4);
	for(s = 0; s < 16; s++)
		thomson_qdd_map[24*16 + s] = q[s%4] + (s/4);
}

std::pair<std::error_condition, std::string> thomson_qdd_image_device::call_load()
{
	if(length() != QDD_IMAGE_LENGTH)
		return std::make_pair(image_error::INVALIDLENGTH, std::string());

#define SYNC_CODE		0x16
#define SECTOR_HEADER_ID	0xa5
#define SECTOR_DATA_ID		0x5a

	uint8_t *dst = &m_track_buffer[0];
	uint8_t *org = dst;
	dst += QDD_DATA_RDY_POS;
	memset(dst, SYNC_CODE, 2796); dst += 2796;
	thomson_qdd_compute_map();

	for(int i = 1; i <= QDD_SECTOR_COUNT; i++) {
		LOGIMG("load: sector %d header [ %d .. %d ] data [ %d .. %d ]\n", i,
				dst - org, dst - org + 3,		// header
				dst - org + 14, dst - org + 14 + 129);	// data
		dst[0] = SECTOR_HEADER_ID;
		dst[1] = i >> 8;
		dst[2] = i & 0xff;
		dst[3] = dst[0] + dst[1] + dst[2];
		dst += 4;
		memset(dst, SYNC_CODE, 10); dst += 10;
		dst[0] = SECTOR_DATA_ID;
		fseek(QDD_SECTOR_LENGTH * thomson_qdd_map[i - 1], SEEK_SET);
		fread(&dst[1], QDD_SECTOR_LENGTH);
		uint8_t crc = 0;
		for(int j = 0; j < QDD_SECTOR_LENGTH + 1; j++)
			crc += dst[j];
		dst[QDD_SECTOR_LENGTH + 1] = crc; dst += QDD_SECTOR_LENGTH + 2;
		memset(dst, SYNC_CODE, 17); dst += 17;
	}
	memset(dst, SYNC_CODE, QDD_TRACK_LEN - (int)(dst - org));
	LOGIMG("load: ready range [ %d .. %d ]\n", QDD_HEAD_READ_SW_POS, QDD_MOTOR_STOP_SW_POS);

	m_disk_present = 1;
	m_write_protected = (is_readonly() ? 1 : 0);
	m_byte_offset = 0;
	LOGHW("%s [%d/%d] disk present\n", machine().time().to_string(), m_byte_offset, QDD_TRACK_LEN);
	LOGHW("%s [%d/%d] write protect is %s\n", machine().time().to_string(),
			m_byte_offset, QDD_TRACK_LEN, is_readonly() ? "on" : "off");

	return std::make_pair(std::error_condition(), std::string());
}

static void seek_sync_code(const uint8_t *src, int &pos, uint8_t val, bool &eos)
{
	while (pos < QDD_TRACK_LEN && src[pos] != val) pos++;
	eos = (pos == QDD_TRACK_LEN);
}

static uint8_t get_next_byte(const uint8_t *src, int &pos, bool &eos)
{
	uint8_t val = 0;
	if (pos < QDD_TRACK_LEN)
		val = src[pos++];
	eos = (pos == QDD_TRACK_LEN);
	return val;
}

void thomson_qdd_image_device::call_unload()
{
	uint8_t *src = &m_track_buffer[0];
	int pos = QDD_DATA_RDY_POS;

	for(int i = 1; i <= QDD_SECTOR_COUNT; i++) {
		uint8_t crc = 0;
		bool eos = false;
		int p0, p1;

		seek_sync_code(src, pos, SYNC_CODE, eos);
		seek_sync_code(src, pos, SECTOR_HEADER_ID, eos);

		if (eos) {
			LOGIMG("unload: header id not found for sector %d at pos %d\n", i, pos);
			break;
		}

		p0 = pos;
		crc = get_next_byte(src, pos, eos);
		uint8_t b1 = get_next_byte(src, pos, eos);
		uint8_t b2 = get_next_byte(src, pos, eos);
		uint8_t b3 = get_next_byte(src, pos, eos);
		uint16_t sect = (b1 << 8 | b2);
		crc += b1 + b2;

		if(eos)
			break;

		if(sect != i) {
			LOGIMG("unload: invalid header id %d (should be %d) at pos %d\n", sect, i, pos);
			break;
		}

		if(crc != b3) {
			LOGIMG("unload: invalid header crc 0x%02x (should be 0x%02x) for sector %d at pos %d\n", b3, crc, i, pos);
			break;
		}

		if (get_next_byte(src, pos, eos) != SYNC_CODE)
			break;
		if (get_next_byte(src, pos, eos) != SYNC_CODE)
			break;
		if (get_next_byte(src, pos, eos) != SYNC_CODE)
			break;
		seek_sync_code(src, pos, SECTOR_DATA_ID, eos);

		if (eos) {
			LOGIMG("unload: data id not found for sector %d at pos %d\n", i, pos);
			break;
		}

		p1 = pos;
		crc = get_next_byte(src, pos, eos);
		for(int j = 0; j < QDD_SECTOR_LENGTH; j++)
			crc += get_next_byte(src, pos, eos);
		b1 = get_next_byte(src, pos, eos);

		if (eos)
			break;

		fseek(QDD_SECTOR_LENGTH*thomson_qdd_map[i - 1], SEEK_SET);
		fwrite(&src[p1 + 1], QDD_SECTOR_LENGTH);
		if(b1 != crc) {
			LOGIMG("unload: invalid data crc 0x%02x (should be 0x%02x) for sector %d at pos %d\n", b1, crc, i, pos);
			break;
		}

		// The CQ90-028 QDD Bios doesnt write any sync code
		// after last sector data, so we do not enforce it
		//
		//if(get_next_byte(src, pos, eos) != SYNC_CODE)
		//	break;

		LOGIMG("unload: sector %d header [ %d .. %d ] data [ %d .. %d ]\n", i,
				p0, p0 + 3,	// header
				p1, p1 + 129);	// data
	}
	LOGIMG("unload: ready range [ %d .. %d ]\n", QDD_HEAD_READ_SW_POS, QDD_MOTOR_STOP_SW_POS);

	if(m_track_buffer.get())
		memset(m_track_buffer.get(), 0, QDD_TRACK_LEN);
	motor_on_w(0);
	m_disk_present = 0;
	LOGHW("%s [%d/%d] media unset\n", machine().time().to_string(), m_byte_offset, QDD_TRACK_LEN);
}

void thomson_qdd_image_device::write(uint8_t data)
{
	static int prev = -1;
	if(m_motor_cmd && m_disk_present && m_ready && m_write_gate && !m_write_protected) {
		LOGWRITE("%s [%d/%d] write 0x%02x replace=0x%02x to the QDD\n", machine().time().to_string(),
				m_byte_offset, QDD_TRACK_LEN, data, m_track_buffer[m_byte_offset]);
		m_track_buffer[m_byte_offset] = data;
		if (m_byte_offset != prev + 1)
			LOGREAD("%s WARNING gap between two writes %d %d\n", machine().time().to_string(),
				prev, m_byte_offset);
	}
	prev = m_byte_offset;
}

uint8_t thomson_qdd_image_device::read()
{
	uint8_t val = 0;
	static int prev = -1;

	if(m_motor_cmd && m_disk_present && m_ready && m_byte_offset > QDD_DATA_RDY_POS) {
		LOGREAD("%s [%d/%d] read 0x%02x from the QDD\n", machine().time().to_string(),
				m_byte_offset, QDD_TRACK_LEN, m_track_buffer[m_byte_offset]);
		val = m_track_buffer[m_byte_offset];
		if (m_byte_offset != prev + 1)
			LOGREAD("%s WARNING gap between two reads %d %d\n", machine().time().to_string(),
				prev, m_byte_offset);
	}
	prev = m_byte_offset;
	return val;
}

void thomson_qdd_image_device::motor_on_w(int state)
{
	if(m_motor_on == state)
		return;

	m_motor_on = state;

	LOGHW("%s [%d/%d] motor_on set %s\n", machine().time().to_string(),
			m_byte_offset, QDD_TRACK_LEN, m_motor_on ? "on":"off");
	if(m_motor_on)
		m_byte_timer->adjust(attotime::zero, 0, attotime::from_hz(QDD_BITRATE / 8));
	if (m_motor_on && !m_motor_cmd) {
		m_motor_cmd = 1;
		LOGHW("%s [%d/%d] motor_cmd set on\n", machine().time().to_string(),
					m_byte_offset, QDD_TRACK_LEN);
	}
}

TIMER_CALLBACK_MEMBER(thomson_qdd_image_device::byte_timer)
{
	int prev = m_ready;

	m_ready = 0;
	if(m_byte_offset > QDD_HEAD_READ_SW_POS && m_byte_offset < QDD_MOTOR_STOP_SW_POS)
		m_ready = 1;
	if(m_ready != prev)
		LOGHW("%s [%d/%d] ready %s\n", machine().time().to_string(),
				m_byte_offset, QDD_TRACK_LEN, m_ready ? "set" : "unset");
	if(m_byte_offset >= QDD_MOTOR_STOP_SW_POS && m_motor_cmd == 1 && (m_motor_on == 0 || m_write_gate == 1)) {
		m_motor_cmd = 0;
		LOGHW("%s [%d/%d] motor_cmd set off\n", machine().time().to_string(), m_byte_offset, QDD_TRACK_LEN);
	}
	m_byte_offset++;
	if(m_byte_offset == QDD_TRACK_LEN) {
		m_byte_offset = 0;
		LOGHW("%s [%d/%d] end of track\n", machine().time().to_string(), m_byte_offset, QDD_TRACK_LEN);
		if (m_motor_cmd == 0)
			m_byte_timer->adjust(attotime::never);
	}
}
