// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, Fabrice Bellet

// Custom Thomson 3.5"/5.25"/QDD diskette controller gate array used in CD 90-351, TO8 and TO9+


// Documentation: http://dcmoto.free.fr/documentation/docto8/index.html
//
// - Emulation of FREE stat0 bit should be tested with real hardware:
//   FREE is not set when the controller received a RESET cmd after an
//   interrupted RSECT cmd (tested with analpiste_to8).
// - FM coding is implemented, but unused in real hardware.
// - RHEAD cmd is implemented, tested in mame, but unused in real
//   hardware

#include "emu.h"
#include "thmfc1.h"
#include "imagedev/thomson_qdd.h"
#include "formats/flopimg.h"

#define LOG_STATE	(1U << 1) // Show state machine
#define LOG_SHIFT	(1U << 2) // Show shift register contents
#define LOG_REGS	(1U << 3) // Show register access
#define LOG_COMMAND	(1U << 4) // Show command invocation
#define LOG_QDD		(1U << 5) // Show QDD specific timings

// #define VERBOSE (LOG_COMMAND)

#include "logmacro.h"

#define LOGSTATE(...)	LOGMASKED(LOG_STATE, __VA_ARGS__)
#define LOGSHIFT(...)	LOGMASKED(LOG_SHIFT, __VA_ARGS__)
#define LOGREGS(...)	LOGMASKED(LOG_REGS, __VA_ARGS__)
#define LOGCOMMAND(...)	LOGMASKED(LOG_COMMAND, __VA_ARGS__)
#define LOGQDD(...)	LOGMASKED(LOG_QDD, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

DEFINE_DEVICE_TYPE(THMFC1_CONNECTOR, thmfc1_connector, "thmfc1_connector", "Connector abstraction for floppy or quick disk drive")

DEFINE_DEVICE_TYPE(THMFC1, thmfc1_device, "thmfc1", "SGS-Thomson THM-FC-1 Diskette Controller") // SGS logo used on silkscreen

thmfc1_device::thmfc1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, THMFC1, tag, owner, clock),
	m_drive(*this, "%u", 0U)
{
}

void thmfc1_device::map(address_map &map)
{
	map(0, 0).rw(FUNC(thmfc1_device::stat0_r), FUNC(thmfc1_device::cmd0_w));
	map(1, 1).rw(FUNC(thmfc1_device::stat1_r), FUNC(thmfc1_device::cmd1_w));
	map(2, 2).w(FUNC(thmfc1_device::cmd2_w));
	map(3, 3).rw(FUNC(thmfc1_device::rdata_r), FUNC(thmfc1_device::wdata_w));
	map(4, 4).w(FUNC(thmfc1_device::wclk_w));
	map(5, 5).w(FUNC(thmfc1_device::wsect_w));
	map(6, 6).w(FUNC(thmfc1_device::wtrck_w));
	map(7, 7).w(FUNC(thmfc1_device::wcell_w));
}

void thmfc1_device::device_start()
{
	m_motor_timer[0] = timer_alloc(FUNC(thmfc1_device::motor_off), this);
	m_motor_timer[1] = timer_alloc(FUNC(thmfc1_device::motor_off), this);

	save_item(NAME(m_cmd0));
	save_item(NAME(m_cmd1));
	save_item(NAME(m_cmd2));
	save_item(NAME(m_stat0));
	save_item(NAME(m_rdata));
	save_item(NAME(m_wdata));
	save_item(NAME(m_clk));
	save_item(NAME(m_sect));
	save_item(NAME(m_trck));
	save_item(NAME(m_cell));
	save_item(NAME(m_last_sync));
	save_item(NAME(m_window_start));
	save_item(NAME(m_shift_reg));
	save_item(NAME(m_shift_data_reg));
	save_item(NAME(m_shift_clk_reg));
	save_item(NAME(m_use_shift_clk_reg));
	save_item(NAME(m_crc));
	save_item(NAME(m_bit));
	save_item(NAME(m_bit_counter));
	save_item(NAME(m_byte_counter));
	save_item(NAME(m_write_buffer));
	save_item(NAME(m_write_buffer_idx));
	save_item(NAME(m_write_buffer_start));
}

void thmfc1_device::device_reset()
{
	m_cmd0 = 0;
	m_cmd1 = 0;
	m_cmd2 = 0;
	m_stat0 = S0_FREE;
	m_rdata = 0;
	m_wdata = 0;
	m_clk = 0;
	m_sect = 0;
	m_trck = 0;
	m_cell = 0;
	m_last_sync = 0;
	m_window_start = 0;
	m_shift_reg = 0;
	m_shift_data_reg = 0;
	m_shift_clk_reg = 0;
	m_use_shift_clk_reg = false;
	m_crc = 0;
	m_bit = 0;
	m_bit_counter = 0;
	m_byte_counter = 0;
	m_write_buffer_idx = 0;
	m_write_buffer_start = 0;
	m_state = S_IDLE;
	m_cur_floppy = nullptr;
	m_cur_qdd = nullptr;
}

TIMER_CALLBACK_MEMBER(thmfc1_device::motor_off)
{
	LOGREGS("motor off\n");
	floppy_image_device *floppy = dynamic_cast<floppy_image_device *>(m_drive[param]->get_device());
	if(floppy)
		floppy->mon_w(1);
}

void thmfc1_device::device_post_load()
{
	if(m_cmd2 & C2_DRS0) {
		m_cur_floppy = dynamic_cast<floppy_image_device *>(m_drive[0]->get_device());
		m_cur_qdd = dynamic_cast<thomson_qdd_image_device *>(m_drive[0]->get_device());
	}
	else if(m_cmd2 & C2_DRS1) {
		m_cur_floppy = dynamic_cast<floppy_image_device *>(m_drive[1]->get_device());
		m_cur_qdd = dynamic_cast<thomson_qdd_image_device *>(m_drive[1]->get_device());
	}
	else {
		m_cur_floppy = nullptr;
		m_cur_qdd = nullptr;
	}
}

void thmfc1_device::cmd0_w(u8 data)
{
	sync();

	static const char *const mode[4] = { "reset", "wsect", "rhead", "rsect" };
	m_cmd0 = data;
	LOGREGS("cmd0_w %02x, code=%s, ensyn=%d nomck=%d wgc=%d mode=%s\n", m_cmd0,
			 m_cmd0 & C0_FM ? "fm" : "mfm",
			 m_cmd0 & C0_ENSYN ? 1 : 0,
			 m_cmd0 & C0_NOMCK ? 1 : 0,
			 m_cmd0 & C0_WGC ? 1 : 0,
			 mode[m_cmd0 & 3]);

	switch(m_cmd0 & 3) {
	case 0:
		if((m_cell & 0x7f) == 0)
			m_state = S_IDLE;
		else if(m_cmd0 & C0_WGC) {
			LOGCOMMAND("command format h=%d t=%d\n",
					 m_cmd1 & C1_SIDE ? 1 : 0,
					 m_trck);
			m_state = S_FORMAT;
			m_bit_counter = 0;
			m_byte_counter = 0;
			m_window_start = m_last_sync;
			LOGSTATE("s_format\n");
		} else {
			flush_flux();
			m_state = S_READ_WAIT_HEADER_SYNC;
			m_window_start = m_last_sync;
			LOGSTATE("s_read_wait_header_sync\n");
		}
		break;
	default:
		LOGCOMMAND("command %s h=%d t=%d s=%d sz=%d\n",
				 mode[m_cmd0 & 3],
				 m_cmd1 & C1_SIDE ? 1 : 0,
				 m_trck,
				 m_sect,
				 128 << ((m_cmd1 >> 5) & 3));
		m_state = S_READ_WAIT_HEADER_SYNC;
		m_bit_counter = 0;
		m_stat0 &= ~S0_FREE;
		m_window_start = m_last_sync;
		LOGSTATE("s_read_wait_header_sync\n");
		break;
	}
	if(m_cur_qdd)
		m_cur_qdd->wg_w(m_cmd0 & C0_WGC);
}

void thmfc1_device::cmd1_w(u8 data)
{
	sync();

	m_cmd1 = data;
	LOGREGS("cmd1_w %02x, sector=(size=%d, side=%d) precomp=%d sync_only_when_ready=%s\n", m_cmd1,
			 128 << ((m_cmd1 >> 5) & 3),
			 m_cmd1 & C1_SIDE ? 1 : 0,
			 (m_cmd1 >> 1) & 7,
			 m_cmd1 & C1_DSYRD ? "on" : "off");
}

void thmfc1_device::cmd2_w(u8 data)
{
	sync();

	u8 prev = m_cmd2;

	m_cmd2 = data;
	LOGREGS("cmd2_w %02x, side=%d dir=%d step=%d motor=%s sel=%c%c\n", m_cmd2,
			 m_cmd2 & C2_SISELB ? 1 : 0,
			 m_cmd2 & C2_DIRECB ? 1 : 0,
			 m_cmd2 & C2_STEP ? 1 : 0,
			 m_cmd2 & C2_MTON ? "on" : "off",
			 m_cmd2 & C2_DRS1 ? 'b' : '-',
			 m_cmd2 & C2_DRS0 ? 'a' : '-');

	if(m_cmd2 & C2_DRS0) {
		m_cur_floppy = dynamic_cast<floppy_image_device *>(m_drive[0]->get_device());
		m_cur_qdd = dynamic_cast<thomson_qdd_image_device *>(m_drive[0]->get_device());
	} else if(m_cmd2 & C2_DRS1) {
		m_cur_floppy = dynamic_cast<floppy_image_device *>(m_drive[1]->get_device());
		m_cur_qdd = dynamic_cast<thomson_qdd_image_device *>(m_drive[1]->get_device());
	}
	else {
		m_cur_floppy = nullptr;
		m_cur_qdd = nullptr;
	}

	if(m_cur_qdd)
		m_cur_qdd->mo_w(m_cmd2 & C2_SISELB ? 0 : 1);
	else if(m_cur_floppy) {
		int i = (m_cmd2 & C2_DRS1 ? 1 : 0);
		if(m_cmd2 & C2_MTON) {
			m_cur_floppy->mon_w(0);
			m_motor_timer[i]->adjust(attotime::never);
		} else if(prev & C2_MTON)
			m_motor_timer[i]->adjust(attotime::from_seconds(2), i);
		m_cur_floppy->ss_w(m_cmd2 & C2_SISELB ? 0 : 1);
		m_cur_floppy->dir_w(m_cmd2 & C2_DIRECB ? 0 : 1);
		m_cur_floppy->stp_w(m_cmd2 & C2_STEP ? 0 : 1);
	}
}

void thmfc1_device::wdata_w(u8 data)
{
	sync();

	m_wdata = data;
	m_stat0 &= ~(S0_BYTE | S0_DREQ);
	LOGREGS("wdata_w %02x\n", data);
}

void thmfc1_device::wclk_w(u8 data)
{
	sync();

	m_clk = data;
	LOGREGS("wclk_w %02x\n", data);
}

void thmfc1_device::wsect_w(u8 data)
{
	sync();

	m_sect = data;
	LOGREGS("wsect_w %02x\n", data);
}

void thmfc1_device::wtrck_w(u8 data)
{
	sync();

	m_trck = data;
	LOGREGS("wtrck_w %02x\n", data);
}

void thmfc1_device::wcell_w(u8 data)
{
	sync();

	m_cell = data;
	LOGREGS("wcell_w %02x\n", data);
}

u8 thmfc1_device::stat0_r()
{
	if(!machine().side_effects_disabled()) {
		sync();
		static int ps = -1;
		if(m_stat0 != ps)
			LOGREGS("stat0_r %02x -%s%s%s%s%s%s\n", m_stat0,
					 m_stat0 & S0_BYTE  ? " byte" : "",
					 m_stat0 & S0_END   ? " end" : "",
					 m_stat0 & S0_FREE  ? " free" : "",
					 m_stat0 & S0_CRCER ? " crcer" : "",
					 m_stat0 & S0_DREQ  ? " dreq" : "",
					 m_stat0 & S0_SYNC  ? " sync" : "");
		ps = m_stat0;
	}
	return m_stat0;
}

u8 thmfc1_device::stat1_r()
{
	u8 res = 0;
	if(m_cur_qdd) {
		if(m_cur_qdd->ms_r())
			res |= S1_INDX;
		if(m_cmd2 & C2_SISELB)
			res |= S1_MTON;
		res |= S1_TRK0;
		if(m_cur_qdd->wp_r())
			res |= S1_WPRT;
		if(!m_cur_qdd->ry_r())
			res |= S1_RDY;
	} else if(m_cur_floppy) {
		if(m_cur_floppy->idx_r())
			res |= S1_INDX;
		if(!m_cur_floppy->dskchg_r())
			res |= S1_DKCH;
		if(!m_cur_floppy->mon_r())
			res |= S1_MTON;
		if(!m_cur_floppy->trk00_r())
			res |= S1_TRK0;
		if(m_cur_floppy->wpt_r())
			res |= S1_WPRT;
		if(!m_cur_floppy->ready_r())
			res |= S1_RDY;
	}

	if(!machine().side_effects_disabled()) {
		static int ps = -1;
		if(res != ps)
			LOGREGS("stat1_r %02x -%s%s%s%s%s%s\n", res,
					 res & S1_INDX ? " index" : "",
					 res & S1_DKCH ? " dskchg" : "",
					 res & S1_MTON ? " mton" : "",
					 res & S1_TRK0 ? " trk0" : "",
					 res & S1_WPRT ? " wprt" : "",
					 res & S1_RDY ? " ready" : "");
		ps = res;
	}
	return res;
}

u8 thmfc1_device::rdata_r()
{
	if(!machine().side_effects_disabled()) {
		m_stat0 &= ~(S0_BYTE | S0_DREQ);
		LOGREGS("rdata_r %02x\n", m_rdata);
	}
	return m_rdata;
}

u64 thmfc1_device::time_to_cycles(const attotime &tm) const
{
	return tm.as_ticks(clock());
}

attotime thmfc1_device::cycles_to_time(u64 cycles) const
{
	return attotime::from_ticks(cycles, clock());
}

bool thmfc1_device::read_one_bit(u64 limit, u64 &next_flux_change)
{
	if(m_cur_qdd)
		return read_one_bit_qdd(limit);
	else
		return read_one_bit_floppy(limit, next_flux_change);
}


bool thmfc1_device::read_one_bit_qdd(u64 limit)
{
#define QDD_BITRATE		101265
	u64 window_end = m_window_start + (8 * 16000000 / QDD_BITRATE);
	if(window_end > limit) {
		LOGQDD("flux_window %s [ %d .. (%d) .. %d ]\n", machine().time().to_string(), m_window_start, limit, window_end);
		return true;
	} else
		LOGQDD("flux_window %s [ %d .. %d ] .. (%d)\n", machine().time().to_string(), m_window_start, window_end, limit);
	while(window_end < limit - (8 * 16000000 / QDD_BITRATE))
		window_end += (8 * 16000000 / QDD_BITRATE);

	m_rdata = m_cur_qdd->read();

	u64 start = time_to_cycles(m_cur_qdd->byte_timer_start());
	u64 expire = time_to_cycles(m_cur_qdd->byte_timer_expire());
	if(!m_cur_qdd->byte_timer_expire().is_never()) {
		LOGQDD("QDD byte_timer %s [ %d .. %d ]\n", machine().time().to_string(), start, expire);
		window_end = expire - (4 * 16000000 / QDD_BITRATE);
	}

	m_stat0 |= S0_BYTE;

	if(m_cmd0 & C0_ENSYN && (m_rdata == m_wdata && m_clk != 0xff))
		m_stat0 |= S0_SYNC;
	else
		m_stat0 &= ~S0_SYNC;

	m_window_start = window_end;
	m_last_sync = window_end;
	return false;
}

bool thmfc1_device::read_one_bit_floppy(u64 limit, u64 &next_flux_change)
{
	while(next_flux_change <= m_last_sync) {
		attotime flux = m_cur_floppy ? m_cur_floppy->get_next_transition(cycles_to_time(m_last_sync + 1)) : attotime::never;
		next_flux_change = flux.is_never() ? u64(-1) : time_to_cycles(flux);
	}

	u64 window_end = m_window_start + ((m_cell & 0x7f) + 1);
	if(window_end > limit)
		return true;

	m_bit = (next_flux_change <= window_end ? 1 : 0);
	if(m_bit && (m_cmd0 & C0_NOMCK))
		m_window_start = next_flux_change + (((m_cell & 0x7f) + 1) >> 1);
	else
		m_window_start = window_end;

	m_last_sync = window_end;

	if(m_cur_floppy == nullptr)
		return false;

	m_shift_reg = (m_shift_reg << 1) | m_bit;
	if(m_bit_counter & 1) {
		m_shift_data_reg = (m_shift_data_reg << 1) | m_bit;
		if((m_crc ^ (m_bit ? 0x8000 : 0x0000)) & 0x8000)
			m_crc = (m_crc << 1) ^ 0x1021;
		else
			m_crc = m_crc << 1;
	} else
		m_shift_clk_reg = (m_shift_clk_reg << 1) | m_bit;

	LOGSHIFT("read %s bit[%x]=%d shift=%04x data=%02x clk=%02x crc=%04x\n",
		m_bit_counter & 1 ? "[d]" : "[c]", m_bit_counter, m_bit,
		m_shift_reg, m_shift_data_reg, m_shift_clk_reg, m_crc);

	m_bit_counter++;
	m_bit_counter &= 0xf;

	if((m_cmd0 & C0_ENSYN) && ((~m_cmd1 & C1_DSYRD) || !m_cur_floppy->ready_r())) {
		if((m_shift_data_reg == m_wdata && m_shift_clk_reg == m_clk) ||
				(m_shift_data_reg == m_clk && m_shift_clk_reg == m_wdata)) {
			m_stat0 |= S0_SYNC;
			m_bit_counter = 0;
		} else if(m_bit_counter == 0)
			m_stat0 &= ~S0_SYNC;
	} else
		m_stat0 &= ~S0_SYNC;

	if(m_bit_counter == 0) {
		m_stat0 |= S0_BYTE;
		m_rdata = m_shift_data_reg;
	}

	return false;
}

bool thmfc1_device::write_one_bit(u64 limit)
{
	if(m_cur_qdd)
		return write_one_bit_qdd(limit);
	else
		return write_one_bit_floppy(limit);
}

bool thmfc1_device::write_one_bit_qdd(u64 limit)
{
	u64 window_end = m_window_start + (8 * 16000000 / QDD_BITRATE);
	if(window_end > limit) {
		LOGQDD("flux_window %s [ %d .. (%d) .. %d ]\n", machine().time().to_string(), m_window_start, limit, window_end);
		return true;
	} else
		LOGQDD("flux_window %s [ %d .. %d ] .. (%d)\n", machine().time().to_string(), m_window_start, window_end, limit);
	while(window_end < limit - (8 * 16000000 / QDD_BITRATE))
		window_end += (8 * 16000000 / QDD_BITRATE);

	if(m_byte_counter > 0 && ~m_stat0 & S0_BYTE)
		m_cur_qdd->write(m_wdata);

	u64 start = time_to_cycles(m_cur_qdd->byte_timer_start());
	u64 expire = time_to_cycles(m_cur_qdd->byte_timer_expire());
	if(!m_cur_qdd->byte_timer_expire().is_never()) {
		LOGQDD("QDD byte_timer %s [ %d .. %d ]\n", machine().time().to_string(), start, expire);
		window_end = expire - (4 * 16000000 / QDD_BITRATE);
	}

	m_stat0 |= S0_BYTE;

	m_window_start = window_end;
	m_last_sync = window_end;
	return false;
}

bool thmfc1_device::write_one_bit_floppy(u64 limit)
{
	u64 window_end = m_window_start + ((m_cell & 0x7f) + 1);
	if(window_end > limit)
		return true;

	if(m_bit_counter == 0 && m_byte_counter == 0 && m_write_buffer_idx == 0)
		m_write_buffer_start = m_window_start;

	if(m_bit_counter & 1)
		m_bit = (m_shift_data_reg >> 7);
	else if(m_use_shift_clk_reg)
		m_bit = (m_shift_clk_reg >> 7);
	else if(m_cmd0 & C0_FM)
		m_bit = 1;
	else
		m_bit = !(m_bit || (m_shift_data_reg >> 7));

	LOGSHIFT("write %s bit[%x]=%d data=%02x clk=%02x crc=%04x\n",
		m_bit_counter & 1 ? "[d]" : "[c]", m_bit_counter, m_bit,
		m_shift_data_reg, m_shift_clk_reg, m_crc);

	if(m_bit_counter & 1) {
		m_shift_data_reg = (m_shift_data_reg & 0x7f) << 1 | (m_shift_data_reg >> 7);

		if((m_crc ^ (m_bit ? 0x8000 : 0x0000)) & 0x8000)
			m_crc = (m_crc << 1) ^ 0x1021;
		else
			m_crc = m_crc << 1;
	} else
		m_shift_clk_reg = (m_shift_clk_reg & 0x7f) << 1 | (m_shift_clk_reg >> 7);

	m_bit_counter++;
	m_bit_counter &= 0xf;

	if(m_bit_counter == 0)
		m_stat0 |= S0_BYTE;

	if(m_bit)
		m_write_buffer[m_write_buffer_idx++] = cycles_to_time (m_window_start + (((m_cell & 0x7f) + 1) >> 1));
	if(m_write_buffer_idx == std::size(m_write_buffer)) {
		m_cur_floppy->write_flux(cycles_to_time(m_write_buffer_start), cycles_to_time(window_end), m_write_buffer_idx, m_write_buffer);
		m_write_buffer_idx = 0;
		m_write_buffer_start = window_end;
	}

	m_window_start = window_end;
	m_last_sync = window_end;

	return false;
}

u16 crc_from_data(u8 data)
{
	u16 crc = 0xffff;

	for (int i = 0; i < 8; i++) {
		if((crc ^ ((data & 0x80) ? 0x8000 : 0x0000)) & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc = crc << 1;
		data = data << 1;
	}
	return crc;
}

void thmfc1_device::sync()
{
	u64 next_sync = machine().time().as_ticks(clock());
	u64 next_flux_change = 0;
	while(m_last_sync < next_sync)
		switch(m_state) {
		case S_IDLE:
			m_last_sync = next_sync;
			break;

		case S_READ_WAIT_HEADER_SYNC:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_stat0 & S0_SYNC) {
				m_crc = crc_from_data(m_wdata);
				m_bit_counter = 0;
				m_byte_counter = 0;
				LOGSTATE("s_read_wait_header_sync %d data=%02x clk=%02x crc=%04x\n",
						m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc);
				m_state = S_READ_VERIFY_HEADER;
				LOGSTATE("s_read_verify_header_verify\n");
			}
			break;

		case S_READ_VERIFY_HEADER: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			bool valid = true;
			LOGSTATE("s_read_verify_header %d data=%02x clk=%02x crc=%04x\n",
					m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc);
			switch(m_byte_counter) {
			case 1:
			case 2:
				valid = m_stat0 & S0_SYNC;
				break;
			case 3:
				valid = m_rdata == 0xfe;
				if(valid && (m_cmd0 & 3) == 2) {
					m_byte_counter = 0;
					m_stat0 |= S0_DREQ;
					m_state = S_READ;
					LOGSTATE("s_read\n");
				}
				break;
			case 4:
				valid = m_rdata == m_trck;
				break;
			case 5:
				// The THMFC1 BIOS always sets side to zero in the sector header.
                                // This is a difference wrt to wd177x MFM/MF track format description
				valid = (m_rdata & 1) == (m_cmd1 & C1_SIDE ? 1 : 0);
				break;
			case 6:
				valid = m_rdata == m_sect;
				break;
			case 7:
				valid = (m_rdata & 3) == ((m_cmd1 >> 5) & 3);
				break;
				// 8 skipped
			case 9:
				valid = m_crc == 0;
				if(valid) {
					m_byte_counter = 0;
					switch(m_cmd0 & 3) {
						case 1:
							m_state = S_WRITE_SKIP_GAP;
							LOGSTATE("s_write_skip_gap\n");
							break;
						case 3:
							m_state = S_READ_SKIP_GAP;
							LOGSTATE("s_read_skip_gap\n");
							break;
						default:
							m_state = S_READ_WAIT_HEADER_SYNC;
							LOGSTATE("s_wait_header_sync\n");
					}
				}
			}
			if(!valid) {
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("s_wait_header_sync\n");
			}
			break;
		}

		case S_READ_SKIP_GAP:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 27) {
				m_byte_counter = 0;
				m_state = S_READ_WAIT_SECTOR_SYNC;
				LOGSTATE("s_read_wait_sector_sync\n");
			}
			break;

		case S_READ_WAIT_SECTOR_SYNC:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_stat0 & S0_SYNC) {
				m_crc = crc_from_data(m_wdata);
				m_bit_counter = 0;
				m_byte_counter = 0;
				m_state = S_READ_VERIFY_SECTOR;
				LOGSTATE("s_read_verify_sector\n");
				break;
			}
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 42) {
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("s_wait_header_sync\n");
			}
			break;

		case S_READ_VERIFY_SECTOR: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			m_stat0 |= S0_DREQ;
			bool valid = true;
			LOGSTATE("s_read_verify_sector %d data=%02x clk=%02x crc=%04x\n",
					m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc);
			switch(m_byte_counter) {
			case 1:
			case 2:
				valid = m_stat0 & S0_SYNC;
				break;
			case 3:
				valid = m_rdata == 0xfb;
				if(valid) {
					m_byte_counter = 0;
					m_state = S_READ;
					LOGSTATE("s_read\n");
				}
			}
			if(!valid) {
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("s_read_wait_header_sync\n");
			}
			break;
		}

		case S_READ: {
			bool overflow = m_stat0 & S0_DREQ;
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("s_read %d data=%02x clk=%02x crc=%04x\n",
					m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc);
			if(overflow) {
				if(m_crc)
					m_stat0 |= S0_CRCER;
				m_stat0 |= S0_FREE;
				m_stat0 &= ~S0_DREQ;
				m_cmd0 &= ~3;
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("s_read_wait_header_sync\n");
				break;
			} else
				m_stat0 |= S0_DREQ;
			break;
		}

		case S_WRITE_SKIP_GAP:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 22) {
				m_byte_counter = 0;
				m_shift_data_reg = 0;
				m_use_shift_clk_reg = false;
				m_state = S_WRITE_SECTOR_SYNC;
				LOGSTATE("s_write_sector_sync\n");
			}
			break;

		case S_WRITE_SECTOR_SYNC:
			if(m_bit_counter == 0)
				LOGSTATE("s_write_sector_sync %d data=%02x clk=%02x crc=%04x use_clk=%d\n",
						m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc, m_use_shift_clk_reg);
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 12) {
				m_crc = 0xffff;
				m_byte_counter = 0;
				m_stat0 |= S0_DREQ;
				m_state = S_WRITE_SECTOR;
				LOGSTATE("s_write_sector\n");
			}
			break;

		case S_WRITE_SECTOR: {
			bool overflow = m_stat0 & S0_DREQ;
			if(m_bit_counter == 0) {
				if(m_shift_data_reg != m_wdata)
					m_use_shift_clk_reg = false;
				if(m_shift_clk_reg != m_clk && m_clk < 0xff)
					m_use_shift_clk_reg = true;
				m_shift_data_reg = m_wdata;
				m_shift_clk_reg = m_clk;
				LOGSTATE("s_write_sector %d data=%02x clk=%02x crc=%04x use_clk=%d\n",
						m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc, m_use_shift_clk_reg);
			}
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			m_stat0 |= S0_DREQ;
			if(overflow) {
				m_byte_counter = 0;
				m_state = S_WRITE_CRC;
				LOGSTATE("s_write_crc\n");
				break;
			}
			break;
		}

		case S_WRITE_CRC:
			if(m_bit_counter == 0){
				m_shift_data_reg = m_crc >> 8;
				LOGSTATE("s_write_crc %d data=%02x clk=%02x crc=%04x use_clk=%d\n",
						m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc, m_use_shift_clk_reg);
			}
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 2) {
				m_stat0 |= S0_FREE;
				m_stat0 &= ~S0_DREQ;
				m_cmd0 &= ~3;
				m_state = S_READ_WAIT_HEADER_SYNC;
				flush_flux();
				LOGSTATE("s_read_wait_header_sync\n");
			} else
				m_stat0 |= S0_DREQ;
			break;

		case S_FORMAT:
			if(m_bit_counter == 0) {
				if(m_shift_data_reg != m_wdata)
					m_use_shift_clk_reg = false;
				if(m_shift_clk_reg != m_clk && m_clk < 0xff)
					m_use_shift_clk_reg = true;
				m_shift_data_reg = m_wdata;
				m_shift_clk_reg = m_clk;
				LOGSTATE("s_format %d data=%02x clk=%02x crc=%04x use_clk=%d\n",
						m_byte_counter, m_shift_data_reg, m_shift_clk_reg, m_crc, m_use_shift_clk_reg);
			}
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter == 0)
				m_byte_counter++;
			break;
		}
}

void thmfc1_device::flush_flux()
{
	if(m_write_buffer_idx) {
		m_cur_floppy->write_flux(cycles_to_time(m_write_buffer_start), cycles_to_time(m_last_sync), m_write_buffer_idx, m_write_buffer);
		m_write_buffer_idx = 0;
	}
}

thmfc1_connector::thmfc1_connector(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, THMFC1_CONNECTOR, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	formats(nullptr),
	m_enable_sound(false),
	m_sectoring_type(floppy_image::SOFT)
{
}

thmfc1_connector::~thmfc1_connector()
{
}

void thmfc1_connector::device_start()
{
}

void thmfc1_connector::device_config_complete()
{
	floppy_image_device *dev = dynamic_cast<floppy_image_device *>(get_card_device());
	if(dev)
	{
		dev->set_formats(formats);
		dev->enable_sound(m_enable_sound);
		dev->set_sectoring_type(m_sectoring_type);
	}
}

device_t *thmfc1_connector::get_device()
{
	return dynamic_cast<device_t *>(get_card_device());
}
