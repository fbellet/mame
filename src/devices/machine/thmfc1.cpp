// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, Fabrice Bellet

// Custom Thomson 3.5"/5.25"/QDD diskette controller gate array used in CD 90-351, TO8 and TO9+


// Documentation: http://dcmoto.free.fr/documentation/docto8/index.html
//
// - Emulation of FREE stat0 bit should be tested with real hardware:
//   FREE is not set when the controller received a RESET cmd after an
//   interrupted RSECT cmd (tested with analpiste_to8).
// - Motor on/off control has issues

#include "emu.h"
#include "thmfc1.h"

DEFINE_DEVICE_TYPE(THMFC1, thmfc1_device, "thmfc1", "SGS-Thomson THM-FC-1 Diskette Controller") // SGS logo used on silkscreen

#define LOG_FLUX         (1U << 1) // Show flux changes
#define LOG_STATE        (1U << 2) // Show state machine
#define LOG_SHIFT        (1U << 3) // Show shift register contents
#define LOG_REGS         (1U << 4) // Show register access
#define LOG_COMMAND      (1U << 5) // Show command invocation
#define LOG_STAT0        (1U << 6) // Show stat0 register updates
#define LOG_CRC          (1U << 7) // Show crc data

// #define VERBOSE (LOG_COMMAND)

#include "logmacro.h"

#define LOGFLUX(...)        LOGMASKED(LOG_FLUX, __VA_ARGS__)
#define LOGSHIFT(...)       LOGMASKED(LOG_SHIFT, __VA_ARGS__)
#define LOGSTATE(...)       LOGMASKED(LOG_STATE, __VA_ARGS__)
#define LOGREGS(...)        LOGMASKED(LOG_REGS, __VA_ARGS__)
#define LOGCOMMAND(...)     LOGMASKED(LOG_COMMAND, __VA_ARGS__)
#define LOGSTAT0(...)       LOGMASKED(LOG_STAT0, __VA_ARGS__)
#define LOGCRC(...)         LOGMASKED(LOG_CRC, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

thmfc1_device::thmfc1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, THMFC1, tag, owner, clock),
	m_floppy(*this, "%u", 0U)
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
	m_timer_motoroff = timer_alloc(FUNC(thmfc1_device::motor_off), this);

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
}

TIMER_CALLBACK_MEMBER(thmfc1_device::motor_off)
{
	LOGREGS("motor off\n");
	if(m_cur_floppy)
		m_cur_floppy->mon_w(1);
}

void thmfc1_device::device_post_load()
{
	if(m_cmd2 & C2_DRS0)
		m_cur_floppy = m_floppy[0]->get_device();
	else if(m_cmd2 & C2_DRS1)
		m_cur_floppy = m_floppy[1]->get_device();
	else
		m_cur_floppy = nullptr;
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
			LOGCOMMAND("format start h=%d t=%d sz=%d\n",
					 m_cmd1 & C1_SIDE ? 1 : 0,
					 m_trck,
					 128 << ((m_cmd1 >> 5) & 3));
			m_state = S_FORMAT;
			m_bit_counter = 0;
			m_byte_counter = 0;
			m_use_shift_clk_reg = (m_clk == 0x0a);
			m_window_start = m_last_sync;
			LOGSTATE("format start\n");
		} else {
			flush_flux();
			m_state = S_READ_WAIT_HEADER_SYNC;
			LOGSTATE("read_wait_header_sync start\n");
		}
		break;
	case 1:
		LOGCOMMAND("write_sector start h=%d t=%d s=%d sz=%d\n",
				 m_cmd1 & C1_SIDE ? 1 : 0,
				 m_trck,
				 m_sect,
				 128 << ((m_cmd1 >> 5) & 3));
		m_state = S_READ_WAIT_HEADER_SYNC;
		m_bit_counter = 0;
		if(m_stat0 & S0_FREE)
			LOGSTAT0("free unset in stat0\n");
		m_stat0 &= ~S0_FREE;
		m_window_start = m_last_sync;
		LOGSTATE("read_wait_header_sync start\n");
		break;
	case 2:
		LOGCOMMAND("rhead unimplemented\n");
		exit(0);
	case 3:
		LOGCOMMAND("read_sector start h=%d t=%d s=%d sz=%d\n",
				 m_cmd1 & C1_SIDE ? 1 : 0,
				 m_trck,
				 m_sect,
				 128 << ((m_cmd1 >> 5) & 3));
		m_state = S_READ_WAIT_HEADER_SYNC;
		m_bit_counter = 0;
		if(m_stat0 & S0_FREE)
			LOGSTAT0("free unset in stat0\n");
		m_stat0 &= ~S0_FREE;
		m_window_start = m_last_sync;
		LOGSTATE("read_wait_header_sync start\n");
		break;
	}
}

void thmfc1_device::cmd1_w(u8 data)
{
	sync();

	m_cmd1 = data;
	m_sect_size = 128 << ((m_cmd1 >> 5) & 3);
	LOGREGS("cmd1_w %02x, sector=(size=%d, side=%d) precomp=%d sync_only_when_ready=%s\n", m_cmd1,
			 m_sect_size,
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

	if(m_cmd2 & C2_DRS0)
		m_cur_floppy = m_floppy[0]->get_device();
	else if(m_cmd2 & C2_DRS1)
		m_cur_floppy = m_floppy[1]->get_device();
	else
		m_cur_floppy = nullptr;

	if(m_cur_floppy) {
		if((prev & C2_MTON) && !(m_cmd2 & C2_MTON))
			m_timer_motoroff->adjust(attotime::from_seconds(2));
		if(m_cmd2 & C2_MTON) {
			m_cur_floppy->mon_w(0);
			m_timer_motoroff->adjust(attotime::never);
		}
		m_cur_floppy->ss_w(m_cmd2 & C2_SISELB ? 0 : 1);
		m_cur_floppy->dir_w(m_cmd2 & C2_DIRECB ? 0 : 1);
		m_cur_floppy->stp_w(m_cmd2 & C2_STEP ? 0 : 1);
	}
}

void thmfc1_device::wdata_w(u8 data)
{
	m_wdata = data;
	if(m_stat0 & S0_BYTE)
		LOGSTAT0("byte unset in stat0\n");
	if(m_stat0 & S0_DREQ)
		LOGSTAT0("dreq unset in stat0\n");
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
	if(m_cur_floppy) {
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
		if(m_stat0 & S0_BYTE)
			LOGSTAT0("byte unset in stat0\n");
		if(m_stat0 & S0_DREQ)
			LOGSTAT0("dreq unset in stat0\n");
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
	while(next_flux_change <= m_last_sync) {
		attotime flux = m_cur_floppy ? m_cur_floppy->get_next_transition(cycles_to_time(m_last_sync+1)) : attotime::never;
		next_flux_change = flux.is_never() ? u64(-1) : time_to_cycles(flux);
	}

	u64 window_end = m_window_start + (m_cell & 0x7f);
	if(window_end > limit)
		return true;

	m_bit = (next_flux_change < window_end ? 1 : 0);
	if(m_bit && (m_cmd0 & C0_NOMCK))
		m_window_start = next_flux_change + ((m_cell & 0x7f) >> 1);
	else
		m_window_start = window_end;

	m_last_sync = window_end;

	m_shift_reg = (m_shift_reg << 1) | m_bit;
	if(m_bit_counter & 1) {
		m_shift_data_reg = (m_shift_data_reg << 1) | m_bit;
		if((m_crc ^ (m_bit ? 0x8000 : 0x0000)) & 0x8000)
			m_crc = (m_crc << 1) ^ 0x1021;
		else
			m_crc = m_crc << 1;
	} else
		m_shift_clk_reg = (m_shift_clk_reg << 1) | m_bit;

	static int count = -1;
	count++;
	if (m_shift_reg || (count & 0xfff) == 0)
		LOGSHIFT("read %s bit[%x]=%d shift_reg=0x%04x c=0x%02x d=0x%02x crc=0x%04x\n",
			m_bit_counter & 1 ? "[d]" : "[c]", m_bit_counter, m_bit,
			m_shift_reg, m_shift_clk_reg, m_shift_data_reg, m_crc);

	m_bit_counter++;
	m_bit_counter &= 0xf;

	if ((m_cmd0 & C0_ENSYN) && (m_cmd1 & C1_DSYRD ? (m_cur_floppy ? m_cur_floppy->ready_r() : 0): 1)) {
		if((m_shift_data_reg == m_wdata && m_shift_clk_reg == m_clk) ||
			(m_shift_data_reg == m_clk && m_shift_clk_reg == m_wdata)) {
			if(~m_stat0 & S0_SYNC)
				LOGSTAT0("sync set in stat0 (wdata=0x%02x clk=0x%02x)\n", m_wdata, m_clk);
			m_stat0 |= S0_SYNC;
			m_bit_counter = 0;
		} else if(m_bit_counter == 0) {
			if(m_stat0 & S0_SYNC)
				LOGSTAT0("sync unset in stat0\n");
			m_stat0 &= ~S0_SYNC;
		}
	}

	if(m_bit_counter == 0) {
		if(~m_stat0 & S0_BYTE)
			LOGSTAT0("byte set in stat0\n");
		m_stat0 |= S0_BYTE;
		m_rdata = m_shift_data_reg;
	}

	return false;
}
bool thmfc1_device::write_one_bit(u64 limit)
{
	u64 window_end = m_window_start + (m_cell & 0x7f);
	if(window_end > limit)
		return true;

	if(m_bit_counter == 0 && m_byte_counter == 0 && m_write_buffer_idx == 0) {
		LOGFLUX("flux write_buffer start at %d\n", m_window_start);
		m_write_buffer_start = m_window_start;
	}

	if(m_bit_counter & 1)
		m_bit = (m_shift_data_reg >> 7);
	else {
		if(m_use_shift_clk_reg)
			m_bit = (m_shift_clk_reg >> 7);
		else
			m_bit = !(m_bit || (m_shift_data_reg >> 7)); // MFM
	}

	LOGSHIFT("write %s bit[%x]=%d c=0x%02x d=0x%02x crc=0x%04x\n",
		m_bit_counter & 1 ? "[d]" : "[c]", m_bit_counter, m_bit,
		m_shift_clk_reg, m_shift_data_reg, m_crc);

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

	if(m_bit_counter == 0) {
		if(~m_stat0 & S0_BYTE)
			LOGSTAT0("byte set in stat0\n");
		m_stat0 |= S0_BYTE;
	}

	if(m_bit)
		m_write_buffer[m_write_buffer_idx++] = cycles_to_time (m_window_start + ((m_cell & 0x7f) >> 1));
	if(m_write_buffer_idx == 128) {
		LOGFLUX("flux write_buffer commited between %d and %d\n", m_write_buffer_start, window_end);
		m_cur_floppy->write_flux(cycles_to_time(m_write_buffer_start), cycles_to_time(window_end), m_write_buffer_idx, m_write_buffer);
		m_write_buffer_idx = 0;
		m_write_buffer_start = window_end;
	}

	m_window_start = window_end;
	m_last_sync = window_end;

	return false;
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
			if(m_shift_reg == 0xaaaa)
				m_crc = 0xffff;
			if(m_stat0 & S0_SYNC) {
				m_bit_counter = 0;
				m_byte_counter = 0;
				m_state = S_READ_VERIFY_HEADER;
				LOGSTATE("read_wait_header_sync end (wdata=0x%02x clk=0x%02x)\n", m_wdata, m_clk);
			}
			break;

		case S_READ_VERIFY_HEADER: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("read_verify_header byte_counter=%d\n", m_byte_counter);
			bool valid = true;
			switch(m_byte_counter) {
			case 1:
			case 2:
				valid = m_stat0 & S0_SYNC;
				LOGSTATE("read_verify_header rdata=0x%02x clk=0x%02x\n", m_rdata, m_clk);
				break;
			case 3:
				valid = m_rdata == 0xfe;
				LOGSTATE("read_verify_header rdata=0x%02x\n", m_rdata);
				break;
			case 4:
				valid = m_rdata == m_trck;
				LOGSTATE("read_verify_header rdata=0x%02x trck=0x%02x\n", m_rdata, m_trck);
				break;
			case 5:
				// The THMFC1 BIOS always sets side to zero in the sector header.
                                // This is a difference wrt to wd177x MFM/MF track format description
				valid = (m_rdata & 1) == (m_cmd1 & C1_SIDE ? 1 : 0);
				LOGSTATE("read_verify_header rdata=0x%02x side=0x%02x\n", m_rdata, m_cmd1 & C1_SIDE ? 1 : 0);
				break;
			case 6:
				valid = m_rdata == m_sect;
				LOGSTATE("read_verify_header rdata=0x%02x sect=0x%02x\n", m_rdata, m_sect);
				break;
			case 7:
				valid = (m_rdata & 3) == ((m_cmd1 >> 5) & 3);
				LOGSTATE("read_verify_header rdata=0x%02x len=0x%02x\n", m_rdata, (m_cmd1 >> 5) & 3);
				break;
				// 8 skipped
			case 9:
				valid = m_crc == 0;
				LOGSTATE("read_verify_header crc=0x%04x\n", m_crc);
				if(valid) {
					m_byte_counter = 0;
					switch(m_cmd0 & 3) {
						case 1:
							m_state = S_WRITE_SKIP_GAP;
							LOGSTATE("read_verify_header end, go to write_skip_gap\n");
							break;
						case 3:
							m_state = S_READ_SKIP_GAP;
							LOGSTATE("read_verify_header end, go to read_skip_gap\n");
							break;
						default:
							m_state = S_READ_WAIT_HEADER_SYNC;
							LOGSTATE("read_verify_header end, go to wait_header_sync\n");
					}
				}
			}
			if(!valid) {
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("read_verify_header failed, restart wait_header_sync\n");
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
				LOGSTATE("read_skip_gap end\n");
			}
			break;

		case S_READ_WAIT_SECTOR_SYNC:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_shift_reg == 0xaaaa)
				m_crc = 0xffff;
			if(m_stat0 & S0_SYNC) {
                                m_bit_counter = 0;
				m_byte_counter = 0;
				if(~m_stat0 & S0_DREQ)
					LOGSTAT0("dreq set in stat0\n");
				m_stat0 |= S0_DREQ;
				m_state = S_READ_VERIFY_SECTOR;
				LOGSTATE("read_wait_sector_sync end (wdata=0x%02x clk=0x%02x)\n", m_wdata, m_clk);
				LOGCRC("crc=0x%04x\n", m_crc);
				break;
			}
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 42) {
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("read_wait_sector_sync failed, restart wait_header_sync\n");
			}
			break;

		case S_READ_VERIFY_SECTOR: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("read_verify_sector byte_counter=%d\n", m_byte_counter);
			bool valid = true;
			switch(m_byte_counter) {
			case 1:
			case 2:
				valid = m_stat0 & S0_SYNC;
				LOGSTATE("read_verify_sector rdata=0x%02x clk=0x%02x\n", m_rdata, m_clk);
				LOGCRC("crc=0x%04x\n", m_crc);
				break;
			case 3:
				valid = m_rdata == 0xfb;
				LOGSTATE("read_verify_sector rdata=0x%02x\n", m_rdata);
				LOGCRC("crc=0x%04x\n", m_crc);
				if(valid) {
					m_byte_counter = 0;
					m_state = S_READ_SECTOR;
					LOGSTATE("read_verify_sector end\n");
				}
			}
			if(!valid) {
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("read_verify_sector failed, restart wait_header_sync\n");
			}
			break;
		}

		case S_READ_SECTOR:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == m_sect_size) {
				m_byte_counter = 0;
				m_state = S_READ_SECTOR_CRC;
				LOGSTATE("read_sector end\n");
				break;
			}
			LOGSTATE("read_sector byte_counter=%d rdata=0x%02x\n", m_byte_counter, m_rdata);
			LOGCRC("crc=0x%04x\n", m_crc);
			break;

		case S_READ_SECTOR_CRC:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("read_sector_crc byte_counter=%d rdata=0x%02x\n", m_byte_counter, m_rdata);
			LOGCRC("crc=0x%04x\n", m_crc);
			if(m_byte_counter == 2) {
				if(m_crc) {
					if(~m_stat0 & S0_CRCER)
						LOGSTAT0("crcer set in stat0\n");
					m_stat0 |= S0_CRCER;
				}
				if(~m_stat0 & S0_FREE)
					LOGSTAT0("free set in stat0\n");
				m_stat0 |= S0_FREE;
				m_cmd0 &= ~3;
				m_state = S_READ_WAIT_HEADER_SYNC;
				LOGSTATE("read_sector_crc end\n");
			}
			break;

		case S_WRITE_SKIP_GAP:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 22) {
				m_byte_counter = 0;
				m_shift_data_reg = 0;
				m_state = S_WRITE_SECTOR_SYNC;
				LOGSTATE("write_skip_gap end\n");
			}
			break;

		case S_WRITE_SECTOR_SYNC:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			if(m_byte_counter == 12) {
				m_crc = 0xffff;
				m_byte_counter = 0;
				m_shift_clk_reg = m_clk;
				m_shift_data_reg = m_wdata;
				m_use_shift_clk_reg = true;
				if(~m_stat0 & S0_DREQ)
					LOGSTAT0("dreq set in stat0\n");
				m_stat0 |= S0_DREQ;
				m_state = S_WRITE_SECTOR;
				LOGSTATE("write_sector_sync end reset crc\n");
			}
			break;

		case S_WRITE_SECTOR:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("write_sector byte_counter=%d shift_data_reg=0x%02x\n", m_byte_counter, m_shift_data_reg);
			LOGCRC("crc=0x%04x\n", m_crc);
			if(m_byte_counter == m_sect_size + 4) {
				m_byte_counter = 0;
				m_shift_data_reg = m_crc >> 8;
				m_state = S_WRITE_SECTOR_CRC;
				LOGSTATE("write_sector end\n");
				break;
			}
			m_shift_data_reg = m_wdata;
			if(m_use_shift_clk_reg && m_wdata != 0xa1)
				m_use_shift_clk_reg = false;
			m_shift_clk_reg = m_clk;
			if(~m_stat0 & S0_DREQ)
				LOGSTAT0("dreq set in stat0\n");
			m_stat0 |= S0_DREQ;
			break;

		case S_WRITE_SECTOR_CRC:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("write_sector_crc byte_counter=%d shift_data_reg=0x%02x\n", m_byte_counter, m_shift_data_reg);
			LOGCRC("crc=0x%04x\n", m_crc);
			m_shift_data_reg = m_crc >> 8;
			if(m_byte_counter == 2) {
				if(~m_stat0 & S0_FREE)
					LOGSTAT0("free set in stat0\n");
				m_stat0 |= S0_FREE;
				m_state = S_READ_WAIT_HEADER_SYNC;
				flush_flux();
				LOGSTATE("write_sector_crc end\n");
			}
			break;

		case S_FORMAT:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter)
				break;
			m_byte_counter++;
			LOGSTATE("format shift_data_reg=0x%02x\n", m_shift_data_reg);
			m_shift_data_reg = m_wdata;
			m_shift_clk_reg = m_clk;
			m_use_shift_clk_reg = (m_clk == 0x0a);
			break;
		}
}

void thmfc1_device::flush_flux()
{
	if(m_write_buffer_idx) {
		LOGFLUX("flux write_buffer flushed between %d and %d (%d changes)\n", m_write_buffer_start, m_last_sync, m_write_buffer_idx);
		m_cur_floppy->write_flux(cycles_to_time(m_write_buffer_start), cycles_to_time(m_last_sync), m_write_buffer_idx, m_write_buffer);
		m_write_buffer_idx = 0;
	}
}

