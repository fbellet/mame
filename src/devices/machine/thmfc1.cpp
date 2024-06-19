// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, Fabrice Bellet

// Custom Thomson 3.5"/5.25"/QDD diskette controller gate array used in CD 90-351, TO8 and TO9+


// Documentation: http://dcmoto.free.fr/documentation/docto8/index.html

#include "emu.h"
#include "thmfc1.h"

DEFINE_DEVICE_TYPE(THMFC1, thmfc1_device, "thmfc1", "SGS-Thomson THM-FC-1 Diskette Controller") // SGS logo used on silkscreen

#define LOG_FLUX         (1U << 1) // Show flux changes
#define LOG_STATE        (1U << 2) // Show state machine
#define LOG_SHIFT        (1U << 3) // Shows shift register contents
#define LOG_REGS         (1U << 4) // Show register access
#define LOG_COMMAND      (1U << 5) // Show command invocation

// #define VERBOSE (LOG_COMMAND)

#include "logmacro.h"

#define LOGFLUX(...)        LOGMASKED(LOG_FLUX, __VA_ARGS__)
#define LOGSHIFT(...)       LOGMASKED(LOG_SHIFT, __VA_ARGS__)
#define LOGSTATE(...)       LOGMASKED(LOG_STATE, __VA_ARGS__)
#define LOGREGS(...)        LOGMASKED(LOG_REGS, __VA_ARGS__)
#define LOGCOMMAND(...)     LOGMASKED(LOG_COMMAND, __VA_ARGS__)

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
	save_item(NAME(m_data));
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
	save_item(NAME(m_data_reg));
	save_item(NAME(m_data_separator_phase));
}

void thmfc1_device::device_reset()
{
	m_cmd0 = 0;
	m_cmd1 = 0;
	m_cmd2 = 0;
	m_stat0 = S0_FREE;
	m_data = 0;
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
	m_data_reg = 0;
	m_data_separator_phase = false;
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

	if(m_stat0 & S0_FREE) {
		switch(m_cmd0 & 3) {
		case 0:
			break;
		case 1:
			LOGCOMMAND("write_sector start h=%d t=%d s=%d sz=%d\n",
					 m_cmd1 & C1_SIDE ? 1 : 0,
					 m_trck,
					 m_sect,
					 128 << ((m_cmd1 >> 5) & 3));
			m_state = S_WAIT_HEADER_SYNC;
			LOGSTATE("wait_header_sync start\n");
			m_stat0 &= ~S0_FREE;
			m_window_start = m_last_sync;
			break;
		case 2:
			LOGCOMMAND("rhead\n");
			exit(0);
		case 3:
			LOGCOMMAND("read_sector start h=%d t=%d s=%d sz=%d\n",
					 m_cmd1 & C1_SIDE ? 1 : 0,
					 m_trck,
					 m_sect,
					 128 << ((m_cmd1 >> 5) & 3));
			m_state = S_WAIT_HEADER_SYNC;
			LOGSTATE("wait_header_sync start\n");
			m_stat0 &= ~S0_FREE;
			m_window_start = m_last_sync;
			break;
		}
	}
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
	m_data = data;
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

	if(!machine().side_effects_disabled())
		LOGREGS("stat1_r %02x -%s%s%s%s%s%s\n", res,
				 res & S1_INDX ? " index" : "",
				 res & S1_DKCH ? " dskchg" : "",
				 res & S1_MTON ? " mton" : "",
				 res & S1_TRK0 ? " trk0" : "",
				 res & S1_WPRT ? " wprt" : "",
				 res & S1_RDY ? " ready" : "");
	return res;
}

u8 thmfc1_device::rdata_r()
{
	if(!machine().side_effects_disabled())
		m_stat0 &= ~(S0_BYTE | S0_DREQ);
	LOGREGS("rdata_r %02x\n", m_data);
	return m_data;
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

	LOGFLUX("flux window_start %d window_end %d limit %d next_flux_change %d\n",
		m_window_start, window_end, limit, next_flux_change);
	m_bit = (next_flux_change < window_end ? 1 : 0);
	if(m_bit && (m_cmd0 & C0_NOMCK))
		m_window_start = next_flux_change + ((m_cell & 0x7f) >> 1);
	else
		m_window_start = window_end;

	m_last_sync = window_end;

	m_shift_reg = (m_shift_reg << 1) | m_bit;
	m_bit_counter++;
	if(m_data_separator_phase) {
		m_data_reg = (m_data_reg << 1) | m_bit;
		if((m_crc ^ (m_bit ? 0x8000 : 0x0000)) & 0x8000)
			m_crc = (m_crc << 1) ^ 0x1021;
		else
			m_crc = m_crc << 1;
	}

	LOGSHIFT("read %s bit=%d bit_counter=0x%x shift_reg=0x%04x clk_bits=0x%02x data_reg=0x%02x crc=0x%04x\n",
		m_data_separator_phase ? "[d]" : "[c]",
		m_bit, m_bit_counter, m_shift_reg, clk_bits(), m_data_reg, m_crc);
	m_data_separator_phase = !m_data_separator_phase;
	return false;
}
bool thmfc1_device::write_one_bit(u64 limit)
{
	u64 window_end = m_window_start + (m_cell & 0x7f);
	if(window_end > limit)
		return true;

	LOGFLUX("flux window_start %d window_end %d limit %d\n", m_window_start, window_end, limit);
	attotime write_buffer = cycles_to_time (m_window_start + ((m_cell & 0x7f) >> 1));
	m_bit_counter++;

	if (m_data_separator_phase) {
		m_bit = (m_shift_data_reg >> 7);
		if((m_crc ^ (m_bit ? 0x8000 : 0x0000)) & 0x8000)
			m_crc = (m_crc << 1) ^ 0x1021;
		else
			m_crc = m_crc << 1;
	} else
		if (m_use_shift_clk_reg)
			m_bit = (m_shift_clk_reg >> 7);
		else
			m_bit = !(m_bit || (m_shift_data_reg >> 7)); // MFM
	LOGSHIFT("write %s bit=%d bit_counter=0x%x shift_clk_reg=0x%02x shift_data_reg=0x%02x crc=0x%04x\n",
		m_data_separator_phase ? "[d]" : "[c]", m_bit,
		m_bit_counter, m_shift_clk_reg, m_shift_data_reg, m_crc);

	if (m_data_separator_phase)
		m_shift_data_reg = (m_shift_data_reg & 0x7f) << 1 | (m_shift_data_reg >> 7);
	else
		m_shift_clk_reg = (m_shift_clk_reg & 0x7f) << 1 | (m_shift_clk_reg >> 7);

	m_cur_floppy->write_flux(cycles_to_time(m_window_start), cycles_to_time(window_end), m_bit, &write_buffer);

	m_window_start = window_end;
	m_last_sync = window_end;
	m_data_separator_phase = !m_data_separator_phase;
	return false;
}

u8 thmfc1_device::clk_bits() const
{
	return
		(m_shift_reg & 0x8000 ? 0x80 : 0x00) |
		(m_shift_reg & 0x2000 ? 0x40 : 0x00) |
		(m_shift_reg & 0x0800 ? 0x20 : 0x00) |
		(m_shift_reg & 0x0200 ? 0x10 : 0x00) |
		(m_shift_reg & 0x0080 ? 0x08 : 0x00) |
		(m_shift_reg & 0x0020 ? 0x04 : 0x00) |
		(m_shift_reg & 0x0008 ? 0x02 : 0x00) |
		(m_shift_reg & 0x0002 ? 0x01 : 0x00);
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

		case S_WAIT_HEADER_SYNC: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_shift_reg == 0xaaaa) {
				m_crc = 0xffff;
				m_data_separator_phase = false;
				LOGSTATE("wait_header_sync, reset crc\n");
			}
			if(m_data_reg == m_data && clk_bits() == m_clk) {
				m_bit_counter = 0;
				m_state = S_VERIFY_HEADER;
				LOGSTATE("wait_header_sync end (data=0x%02x clk=0x%02x)\n", m_data, m_clk);
			}
			break;
		}

		case S_VERIFY_HEADER: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter & 0xf)
				break;
			LOGSTATE("verify_header, m_bit_counter=0x%x\n", m_bit_counter);
			bool valid = true;
			switch(m_bit_counter >> 4) {
			case 1:
			case 2:
				valid = m_data_reg == m_data && clk_bits() == m_clk;
				LOGSTATE("verify_header, data_reg=0x%02x data=0x%02x clk_bits=0x%02x clk=0x%02x\n", m_data_reg, m_data, clk_bits(), m_clk);
				break;
			case 3:
				valid = m_data_reg == 0xfe;
				LOGSTATE("verify_header, data_reg=0x%x\n", m_data_reg);
				break;
			case 4:
				valid = m_data_reg == m_trck;
				LOGSTATE("verify_header, data_reg=0x%x trck=0x%x\n", m_data_reg, m_trck);
				break;
			case 5:
				// The THMFC1 BIOS always sets side to zero in the sector header.
                                // This is a difference wrt to wd177x MFM/MF track format description
				valid = (m_data_reg & 1) == (m_cmd1 & C1_SIDE ? 1 : 0);
				LOGSTATE("verify_header, data_reg=0x%x side=0x%x\n", m_data_reg, m_cmd1 & C1_SIDE ? 1 : 0);
				break;
			case 6:
				valid = m_data_reg == m_sect;
				LOGSTATE("verify_header, data_reg=0x%x sect=0x%x\n", m_data_reg, m_sect);
				break;
			case 7:
				valid = (m_data_reg & 3) == ((m_cmd1 >> 5) & 3);
				LOGSTATE("verify_header, data_reg=0x%x len=0x%x\n", m_data_reg, (m_cmd1 >> 5) & 3);
				break;
				// 8 skipped
			case 9:
				valid = m_crc == 0;
				m_bit_counter = 0;
				m_state = ((m_cmd0 & 3) == 1) ? S_SKIP_WRITE_GAP:S_SKIP_READ_GAP;
				LOGSTATE("verify_header end\n");
				break;
			}
			if(!valid) {
				m_state = S_WAIT_HEADER_SYNC;
				LOGSTATE("verify_header failed, restart wait_header_sync\n");
			}
			break;
		}

		case S_SKIP_READ_GAP:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter == 27 << 4) {
				m_bit_counter = 0;
				m_state = S_WAIT_READ_SECTOR_SYNC;
				LOGSTATE("skip_read_gap end\n");
			}
			break;

		case S_SKIP_WRITE_GAP:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter == 22 << 4) {
				m_bit_counter = 0;
				m_shift_data_reg = 0x00;
				m_state = S_WAIT_WRITE_SECTOR_SYNC;
				LOGSTATE("skip_write_gap end\n");
			}
			break;

		case S_WAIT_READ_SECTOR_SYNC: {
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_shift_reg == 0xaaaa) {
				m_crc = 0xffff;
				m_data_separator_phase = false;
				LOGSTATE("wait_read_sector_sync, reset crc\n");
			}
			if(m_data_reg == m_data && clk_bits() == m_clk) {
				m_bit_counter = 0;
				m_stat0 |= S0_DREQ;
				LOGSTATE("wait_read_sector_sync end (data=0x%02x clk=0x%02x)\n", m_data, m_clk);
				m_state = S_READ_SECTOR;
			}
			if(m_bit_counter == 42 << 4) {
				m_state = S_WAIT_HEADER_SYNC;
				LOGSTATE("wait_read_sector_sync failed, restart wait_header_sync\n");
			}
			break;
		}

		case S_READ_SECTOR:
			if(read_one_bit(next_sync, next_flux_change))
				return;
			if(m_bit_counter != 16)
				break;
			if(m_stat0 & (S0_BYTE|S0_DREQ)) {
				LOGSTATE("read_sector end\n");
				if(m_crc)
					m_stat0 |= S0_CRCER;
				m_stat0 &= ~S0_BYTE;
				m_stat0 |= S0_FREE;
				m_cmd0 &= ~3;
				m_state = S_IDLE;
				break;
			}
			LOGSTATE("read_sector data_reg=0x%02x crc=0x%04x\n", m_data_reg, m_crc);
			m_stat0 |= S0_BYTE;
			m_data = m_data_reg;
			m_bit_counter = 0;
			break;

		case S_WAIT_WRITE_SECTOR_SYNC:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter == 12 << 4) {
				m_bit_counter = 0;
				m_crc = 0xffff;
				m_shift_clk_reg = m_clk;
				m_shift_data_reg = m_data;
				m_use_shift_clk_reg = true;
				m_stat0 |= (S0_DREQ | S0_BYTE);
				m_state = S_WRITE_SECTOR;
				LOGSTATE("wait_write_sector_sync end, reset crc\n");
			}
			break;

		case S_WRITE_SECTOR:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter != 16)
				break;
			LOGSTATE("write_sector shift_data_reg=0x%02x crc=0x%04x\n", m_shift_data_reg, m_crc);
			if(m_stat0 & S0_DREQ) {
				m_bit_counter = 0;
				m_shift_data_reg = m_crc >> 8;
				m_stat0 &= ~(S0_DREQ | S0_BYTE);
				m_state = S_WRITE_SECTOR_CRC;
				LOGSTATE("write_sector end\n");
				break;
			}
			m_bit_counter = 0;
			m_shift_data_reg = m_data;
			if (m_use_shift_clk_reg && m_data != 0xa1)
				m_use_shift_clk_reg = false;
			m_shift_clk_reg = m_clk;
			m_stat0 |= (S0_BYTE | S0_DREQ);
			break;

		case S_WRITE_SECTOR_CRC:
			if(write_one_bit(next_sync))
				return;
			if(m_bit_counter == 16)
				m_shift_data_reg = m_crc >> 8;
			if(m_bit_counter != 32)
				break;
			LOGSTATE("write_sector_crc end\n");
			m_stat0 |= S0_FREE;
			m_state = S_IDLE;
			break;
		}
}
