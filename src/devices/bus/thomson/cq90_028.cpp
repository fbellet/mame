// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// CQ 90-028 - QDD drive controller built from a motorola 6852 (serial chip)
//
// Handles a single QDD drive (QD 90-128)

#include "emu.h"
#include "cq90_028.h"

#define VERBOSE 0

#include "logmacro.h"

#define QDD_BITRATE		101265

DEFINE_DEVICE_TYPE(CQ90_028, cq90_028_device, "cq90_028", "Thomson CQ 90-028 Quick Disk Drive Controller")

cq90_028_device::cq90_028_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CQ90_028, tag, owner, clock),
	thomson_extension_interface(mconfig, *this),
	m_mc6852(*this, "mc6852"),
	m_qdd(*this, "qdd"),
	m_rom(*this, "rom")
{
}

ROM_START(cq90_028)
	ROM_REGION( 0x7c0, "rom", 0 )
	ROM_LOAD ( "cq90-028.rom", 0x000, 0x7c0, CRC(ca4dba3d) SHA1(949c1f777c892da62c242215d79757d61e71e62b) )
ROM_END

void cq90_028_device::rom_map(address_map &map)
{
	map(0x000, 0x7bf).rom().region(m_rom, 0);
}

void cq90_028_device::io_map(address_map &map)
{
	map(0x10, 0x11).rw(m_mc6852, FUNC(mc6852_device::read), FUNC(mc6852_device::write));
	map(0x18, 0x18).rw(FUNC(cq90_028_device::status_r), FUNC(cq90_028_device::wg_w));
	map(0x1c, 0x1c).w(FUNC(cq90_028_device::reg_w));
}

const tiny_rom_entry *cq90_028_device::device_rom_region() const
{
	return ROM_NAME(cq90_028);
}

void cq90_028_device::device_add_mconfig(machine_config &config)
{
	MC6852(config, m_mc6852, DERIVED_CLOCK(1, 1)); // Comes from the main board
	// Base tx/rx clock is 101564Hz
	// There's probably a pll in the gate array
	THOMSON_QDD(config, m_qdd);
}

void cq90_028_device::device_start()
{
	m_byte_timer = timer_alloc(FUNC(cq90_028_device::byte_timer), this);
	m_byte_timer->adjust(attotime::zero, 0, attotime::from_hz(QDD_BITRATE / 8));

	save_item(NAME(m_wg));
	save_item(NAME(m_reg));
	save_item(NAME(m_status));
}

void cq90_028_device::device_reset()
{
	m_mc6852->reset();
	m_mc6852->set_data_bus_reversed(true);
	m_wg = 0;
	m_reg = 0;
	m_status = 0;
}

void cq90_028_device::wg_w(uint8_t data)
{
	m_wg = data;
	m_qdd->wg_w(data & 0x80 ? 0 : 1);
	LOG("wg_w %02x\n", data);
}

void cq90_028_device::reg_w(uint8_t data)
{
	m_reg = data;
	LOG("reg_w %02x\n", data);
}

uint8_t cq90_028_device::status_r()
{
	// 40 = disk absent
	// 80 = drive not ready
	m_status = 0;
	if(m_qdd->ms_r())
		m_status |= 0x40;
	if(m_qdd->ry_r())
		m_status |= 0x80;

	if(!machine().side_effects_disabled()) {
		static int ps = -1;
		if(m_status != ps)
			LOG("status_r %02x -%s%s\n", m_status,
				m_status & 0x40 ? "" : " disk",
				m_status & 0x80 ? "" : " rdy");
		ps = m_status;
	}
	return m_status;
}

TIMER_CALLBACK_MEMBER(cq90_028_device::byte_timer)
{
	// MTONN is wired to SM/DTRN of mc6852
	uint8_t mo = m_mc6852->sm_dtr_r();
	m_qdd->mo_w(mo);

	// and WRPRN is wired to CTSN of mc6852
	m_mc6852->cts_w(m_qdd->wp_r());

	if(!mo) {
		if(~m_wg & 0x80) {
			int tuf;
			uint8_t data;
			data = m_mc6852->get_tx_byte(&tuf);
			if(!tuf)
				m_qdd->write(data);
		}
		m_mc6852->receive_byte(m_qdd->read());
	}
}
