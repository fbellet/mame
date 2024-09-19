// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// CQ 90-028 - QDD drive controller built from a motorola 6852 (serial chip)
//
// Handles a single QDD drive (QD 90-128)

#include "emu.h"
#include "cq90_028.h"

#define VERBOSE 0
#include "logmacro.h"

DEFINE_DEVICE_TYPE(CQ90_028, cq90_028_device, "cq90_028", "Thomson CQ 90-028 QDD controller")

cq90_028_device::cq90_028_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CQ90_028, tag, owner, clock),
	thomson_extension_interface(mconfig, *this),
	m_ssda(*this, "ssda"),
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
	map(0x10, 0x11).rw(m_ssda, FUNC(mc6852_device::read), FUNC(mc6852_device::write));
	map(0x18, 0x18).rw(FUNC(cq90_028_device::status_r), FUNC(cq90_028_device::drive_w));
	map(0x1c, 0x1c).w(FUNC(cq90_028_device::motor_w));
}

const tiny_rom_entry *cq90_028_device::device_rom_region() const
{
	return ROM_NAME(cq90_028);
}

void cq90_028_device::device_add_mconfig(machine_config &config)
{
	MC6852(config, m_ssda, DERIVED_CLOCK(1, 1)); // Comes from the main board
	// Base tx/rx clock is 101564Hz
	// There's probably a pll in the gate array
	//
	// WRPRN signal from the drive is hard wired to the CTSN pin
	// (24) of the SSDA, ensuring that data transmit is inhibited
	// when the floppy is write-protected.
	THOMSON_QDD(config, m_qdd);
}

void cq90_028_device::device_start()
{
}

void cq90_028_device::device_reset()
{
	m_ssda->reset();
	m_ssda->set_data_bus_reversed(true);
	m_qdd->set_ssda(m_ssda);
}

void cq90_028_device::drive_w(u8 data)
{
	LOG("drive_w %02x\n", data);
	m_qdd->set_write_enable (data & 0x80 ? false : true);
}

void cq90_028_device::motor_w(u8 data)
{
	LOG("motor_w %02x\n", data);
}

u8 cq90_028_device::status_r()
{
	// 40 = disk absent
	// 80 = index pulse
	u8 data =  (m_qdd->disk_present() ? 0 : 0x40) | (m_qdd->index() ? 0x80 : 0);
	static u8 prev;
	if (data != prev) {
		prev = data;
		LOG("status_r %02x\n", data);
	}
	return data;
}
