// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// CQ 90-028 - QDD drive controller built from a motorola 6852 (serial chip)
//
// Handles a single QDD drive (QD 90-128)

#ifndef MAME_BUS_THOMSON_CQ90_028_H
#define MAME_BUS_THOMSON_CQ90_028_H

#include "extension.h"
#include "machine/mc6852.h"
#include "imagedev/thomson_qdd.h"

class cq90_028_device : public device_t, public thomson_extension_interface
{
public:
	cq90_028_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~cq90_028_device() = default;

	static constexpr feature_type unemulated_features() { return feature::DISK; }

	virtual void rom_map(address_map &map) override ATTR_COLD;
	virtual void io_map(address_map &map) override ATTR_COLD;

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(byte_timer);

private:
	required_device<mc6852_device> m_mc6852;
	required_device<thomson_qdd_image_device> m_qdd;
	required_memory_region m_rom;

	void wg_w(uint8_t data);
	void reg_w(uint8_t data);
	uint8_t status_r();

	uint8_t m_wg;
	uint8_t m_reg;
	uint8_t m_status;

	emu_timer *m_byte_timer;
};

DECLARE_DEVICE_TYPE(CQ90_028, cq90_028_device)

#endif
