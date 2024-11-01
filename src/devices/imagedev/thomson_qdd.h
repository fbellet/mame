// license:BSD-3-Clause
// copyright-holders:Fabrice Bellet
/*********************************************************************

    thomson_qdd.h

    MAME interface to the Thomson Quick Disk Drive image abstraction code

*********************************************************************/

#ifndef MAME_DEVICES_IMAGEDEV_THOMSON_QDD_H
#define MAME_DEVICES_IMAGEDEV_THOMSON_QDD_H

#pragma once

#include "magtape.h"


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> thomson_qdd_image_device

class thomson_qdd_image_device : public microtape_image_device
{
public:
	// construction/destruction
	thomson_qdd_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	virtual ~thomson_qdd_image_device();

	// device_image_interface implementation
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual void call_unload() override;

	virtual const char *image_type_name() const noexcept override { return "quickdisk"; }
	virtual const char *image_brief_type_name() const noexcept override { return "qdd"; }
	virtual const char *image_interface() const noexcept override { return "qdd"; }
	virtual const char *file_extensions() const noexcept override { return "qd"; }

	uint8_t ry_r() { return m_ry; }
	uint8_t ms_r () { return m_ms; }
	uint8_t wp_r() { return m_wp; }
	void wg_w(int state) { m_wg = state; }
	void mo_w(int state);
	attotime byte_timer_start() { return m_byte_timer->start(); }
	attotime byte_timer_expire() { return m_byte_timer->expire(); }

	void write(uint8_t data);
	uint8_t read();

protected:
	// device_t implementation
	virtual void device_start() override;
	virtual void device_reset() override;
	void save();

	TIMER_CALLBACK_MEMBER(byte_timer);

private:
	std::unique_ptr<uint8_t[]> m_track_buffer;
	int m_byte_offset;

	int m_ms;	// Media sense (active low)
	int m_mo;	// Motor on (active low)
	int m_wg;	// Write gate (active high)
	int m_wp;	// Write protected (active high)
	int m_ry;	// Ready (active low)
	int m_dirty;

	emu_timer *m_byte_timer;
};

// device type definition
DECLARE_DEVICE_TYPE(THOMSON_QDD, thomson_qdd_image_device)

#endif // MAME_DEVICES_IMAGEDEV_THOMSON_QDD_H
