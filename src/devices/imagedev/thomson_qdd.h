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
#include "machine/mc6852.h"


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> thomson_qdd_image_device

typedef void (*byte_ready_cb_t)(mc6852_device, u8);

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

	uint8_t ready_r() { return m_ready; }
	uint8_t disk_present_r () { return m_disk_present; }
	uint8_t write_protected_r() { return m_write_protected; }
	void write_gate_w(int value) { m_write_gate = value; }
	void motor_on_w(int value);

	void write(uint8_t data);
	uint8_t read();

protected:
	// device_t implementation
	virtual void device_start() override;
	virtual void device_reset() override;

	TIMER_CALLBACK_MEMBER(byte_timer);

private:
	std::unique_ptr<uint8_t[]> m_track_buffer;

	int m_byte_offset;
	int m_motor_cmd;

	int m_disk_present;
	int m_motor_on;
	int m_ready;
	int m_write_gate;
	int m_write_protected;

	emu_timer *m_byte_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(THOMSON_QDD, thomson_qdd_image_device)

#endif // MAME_DEVICES_IMAGEDEV_THOMSON_QDD_H
