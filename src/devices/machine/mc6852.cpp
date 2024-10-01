// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Motorola MC6852 Synchronous Serial Data Adapter emulation

**********************************************************************/

/*

    TODO:

    - FIFO flags
    - receive
    - transmit
    - parity
    - 1-sync-character mode
    - 2-sync-character mode
    - external sync mode
    - interrupts

*/

#include "emu.h"
#include "mc6852.h"

#define LOG_CTRL	(1U << 1) // Show Control Registers operations
#define LOG_STAT	(1U << 2) // Show Status Register queries
#define LOG_TX		(1U << 3) // Show Tx-FIFO operations
#define LOG_RX		(1U << 4) // Show Rx-FIFO operations
#define LOG_SYNC	(1U << 5) // Show Sync Code value

// #define VERBOSE (LOG_STAT|LOG_SYNC|LOG_TX)

#include "logmacro.h"

#define LOGCTRL(...)	LOGMASKED(LOG_CTRL, __VA_ARGS__)
#define LOGSTAT(...)	LOGMASKED(LOG_STAT, __VA_ARGS__)
#define LOGTX(...)	LOGMASKED(LOG_TX, __VA_ARGS__)
#define LOGRX(...)	LOGMASKED(LOG_RX, __VA_ARGS__)
#define LOGSYNC(...)	LOGMASKED(LOG_SYNC, __VA_ARGS__)


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(MC6852, mc6852_device, "mc6852", "Motorola MC6852 SSDA")

#define REVERSE_BYTE(a) \
	(((a) & 0x80 ? 0x01 : 0) | \
	((a) & 0x40 ? 0x02 : 0) | \
	((a) & 0x20 ? 0x04 : 0) | \
	((a) & 0x10 ? 0x08 : 0) | \
	((a) & 0x08 ? 0x10 : 0) | \
	((a) & 0x04 ? 0x20 : 0) | \
	((a) & 0x02 ? 0x40 : 0) | \
	((a) & 0x01 ? 0x80 : 0))


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  mc6852_device - constructor
//-------------------------------------------------

mc6852_device::mc6852_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, MC6852, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	m_write_tx_data(*this),
	m_write_irq(*this),
	m_write_sm_dtr(*this),
	m_write_tuf(*this),
	m_rx_clock(0),
	m_tx_clock(0),
	m_cts(1),
	m_dcd(1),
	m_sm_dtr(1),
	m_tuf(0),
	m_in_sync(0),
	m_data_bus_reversed(false)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mc6852_device::device_start()
{
	set_rcv_rate(m_rx_clock);
	set_tra_rate(m_tx_clock);

	// register for state saving
	save_item(NAME(m_status));
	save_item(NAME(m_cr));
	save_item(NAME(m_scr));
	save_item(NAME(m_tdr));
	save_item(NAME(m_tsr));
	save_item(NAME(m_rdr));
	save_item(NAME(m_rsr));
	save_item(NAME(m_cts));
	save_item(NAME(m_dcd));
	save_item(NAME(m_sm_dtr));
	save_item(NAME(m_tuf));
	save_item(NAME(m_in_sync));
	save_item(NAME(m_data_bus_reversed));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mc6852_device::device_reset()
{
	m_rx_fifo = std::queue<uint8_t>();
	m_tx_fifo = std::queue<uint8_t>();

	receive_register_reset();
	transmit_register_reset();

	/* reset and inhibit receiver/transmitter sections */
	m_cr[0] |= (C1_TX_RS | C1_RX_RS);
	m_cr[1] &= ~(C2_EIE | C2_PC2 | C2_PC1);
	m_status |= S_TDRA;

	/* set receiver shift register to all 1's */
	m_rsr = 0xff;
}


//-------------------------------------------------
//  tra_callback -
//-------------------------------------------------

void mc6852_device::tra_callback()
{
	m_write_tx_data(transmit_register_get_data_bit());
}


//-------------------------------------------------
//  tra_complete -
//-------------------------------------------------

void mc6852_device::tra_complete()
{
	// TODO
}

//-------------------------------------------------
//  receive_byte -
//-------------------------------------------------
void mc6852_device::receive_byte(uint8_t data)
{
	// Ignore if the receiver is in reset or sync is not enabled
	if (m_cr[0] & (C1_RX_RS | C1_CLEAR_SYNC))
		return;

	// Handle sync detection
	if (!m_in_sync)
	{
		// TODO also handle two sync codes.
		if (data == m_scr)
		{
			m_in_sync = 1;
			// TODO handle the various SM responses
			if ((m_cr[1] & (C2_PC2 | C2_PC1)) == 1)
				m_sm_dtr = 1;
		}
		return;
	}

	if ((m_cr[0] & C1_STRIP_SYNC) && (data == m_scr))
		return;

	int size = m_rx_fifo.size();

	if (size < 3)
	{
		LOGRX("MC6852 Push byte 0x%02x to FIFO\n", data);
		m_rx_fifo.push(data);
		size++;
	}
	else
	{
		// Overrun.
		// TODO this should override the last data pushed
		m_status |= S_RX_OVRN;
	}

	int trigger = (m_cr[1] & C2_1_2_BYTE) ? 1 : 2;

	if (size >= trigger)
	{
		m_status |= S_RDA;
	}
}

//-------------------------------------------------
//  rcv_complete -
//-------------------------------------------------

void mc6852_device::rcv_complete()
{
	// TODO
}

//-------------------------------------------------
//  read -
//-------------------------------------------------

// TODO each RX fifo element needs an associated PE status flag, and reading
// the status register should return the PE for the last element of the fifo.

// TODO RX overrun should be cleared by reading the status register followed
// by reading the RX fifo.

uint8_t mc6852_device::read(offs_t offset)
{
	uint8_t data = 0;

	if (BIT(offset, 0))
	{
		int size = m_rx_fifo.size();
		if (size > 0)
		{
			data = m_rx_fifo.front();
			if (!machine().side_effects_disabled())
			{
				m_rx_fifo.pop();
				int trigger = (m_cr[1] & C2_1_2_BYTE) ? 1 : 2;
				if (size <= trigger)
				{
					m_status &= ~S_RDA;
				}
				LOGRX("MC6852 Receive Data FIFO 0x%02x\n", data);
			}
		}
	}
	else
	{
		data = m_status;

		// TS reset inhibits the TDRA status bit (in the
		// one-sync-character and two-sync-character modes) The
		// m_status S_TDRA bit is allowed to reflect the actual fifo
		// availability, masking it here on a read of the status, so
		// that the TDRA status bit is simply unmasked here when the
		// TX is taken out of reset.
		if (m_cr[0] & C1_TX_RS)
		{
			data &= ~S_TDRA;
		}

		static uint8_t prev_data;

		if (data != prev_data) {
			LOGSTAT("MC6852 Status 0x%02x irq=%i pe=%i ovr=%i und=%i cts=%i tr=%i rd=%i\n",
					data,
					data & S_IRQ  ? 1 : 0,
					data & S_PE   ? 1 : 0,
					data & S_RX_OVRN  ? 1 : 0,
					data & S_TUF  ? 1 : 0,
					data & S_CTS ? 1 : 0,
					data & S_TDRA ? 1 : 0,
					data & S_RDA  ? 1 : 0);
			prev_data = data;
		}

		if (!machine().side_effects_disabled())
		{
			// TODO this might not be quite right, the datasheet
			// states that the RX overrun flag is cleared by
			// reading the status, and the RX data fifo?
			m_status &= S_RX_OVRN;
		}

		if (m_data_bus_reversed)
			data = REVERSE_BYTE(data);
	}

	return data;
}

//-------------------------------------------------
//  tx_start -
//-------------------------------------------------

// The corresponds in time to just before the first bit of the next word is
// transmitted by this device. At this time the TX shift register is loaded
// the TUF line may be asserted if there is a TX FIFO underflow.
uint8_t mc6852_device::get_tx_byte(int *tuf)
{
	if (m_cr[0] & C1_TX_RS)
	{
		// FIFO is not popped when the TX is reset, but may be loaded
		// so that it is pre-loaded when the reset is cleared.  But
		// will is send a sync code if that is enabled, of just ones?
		*tuf = 0;
		return 0xff;
	}

	int size = m_tx_fifo.size();

	if (size == 0)
	{
		// TX underflow
		if (m_cr[1] & C2_TX_SYNC)
		{
			m_status |= S_TUF;
			// TODO should the TUF callback be called, TUF is to
			// be pulsed.
			*tuf = 1;
			return m_scr;
		}

		*tuf = 0;
		return 0xff;
	}

	uint8_t data = m_tx_fifo.front();
	m_tx_fifo.pop();
	size--;

	int trigger = (m_cr[1] & C2_1_2_BYTE) ? 1 : 2;
	int available = 3 - size;

	if (available >= trigger)
	{
		m_status |= S_TDRA;
	}

	*tuf = 0;
	return data;
}


//-------------------------------------------------
//  write -
//-------------------------------------------------

void mc6852_device::write(offs_t offset, uint8_t data)
{
	if (BIT(offset, 0))
	{
		switch (m_cr[0] & C1_AC_MASK)
		{
		case C1_AC_C2: {
			if (m_data_bus_reversed)
				data = REVERSE_BYTE(data);
			/* control 2 */
			static const int bit[8] = { 6, 6, 7, 8, 7, 7, 8, 8 };
			static const int par[8] = { 2, 1, 0, 0, 2, 1, 2, 1 };
			static const char *const parname[3] = { "none", "odd", "even" };
			int bits   = bit[ (data >> 3) & 7 ];
			int parval = par[ (data >> 3) & 7 ];
			static const char *const sm_dtr_name[4] = { "1", "pulse", "0", "0" };

			LOGCTRL("MC6852 Control 2 0x%02x bits=%i par=%s blen=%i under=%s%s sm_dtr=%s\n", data,
					bits, parname[parval], data & C2_1_2_BYTE ? 1 : 2,
					data & C2_TX_SYNC ? "sync" : "ff",
					data & C2_EIE ? "irq-err" : "",
					sm_dtr_name[data & (C2_PC1|C2_PC2)]);
			m_cr[1] = data;

			int data_bit_count = 0;
			parity_t parity = PARITY_NONE;
			stop_bits_t stop_bits = STOP_BITS_1;

			switch ((data & C2_WS_MASK) >> 3)
			{
			case 0: data_bit_count = 6; parity = PARITY_EVEN; break;
			case 1: data_bit_count = 6; parity = PARITY_ODD; break;
			case 2: data_bit_count = 7; parity = PARITY_NONE; break;
			case 3: data_bit_count = 8; parity = PARITY_NONE; break;
			case 4: data_bit_count = 7; parity = PARITY_EVEN; break;
			case 5: data_bit_count = 7; parity = PARITY_ODD; break;
			case 6: data_bit_count = 8; parity = PARITY_EVEN; break;
			case 7: data_bit_count = 8; parity = PARITY_ODD; break;
			}

			switch (data & (C2_PC1|C2_PC2))
			{
			case 0: m_sm_dtr = 1; break;
			case 1: m_sm_dtr = (m_in_sync ? 1 : 0); break;
			default: m_sm_dtr = 0; break;
			}

			set_data_frame(1, data_bit_count, parity, stop_bits);

			// The fifo trigger levels may have changed, so update
			// the status bits.

			int trigger = (m_cr[1] & C2_1_2_BYTE) ? 1 : 2;

			if (m_rx_fifo.size() >= trigger)
				m_status |= S_RDA;
			else
				m_status &= ~S_RDA;

			int tx_fifo_available = 3 - m_tx_fifo.size();
			if (tx_fifo_available >= trigger)
				m_status |= S_TDRA;
			else
				m_status &= ~S_TDRA;

			break;
			}

		case C1_AC_C3:
			/* control 3 */
			if (m_data_bus_reversed)
				data = REVERSE_BYTE(data);
			LOGCTRL("MC6852 Control 3 0x%02x %s%ssync-len=%i sync-mode=%s\n",
					data,
					data & C3_CTUF ? "clr-tuf " : "",
					data & C3_CTS ? "clr-cts " : "",
					data & C3_1_2_SYNC ? 1 : 2,
					data & C3_E_I_SYNC ? "ext" : "int" );
			m_cr[2] = data;
			if (m_cr[2] & C3_CTUF)
			{
				m_cr[2] &= ~C3_CTUF;
				m_status &= ~S_TUF;
			}
			if (m_cr[2] & C3_CTS)
			{
				m_cr[2] &= ~C3_CTS;
				if (m_cts)
					m_status |= S_CTS;
				else
					m_status &= ~S_CTS;
			}
			break;

		case C1_AC_SYNC:
			/* sync code */
			LOGSYNC("MC6852 Sync Code 0x%02x\n", data);
			m_scr = data;
			break;

		case C1_AC_TX_FIFO: {
			/* transmit data FIFO */
			int available = 3 - m_tx_fifo.size();
			if (available > 0)
			{
				LOGTX("MC6852 Transmit FIFO 0x%02x\n", data);
				m_tx_fifo.push(data);
				available--;
			}
			else
			{
				LOGTX("MC6852 Transmit FIFO OVERFLOW 0x%02x\n", data);
			}
			int trigger = (m_cr[1] & C2_1_2_BYTE) ? 1 : 2;
			if (available < trigger)
			{
				m_status &= ~S_TDRA;
			}
			break;
			}
		}
	}
	else
	{
		if (m_data_bus_reversed)
			data = REVERSE_BYTE(data);
		LOGCTRL("MC6852 Control 1 0x%02x reset=%c%c %s%sirq=%c%c\n", data,
				data & C1_RX_RS ? 'r' : '-', data & C1_TX_RS ? 't' : '-',
				data & C1_STRIP_SYNC ? "strip-sync " : "",
				data & C1_CLEAR_SYNC ? "clear-sync " : "",
				data & C1_RIE ? 'r' : '-',
				data & C1_TIE ? 't' : '-');

		/* receiver reset */
		if (data & C1_RX_RS)
		{
			/* When Rx Rs is set, it clears the receiver
			control logic, sync logic, error logic, Rx Data FIFO Control,
			Parity Error status bit, and DCD interrupt. The Receiver Shift
			Register is set to ones.
			*/

			m_status &= ~(S_RX_OVRN | S_PE | S_DCD | S_RDA);
			m_rsr = 0xff;
			m_rx_fifo = std::queue<uint8_t>();

			receive_register_reset();
		}

		/* transmitter reset */
		if (data & C1_TX_RS)
		{
			// When Tx Rs is set, it clears the transmitter
			// control section, Transmitter Shift Register, Tx
			// Data FIFO Control (the Tx Data FIFO can be reloaded
			// after one E clock pulse), the Transmitter Underflow
			// status bit, and the CTS interrupt.

			m_status &= ~(S_TUF | S_CTS);
			m_status |= S_TDRA;
			m_tx_fifo = std::queue<uint8_t>();

			transmit_register_reset();
		}

		if (data & C1_CLEAR_SYNC)
		{
			m_in_sync = 0;
			if ((m_cr[1] & (C2_PC1|C2_PC2)) == 1)
				m_sm_dtr = 0;
		}
		m_cr[0] = data;
	}
}
