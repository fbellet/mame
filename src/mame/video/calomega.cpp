// license:BSD-3-Clause
// copyright-holders: Roberto Fresca
/***********************************************

    .-----------------------------------------.
    |                                         |
    |         CAL OMEGA / CEI / UCMC          |
    |    SYSTEMS 903 / 904 / 905 / 906-III    |
    |                                         |
    |        Driver by Roberto Fresca.        |
    |                                         |
    '-----------------------------------------'

               * Video Hardware *

************************************************/


#include "emu.h"
#include "includes/calomega.h"


void calomega_state::calomega_videoram_w(offs_t offset, uint8_t data)
{
	m_videoram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}

void calomega_state::calomega_colorram_w(offs_t offset, uint8_t data)
{
	m_colorram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}

TILE_GET_INFO_MEMBER(calomega_state::get_bg_tile_info)
{
/*  - bits -
    7654 3210
    --xx xx--   tiles color.
    ---- --x-   tiles bank.
    x--- ---x   extended tiles addressing.
    -x-- ----   seems unused. */

	int attr = m_colorram[tile_index];
	int code = ((attr & 1) << 8) | m_videoram[tile_index];  // bit 0 extends the the tiles addressing
	int bank = (attr & 0x02) >> 1;                          // bit 1 switch the gfx banks
	int color = (attr & 0x3c) >> 2;                         // bits 2-3-4-5 for color

	tileinfo.set(bank, code, color, 0);
}

void calomega_state::video_start()
{
	m_gfxdecode->gfx(0)->set_granularity(8);
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(*this, FUNC(calomega_state::get_bg_tile_info)), TILEMAP_SCAN_ROWS, 8, 8, 32, 31);
}

uint32_t calomega_state::screen_update_calomega(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}

void calomega_state::calomega_palette(palette_device &palette) const
{
/*  the proms are 256x4 bit, but the games only seem to need the first 128 entries,
    and the rest of the PROM data looks like junk rather than valid colors

    prom bits
    3210
    ---x   red component
    --x-   green component
    -x--   blue component
    x---   foreground (colors with this bit set are full brightness,
           colors with it clear are attenuated by the background color pots)
*/

    // TODO: hook pots up as PORT_ADJUSTERs instead of hard coding them here

    // let's make the BG a little darker than FG blue
	constexpr int r_pot = 0x00;
	constexpr int g_pot = 0x00;
	constexpr int b_pot = 0xc0;

    // 00000BGR
	uint8_t const *const color_prom = memregion("proms")->base();
	if (!color_prom)
		return;

	for (int i = 0; i < palette.entries(); i++)
	{
		int const nibble = color_prom[i];

		int const fg = BIT(nibble, 3);

        // red component
		int const r = BIT(nibble, 0) * (fg ? 0xff : r_pot);

        // green component
		int const g = BIT(nibble, 1) * (fg ? 0xff : g_pot);

        // blue component
		int const b = BIT(nibble, 2) * (fg ? 0xff : b_pot);

		palette.set_pen_color(i, rgb_t(r, g, b));
	}
}
