// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "thom_dsk.h"

thomson_525_format::thomson_525_format() : wd177x_format(formats)
{
}

const char *thomson_525_format::name() const noexcept
{
	return "thomson_525";
}

const char *thomson_525_format::description() const noexcept
{
	return "Thomson 5.25 disk image";
}

const char *thomson_525_format::extensions() const noexcept
{
	return "fd";
}

int thomson_525_format::get_image_offset(const format &f, int head, int track) const
{
        return (f.track_count * head + track) * compute_track_size(f);
}

const thomson_525_format::format thomson_525_format::formats[] = {
	{
		floppy_image::FF_525, floppy_image::SSSD, floppy_image::FM,
		4000,
		16, 40, 1,
		128, {},
		1, {},
		17, 22, 12
	},
	{
		floppy_image::FF_525, floppy_image::SSDD, floppy_image::MFM,
		2000,
		16, 40, 1,
		256, {},
		1, {},
		31, 22, 44
	},
	{
		floppy_image::FF_525, floppy_image::DSDD, floppy_image::MFM,
		2000,
		16, 40, 2,
		256, {},
		1, {},
		31, 22, 44
	},
	{}
};


thomson_35_format::thomson_35_format() : wd177x_format(formats)
{
}

const char *thomson_35_format::name() const noexcept
{
	return "thomson_35";
}

const char *thomson_35_format::description() const noexcept
{
	return "Thomson 3.5 disk image";
}

const char *thomson_35_format::extensions() const noexcept
{
	return "fd";
}

int thomson_35_format::get_image_offset(const format &f, int head, int track) const
{
        return (f.track_count * head + track) * compute_track_size(f);
}

const thomson_35_format::format thomson_35_format::formats[] = {
	{
		floppy_image::FF_35, floppy_image::SSDD, floppy_image::MFM,
		2000,
		16, 80, 1,
		256, {},
		1, {},
		31, 22, 44
	},
	{
		floppy_image::FF_35, floppy_image::DSDD, floppy_image::MFM,
		2000,
		16, 80, 2,
		256, {},
		1, {},
		31, 22, 44
	},
	{}
};

floppy_image_format_t::desc_e* thomson_35_format::get_desc_mfm(const format &f, int &current_size, int &end_gap_index) const
{
	// The format description differs from the wd177x format:
	// the head id is always zero (in field 7)
	static floppy_image_format_t::desc_e desc[25] = {
		/* 00 */ { MFM, 0x4e, 0 },
		/* 01 */ { SECTOR_LOOP_START, 0, 0 },
		/* 02 */ {   MFM, 0x00, 12 },
		/* 03 */ {   CRC_CCITT_START, 1 },
		/* 04 */ {     RAW, 0x4489, 3 },
		/* 05 */ {     MFM, 0xfe, 1 },
		/* 06 */ {     TRACK_ID },
		/* 07 */ {     MFM, 0x00, 1 },
		/* 08 */ {     SECTOR_ID },
		/* 09 */ {     SIZE_ID },
		/* 10 */ {   CRC_END, 1 },
		/* 11 */ {   CRC, 1 },
		/* 12 */ {   MFM, 0x4e, 0 },
		/* 13 */ {   MFM, 0x00, 12 },
		/* 14 */ {   CRC_CCITT_START, 2 },
		/* 15 */ {     RAW, 0x4489, 3 },
		/* 16 */ {     MFM, 0xfb, 1 },
		/* 17 */ {     SECTOR_DATA, -1 },
		/* 18 */ {   CRC_END, 2 },
		/* 19 */ {   CRC, 2 },
		/* 20 */ {   MFM, 0x4e, 0 },
		/* 21 */ { SECTOR_LOOP_END },
		/* 22 */ { MFM, 0x4e, 0 },
		/* 23 */ { RAWBITS, 0x9254, 0 },
		/* 24 */ { END }
	};

	desc[0].p2 = f.gap_1;
	desc[1].p2 = f.sector_count - 1;
	desc[12].p2 = f.gap_2;
	desc[20].p2 = f.gap_3;

	wd177x_format::get_desc_mfm(f, current_size, end_gap_index);

	return desc;
}

const thomson_525_format FLOPPY_THOMSON_525_FORMAT;
const thomson_35_format FLOPPY_THOMSON_35_FORMAT;
