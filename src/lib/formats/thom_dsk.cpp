// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "thom_dsk.h"
#include "ioprocs.h"
#include "osdcore.h"

#define VERBOSE 1

#ifndef LOG_FORMATS
#define LOG_FORMATS(...) do { if (VERBOSE) osd_printf_info(__VA_ARGS__); } while (false)
#endif

thomson_525_fd_format::thomson_525_fd_format() : wd177x_format(formats)
{
}

const char *thomson_525_fd_format::name() const noexcept
{
	return "thomson_525_fd";
}

const char *thomson_525_fd_format::description() const noexcept
{
	return "Thomson 5.25 FD disk image";
}

const char *thomson_525_fd_format::extensions() const noexcept
{
	return "fd";
}

const thomson_525_fd_format::format thomson_525_fd_format::formats[] = {
	{
		floppy_image::FF_525, floppy_image::DSDD, floppy_image::MFM,
		2000,
		16, 40, 2,
		256, {},
		1, {},
		31, 22, 44
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
		floppy_image::FF_525, floppy_image::SSSD, floppy_image::FM,
		4000,
		16, 40, 1,
		128, {},
		1, {},
		17, 22, 12
	},
	{}
};

int thomson_525_fd_format::get_image_offset(const format &f, int head, int track) const
{
	return (track + (head ? f.track_count : 0)) * compute_track_size(f);
}



thomson_35_fd_format::thomson_35_fd_format() : wd177x_format(formats)
{
}

const char *thomson_35_fd_format::name() const noexcept
{
	return "thomson_35_fd";
}

const char *thomson_35_fd_format::description() const noexcept
{
	return "Thomson 3.5 FD disk image";
}

const char *thomson_35_fd_format::extensions() const noexcept
{
	return "fd";
}

// 1280K .fd images exist but are not supported. They represent a notional type
// of 4-sided disk that can be inserted into 2 drives at once.
const thomson_35_fd_format::format thomson_35_fd_format::formats[] = {
	{
		floppy_image::FF_35, floppy_image::DSDD, floppy_image::MFM,
		2000,
		16, 80, 2,
		256, {},
		1, {},
		31, 22, 44
	},
	{
		floppy_image::FF_35, floppy_image::SSDD, floppy_image::MFM,
		2000,
		16, 80, 1,
		256, {},
		1, {},
		31, 22, 44
	},
	{
		floppy_image::FF_35, floppy_image::SSSD, floppy_image::FM,
		4000,
		16, 80, 1,
		128, {},
		1, {},
		17, 12, 22
	},
	{}
};

int thomson_35_fd_format::get_image_offset(const format &f, int head, int track) const
{
	return (track + (head ? f.track_count : 0)) * compute_track_size(f);
}

floppy_image_format_t::desc_e* thomson_35_fd_format::get_desc_fm(const format &f, int &current_size, int &end_gap_index) const
{
	floppy_image_format_t::desc_e *desc = wd177x_format::get_desc_fm(f, current_size, end_gap_index);

	// The format description differs from the wd177x format:
	// the head id is always zero (in field 6)
	desc[6] = { FM, 0x00, 1 };

	return desc;
}

floppy_image_format_t::desc_e* thomson_35_fd_format::get_desc_mfm(const format &f, int &current_size, int &end_gap_index) const
{
	floppy_image_format_t::desc_e *desc = wd177x_format::get_desc_mfm(f, current_size, end_gap_index);

	// The format description differs from the wd177x format:
	// the head id is always zero (in field 7)
	desc[7] = { MFM, 0x00, 1 };

	return desc;
}

#define SAP_SIGNATURE   "SYSTEME D'ARCHIVAGE PUKALL S.A.P. " \
			"(c) Alexandre PUKALL Avril 1998"

#pragma pack(1)

static const int sap_magic_num = 0xB3; /* simple XOR crypt */

struct sap_header
{
	uint8_t version;
	uint8_t signature[65];
};

thomson_sap_format::thomson_sap_format() : wd177x_format(formats)
{
}

const char *thomson_sap_format::name() const noexcept
{
	return "sap";
}

const char *thomson_sap_format::description() const noexcept
{
	return "Thomson SAP disk image";
}

const char *thomson_sap_format::extensions() const noexcept
{
	return "sap";
}

const thomson_sap_format::format thomson_sap_format::formats[] = {
	{
		floppy_image::FF_35, floppy_image::SSDD, floppy_image::MFM,
		2000,
		16, 80, 1,
		256, {},
		1, {},
		31, 22, 44
	},
	{
		floppy_image::FF_525, floppy_image::SSSD, floppy_image::FM,
		4000,
		16, 40, 1,
		128, {},
		1, {},
		17, 22, 12
	},
	{}
};

int thomson_sap_format::identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	struct sap_header header;
	auto const [err, actual] = read_at(io, 0, &header, sizeof(header));
	if (err)
	{
		return 0;
	}
	if (!memcmp(header.signature, SAP_SIGNATURE, 65))
	{
		return FIFID_SIGN;
	}
	return 0;
}

static const uint16_t sap_crc[] = {
	0x0000, 0x1081, 0x2102, 0x3183,   0x4204, 0x5285, 0x6306, 0x7387,
	0x8408, 0x9489, 0xa50a, 0xb58b,   0xc60c, 0xd68d, 0xe70e, 0xf78f,
};

static uint16_t compute_sap_crc(uint8_t* data, int size)
{
	int i;
	uint16_t crc = 0xffff, crc2;
	for (i = 0; i < size; i++)
	{
		crc2 = (crc >> 4) ^ sap_crc[(crc ^ data[i]) & 15];
		crc = (crc2 >> 4) ^ sap_crc[(crc2 ^ (data[i] >> 4)) & 15];
	}
	return crc;
}

bool thomson_sap_format::load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const
{
	struct sap_header sap_hdr;
	uint64_t file_size;
	if (io.length(file_size))
		return false;

	// read header
	read_at(io, 0, &sap_hdr, sizeof(sap_hdr));

	if (sap_hdr.version > 2)
		return false;

	const format &f = formats[sap_hdr.version - 1];
	uint64_t file_offset = sizeof(sap_hdr);

	for(int track=0; track < f.track_count; track++) {
		uint8_t sectdata[40*512];
		desc_s sectors[40];
		floppy_image_format_t::desc_e *desc;
		int current_size;
		int end_gap_index;
		const format &tf = get_track_format(f, 0, track);

		switch (tf.encoding)
		{
		case floppy_image::FM:
			desc = get_desc_fm(tf, current_size, end_gap_index);
			break;
		case floppy_image::MFM:
		default:
			desc = get_desc_mfm(tf, current_size, end_gap_index);
			break;
		}

		int total_size = 200000000/tf.cell_size;
		int remaining_size = total_size - current_size;
		if(remaining_size < 0) {
			return false;
		}

		// Fixup the end gap
		desc[end_gap_index].p2 = remaining_size / 16;
		desc[end_gap_index + 1].p2 = remaining_size & 15;
		desc[end_gap_index + 1].p1 >>= 16-(remaining_size & 15);

		if (tf.encoding == floppy_image::FM)
			desc[14].p1 = get_track_dam_fm(tf, 0, track);
		else
			desc[16].p1 = get_track_dam_mfm(tf, 0, track);

		build_sector_description(tf, sectdata, sectors, track, 0);

		for (int i = 0; i < f.sector_count; i++) {
			uint8_t buffer[262];
			read_at(io, file_offset, buffer, f.sector_base_size + 6);
			// consistency checks
			if (buffer[0]) {
				LOG_FORMATS("thomson_sap_format: format > 0 in sector header at "
					"offset %d (track %d, sector %d)\n", file_offset, track, i + 1);
				return false;
			}
			if (buffer[1]) {
				LOG_FORMATS("thomson_sap_format: protection > 0 in sector header at "
					"offset %d (track %d, sector %d)\n", file_offset + 1, track, i + 1);
				return false;
			}
			if (buffer[2] != track) {
				LOG_FORMATS("thomson_sap_format: mistmatched track number (%d) in sector "
					"header at offset %d (track %d, sector %d)\n",
					buffer[2], file_offset + 2, track, i + 1);
				return false;
			}
			if (buffer[3] != i + 1) {
				LOG_FORMATS("thomson_sap_format: mistmatched sector number (%d) in sector "
					"header at offset %d (track %d, sector %d)\n",
					buffer[3], file_offset + 3, track, i + 1);
				return false;
			}
			// decrypt
			for (int j = 0; j < f.sector_base_size; j++)
				buffer[4 + j] ^= sap_magic_num;
			// check CRC
			uint16_t crc = compute_sap_crc (buffer, f.sector_base_size + 4);
			if (((crc >> 8) != buffer[f.sector_base_size + 4]) ||
				((crc & 0xff) != buffer[f.sector_base_size + 5])) {
				LOG_FORMATS("thomson_sap_format: crc error for sector at offset %d "
					"(track %d, sector %d)\n", file_offset, track, i + 1);
				return false;
			}
			memcpy((void *)sectors[i].data, buffer + 4, f.sector_base_size);
			file_offset += f.sector_base_size + 6;
		}

		generate_track(desc, track, 0, sectors, tf.sector_count, total_size, image);
	}

	image.set_form_variant(f.form_factor, f.variant);

	return true;
}

bool thomson_sap_format::save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const
{
	struct sap_header sap_hdr;
	uint32_t variant = image.get_variant();

	switch (variant)
	{
	case floppy_image::SSDD:
		sap_hdr.version = 1;
		break;
	case floppy_image::DSDD:
		LOG_FORMATS("thomson_sap_format: SAP cannot handle double sided floppy,"
			"only first side will be saved.\n");
		sap_hdr.version = 1;
		break;
	default:
		sap_hdr.version = 2;
	}
	LOG_FORMATS("thomson_sap_format: using SAP version %d for floppy variant %s\n",
		sap_hdr.version, image.get_variant_name(0, variant));

	memcpy(sap_hdr.signature, SAP_SIGNATURE, 65);

	// write header
	write_at(io, 0, &sap_hdr, sizeof(sap_hdr));

	const format &f = formats[sap_hdr.version - 1];
	uint64_t file_offset = sizeof(sap_hdr);

	uint8_t sectdata[40*512];
	desc_s sectors[40];

	for(int track=0; track < f.track_count; track++) {
		const format &tf = get_track_format(f, 0, track);
		build_sector_description(tf, sectdata, sectors, track, 0);
		extract_sectors(image, tf, sectors, track, 0);

		for (int i = 0; i < f.sector_count; i++) {
			uint8_t buffer[262];
			buffer[0] = 0;
			buffer[1] = 0;
			buffer[2] = track;
			buffer[3] = i + 1;
			memcpy(buffer + 4, (void *)sectors[i].data, f.sector_base_size);
			// compute CRC
			uint16_t crc = compute_sap_crc (buffer, f.sector_base_size + 4);
			buffer[f.sector_base_size + 4] = crc >> 8;
			buffer[f.sector_base_size + 5] = crc & 0xff;
			// encrypt
			for (int j = 0; j < f.sector_base_size; j++)
				buffer[4 + j] ^= sap_magic_num;
			write_at(io, file_offset, buffer, f.sector_base_size + 6);
			file_offset += f.sector_base_size + 6;
		}
	}

	return true;
}

const thomson_525_fd_format FLOPPY_THOMSON_525_FD_FORMAT;
const thomson_35_fd_format FLOPPY_THOMSON_35_FD_FORMAT;
const thomson_sap_format FLOPPY_THOMSON_SAP_FORMAT;
