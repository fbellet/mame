// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "thom_dsk.h"
#include "ioprocs.h"
#include "osdcore.h"

#define VERBOSE 1

#ifndef LOG_FORMATS
#define LOG_FORMATS(...) do { if (VERBOSE) osd_printf_info(__VA_ARGS__); } while (false)
#endif

thomson_fd_format::thomson_fd_format(const format *formats) : wd177x_format(formats)
{
}

bool thomson_fd_format::validate_fat(util::random_read &io, const format &f, int offset) const
{
	// Basic Microsoft FAT representation (track 20, sector 2)
	//   $ff: unallocated
	//   $fe: reserved
	//   <$c0: allocated block
	//   >$c0 and <$c9: last block
	int nblocks = (f.track_count * 2 < f.sector_base_size - 1 ? f.track_count * 2 : f.sector_base_size - 1);
	uint8_t fat[256];
	read_at(io, offset + (20 * f.sector_count + 1) * f.sector_base_size, fat, f.sector_base_size);
	// offset 0 is always zero
	if (fat[0])
		return false;
	 // offsets 41 and 42 are reserved: two blocks for FAT
	if (fat[41] != 0xfe)
		return false;
	if (fat[42] != 0xfe)
		return false;
	for(int i = 1; i <= nblocks; i++) {
		if (fat[i] > 0xc8 && fat[i] < 0xfe)
			return false;
		// - value $c0 is not possible (the last block contains at least one sector)
		// - value should be bound by the number of blocks
		if (fat[i] >= nblocks && fat[i] < 0xc1)
			return false;
	}
	return true;
}

bool thomson_fd_format::validate_catalog(util::random_read &io, const format &f, int offset) const
{
	// Basic Microsoft catalog representation (track 20, sector 3 to 15)
	// A file descriptor uses 32 bytes:
	//  - offset 0: (0 unallocated entry, $20-$7f allocated entry, $ff end of catalog
	//  - offset 0-7: filename, left centered, completed by spaces
	//  - offset 8-10: extension
	//  - offset 11: file type (0, 1, 2, 3)
	//  - offset 12: $ff for ascii, 0 for binary
	//  - offset 13: first block
	//  - offset 14-15: number of bytes in the last sector
	//  - offset 16-23: comments
	//  - offset 24-31: reserved

	int nblocks = (f.track_count * 2 < f.sector_base_size - 1 ? f.track_count * 2 : f.sector_base_size - 1);
	bool end_of_catalog = false;
	for(int sect=2; sect < 16; sect++) {
		uint8_t cat[256];
		read_at(io, offset + (20 * f.sector_count + sect) * f.sector_base_size, cat, f.sector_base_size);
		for(int i=0; i < f.sector_base_size; i += 32) {
			bool end_of_filename = false;
			bool end_of_suffix = false;
			if (end_of_catalog) {
				if (cat[i] < 0xff)
					return false;
				continue;
			}
			if (cat[i] == 0)
				continue;
			if (cat[i] == 0xff) {
				end_of_catalog = true;
				continue;
			}
			for(int j=i; j < i+8; j++) {
				if (cat[j] < 0x20 || cat[j] > 0x7f)
					return false;
				if (end_of_filename && cat[j] != 0x20)
					return false;
				end_of_filename = (cat[j] == 0x20);
			}
			for(int j=i+8; j < i+11; j++) {
				if (cat[j] < 0x20 || cat[j] > 0x7f)
					return false;
				if (end_of_suffix && cat[j] != 0x20)
					return false;
				end_of_suffix = (cat[j] == 0x20);
			}
			if (cat[i + 11] > 3)
				return false;
			if (cat[i + 12] > 0 && cat[i + 12] < 0xff)
				return false;
			if (cat[i + 13] >= nblocks)
				return false;
		}
	}
	return true;
}

int thomson_fd_format::find_size(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	uint64_t size;
	if(io.length(size))
		return -1;

	for(int i=0; formats[i].form_factor; i++) {
		const format &f = get_track_format(formats[i], 0, 0);;
		int head;
		int track_size = 0;
		for(head=0; head < f.head_count; head++) {
			const format &tf = get_track_format(formats[i], head, 0);;
			if(form_factor != floppy_image::FF_UNKNOWN && form_factor != tf.form_factor)
				break;;
			if(!variants.empty() && !has_variant(variants, tf.variant))
				break;
			if(!validate_fat(io, tf, track_size))
				break;
			if(!validate_catalog(io, tf, track_size))
				break;
			track_size += (tf.track_count * tf.sector_base_size * tf.sector_count);
		}
		if(head < f.head_count)
			continue;
		if(size != track_size)
			continue;
		for(int head=0; head < f.head_count; head++) {
			const format &tf = get_track_format(formats[i], head, 0);;
			LOG_FORMATS("find_size: identified %s, %s, %s for head %d with size %d\n",
					floppy_image::get_form_factor_name(tf.form_factor),
					tf.variant == floppy_image::SSDD || tf.variant == floppy_image::DSDD ? "DD" : "SD",
					floppy_image::get_encoding_name(tf.encoding), head,
					tf.track_count * tf.sector_base_size * tf.sector_count);
		}
		return i;
	}
	LOG_FORMATS("find_size: unidentified for size %d\n", size);
	return -1;
}

int thomson_fd_format::identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	int const type = find_size(io, form_factor, variants);

	if(type != -1)
		return FIFID_SIZE | FIFID_STRUCT;

	return 0;
}

int thomson_fd_format::get_image_offset(const format &f, int head, int track) const
{
	int offset = 0;

	for(int hd=0; hd < head; hd++) {
		for(int trk=0; trk < f.track_count; trk++) {
			const format &tf = get_track_format(f, hd, trk);
			offset += compute_track_size(tf);
		}
	}

	for(int trk=0; trk < track; trk++) {
		const format &tf = get_track_format(f, head, trk);
		offset += compute_track_size(tf);
	}

	return offset;
}

const wd177x_format::format &thomson_fd_format::get_track_format(const format &f, int head, int track) const
{
	int n = -1;
	for(int i = 0; formats[i].form_factor; i++) {
		if(&formats[i] == &f) {
			n = i;
			break;
		}
	}
	if(n < 0) {
		LOG_FORMATS("Error format not found\n");
		return f;
	}
	if(head >= f.head_count) {
		LOG_FORMATS("Error invalid head %d\n", head);
		return f;
	}
	if(track >= f.track_count) {
		LOG_FORMATS("Error invalid track %d\n", track);
		return f;
	}
	if(n==1 && head==1) // DS, DD on side 0, SD on side 1
		n=3;
	if(n==4 && head==1) // DS, SD on side 0, DD on side 1
		n=0;
	return formats[n];
}

floppy_image_format_t::desc_e* thomson_fd_format::get_desc_fm(const format &f, int &current_size, int &end_gap_index) const
{
	floppy_image_format_t::desc_e *desc = wd177x_format::get_desc_fm(f, current_size, end_gap_index);

	// The format description differs from the wd177x format:
	// the head id is always zero (in field 6)
	desc[6] = { FM, 0x00, 1 };

	return desc;
}

floppy_image_format_t::desc_e* thomson_fd_format::get_desc_mfm(const format &f, int &current_size, int &end_gap_index) const
{
	floppy_image_format_t::desc_e *desc = wd177x_format::get_desc_mfm(f, current_size, end_gap_index);

	// The format description differs from the wd177x format:
	// the head id is always zero (in field 7)
	desc[7] = { MFM, 0x00, 1 };

	return desc;
}

thomson_525_fd_format::thomson_525_fd_format() : thomson_fd_format(formats)
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
	{	// DS, DD on side 0, SD on side 1
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
		floppy_image::FF_525, floppy_image::DSSD, floppy_image::FM,
		4000,
		16, 40, 2,
		128, {},
		1, {},
		27, 11, 27
	},
	{	// DS, SD on side 0, DD on side 1
		floppy_image::FF_525, floppy_image::DSSD, floppy_image::FM,
		4000,
		16, 40, 2,
		128, {},
		1, {},
		27, 11, 27
	},
	{
		floppy_image::FF_525, floppy_image::SSSD, floppy_image::FM,
		4000,
		16, 40, 1,
		128, {},
		1, {},
		27, 11, 27
	},
	{}
};

thomson_35_fd_format::thomson_35_fd_format() : thomson_fd_format(formats)
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
	{	// DS, DD on side 0, SD on side 1
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
	{	// DS, SD on side 0, DD on side 1
		floppy_image::FF_35, floppy_image::DSSD, floppy_image::FM,
		4000,
		16, 80, 2,
		128, {},
		1, {},
		17, 12, 22
	},
	{
		floppy_image::FF_35, floppy_image::DSSD, floppy_image::FM,
		4000,
		16, 80, 2,
		128, {},
		1, {},
		17, 12, 22
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
		27, 11, 27
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
	for(i = 0; i < size; i++)
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

		for(int i = 0; i < f.sector_count; i++) {
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
			for(int j = 0; j < f.sector_base_size; j++)
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
		sap_hdr.version, image.get_variant_name(variant));

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

		for(int i = 0; i < f.sector_count; i++) {
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
			for(int j = 0; j < f.sector_base_size; j++)
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
