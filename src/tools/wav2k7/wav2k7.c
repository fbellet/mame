/***************************************************************************

    Copyright (C) Fabrice Bellet 2012

    Thomson 8-bit computers

***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#include "corestr.h"
#include "ioprocs.h"
#include "formats/cassimg.h"

#define VERBOSE       0 /* 0, 1 or 2 */

#define LOG(x)	do { if (VERBOSE > 0) printf x; } while (0)
#define VLOG(x)	do { if (VERBOSE > 1) printf x; } while (0)


/*
 * see src/mess/machine/thomson.c for the description of the
 * format
 */
#define TO7_BIT_LENGTH ( 7/6300. )
#define MO5_BIT_LENGTH   0.000833
#define MO5_HBIT_LENGTH (MO5_BIT_LENGTH / 2.)

static void usage(void)
{
	fprintf( stderr, "Usage:\n");
	fprintf( stderr, "	wav2k7 <format> <inputfile.wav> <outputfile>\n");
	fprintf( stderr, "	with format=to7 or mo5\n");
}

#define same_sign(x,y)  (((x) >= 0 && (y) >= 0 ) || ((x) < 0 && (y) < 0))

static double mo5_next_sign_change (cassette_image::ptr &cassette, double pos, double range)
{
	cassette_image::Info info = cassette->get_info( );
	double len = info.sample_count / info.sample_frequency;
	int8_t cur_data, next_data;
	double step = MO5_HBIT_LENGTH / 10;
	double pos1;

	pos1 = pos;
	if ( pos1 < len ) {
	    cassette->get_samples(0, pos1, MO5_BIT_LENGTH, 1, 1, &cur_data, 0);
	    pos1 += step;
	}
	if ( pos1 < len && (pos1 - pos) < range)
	    cassette->get_samples(0, pos1, MO5_BIT_LENGTH, 1, 1, &next_data, 0);
	while ( pos1 + step < len &&
		(pos1 - pos) < range &&
		same_sign (cur_data, next_data)) {
	    cur_data = next_data;
	    pos1 += step;
	    cassette->get_samples(0, pos1, MO5_BIT_LENGTH, 1, 1, &next_data, 0);
	}
	if (pos1 + step >= len) {
	    /* end of stream detection */
	    VLOG (("mo5_next_sign_change(): end of stream reached %f-%f\n", pos, len));
	    return len;
	}
	if (pos1 - pos >= range)
	    VLOG (("mo5_next_sign_change(): no sign change in range %f-%f\n", pos, pos1));
	else
	    VLOG (("mo5_next_sign_change(): sign change at pos=%f\n", pos1));
	return pos1;
}

static void mo5_next_bit (cassette_image::ptr &cassette, double *pos, uint8_t *bit)
{
	cassette_image::Info info = cassette->get_info( );
	double len = info.sample_count / info.sample_frequency;
	double pos1, pos2, pos3;
	double period;
	uint8_t b;
	double next_pos;
	static int c=0;

#define is_hbit(v) ((v) > 0.8 * MO5_HBIT_LENGTH)
#define is_bit(v) ((v) > 0.8 * MO5_BIT_LENGTH)

	c++;
	pos1 = *pos;
	if ( pos1 >= len )
	    return;
	pos2 = mo5_next_sign_change (cassette, pos1, 1.2 * MO5_BIT_LENGTH);
	period = pos2 - pos1;
	if ( period > 0.8 * MO5_BIT_LENGTH ) {
	    b = 0;
	    next_pos = pos2;
	} else if ( period > 0.8 * MO5_HBIT_LENGTH ) {
	    if ( pos2 >= len )
		return;
	    pos3 = mo5_next_sign_change (cassette, pos2, 1.2 * MO5_HBIT_LENGTH);
	    b = 1;
	    next_pos = pos3;
	} else
	    return;
	*bit = b;
	*pos = next_pos;
	VLOG (("mo5_next_bit(): call #%d bit=%d pos=%f\n", c,b,next_pos));
}

static int mo5_next_byte (cassette_image::ptr &cassette, double *pos, uint8_t *byte)
{
	cassette_image::Info info = cassette->get_info( );
	double len = info.sample_count / info.sample_frequency;
	uint8_t b = 0;
	uint8_t bit = 0;
	int i;
	int found = 0;

	for ( i=0; i<8 && *pos < len; i++) {
	    mo5_next_bit (cassette, pos, &bit);
	    if ( bit )
		    b |= ( bit << ( 7 - i) );
	}
	if ( i == 8 ) {
	    *byte = b;
	    found = 1;
	    VLOG (("mo5_next_byte(): %02x\n", b));
	}
	return found;
}

static int do_convert_mo5 (cassette_image::ptr &cassette, FILE *f)
{
	cassette_image::Info info = cassette->get_info( );
	double len = info.sample_count / info.sample_frequency;
	uint8_t data[ 260 ];
	uint8_t b = 0;
	uint8_t crc;
	int size;
	double pos = 0;
	int valid = 1;
	int found = 0;
	int i, j;

	if (pos < len)
	    pos = mo5_next_sign_change (cassette, pos, len - pos);

	while (pos < len) {

	    /* skip zero fillers */
	    mo5_next_bit (cassette, &pos, &b);
	    while (pos < len && b == 0) {
		mo5_next_bit (cassette, &pos, &b);
	    }

	    if (pos >= len)
		break;

	    /* when the first bit=1 is seen, we consider this
	     * is the last bit of the first 0x01 byte, and
	     * we sync from there
	     */
	    if (pos < len && b == 1) {
		found = 1;
		data[ 0 ] = 0x01;
	    }

	    while (found && data[ 0 ] == 0x01) {
		fputc (data[ 0 ], f);
		VLOG (("pos=%f: %02x\n", pos, data[ 0 ]));
		found = mo5_next_byte (cassette, &pos, &data[ 0 ]);
	    }

	    for ( i = 1; found && i < 4 ; i++ ) {
		found = mo5_next_byte (cassette, &pos, &data[ i ]);
	    }

	    if ( found ) {
		if ( data[ 0 ] != 0x3c || data[ 1 ] != 0x5a )
		    valid = 0;
		if ( data[ 2 ] != 0x00 && data[ 2 ] != 0x01 && data[ 2 ] != 0xff )
		    valid = 0;
		if (!valid) {
		    LOG(("pos=%f: bad header found.\n", pos));
		    return -1;
		}
	    }
	    crc = 0;
	    size = ( data[ 3 ] ? data[ 3 ] : 256 );
	    for ( i = 4 ; found && i < size + 3; i++ ) {
		found = mo5_next_byte (cassette, &pos, &data[ i ]);
		crc += data[ i ];
	    }
	    valid = (found && !crc);
	    if (!found)
		LOG(("pos=%f: incomplete last byte.\n", pos));
	    if (crc)
		LOG(("pos=%f: wrong crc.\n", pos));
	    if (!valid)
		return -1;

	    LOG (("pos=%f: bloc=", pos));
	    for ( j = 0; j < i ; j++ ) {
		fputc (data[ j ], f);
		LOG (("%02x ", data[ j ]));
	    }
	    LOG (("\n"));

	    if (pos < len)
		pos = mo5_next_sign_change (cassette, pos, len - pos);
	}
	return 0;
}

static void do_convert_to7 (cassette_image::ptr &cassette, FILE *f)
{
	cassette_image::Info info = cassette->get_info( );
	double len = info.sample_count / info.sample_frequency;
	uint8_t   to7_k7_byte = 0;
	int to7_k7_bitpos = 0;
	int i, chg, bit, synced = 0;
	int8_t data[38];
	double pos = 0;
	double offset = 0;
	/* step when looking for the synchro bit 0 */
	double step = TO7_BIT_LENGTH / 10;

	/* initial padding */
	for ( i = 0; i < 10 ; i++ ) {
		fputc (0x04, f);
		LOG(("04\n"));
	}

	for (; pos + offset < len ;) {
		cassette->get_samples(0, pos + offset,
			TO7_BIT_LENGTH * 38. / 35., 38, 1, data, 0);
		for ( i = 1, chg = 0; i < 38; i++ ) {
			if ( data[ i - 1 ] >= 0 && data[ i ] < 0 )
				chg++;
			if ( data[ i - 1 ] <= 0 && data[ i ] > 0 )
				chg++;
		}
		bit = (chg >= 13 ? 1 : 0);
		to7_k7_bitpos = offset / TO7_BIT_LENGTH;
		LOG (("pos=%f samppos=%d bit=%d (%d)\n",
			pos + offset, to7_k7_bitpos, bit, chg));

		if ( !synced ) {
		    /* when not in sync, looking for the next
		     * 0 bit, progressing step by step
		     */
		    if ( bit )
			offset += step;
		    else {
			/* we found the 0 start bit
			 * changing state to synchronized and
			 * realigning position index
			 */
			synced = 1;
			to7_k7_byte = 0;
			pos = pos + offset + ( TO7_BIT_LENGTH / 2);
			offset = 0.;
		    }
		} else {
		    /* when in sync, progress by step of size
		     * TO7_BIT_LENGTH
		     */
		    if ( to7_k7_bitpos == 0 ) {
			/* this bit is the 0 start bit, skip it */
			offset += TO7_BIT_LENGTH;
			continue;
		    } else if ( to7_k7_bitpos >=1 && to7_k7_bitpos <= 8 ) {
			/* this bit is part of the data */
			if ( bit )
			    to7_k7_byte |= ( bit << ( to7_k7_bitpos - 1) );
			offset += TO7_BIT_LENGTH;
		    } else if ( to7_k7_bitpos <= 11 ) {
			/* we skip the next two 1 stop bits and
			 * expect the next 0 start bit
			 */
			if ( !bit ) {
			    fputc (to7_k7_byte, f);
			    LOG (("%02x\n", to7_k7_byte));
			    to7_k7_byte = 0;
			    pos = pos + offset + ( TO7_BIT_LENGTH / 2 );
			    offset = 0;
			} else
			    offset += step;
		    } else {
			/* no 0 start bit seen for too long
			 * changing state to unsynchronized
			 */
			fputc (to7_k7_byte, f);
			LOG (("%02x\n", to7_k7_byte));
			to7_k7_byte = 0;
			pos = pos + offset;
			offset = 0;
			synced = 0;
		    }
		}

	}
	/* flush */
	if ( synced ) {
	    if ( to7_k7_bitpos ) {
		fputc (to7_k7_byte, f);
		LOG (("%02x\n", to7_k7_byte));
	    }
	}
}
int main(int argc, char *argv[])
{
	cassette_image::ptr cassette;
	const struct cassette_image::Format * const *formats = cassette_default_formats;
	FILE *fin, *fout;
	int ret = 0;

	if ( argc != 4 ) {
		usage();
		return 0;
	}
	fin = fopen( argv[2], "rb" );
	if ( !fin ) {
		fprintf(stderr, "File %s not found.\n",argv[1]);
		return -1;
	}

	auto io = util::stdio_read_write( fin, 0x00 );
	fin = nullptr;
	if ( !io ) {
		fprintf(stderr, "Out of memory.\n");
		return -1;
	}

	if ( cassette_image::open_choices( std::move(io), "wav", formats,
		cassette_image::FLAG_READONLY, cassette) !=
		cassette_image::error::SUCCESS ) {
		fprintf(stderr, "Invalid format of input file.\n");
		fclose(fin);
		return -1;
	}

	fout = fopen( argv[3], "w" );
	if ( !fout ) {
		fprintf(stderr, "File %s not found be opened.\n",argv[2]);
		fclose(fin);
		return -1;
	}

	if ( !strcmp( argv[1], "to7" ))
		do_convert_to7 ( cassette, fout );
	else if (!strcmp( argv[1], "mo5" ))
		ret = do_convert_mo5 ( cassette, fout );
	else
		usage();
	cassette.reset();
	fclose( fin );
	fclose( fout );
	return ret;
}
