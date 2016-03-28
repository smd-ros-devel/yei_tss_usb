#include "yei_tss_usb/tss_usb.h"

#include <libxml/xmlreader.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static inline int parse_hex( const xmlChar *ch, unsigned char *dest );
static int fw_update( const int tssd, const char *fw, const unsigned int page_size, const unsigned char fake );

int main( int argc, const char *argv[] )
{
	const char *dev;
	const char *fw;
	int ret = -1;
	int tssd = -1;
	unsigned char sernum[4];
	struct tss_bootloader_info bl_info;

	if( argc < 3 )
	{
		printf( "Usage: %s <path to device> <path to firmware>\n", argv[0] );

		exit( EXIT_FAILURE );
	}

	dev = argv[1];
	fw = argv[2];

	tssd = tss_usb_open( dev );
	if( tssd < 0 )
	{
		fprintf( stderr, "Failed to open TSS device (%d)\n", tssd );

		exit( EXIT_FAILURE );
	}

	ret = tss_bootloader_ping( tssd );
	if( ret != 1 )
	{
		if( ret == 0 )
			fprintf( stderr, "Please place device in bootloader and try again\n" );
		else
			fprintf( stderr, "Failed to determine bootloader state (%d)\n", ret );

		goto fail;
	}

	ret = tss_get_serial( tssd, sernum );
	if( ret < 0 )
	{
		fprintf( stderr, "WARNING: Failed to query for device serial number (%d)\n", ret );
	}
	else
	{
		printf( "Device serial number: %02X%02X%02X%02X\n", sernum[0], sernum[1], sernum[2], sernum[3] );
	}

	ret = tss_bootloader_info( tssd, &bl_info );
	if( ret < 0 )
	{
		fprintf( stderr, "ERROR: Failed to get device information from bootloader (%d)\n", ret );

		goto fail;
	}

	printf( "Device page size: %u\n", bl_info.page_size );

	fprintf(stderr, "WARNING: This firmware updater is incomplete! DO NOT USE IT!\n");

	ret = fw_update( tssd, fw, bl_info.page_size, 1 );
	if( ret < 0 )
	{
		fprintf( stderr, "ERROR: Failed to update firmware (%d)\n", ret );

		goto fail;
	}

	tss_usb_close( tssd );

	exit( EXIT_SUCCESS );

fail:
	tss_usb_close( tssd );

	exit( EXIT_FAILURE );
}

static int fw_update( const int tssd, const char *fw, const unsigned int page_size, const unsigned char fake )
{
	xmlTextReaderPtr reader;
	int ret;
	const xmlChar *name, *value;

	reader = xmlReaderForFile( fw, NULL, XML_PARSE_DTDATTR | XML_PARSE_NOENT );
	if( reader == NULL )
	{
		return -1;
	}

	ret = xmlTextReaderRead( reader );
	while( ret == 1 )
	{
		name = xmlTextReaderConstName( reader );

		if (name != NULL)
		{
			if( strcmp( name, "SetAddr" ) == 0 )
			{
				ret = xmlTextReaderHasValue( reader );
				if( ret != 1 )
				{
					ret = xmlTextReaderRead( reader );
					if( ret != 1 )
					{
						ret = -24;

						goto exit;
					}

					ret = xmlTextReaderHasValue( reader );
					if( ret != 1 )
					{
						ret = -25;

						goto exit;
					}
				}

				if( xmlTextReaderNodeType( reader ) == XML_READER_TYPE_TEXT )
				{
					unsigned int set_addr;

					value = xmlTextReaderConstValue( reader );
					if( value == NULL )
					{
						ret = -26;

						goto exit;
					}

					ret = sscanf( value, "%X", &set_addr );
					if( ret != 1 )
					{
						ret = -27;

						goto exit;
					}

					printf( "Setting address to 0x%X\n", set_addr);

					if( fake == 0 )
					{
						ret = tss_bootloader_set_addr( tssd, set_addr );
						if( ret < 0 )
						{
							printf( "ret was %d\n", ret );

							ret = -28;

							goto exit;
						}
						else if( ret > 0 )
						{
							ret = -29;

							goto exit;
						}
					}
				}
			}
			else if( strcmp( name, "MemProg" ) == 0 )
			{
				ret = xmlTextReaderHasValue( reader );
				if( ret != 1 )
				{
					ret = xmlTextReaderRead( reader );
					if( ret != 1 )
					{
						ret = -4;

						goto exit;
					}

					ret = xmlTextReaderHasValue( reader );
					if( ret != 1 )
					{
						ret = -5;

						goto exit;
					}
				}

				if( xmlTextReaderNodeType(reader) == XML_READER_TYPE_TEXT )
				{
					unsigned int chars;
					unsigned int len;
					unsigned char tmp;
					unsigned char *data = NULL;

					value = xmlTextReaderConstValue( reader );
					if( value == NULL )
					{
						ret = -6;

						goto exit;
					}

					chars = xmlStrlen( value );
					if( chars % 2 != 0 )
					{
						ret = -7;

						goto exit;
					}

					len = chars / 2;

					data = malloc( len );
					if( data == NULL )
					{
						ret = -8;

						goto exit;
					}

					for( unsigned char *d = data; chars > 0; chars -= 2, d++, value += 2 )
					{
						if( parse_hex(value, d) < 0 )
						{
							free( data );

							ret = -9;

							goto exit;
						}
					}

					printf( "Writing %u bytes of firmware...", len );

					if( fake == 0 )
					{
						ret = tss_bootloader_mem_prog( tssd, data, len, page_size );
						if( ret != 0 )
						{
							if( ret < 0 )
								printf( "i/o failure.\n" );
							else
								printf( "device reported failure.\n" );

							free( data );

							ret = -10;

							goto exit;
						}
					}

					free( data );

					printf( "done.\n" );
				}
			}
			else if( strcmp( name, "MemProgC" ) == 0 )
			{
				ret = xmlTextReaderHasValue( reader );
				if( ret != 1 )
				{
					ret = xmlTextReaderRead( reader );
					if( ret != 1 )
					{
						ret = -4;

						goto exit;
					}

					ret = xmlTextReaderHasValue( reader );
					if( ret != 1 )
					{
						ret = -5;

						goto exit;
					}
				}

				if( xmlTextReaderNodeType(reader) == XML_READER_TYPE_TEXT )
				{
					unsigned int chars;
					unsigned int len;
					unsigned char tmp;
					unsigned char *data = NULL;

					value = xmlTextReaderConstValue( reader );
					if( value == NULL )
					{
						ret = -6;

						goto exit;
					}

					chars = xmlStrlen( value );
					if( chars % 2 != 0 )
					{
						ret = -7;

						goto exit;
					}

					len = chars / 2;

					data = malloc( len );
					if( data == NULL )
					{
						ret = -8;

						goto exit;
					}

					for( unsigned char *d = data; chars > 0; chars -= 2, d++, value += 2 )
					{
						if( parse_hex(value, d) < 0 )
						{
							free( data );

							ret = -9;

							goto exit;
						}
					}

					printf( "Writing %u bytes of firmware...", len );

					if( fake == 0 )
					{
						ret = tss_bootloader_mem_prog_c( tssd, data, len, page_size );
						if( ret != 0 )
						{
							if( ret < 0 )
								printf( "i/o failure.\n" );
							else
								printf( "device reported failure.\n" );

							free( data );

							ret = -10;

							goto exit;
						}
					}

					free( data );

					printf( "done.\n" );
				}
			}
			else if( strcmp( name, "Finish" ) == 0 )
			{
				if( xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT )
				{
					printf( "Finished writing firmware.\n" );

					if( fake == 0 )
					{
						ret = tss_bootloader_finish( tssd );
						if( ret != 0 )
						{
							ret = ( ret < 0 ) ? -50 : -51;

							goto exit;
						}
					}
				}
			}
			else if( strcmp( name, "Run" ) == 0 )
			{
				if( xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT )
				{
					printf( "Running new firmware.\n" );

					if( fake == 0 )
					{
						ret = tss_bootloader_run( tssd );
						if( ret < 0 )
						{
							ret = -60;

							goto exit;
						}
					}
				}
			}
		}

		ret = xmlTextReaderRead( reader );
	}

	ret = 0;

exit:
	xmlFreeTextReader( reader );

	return ret;
}

static inline int parse_hex( const xmlChar *ch, unsigned char *dest )
{
	const unsigned char conv[104] =
	{
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  0,  0,  0,  0,  0,  0,
		0,  10, 11, 12, 13, 14, 15, 0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0,  10, 11, 12, 13, 14, 15, 0,
	};

	if( *ch < 48 || ( *ch > 57 && *ch < 65 ) || ( *ch > 70 && *ch < 97 ) || *ch > 102)
	{
		return -1;
	}

	*dest = conv[*ch];
	ch++;

	if( *ch < 48 || ( *ch > 57 && *ch < 65 ) || ( *ch > 70 && *ch < 97 ) || *ch > 102)
	{
		return -2;
	}

	*dest = (*dest << 4) | conv[*ch];

	return 0;
}

