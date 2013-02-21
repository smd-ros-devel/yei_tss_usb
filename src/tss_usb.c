/***************************************************************************//**
* \file tss_usb.c
*
* \brief Standalone C Driver for the YEI 3-Space Sensor USB
* \author Scott K Logan
* \date January 07, 2013
*
* This is a standolone C driver for the Pololu SMC family of motor
* controllers. It uses LibUSB to interface with the system's USB drivers, and
* is interfaced with similarly to files, in which a device is opened and is
* referenced with an integer handle.
*
* \section license License (BSD-3)
* Copyright (c) 2013, Scott K Logan\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "yei_tss_usb/tss_usb.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

/*!
 * \brief Maximum number of simultaneously open device handles.
 */
#define MAX_HANDLES 256

#define CMD_HEADER 0xF7

#define NO_FLUSH_BUFFER 1

enum tss_usb_commands
{
	CMD_READ_FILTERED_TARED_ORIENTATION_QUATERNION = 0x00,
	CMD_READ_GYROS = 0x21,
	CMD_READ_ACCELEROMETER = 0x22,
	CMD_TARE_WITH_CURRENT_ORIENTATION = 0x60,
	CMD_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION = 0x68,
	CMD_SET_REFERENCE_VECTOR_MODE = 0x69,
	CMD_SET_AXIS_DIRECTIONS = 0x74,
	CMD_RESET_KALMAN_FILTER = 0x78,
	CMD_READ_KALMAN_FILTERS_COVARIANCE_MATRIX = 0x93,
	CMD_READ_VERSION_EXTENDED = 0xDF,
	CMD_RESTORE_FACTORY_SETTINGS = 0xE0,
	CMD_COMMIT_SETTINGS = 0xE1,
	CMD_SOFTWARE_RESET = 0xE2,
	CMD_GET_VERSION = 0xE6,
	CMD_SET_UART_BAUD_RATE = 0xE7,
	CMD_GET_UART_BAUD_RATE = 0xE8,
	CMD_SET_LED_COLOR = 0xEE,
	CMD_GET_LED_COLOR = 0xEF
};

struct tss_usb_priv
{
	int fd;
	int baud;
	char *port;
	char *version;
	char *version_extended;
};

/*!
 * \brief List of communication handles.
 */
static struct tss_usb_priv * tss_usb_list[MAX_HANDLES] = { NULL };

/*!
 * \brief Grabs the next available device handle slot.
 *
 * Iterates through the ::MAX_HANDLES slots for the lowest available index.
 *
 * \returns Open slot index between 0 and ::MAX_HANDLES
 * \retval -1 No available slots
 */
static int next_available_handle( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( !tss_usb_list[i] )
			return i;
	}
	return -1;
}

//This is a convenience function to calc the last byte in the packet
// commands without parameters can use the same number as the command
static unsigned char create_checksum( const unsigned char *command_bytes,
        const unsigned int num_bytes )
{
	unsigned int chkSum = 0;
	unsigned int i;
	for( i = 0; i < num_bytes; i++ )
		chkSum += command_bytes[i];
	return (unsigned char)(chkSum % 256);
}

//The 3-Space sensors are Big Endian and x86 is Little Endian
//So the bytes need be swapped around, this function can convert from and
// to big endian
static inline void endian_swap( unsigned int *x )
{
	*x = ( *x >> 24 ) |
		( ( *x << 8 ) & 0x00FF0000 ) |
		( ( *x >> 8 ) & 0x0000FF00 ) |
		( *x << 24 );
}

static int baud2term( int *baud )
{
	switch( *baud )
	{
	case 1200:
		return B1200;
		break;
	case 2400:
		return B2400;
		break;
	case 4800:
		return B4800;
		break;
	case 9600:
		return B9600;
		break;
	case 19200:
		return B19200;
		break;
	case 38400:
		return B38400;
		break;
	case 57600:
		return B57600;
		break;
	case 115200:
		return B115200;
		break;
	case 230400:
		return B230400;
		break;
	case 460800:
		return B460800;
		break;
	case 921600:
		return B921600;
		break;
	default:
		return B0;
		break;
	}
}

static int send_cmd( const int fd, const unsigned char *data, const size_t num_bytes )
{
	int bytes_sent = write( fd, data, num_bytes );
	if( bytes_sent < 0 )
		return TSS_USB_ERROR_IO;
	else if( bytes_sent == 0 )
		return TSS_USB_ERROR_TIMEOUT;
	else if( (unsigned)bytes_sent != num_bytes )
		return TSS_USB_ERROR_IO;
	return TSS_USB_SUCCESS;
}

static int read_data( const int fd, unsigned char *data, size_t num_bytes )
{
	int bytes_recv;
	while( num_bytes )
	{
		bytes_recv = read( fd, data, num_bytes );
		if( bytes_recv < 0 )
			return TSS_USB_ERROR_IO;
		if( bytes_recv == 0 )
			return TSS_USB_ERROR_TIMEOUT;
		num_bytes -= bytes_recv;
	}
	return TSS_USB_SUCCESS;
}

int tss_usb_open( const char *port )
{
	/* Step 1: Make sure the device opens OK */
	int fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
		return TSS_USB_ERROR_NO_DEVICE;

	fcntl( fd, F_SETFL, 0 );

	struct termios options;
	cfmakeraw( &options );
	if( cfsetispeed( &options, B115200 ) < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_IO;
	}
	if( cfsetospeed( &options, B115200 ) < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_IO;
	}
	options.c_cflag &= ~HUPCL;
	options.c_lflag &= ~ICANON;
	options.c_cc[VTIME] = 2;
	options.c_cc[VMIN] = 0;
	if( tcsetattr( fd, TCSANOW, &options ) < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_IO;
	}

	/* Step 2: Set our baud rate to match the device, if necessary */
	unsigned char buf[3] = { CMD_HEADER, CMD_GET_UART_BAUD_RATE, CMD_GET_UART_BAUD_RATE };
	int ret;
	if( ( ret = send_cmd( fd, buf, sizeof( buf ) ) ) < 0 )
	{
		close( fd );
		return ret;
	}
	int dev_baud;
	if( ( ret = read_data( fd, (unsigned char *)&dev_baud, sizeof( int ) ) ) < 0 )
        {
		close( fd );
		return ret;
        }
	endian_swap( (unsigned int *)&dev_baud );
	int new_baud = baud2term( &dev_baud );
	if( new_baud != 115200 )
	{
		if( cfsetispeed( &options, new_baud ) == -1 )
		{
			close( fd );
			return TSS_USB_ERROR_IO;
		}
		if( cfsetospeed( &options, new_baud ) == -1 )
		{
			close( fd );
			return TSS_USB_ERROR_IO;
		}
		if( tcsetattr( fd, TCSANOW, &options ) == -1 )
		{
			close( fd );
			return TSS_USB_ERROR_IO;
		}
	}

	/* Step 3: Allocate a private struct */
	int mydev = next_available_handle( );
	if( mydev < 0 )
	{
		close( fd );
		return TSS_USB_ERROR_NO_MEM;
	}

	tss_usb_list[mydev] = malloc( sizeof( struct tss_usb_priv ) );
	if( !tss_usb_list[mydev] )
	{
		close( fd );
		return TSS_USB_ERROR_NO_MEM;
	}

	memset( tss_usb_list[mydev], 0, sizeof( struct tss_usb_priv ) );

	tss_usb_list[mydev]->port = malloc( strlen( port ) + 1 );
	if( !tss_usb_list[mydev]->port )
	{
		free( tss_usb_list[mydev] );
		close( fd );
		return TSS_USB_ERROR_NO_MEM;
	}
	memcpy( tss_usb_list[mydev]->port, port, strlen( port ) + 1 );
	tss_usb_list[mydev]->fd = fd;
	tss_usb_list[mydev]->baud = dev_baud;

	return TSS_USB_SUCCESS;
}

void tss_usb_close( const int tssd )
{
	if( tssd < 0 || tssd > MAX_HANDLES || !tss_usb_list[tssd] )
		return;

	close( tss_usb_list[tssd]->fd );

	free( tss_usb_list[tssd]->port );
	free( tss_usb_list[tssd]->version );
	free( tss_usb_list[tssd]->version_extended );

	free( tss_usb_list[tssd] );
	tss_usb_list[tssd] = NULL;
}

int tss_get_led( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_GET_LED_COLOR, CMD_GET_LED_COLOR };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* Read Response */
	if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
		return ret;

	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_set_led( const int tssd, const float vals[3] )
{
	/* Construct Packet Payload */
	unsigned char buf[2 + 3 * sizeof( float ) + 1] = { CMD_HEADER, CMD_SET_LED_COLOR };
	memcpy( &buf[2], &vals[0], sizeof( float ) );
	endian_swap( (unsigned int *)&buf[2] );
	memcpy( &buf[2 + sizeof( float )], &vals[1], sizeof( float ) );
	endian_swap( (unsigned int *)&buf[2 + sizeof( float )] );
	memcpy( &buf[2 + 2 * sizeof( float )], &vals[2], sizeof( float ) );
	endian_swap( (unsigned int *)&buf[2 + 2 * sizeof( float )] );
	buf[2 + 3 * sizeof( float )] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_get_orientation_quaternion( const int tssd, float vals[4] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_READ_FILTERED_TARED_ORIENTATION_QUATERNION, CMD_READ_FILTERED_TARED_ORIENTATION_QUATERNION };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* Read Response */
	if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 4 * sizeof( float ) ) ) < 0 )
		return ret;

	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );
	endian_swap( (unsigned int *)&vals[3] );

	return TSS_USB_SUCCESS;
}

int tss_get_filtered_gyro( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_READ_GYROS, CMD_READ_GYROS };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* Read Response */
	if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
		return ret;

	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_get_accel( const int tssd, float vals[3] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_READ_ACCELEROMETER, CMD_READ_ACCELEROMETER };
	int ret;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* Read Response */
	if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 3 * sizeof( float ) ) ) < 0 )
		return ret;

	endian_swap( (unsigned int *)&vals[0] );
	endian_swap( (unsigned int *)&vals[1] );
	endian_swap( (unsigned int *)&vals[2] );

	return TSS_USB_SUCCESS;
}

int tss_get_covariance( const int tssd, float vals[16] )
{
	#ifndef NO_FLUSH_BUFFER
	/* Clear Response Buffer */
	tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
	#endif

	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_READ_KALMAN_FILTERS_COVARIANCE_MATRIX, CMD_READ_KALMAN_FILTERS_COVARIANCE_MATRIX };
	int ret;
	unsigned char i;

	if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
		return ret;

	/* Read Response */
	if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)vals, 9 * sizeof( float ) ) ) < 0 )
		return ret;

	for( i = 0; i < 16; i++ )
		endian_swap( (unsigned int *)&vals[i] );

	return TSS_USB_SUCCESS;
}

int tss_set_axis_directions( const int tssd, const unsigned char val )
{
	/* Construct Packet Payload */
	unsigned char buf[2 + sizeof( unsigned char ) + 1] = { CMD_HEADER, CMD_SET_AXIS_DIRECTIONS };
	memcpy( &buf[2], &val, sizeof( unsigned char ) );
	buf[2 + sizeof( unsigned char )] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_get_version( const int tssd, char vals[33] )
{
	if( !tss_usb_list[tssd]->version )
	{
		#ifndef NO_FLUSH_BUFFER
		/* Clear Response Buffer */
		tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
		#endif

		/* Construct Packet Payload */
		unsigned char buf[3] = { CMD_HEADER, CMD_GET_VERSION, CMD_GET_VERSION };
		int ret;

		if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
			return ret;

		tss_usb_list[tssd]->version = malloc( 33 * sizeof( char ) );
		tss_usb_list[tssd]->version[32] = '\0';

		/* Read Response */
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)tss_usb_list[tssd]->version, 32 * sizeof( char ) ) ) < 0 )
		{
			free( tss_usb_list[tssd]->version );
			tss_usb_list[tssd]->version = NULL;
			return ret;
		}
	}

	memcpy( vals, tss_usb_list[tssd]->version, 33 * sizeof( char ) );

	return TSS_USB_SUCCESS;
}

int tss_get_version_extended( const int tssd, char vals[13] )
{
	if( !tss_usb_list[tssd]->version_extended )
	{
		#ifndef NO_FLUSH_BUFFER
		/* Clear Response Buffer */
		tcflush( tss_usb_list[tssd]->fd, TCIOFLUSH );
		#endif

		/* Construct Packet Payload */
		unsigned char buf[3] = { CMD_HEADER, CMD_READ_VERSION_EXTENDED, CMD_READ_VERSION_EXTENDED };
		int ret;

		if( ( ret = send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) ) ) < 0 )
			return ret;

		tss_usb_list[tssd]->version_extended = malloc( 13 * sizeof( char ) );
		tss_usb_list[tssd]->version_extended[12] = '\0';

		/* Read Response */
		if( ( ret = read_data( tss_usb_list[tssd]->fd, (unsigned char *)tss_usb_list[tssd]->version_extended, 12 * sizeof( char ) ) ) < 0 )
		{
			printf( "Got %s\n", tss_usb_list[tssd]->version_extended );
			free( tss_usb_list[tssd]->version_extended );
			tss_usb_list[tssd]->version_extended = NULL;
			return ret;
		}
	}

	memcpy( vals, tss_usb_list[tssd]->version_extended, 13 * sizeof( char ) );

	return TSS_USB_SUCCESS;
}

int tss_reset_kalman_filter( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_RESET_KALMAN_FILTER, CMD_RESET_KALMAN_FILTER };
	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_tare( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_TARE_WITH_CURRENT_ORIENTATION, CMD_TARE_WITH_CURRENT_ORIENTATION };
	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_commit( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_COMMIT_SETTINGS, CMD_COMMIT_SETTINGS };
	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_reset( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_SOFTWARE_RESET, CMD_SOFTWARE_RESET };
	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_restore_factory_settings( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_RESTORE_FACTORY_SETTINGS, CMD_RESTORE_FACTORY_SETTINGS };
	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_set_multi_reference_vectors( const int tssd )
{
	/* Construct Packet Payload */
	unsigned char buf[3] = { CMD_HEADER, CMD_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION, CMD_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION };
	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

int tss_set_reference_mode( const int tssd, const unsigned char val )
{
	/* Construct Packet Payload */
	unsigned char buf[2 + sizeof( unsigned char ) + 1] = { CMD_HEADER, CMD_SET_REFERENCE_VECTOR_MODE };
	memcpy( &buf[2], &val, sizeof( unsigned char ) );
	buf[2 + sizeof( unsigned char )] = create_checksum( &buf[1], sizeof( buf ) - 2 );

	return send_cmd( tss_usb_list[tssd]->fd, buf, sizeof( buf ) );
}

