/***************************************************************************//**
* \file tss_usb.h
*
* \brief Standalone C Driver for the YEI 3-Space Sensor USB (header)
* \author Scott K Logan
* \date February 08, 2013
*
* API for the standalone C driver
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

#ifndef _tss_usb_h
#define _tss_usb_h

#ifdef __cplusplus
extern "C" {
#endif

enum tss_usb_error
{
	TSS_USB_SUCCESS = 0,
	TSS_USB_ERROR_IO = -1,
	TSS_USB_ERROR_NO_DEVICE = -4,
	TSS_USB_ERROR_NO_MEM = -11,
	TSS_USB_ERROR_OTHER = -13,
	TSS_USB_ERROR_TIMEOUT = -14
};

enum tss_usb_axis_configurations
{
	TSS_USB_AXIS_XYZ = 0,
	TSS_USB_AXIS_XZY = 1,
	TSS_USB_AXIS_YXZ = 2,
	TSS_USB_AXIS_YZX = 3,
	TSS_USB_AXIS_ZXY = 4,
	TSS_USB_AXIS_ZYX = 5
};

enum tss_usb_axis_inversion
{
	TSS_USB_INVERT_X = 32,
	TSS_USB_INVERT_Y = 16,
	TSS_USB_INVERT_Z = 8,
	TSS_USB_INVERT_NONE = 0
};

enum tss_usb_reference_vector_mode
{
	TSS_USB_REFERENCE_SINGLE_STATIC = 0,
	TSS_USB_REFERENCE_SINGLE_AUTO,
	TSS_USB_REFERENCE_SINGLE_AUTO_CONTINUOUS,
	TSS_USB_REFERENCE_MULTI
};

int tss_usb_open( const char *port );
void tss_usb_close( const int tssd );

int tss_get_led( const int tssd, float vals[3] );
int tss_set_led( const int tssd, const float vals[3] );

int tss_get_orientation_quaternion( const int tssd, float vals[4] );
int tss_get_filtered_gyro( const int tssd, float vals[3] );
int tss_get_accel( const int tssd, float vals[3] );
int tss_get_covariance( const int tssd, float vals[16] );
int tss_set_axis_directions( const int tssd, const unsigned char val );
int tss_get_version( const int tssd, char vals[33] );
int tss_get_version_extended( const int tssd, char vals[13] );
int tss_reset_kalman_filter( const int tssd );
int tss_tare( const int tssd );
int tss_commit( const int tssd );
int tss_reset( const int tssd );
int tss_restore_factory_settings( const int tssd );
int tss_set_multi_reference_vectors( const int tssd );
int tss_set_reference_mode( const int tssd, const unsigned char val );
int tss_get_temperature_c( const int tssd, float *val );
int tss_read_compass( const int tssd, float vals[3] );

#ifdef __cplusplus
}
#endif

#endif /* _tss_usb_h */
