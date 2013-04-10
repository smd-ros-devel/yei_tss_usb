#include "yei_tss_usb/tss_usb_driver.hpp"

#include <iomanip>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <ros/time.h>
#include <tf/tf.h>

namespace yei_tss_usb
{
	TSSUSB::TSSUSB( const ros::NodeHandle &_nh,
		const ros::NodeHandle &_nh_priv,
		const std::string _port ) :
		nh( _nh ),
		nh_priv( _nh_priv ),
		port( _port ),
		frame_id( "imu" ),
		tssd( -1 ),
		min_update_rate( 50.0 ),
		max_update_rate( 230.0 ),
		diag_pub_freq( diagnostic_updater::FrequencyStatusParam( &min_update_rate, &max_update_rate, 0.1, 10 ) ),
		io_failure_count( 0 ),
		open_failure_count( 0 ),
		axis_config( TSS_USB_AXIS_YZX ),
		invert_x_axis( false ),
		invert_y_axis( true ),
		invert_z_axis( false ),
		reference_vector_mode( 1 ),
		orientation_covariance( 0.1 ),
		angular_velocity_covariance( 0.1 ),
		linear_acceleration_covariance( 0.1 ),
		grav_vect( 0, 0, GRAVITATIONAL_ACCELERATION ),
		spin_rate( 100 ),
		spin_thread( &TSSUSB::spin, this )
	{
		cmd_lock.unlock( );
		diag.setHardwareID( "YEI 3-Space Sensor (not connected)" );
		diag.add( "YEI TSS Status", this, &TSSUSB::DiagCB );
		diag.add( diag_pub_freq );

		nh_priv.param( "frame_id", frame_id, (std::string)"imu" );
		std::string temp_axis_config;
		nh_priv.param( "axis_config", temp_axis_config, (std::string)"yzx" );
		axis_config = str_to_tss_axis( temp_axis_config.c_str( ) );
		nh_priv.param( "invert_x_axis", invert_x_axis, false );
		nh_priv.param( "invert_y_axis", invert_y_axis, true );
		nh_priv.param( "invert_z_axis", invert_z_axis, false );
		XmlRpc::XmlRpcValue temp_gravity_vector;
		temp_gravity_vector[0] = 0.0;
		temp_gravity_vector[1] = 0.0;
		temp_gravity_vector[2] = GRAVITATIONAL_ACCELERATION;
		nh_priv.param( "gravity_vector", temp_gravity_vector, temp_gravity_vector );
		ROS_ASSERT( temp_gravity_vector.getType() == XmlRpc::XmlRpcValue::TypeArray );
		ROS_ASSERT( temp_gravity_vector.size( ) == 3 );
		ROS_ASSERT( temp_gravity_vector[0].getType() == XmlRpc::XmlRpcValue::TypeDouble );
		ROS_ASSERT( temp_gravity_vector[1].getType() == XmlRpc::XmlRpcValue::TypeDouble );
		ROS_ASSERT( temp_gravity_vector[2].getType() == XmlRpc::XmlRpcValue::TypeDouble );
		grav_vect = tf::Vector3( temp_gravity_vector[0], temp_gravity_vector[1], temp_gravity_vector[2] );
		nh_priv.param( "reference_vector_mode", reference_vector_mode, 1 );
		ROS_ASSERT( reference_vector_mode >= 0 && reference_vector_mode <= 3 );
		nh_priv.param( "orientation_covariance", orientation_covariance, 0.0 );
		nh_priv.param( "angular_velocity_covariance", angular_velocity_covariance, 0.0 );
		nh_priv.param( "linear_acceleration_covariance", linear_acceleration_covariance, 0.0 );
		nh_priv.param( "temperature_variance", temperature_variance, 0.0 );
		nh_priv.param( "magnetic_field_covariance", magnetic_field_covariance, 0.0 );
	}

	TSSUSB::~TSSUSB( )
	{
		spin_thread.interrupt( );
		TSSClose( );
	}

	enum tss_usb_axis_configurations TSSUSB::str_to_tss_axis( const char *str )
	{
		if( std::strlen( str ) == 3 )
		{
			const char lowerstr[4] = { tolower( str[0] ), tolower( str[1] ), tolower( str[2] ), '\0' };
			if( lowerstr[0] == 'x' && lowerstr[1] == 'y' && lowerstr[2] == 'z' )
				return TSS_USB_AXIS_XYZ;
			if( lowerstr[0] == 'x' && lowerstr[1] == 'z' && lowerstr[2] == 'y' )
				return TSS_USB_AXIS_XZY;
			if( lowerstr[0] == 'y' && lowerstr[1] == 'x' && lowerstr[2] == 'z' )
				return TSS_USB_AXIS_YXZ;
			if( lowerstr[0] == 'y' && lowerstr[1] == 'z' && lowerstr[2] == 'x' )
				return TSS_USB_AXIS_YZX;
			if( lowerstr[0] == 'z' && lowerstr[1] == 'x' && lowerstr[2] == 'y' )
				return TSS_USB_AXIS_ZXY;
			if( lowerstr[0] == 'z' && lowerstr[1] == 'y' && lowerstr[2] == 'x' )
				return TSS_USB_AXIS_ZYX;
		}
		ROS_ASSERT_MSG(1, "WARNING: Invalid axis configuration '%s'", str );
		return TSS_USB_AXIS_XYZ;
	}

	bool TSSUSB::TSSOpen( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		return TSSOpenNoLock( );
	}

	bool TSSUSB::TSSOpenNoLock( )
	{
		if( tssd >= 0 )
			return true;

		tssd = tss_usb_open( port.c_str( ) );

		if( tssd < 0 )
		{
			open_failure_count++;
			return false;
		}

		if( tss_set_axis_directions( tssd, axis_config | ( invert_x_axis ? TSS_USB_INVERT_X : 0 )
			| ( invert_y_axis ? TSS_USB_INVERT_Y : 0 ) | ( invert_z_axis ? TSS_USB_INVERT_Z : 0 ) ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return false;
		}

		if( tss_set_reference_mode( tssd, reference_vector_mode ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return false;
		}

		diag.setHardwareIDf( "YEI TSS on %s", port.c_str( ) );

		imu_pub = nh.advertise<sensor_msgs::Imu>( "imu/data", 1 );
		temp_pub = nh.advertise<sensor_msgs::Temperature>( "imu/temp", 1 );
		mag_pub = nh.advertise<sensor_msgs::MagneticField>( "imu/mag", 1 );
		tare_srv = nh_priv.advertiseService( "tare", &TSSUSB::TareCB, this );
		commit_srv = nh_priv.advertiseService( "commit", &TSSUSB::CommitCB, this );
		reset_srv = nh_priv.advertiseService( "reset", &TSSUSB::ResetCB, this );
		factory_srv = nh_priv.advertiseService( "restore_factory_settings", &TSSUSB::FactoryCB, this );
		led_color_srv = nh_priv.advertiseService( "set_led_color", &TSSUSB::LEDColorCB, this );

		return true;
	}

	void TSSUSB::TSSClose( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		TSSCloseNoLock( );
	}

	void TSSUSB::TSSCloseNoLock( )
	{
		int old_tssd = tssd;
		if( tssd < 0 )
		{
			cmd_lock.unlock( );
			return;
		}
		tssd = -1;
		tss_usb_close( old_tssd );
		if( imu_pub )
			imu_pub.shutdown( );
		if( temp_pub )
			temp_pub.shutdown( );
		if( mag_pub )
			mag_pub.shutdown( );
		if( tare_srv )
			tare_srv.shutdown( );
		if( commit_srv )
			commit_srv.shutdown( );
		if( reset_srv )
			reset_srv.shutdown( );
		if( factory_srv )
			factory_srv.shutdown( );
		if( led_color_srv )
			led_color_srv.shutdown( );
	}

	void TSSUSB::spin( )
	{
		while( ros::ok( ) )
		{
			boost::this_thread::interruption_point( );
			spinOnce( );
			diag.update( );
			spin_rate.sleep( );
		}
	}

	void TSSUSB::spinOnce( )
	{
		cmd_lock.lock( );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return;

		int ret;

		/*
		 * IMU Data
		 */

		float quat[4];
		if( ( ret = tss_get_orientation_quaternion( tssd, quat ) ) < 0 )
		{
			TSSCloseNoLock( );
			cmd_lock.unlock( );
			io_failure_count++;
			return;
		}

		float gyro[3];
		if( ( ret = tss_get_filtered_gyro( tssd, gyro ) ) < 0 )
		{
			TSSCloseNoLock( );
			cmd_lock.unlock( );
			io_failure_count++;
			return;
		}

		float accel[3];
		if( ( ret = tss_get_accel( tssd, accel ) ) < 0 )
		{
			TSSCloseNoLock( );
			cmd_lock.unlock( );
			io_failure_count++;
			return;
		}

		cmd_lock.unlock( );

		sensor_msgs::ImuPtr msg( new sensor_msgs::Imu );

		msg->header.frame_id = frame_id;
		msg->header.stamp = ros::Time::now( );

		msg->orientation.x = quat[0];
		msg->orientation.y = quat[1];
		msg->orientation.z = quat[2];
		msg->orientation.w = quat[3];

		msg->angular_velocity.x = gyro[0];
		msg->angular_velocity.y = gyro[1];
		msg->angular_velocity.z = gyro[2];

		switch( axis_config )
		{
			case TSS_USB_AXIS_XYZ:
				msg->linear_acceleration.x = accel[0] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.y = accel[1] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.z = accel[2] * GRAVITATIONAL_ACCELERATION;
				break;
			case TSS_USB_AXIS_XZY:
				msg->linear_acceleration.x = accel[0] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.y = accel[2] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.z = accel[1] * GRAVITATIONAL_ACCELERATION;
				break;
			case TSS_USB_AXIS_YXZ:
				msg->linear_acceleration.x = accel[1] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.y = accel[0] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.z = accel[2] * GRAVITATIONAL_ACCELERATION;
				break;
			case TSS_USB_AXIS_YZX:
				msg->linear_acceleration.x = accel[2] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.y = accel[0] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.z = accel[1] * GRAVITATIONAL_ACCELERATION;
				break;
			case TSS_USB_AXIS_ZXY:
				msg->linear_acceleration.x = accel[1] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.y = accel[2] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.z = accel[0] * GRAVITATIONAL_ACCELERATION;
				break;
			case TSS_USB_AXIS_ZYX:
				msg->linear_acceleration.x = accel[2] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.y = accel[1] * GRAVITATIONAL_ACCELERATION;
				msg->linear_acceleration.z = accel[0] * GRAVITATIONAL_ACCELERATION;
				break;
		}
		if( invert_x_axis )
			msg->linear_acceleration.x *= -1;
		if( invert_y_axis )
			msg->linear_acceleration.y *= -1;
		if( invert_z_axis )
			msg->linear_acceleration.z *= -1;

		tf::Quaternion orient;
		tf::quaternionMsgToTF( msg->orientation, orient );
		const tf::Vector3 tmp_grav_vect = tf::quatRotate( orient.inverse( ), grav_vect );
		msg->linear_acceleration.x += tmp_grav_vect.x( );
		msg->linear_acceleration.y += tmp_grav_vect.y( );
		msg->linear_acceleration.z += tmp_grav_vect.z( );

		/* Dummy Values */
		msg->orientation_covariance[0] = orientation_covariance;
		msg->orientation_covariance[4] = orientation_covariance;
		msg->orientation_covariance[8] = orientation_covariance;
		msg->angular_velocity_covariance[0] = angular_velocity_covariance;
		msg->angular_velocity_covariance[4] = angular_velocity_covariance;
		msg->angular_velocity_covariance[8] = angular_velocity_covariance;
		msg->linear_acceleration_covariance[0] = linear_acceleration_covariance;
		msg->linear_acceleration_covariance[4] = linear_acceleration_covariance;
		msg->linear_acceleration_covariance[8] = linear_acceleration_covariance;

		imu_pub.publish( msg );

		/*
		 * Temperature
		 */

		cmd_lock.lock( );

		float temp;
		if( ( ret = tss_get_temperature_c( tssd, &temp ) ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return;
		}

		cmd_lock.unlock( );

		sensor_msgs::TemperaturePtr temp_msg( new sensor_msgs::Temperature );
		temp_msg->header = msg->header;
		temp_msg->temperature = temp;
		temp_msg->variance = temperature_variance;

		temp_pub.publish( temp_msg );

		/*
		 * Magnetic Field
		 */

		cmd_lock.lock( );

		float mag[3];
		if( ( ret = tss_read_compass( tssd, mag ) ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return;
		}

		cmd_lock.unlock( );

		sensor_msgs::MagneticFieldPtr mag_msg( new sensor_msgs::MagneticField );
		mag_msg->header = msg->header;
		mag_msg->magnetic_field.x = 0.0001 * mag[0];
		mag_msg->magnetic_field.y = 0.0001 * mag[1];
		mag_msg->magnetic_field.z = 0.0001 * mag[2];
		mag_msg->magnetic_field_covariance[0] = magnetic_field_covariance;
		mag_msg->magnetic_field_covariance[4] = magnetic_field_covariance;
		mag_msg->magnetic_field_covariance[8] = magnetic_field_covariance;

		mag_pub.publish( mag_msg );

		diag_pub_freq.tick( );
	}

	void TSSUSB::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected" );
			return;
		}

		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "TSS status OK" );

		int ret;
		char version[33];
		if( ( ret = tss_get_version( tssd, version ) ) < 0 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to fetch version" );
			io_failure_count++;
			TSSCloseNoLock( );
			return;
		}
		stat.add( "version", version );

		char ext_version[13];
		if( ( ret = tss_get_version_extended( tssd, ext_version ) ) < 0 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to fetch extended_version" );
			io_failure_count++;
			TSSCloseNoLock( );
			return;
		}
		stat.add( "version_extended", ext_version );

		static unsigned int last_io_failure_count = io_failure_count;
		if( io_failure_count > last_io_failure_count )
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "I/O Failure Count Increase" );
		stat.add( "io_failure_count", io_failure_count );
		last_io_failure_count = io_failure_count;

		static unsigned int last_open_failure_count = open_failure_count;
		if( open_failure_count > last_open_failure_count )
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Open Failure Count Increase" );
		stat.add( "open_failure_count", open_failure_count );
		last_open_failure_count = open_failure_count;
	}

	bool TSSUSB::TareCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return false;

		if( tss_tare( tssd ) < 0 )
			return false;

		return true;
	}

	bool TSSUSB::CommitCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return false;

		if( tss_commit( tssd ) < 0 )
			return false;

		return true;
	}

	bool TSSUSB::ResetCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return false;

		if( tss_reset( tssd ) < 0 )
			return false;

		return true;
	}

	bool TSSUSB::FactoryCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return false;

		if( tss_restore_factory_settings( tssd ) < 0 )
			return false;

		return true;
	}

	bool TSSUSB::LEDColorCB( yei_tss_usb::LEDColor::Request &req, yei_tss_usb::LEDColor::Response &res )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return false;

		if( tss_set_led( tssd, &req.color[0] ) < 0 )
			return false;

		return true;
	}
}
