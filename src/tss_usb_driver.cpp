#include "yei_tss_usb/tss_usb_driver.hpp"

#include <iomanip>

#include <sensor_msgs/Imu.h>
#include <ros/time.h>

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
		spin_rate( 100 ),
		spin_thread( &TSSUSB::spin, this )
	{
		cmd_lock.unlock( );
		diag.setHardwareID( "YEI 3-Space Sensor (not connected)" );
		diag.add( "YEI TSS Status", this, &TSSUSB::DiagCB );
		diag.add( diag_pub_freq );

		nh_priv.param( "frame_id", frame_id, (std::string)"imu" );
	}

	TSSUSB::~TSSUSB( )
	{
		spin_thread.interrupt( );
		TSSClose( );
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

		if( tss_set_axis_directions( tssd, TSS_USB_YZX | TSS_USB_INVERT_Y ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return false;
		}

		if( tss_set_reference_mode( tssd, TSS_USB_REFERENCE_MULTI ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return false;
		}

		diag.setHardwareIDf( "YEI TSS on %s", port.c_str( ) );

		imu_pub = nh.advertise<sensor_msgs::Imu>( "imu/data", 1 );
		tare_srv = nh_priv.advertiseService( "tare", &TSSUSB::TareCB, this );
		commit_srv = nh_priv.advertiseService( "commit", &TSSUSB::CommitCB, this );
		reset_srv = nh_priv.advertiseService( "reset", &TSSUSB::ResetCB, this );
		factory_srv = nh_priv.advertiseService( "restore_factory_settings", &TSSUSB::FactoryCB, this );
		multi_ref_srv = nh_priv.advertiseService( "set_multi_reference_vectors", &TSSUSB::MultiRefCB, this );

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
		if( tare_srv )
			tare_srv.shutdown( );
		if( commit_srv )
			commit_srv.shutdown( );
		if( reset_srv )
			reset_srv.shutdown( );
		if( factory_srv )
			factory_srv.shutdown( );
		if( multi_ref_srv )
			multi_ref_srv.shutdown( );
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
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return;

		int ret;

		float quat[4];
		if( ( ret = tss_get_orientation_quaternion( tssd, quat ) ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return;
		}

		float gyro[3];
		if( ( ret = tss_get_filtered_gyro( tssd, gyro ) ) < 0 )
		{
			TSSCloseNoLock( );
			io_failure_count++;
			return;
		}

		float accel[3];
		if( ( ret = tss_get_accel( tssd, accel ) ) < 0 )
		{
			TSSCloseNoLock( );
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

		msg->linear_acceleration.x = accel[0] * -9.80665;
		msg->linear_acceleration.y = accel[1] * -9.80665;
		msg->linear_acceleration.z = accel[2] * -9.80665;

		/* Dummy Values */
		msg->orientation_covariance[0] = .1;
		msg->orientation_covariance[4] = .1;
		msg->orientation_covariance[8] = .1;

		imu_pub.publish( msg );
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

	bool TSSUSB::MultiRefCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( tssd < 0 && !TSSOpenNoLock( ) )
			return false;

		if( tss_set_multi_reference_vectors( tssd ) < 0 )
			return false;

		return true;
	}
}
