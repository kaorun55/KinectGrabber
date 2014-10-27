// Sample.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "kinect2_grabber.h"
#include <pcl/visualization/cloud_viewer.h>


int _tmain( int argc, _TCHAR* argv[] )
{
	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );

#if 0
    // Callback Function to be called when Updating Data(Depth)
    boost::function<void( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& )> function =
        [&viewer]( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud ){
        if ( !viewer.wasStopped() ){
            viewer.showCloud( cloud );
        }
    };
#else
    // Callback Function to be called when Updating Data(RGB-D)
    boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> function =
        [&viewer]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud ){
        if ( !viewer.wasStopped() ){
            viewer.showCloud( cloud );
        }
    };
#endif

	// Create KinectGrabber
	pcl::Grabber* grabber = new pcl::Kinect2Grabber();

	// Regist Callback Function
	grabber->registerCallback( function );

	// Start Retrieve Data
	grabber->start();

	while( !viewer.wasStopped() ){
		// Input Key ( Exit ESC key )
		if( GetKeyState( VK_ESCAPE ) < 0 ){
			break;
		}
	}

	// Stop Retrieve Data
	grabber->stop();

	return 0;
}

