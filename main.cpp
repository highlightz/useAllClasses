// Integration of all classes
// Date: 2015-09-05
#if 0
// Camera Class --------------------------------------------------------
#include "bb2_wrapper/bb2_wrapper.h"

#define S2C38 

#ifdef HIGHER
 #define WIDTH 1024
 #define HEIGHT 768
#else
 #define WIDTH 640
 #define HEIGHT 480
#endif

// Global Resources
bb2_wrapper m_camera( WIDTH, HEIGHT );

// Raw four-channel images
IplImage* pframeL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
IplImage* pframeR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );

// Rectified three-channel images
IplImage* pfL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
IplImage* pfR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );

// Images for VO, single-channel gray images
IplImage* pGrayL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
IplImage* pGrayR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );

// 8-bit disparity image, for display
IplImage* disp8 = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
// ---------------------------------------------------------------------

// VO Class ----------------------------------
#include "libviso2_wrapper/libviso2_wrapper.h"
// -------------------------------------------

// Log Class -----------------------------------------------------------
#include "log_wrapper/log_wrapper.h"
#include "structures.h"
// Default log folder
log_wrapper< odometry > imageLogger( "D:\\HighlightClark\\dog_data\\" );
// ---------------------------------------------------------------------

// Hokuyo Class -------------------------------------------------------
#include "hokuyo_wrapper/hokuyo_wrapper.h"
// Draw laser scan points on this image
IplImage* laserPoints = cvCreateImage( cvSize( HEIGHT, HEIGHT ), 8, 3 );
// ---------------------------------------------------------------------

// System Includes------
#include <iostream>
using std::cout;
using std::endl;
#include <vector>
using std::vector;
#include <stdlib.h>
#include <time.h>
// ---------------------

int main( int argc, char** argv )
{
	// libviso2_wrapper class declarations ----------------------------------
	
	// Calibration parameters for bb2 
	VisualOdometryStereo::parameters param;

#ifdef S2C38
	#ifdef HIGHER
		double f  = 786.66; 
		double cu = 515.40; 
		double cv = 393.52; 
		double parambase = 0.12;
	#else
		double f  = 491.66; 
		double cu = 321.94; 
		double cv = 245.76; 
		double parambase = 0.12;
	#endif
#else  // S2C25
	#ifdef HIGHER
		double f  = 445.90; 
		double cu = 507.51; 
		double cv = 395.32; 
		double parambase = 0.12;
	#else
		double f  = 278.69; 
		double cu = 317.00; 
		double cv = 246.89; 
		double parambase = 0.12;
	#endif
#endif

	param.calib.f  = f;   // Focal length in pixels
	param.calib.cu = cu;  // Principal point (u-coordinate) in pixels
	param.calib.cv = cv;         // Principal point (v-coordinate) in pixels
	param.base     = parambase;  // Baseline in meters

	libviso2_wrapper libviso2( param );
	// ----------------------------------------------------------------------

	// Start camera ----------------------------
	if ( !m_camera.StartCamera( ) )
	{
		cout << "Start Camera Failed!" << endl;
		return -1;
	}
	m_camera.showCameraInfo( );
	m_camera.EnableStereoMatch( );
	// -----------------------------------------

	// Start laser ----------------------
	hokuyo_wrapper laser( argc, argv );
	if ( !laser.startHokuyo( ) )
	{
		cout << "Hokuyo Failed!" << endl;
	}
	// ----------------------------------
	
	// Data Logger ------------------------------------
	// Set an initial counter for the image log_wrapper
	long img_counter = 0;
	clock_t start, finish;  // Timing utility
	double duration = 0;
	const double totalRunningTime = 300;  // Unit: second
	start = clock( );
	// ------------------------------------------------
	
	// Main processing loop
	while ( duration < totalRunningTime )
	{
		/*
		if ( laser.bufferDistance( ) )
		{
			cout << laser.filterSafeAngle( ) << endl;
		}
		
		laser.showDistancePoints( laserPoints );
		cvShowImage( "laserPoints", laserPoints );
		cvWaitKey( 33 );
		cvZero( laserPoints );*/

		if ( m_camera.AcquireFrame( ) && m_camera.StereoMatch( ) )
		{
			pframeL = m_camera.GetRetfImgL( );
			pframeR = m_camera.GetRetfImgR( );
			m_camera.Four2Three( pframeL, pfL );
			m_camera.Four2Three( pframeR, pfR );

			disp8 = m_camera.getStereo( );
			m_camera.showAvrgDepth( );

			// Start the image logger
			imageLogger.start_log_image_pair( img_counter, pfL, pfR );

			cvShowImage( "LeftColor", pfL );
			cvShowImage( "RightColor", pfR );
			cvShowImage( "disp8", disp8 );
			cvWaitKey( 33 );
	
			cvCvtColor( pfL, pGrayL, CV_BGR2GRAY );
			cvCvtColor( pfR, pGrayR, CV_BGR2GRAY );
		}
	
		iplImageWrapper left_image( pGrayL );
		iplImageWrapper right_image( pGrayR );

		libviso2.run( left_image, right_image );
		
		odometry anOdom = libviso2.getOdometry( );
		Matrix aPose = libviso2.getPose( );
		//cout << aPose << std::endl << endl;
		
		// Start the odometry logger
		imageLogger.start_log_odom( anOdom );

		// Update the logger counter
		img_counter++;

		// Get the elapsed time
		finish = clock( );
		duration = static_cast< double >( finish - start ) / CLOCKS_PER_SEC;
	}	

	return 0;
}
#endif  // End: Integration of all classes

// bb2_wrapper Class Test
#if 0
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <vector>
using std::cout;
using std::endl;
using std::vector;

#include "bb2_wrapper/bb2_wrapper.h"
//#include "ransac.h"

#include <opencv2/opencv.hpp>
using namespace cv;
namespace miscellaneous
{
	template < class DataType > 
	class Image
	{
	public:
		Image( IplImage *img = 0 ){ imgp = img; }
		~Image(  ){ imgp = 0; }
		void operator =( IplImage *img ){ imgp = img; }
		inline DataType* operator[]( const int rowIndx )
		{
			return ( ( DataType* )( imgp->imageData + rowIndx * imgp->widthStep ) );
		}
	private:
		IplImage *imgp;
	};

	typedef struct
	{
		unsigned char b, g, r;
	} RgbPixel;

	typedef struct
	{
		float b, g, r;
	} RgbPixelFloat;

	typedef Image<RgbPixel> RgbImage;
	typedef Image<RgbPixelFloat> RgbImageFloat;
	typedef Image<unsigned char> BwImage;
	typedef Image<float> BwImageFloat;

	void drawInterestPointsOnImage( IplImage* src, int flag )
	{
		if ( flag == 1 )
		{
			// First line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 4 ), 3, cvScalar(0, 0, 255) );

			// Second line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 2 ), 3, cvScalar(0, 0, 255) );

			// Third line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
		}
		if ( flag == 0 )
		{
			// First line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 4 ), 3,cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 4 ), 3, cvScalar(255, 255, 255) );

			// Second line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 2 ), 3, cvScalar(255, 255, 255) );

			// Third line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
		}
	}

	void drawInterestLinesOnImage( IplImage* src, int flag )
	{
		CvPoint top_left = cvPoint( 10, 10 );     CvPoint top_mid_left = cvPoint( 210, 10 );     CvPoint top_mid_right = cvPoint( 430, 10 );     CvPoint top_right = cvPoint( 630, 10 );
		CvPoint bottom_left = cvPoint( 10, 470 ); CvPoint bottom_mid_left = cvPoint( 210, 470 ); CvPoint bottom_mid_right = cvPoint( 430, 470 ); CvPoint bottom_right = cvPoint( 630, 470 );

		if ( flag == 1 )
		{
			cvLine( src, top_left, bottom_left, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_mid_left, bottom_mid_left, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_mid_right, bottom_mid_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_right, bottom_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_left, top_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, bottom_left, bottom_right, cvScalar(0, 255, 0), 2 );
		}

		if ( flag == 0 )
		{
			cvLine( src, top_left, bottom_left, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_mid_left, bottom_mid_left, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_mid_right, bottom_mid_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_right, bottom_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_left, top_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, bottom_left, bottom_right, cvScalar(255, 255, 255), 2 );
		}

	}

	// maxDisp is needed
	// ����ͳ�Ƽ��㣬��ȷ��V�Ӳ�ͼ����������������maxDisp
	void computeVDisparity( IplImage* src, IplImage* dst )
	{
		for ( int rowComm = 0; rowComm < src->height; rowComm++ )
		{
			for ( int colDst = 0; colDst < dst->width; colDst++ )
			{
				// Do the counting of a row of the src disparity image
				int counter = 0;
				for ( int colSrc = 0; colSrc < src->width; colSrc++ )
				{
					if ( src->imageData[ rowComm * src->widthStep + colSrc ] == colDst )
					{
						counter++;
					}
				}

				dst->imageData[ rowComm * dst->widthStep + colDst ] = counter;
			}
		}
	}

	// maxDisp is needed
	// ����ͳ�Ƽ��㣬��ȷ��U�Ӳ�ͼ����������������maxDisp
	void computeUDisparity( IplImage* src, IplImage* dst )
	{
		for ( int colComm = 0; colComm < src->width; colComm++ )
		{
			for ( int rowDst = 0; rowDst < dst->height; rowDst++ )
			{
				// Do the counting of a column of the src disparity image
				int counter = 0;
				for ( int rowSrc = 0; rowSrc < src->height; rowSrc++ )
				{
					if ( src->imageData[ rowSrc * src->widthStep + colComm ] == rowDst )
					{
						counter++;
					}
				}

				dst->imageData[ rowDst * dst->widthStep + colComm ] = counter;
			}
		}
	}

	void houghTransform( Mat& src )
	{
		const double pi = 3.14;

		Mat contours;
		Canny( src, contours, 125, 350 );
		vector< cv::Vec2f > lines;
		// Hough transform, gathering a series of parameters (rho, theta), 
		// in which each pair parameter corresponds to a line
		HoughLines( contours, lines, 1, pi / 180, 80 );

		vector< Vec2f >::const_iterator it = lines.begin( );
		cout << lines.size( ) << endl;
		for ( it = lines.begin( ); it < lines.end( ); it++ )
		{
			float rho = (*it)[0];
			float theta = (*it)[1];
			if ( theta < pi / 4 || theta > pi * 3 / 4 )
			{
				Point pt1( rho/cos(theta),0);
				Point pt2( ( rho - src.rows*sin(theta))/cos(theta), src.rows );
				line( src, pt1, pt2, Scalar( 255 ), 1 );
			}
			else
			{
				Point pt1( 0, rho/sin(theta));
				Point pt2( src.cols, ( rho - src.cols*cos(theta))/sin(theta) );
				line( src, pt1, pt2, Scalar( 255 ), 1 );
			}
		}
	}
}


bb2_wrapper m_camera(640,480);

const int maxDisp = 255;

IplImage* pfL;
IplImage* pfR;
IplImage* pframeL;
IplImage* pframeR;
IplImage* disp8;
IplImage* vdisp;
IplImage* udisp;
IplImage* edgesOfDepth;

void miscellaneous::drawInterestPointsOnImage( IplImage* src, int flag );
void miscellaneous::drawInterestLinesOnImage( IplImage* src, int flag );
void miscellaneous::computeUDisparity( IplImage* src, IplImage* dst );
void miscellaneous::computeVDisparity( IplImage* src, IplImage* dst );
void miscellaneous::houghTransform( Mat& src );

int main( )
{
	//ransac aR( 5 );
	
	pfL = cvCreateImage(cvSize(640,480),8,3);
	pfR = cvCreateImage(cvSize(640,480),8,3);
	disp8 = cvCreateImage(cvSize(640,480),8,1);
	udisp = cvCreateImage( cvSize( 640, maxDisp ), 8, 1 );
	vdisp = cvCreateImage( cvSize( maxDisp, 480 ), 8, 1 );
	edgesOfDepth = cvCreateImage( cvSize( 640, 480 ), 8, 1 );

	//m_camera.setDispRange( 0, 200 );

	if( !m_camera.StartCamera() )
	{
		cout<<"StartCamera failed!"<<endl;
		return -1;
	}
	else
	{
		m_camera.showCameraInfo( );
		m_camera.EnableStereoMatch( );
		while (1)
		{
			if(m_camera.AcquireFrame()&&m_camera.StereoMatch())
			{
				pframeL = m_camera.GetRetfImgL();
				pframeR = m_camera.GetRetfImgR();
				m_camera.Four2Three(pframeL,pfL);
				m_camera.Four2Three(pframeR,pfR);

				disp8 = m_camera.getStereo( );
				/*BwImage disp8_wrapper( disp8 );
				for ( int i = 0; i < 640; i++ )
					for ( int j = 0; j < 480; j++ )
					{
						cout << (int)disp8_wrapper[j][i] << '\t';
					}
				*/
				
				/*
				miscellaneous::computeUDisparity( disp8, udisp );
				miscellaneous::computeVDisparity( disp8, vdisp );
				cvShowImage( "udisp", udisp );
				cvShowImage( "vdisp", vdisp );
				*/

				/*
				Mat iplWrapper( disp8 );
				miscellaneous::houghTransform( iplWrapper );
				imshow( "lines", iplWrapper );
				*/

				m_camera.showInterestPointsDepth( );

				//m_camera.showInterestPoints3D();

				//m_camera.showAvrgDepth( );

				//vector< CvPoint3D32f > iA( 620 * 240 );
				//iA = m_camera.showInterestArea( );

				//aR.mainLoop( iA );
				
				//vector< int > randomIndex( 3 );
				//randomIndex = aR.genRandomIndex( );		
				//cout << randomIndex[0] << ": " << iA[randomIndex[0]].x << ", " << iA[randomIndex[0]].y << ", " << iA[randomIndex[0]].z << endl;

				//cvCanny(disp8, edgesOfDepth, 20, 100 );

				miscellaneous::drawInterestPointsOnImage( pfR, 1 );
				miscellaneous::drawInterestLinesOnImage( pfR, 1 );

				miscellaneous::drawInterestPointsOnImage( disp8, 0 );
				miscellaneous::drawInterestLinesOnImage( disp8, 0 );

				miscellaneous::drawInterestPointsOnImage( pfL, 1 );
				miscellaneous::drawInterestLinesOnImage( pfL, 1 );

				
				
				cvShowImage("Left",pfL);
				cvShowImage("Right",pfR);
				
				cvShowImage("Disp8", disp8 );
				//cvShowImage("edge", edgesOfDepth );
				//if(cvWaitKey(20)==27) break;
				cvWaitKey( 5 );
			}
		}	
	}

	//====================
	m_camera.StopCamera();
	cvDestroyWindow("Left");
	cvDestroyWindow("Right");
	system("Pause");
	return 0;
}
#endif

#if 0
#include "hokuyo_wrapper\hokuyo_wrapper.h"

#include "hokuyo_wrapper\DirectionGenerator.h"

#include "bb2_wrapper/bb2_wrapper.h"
#include "libviso2_wrapper/libviso2_wrapper.h"

int main( int argc, char** argv )
{
	// Initialize hokuyo_wrapper object, and start the laser range finder
	hokuyo_wrapper laser( argc, argv );
	laser.startHokuyo( );
    	laser.setInterestRadius( 10000 );  // 10 meters
	
	// Initialize DirectionGenerator object    
    	DirectionGenerator dg;
	dg.setInterestRadius( 4500 );  // 4.5 meters
    
    	cv::Mat laserPoints;
    
    	// Initialize bb2_wrapper object, and start the stereo camera
    	const int WIDTH = 640;
    	const int HEIGHT = 480;
	bb2_wrapper m_camera( WIDTH, HEIGHT );
	IplImage* pframeL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
	IplImage* pframeR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
	IplImage* pfL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	IplImage* pfR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	IplImage* pGrayL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
	IplImage* pGrayR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
	
	// Start camera
	if ( !m_camera.StartCamera( ) )
	{
		std::cout << "StartCamera failed!" << std::endl;
		return -1;
	}
	
	// Initialize libviso2_wrapper object
	// Calibration parameters for bb2 
	VisualOdometryStereo::parameters param;
	
#if 1
	// S2C25
	double f  = 278.69; 
	double cu = 317.00; 
	double cv = 246.89; 
	double parambase = 0.12;
#else
	double f  = 491.66; 
	double cu = 321.94; 
	double cv = 245.876; 
	double parambase = 0.12;
#endif
	
	param.calib.f  = f;  // focal length in pixels
	param.calib.cu = cu;  // principal point (u-coordinate) in pixels
	param.calib.cv = cv;  // principal point (v-coordinate) in pixels
	param.base     = parambase;  // baseline in meters
	
	// Draw odometry curve on this image
	cv::Mat odomCurve( 640, 480, CV_8UC3 );
	
	libviso2_wrapper libviso2( param );
	
	// Acquire initial laser sequence data
	laser.bufferDistance( );
    vector< long > distance = laser.getDistance( );

	// Main processing loop
	while ( 1 )
	{        
        	// Generate next waypoint
        	double x_next = 0.0;
        	double y_next = 0.0;
        	dg.genWaypoint( distance, x_next, y_next );
			
			laser.showDistancePoints( laserPoints );
			cv::imshow( "laser", laserPoints );
			cv::waitKey( 5 );
			laserPoints.setTo( cv::Scalar( 0 ) );
        	
        	const double toleranceThreshold = 2 ;  // 0.2 meter, wild value, to be tuned
        	double distanceToWaypoint = libviso2.distanceToWaypoint( x_next/1000, y_next/1000 );
			
        	while ( distanceToWaypoint > toleranceThreshold )
        	{
        		// TODO:
        		// Here, drive the car towards the target
        		
        		// Start visual odometry, and init its coordinate frame;
        		// meanwhile, map the generated waypoint into odometry frame
        		if ( m_camera.AcquireFrame( ) && m_camera.StereoMatch( ) )
			{
				pframeL = m_camera.GetRetfImgL( );
				pframeR = m_camera.GetRetfImgR( );
				m_camera.Four2Three( pframeL, pfL );
				m_camera.Four2Three( pframeR, pfR );
	
				cvCvtColor( pfL, pGrayL, CV_BGR2GRAY );
				cvCvtColor( pfR, pGrayR, CV_BGR2GRAY );
			}
			
			iplImageWrapper left_image( pGrayL );
			iplImageWrapper right_image( pGrayR );
			
			libviso2.run( left_image, right_image );
			
			odometry anOdom = libviso2.getOdometry( );
			cout << "Target: " << -y_next/1000 << ", " << x_next/1000;
			cout << "Current: " << anOdom.x << ", " << anOdom.z;
			cout << "Distance: " << distanceToWaypoint << endl;

			
			libviso2.drawOdometryCurve( odomCurve );
			cv::imshow( "Odometry", odomCurve );
			cv::waitKey( 5 );
			
			// Update the distance to the next target 
			distanceToWaypoint = libviso2.distanceToWaypoint( x_next/1000, y_next/1000 );

			// Update the laser sequence too
			laser.bufferDistance( );
			distance = laser.getDistance( );
        	}
       	
        	// When the target has been reached, 
        	// the odometry coordinate should be reinitialized
        	libviso2.reinitializePose( );
	}

	return 0;
}
#endif

// bb2_wrapper Class Test
// Date: 150905
#if 1
// System Includes
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <iostream>
using std::cout;
using std::endl;

#include <vector>
using std::vector;

// OpenCV Includes
#include <opencv2/opencv.hpp>

// Structures used
#include "structures.h"

// Camera derived class
#include "bb2_wrapper\bb2_wrapper.h"

// Serial Port class
#include "serial\syncSerialComm.h"

namespace miscellaneous
{
	vector< DepthWindow > computeAvrgDepth( vector< PointCloud > pc_seq, GroundPlane groundPlane )
	{
		// Those rows of point clouds below this row is considered.
		const int row_target = 240;

		// Depth that is smaller than it is considered to be too near.
		const float fixedDepth = 1.0f;

		const float belongToGroundThreshold = 0.05f;  // Unit: meter

		// Declares three depth windows: left, middle, and right
		vector< DepthWindow > windows( 3 );

		// Normalize the norm vector of ground plane
		float normed_vector = 1 / sqrt( groundPlane.coefs_0 * groundPlane.coefs_0 
			                + groundPlane.coefs_1 * groundPlane.coefs_1
			                + groundPlane.coefs_2 * groundPlane.coefs_2 );

		for ( int i = 0; i < pc_seq.size( ); i++ )
		{
			// Left window
			if (   pc_seq[i].row > row_target 
				&& pc_seq[i].row < 480
				&& pc_seq[i].col > 10  // The left 10 columns of points not considered 
				&& pc_seq[i].col < 210 )
			{
				float distanceToGround = abs( groundPlane.coefs_0 * pc_seq[i].x
				                   + groundPlane.coefs_1 * pc_seq[i].y
				                   + groundPlane.coefs_2 * pc_seq[i].z 
				                   + groundPlane.coefs_3 ) * normed_vector;
				if ( distanceToGround > belongToGroundThreshold )  // If the point cloud is above the ground
				{
					windows[0].avrgDepth += pc_seq[i].z;
					windows[0].numOfPoints++;
					
					// Count the number of pixels with depth less than fixedDepth
					if ( pc_seq[i].z < fixedDepth )
					{
						windows[0].numPixelLessThanFixedDepthThreshold++;
					}
				}
			}

			// Middle window
			if (   pc_seq[i].row > row_target 
				&& pc_seq[i].row < 480
				&& pc_seq[i].col >= 210  
				&& pc_seq[i].col < 430 )
			{
				float distanceToGround = abs( groundPlane.coefs_0 * pc_seq[i].x
				                   + groundPlane.coefs_1 * pc_seq[i].y
				                   + groundPlane.coefs_2 * pc_seq[i].z 
				                   + groundPlane.coefs_3 ) * normed_vector;
				if ( distanceToGround > belongToGroundThreshold )  // If the point cloud is above the ground
				{
					windows[1].avrgDepth += pc_seq[i].z;
					windows[1].numOfPoints++;
					
					// Count the number of pixels with depth less than fixedDepth
					if ( pc_seq[i].z < fixedDepth )
					{
						windows[1].numPixelLessThanFixedDepthThreshold++;
					}
				}
			}

			// Right window
			if (   pc_seq[i].row > row_target 
				&& pc_seq[i].row < 480
				&& pc_seq[i].col >= 430  // The right 10 columns of points not considered 
				&& pc_seq[i].col < 630 )
			{
				float distanceToGround = abs( groundPlane.coefs_0 * pc_seq[i].x
				                   + groundPlane.coefs_1 * pc_seq[i].y
				                   + groundPlane.coefs_2 * pc_seq[i].z 
				                   + groundPlane.coefs_3 ) * normed_vector;
				if ( distanceToGround > belongToGroundThreshold )  // If the point cloud is above the ground
				{
					windows[2].avrgDepth += pc_seq[i].z;
					windows[2].numOfPoints++;
					
					// Count the number of pixels with depth less than fixedDepth
					if ( pc_seq[i].z < fixedDepth )
					{
						windows[2].numPixelLessThanFixedDepthThreshold++;
					}
				}
			}
		}

		return windows;
	}

	// Consider artificial forces composed of tens of thousands of point clouds.
	// Take ground plane as a constraint.
	// Returns an safest direction angle( in degrees ).
	float sumArtificialForces( vector< PointCloud > pc_seq, GroundPlane groundPlane )
	{
		int counter = 0;
		// Initialize sum force.
		// y direction force not considered, 
		// this equals to projecting 3d point to x-z plane.
		float sum_force_x = 0.0f;
		float sum_force_z = 0.0f;

		const float belongToGroundThreshold = 0.05f;  // Unit: meter

		// Normalize the norm vector of ground plane
		float normed_vector = 1 / sqrt( groundPlane.coefs_0 * groundPlane.coefs_0 
			                + groundPlane.coefs_1 * groundPlane.coefs_1
			                + groundPlane.coefs_2 * groundPlane.coefs_2 );

		for ( int i = 0; i < pc_seq.size( ); i++ )
		{
			float distanceToGround = abs( groundPlane.coefs_0 * pc_seq[i].x
				                   + groundPlane.coefs_1 * pc_seq[i].y
				                   + groundPlane.coefs_2 * pc_seq[i].z 
				                   + groundPlane.coefs_3 ) * normed_vector;

			if ( distanceToGround > belongToGroundThreshold )  // If the point cloud is above the ground
			{
				sum_force_x += pc_seq[i].x;
				sum_force_z += pc_seq[i].z;
				counter++;
			}
		}

		// Coordinate transform
		float x = sum_force_z;
		float y = -sum_force_x;

		float cos_theta = y / sqrt( x * x + y * y );
		float angle_in_radian = asin( cos_theta );
		float angle_in_deg = angle_in_radian * 180 / 3.14;
		//cout << counter << endl;
		return angle_in_deg;
	}

	template < class DataType > 
	class Image
	{
	public:
		Image( IplImage *img = 0 ){ imgp = img; }
		~Image(  ){ imgp = 0; }
		void operator =( IplImage *img ){ imgp = img; }
		inline DataType* operator[]( const int rowIndx )
		{
			return ( ( DataType* )( imgp->imageData + rowIndx * imgp->widthStep ) );
		}
	private:
		IplImage *imgp;
	};

	typedef struct
	{
		unsigned char b, g, r;
	} RgbPixel;

	typedef struct
	{
		float b, g, r;
	} RgbPixelFloat;

	typedef Image<RgbPixel> RgbImage;
	typedef Image<RgbPixelFloat> RgbImageFloat;
	typedef Image<unsigned char> BwImage;
	typedef Image<float> BwImageFloat;

	void drawInterestPointsOnImage( IplImage* src, int flag )
	{
		if ( flag == 1 )
		{
			// First line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 4 ), 3, cvScalar(0, 0, 255) );

			// Second line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 2 ), 3, cvScalar(0, 0, 255) );

			// Third line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
		}
		if ( flag == 0 )
		{
			// First line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 4 ), 3,cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 4 ), 3, cvScalar(255, 255, 255) );

			// Second line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 2 ), 3, cvScalar(255, 255, 255) );

			// Third line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
		}
	}

	void drawInterestLinesOnImage( IplImage* src, int flag )
	{
		CvPoint top_left = cvPoint( 10, 10 );     CvPoint top_mid_left = cvPoint( 210, 10 );     CvPoint top_mid_right = cvPoint( 430, 10 );     CvPoint top_right = cvPoint( 630, 10 );
		CvPoint bottom_left = cvPoint( 10, 470 ); CvPoint bottom_mid_left = cvPoint( 210, 470 ); CvPoint bottom_mid_right = cvPoint( 430, 470 ); CvPoint bottom_right = cvPoint( 630, 470 );

		if ( flag == 1 )
		{
			cvLine( src, top_left, bottom_left, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_mid_left, bottom_mid_left, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_mid_right, bottom_mid_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_right, bottom_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_left, top_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, bottom_left, bottom_right, cvScalar(0, 255, 0), 2 );
		}

		if ( flag == 0 )
		{
			cvLine( src, top_left, bottom_left, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_mid_left, bottom_mid_left, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_mid_right, bottom_mid_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_right, bottom_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_left, top_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, bottom_left, bottom_right, cvScalar(255, 255, 255), 2 );
		}

	}

	// maxDisp is needed
	// ����ͳ�Ƽ��㣬��ȷ��V�Ӳ�ͼ����������������maxDisp
	void computeVDisparity( IplImage* src, IplImage* dst )
	{
		for ( int rowComm = 0; rowComm < src->height; rowComm++ )
		{
			for ( int colDst = 0; colDst < dst->width; colDst++ )
			{
				// Do the counting of a row of the src disparity image
				int counter = 0;
				for ( int colSrc = 0; colSrc < src->width; colSrc++ )
				{
					if ( src->imageData[ rowComm * src->widthStep + colSrc ] == colDst )
					{
						counter++;
					}
				}

				dst->imageData[ rowComm * dst->widthStep + colDst ] = counter;
			}
		}
	}

	// maxDisp is needed
	// ����ͳ�Ƽ��㣬��ȷ��U�Ӳ�ͼ����������������maxDisp
	void computeUDisparity( IplImage* src, IplImage* dst )
	{
		for ( int colComm = 0; colComm < src->width; colComm++ )
		{
			for ( int rowDst = 0; rowDst < dst->height; rowDst++ )
			{
				// Do the counting of a column of the src disparity image
				int counter = 0;
				for ( int rowSrc = 0; rowSrc < src->height; rowSrc++ )
				{
					if ( src->imageData[ rowSrc * src->widthStep + colComm ] == rowDst )
					{
						counter++;
					}
				}

				dst->imageData[ rowDst * dst->widthStep + colComm ] = counter;
			}
		}
	}

	void houghTransform( Mat& src )
	{
		const double pi = 3.14;

		Mat contours;
		Canny( src, contours, 125, 350 );
		vector< cv::Vec2f > lines;
		// Hough transform, gathering a series of parameters (rho, theta), 
		// in which each pair parameter corresponds to a line
		HoughLines( contours, lines, 1, pi / 180, 80 );

		vector< cv::Vec2f >::const_iterator it = lines.begin( );
		cout << lines.size( ) << endl;
		for ( it = lines.begin( ); it < lines.end( ); it++ )
		{
			float rho = (*it)[0];
			float theta = (*it)[1];
			if ( theta < pi / 4 || theta > pi * 3 / 4 )
			{
				cv::Point pt1( rho/cos(theta),0);
				cv::Point pt2( ( rho - src.rows*sin(theta))/cos(theta), src.rows );
				cv::line( src, pt1, pt2, cv::Scalar( 255 ), 1 );
			}
			else
			{
				cv::Point pt1( 0, rho/sin(theta));
				cv::Point pt2( src.cols, ( rho - src.cols*cos(theta))/sin(theta) );
				cv::line( src, pt1, pt2, cv::Scalar( 255 ), 1 );
			}
		}
	}

	void computeAverageDepth( IplImage* disparity, 
		                      double& left, double& middle, double& right )							
	{
		left = middle = right = 0;
		const int row_target = 240;
		for ( int row = row_target; row < disparity->height; row++ )
		{
			for ( int col = 10; col < 210; col++ )
			{
				left += static_cast< double >( disparity->imageData[ row * disparity->widthStep + col ] ) ;// ( disparity->height - row_target ) / ( 210 - 10 );
			}
		}

		for ( int row = row_target; row < disparity->height; row++ )
		{
			for ( int col = 210; col < 430; col++ )
			{
				cout << static_cast< double >( disparity->imageData[ row * disparity->widthStep + col ] );// ( disparity->height - row_target ) / ( 430 - 210 );
			}
		}
		//cout << middle << endl;
		for ( int row = row_target; row < disparity->height; row++ )
		{
			for ( int col = 430; col < 630; col++ )
			{
				right += static_cast< double >( disparity->imageData[ row * disparity->widthStep + col ] );// ( disparity->height - row_target ) / ( 630 - 430 );
			}
		}

		/*
		if ( row > row_target && pc.row < 470 && pc.col > 10 && pc.col < 210 )
		{
			avrgDepth[0] += pc.z;
		}

		if ( pc.row > row_target && pc.row < 470 && pc.col >= 210 && pc.col < 430 )
		{
			avrgDepth[1] += pc.z;
		}

		if ( pc.row > row_target && pc.row < 470 && pc.col >= 430 && pc.col < 630 )
		{
			avrgDepth[2] += pc.z;
		}*/
	}
}

namespace controls
{
	// Generate controls with some kind of algorithm
	void genControls( vector< float >& angles, vector< float >& speeds )
	{
		// Controls length
		const int numOfControls = 10;
	
		// Controls ranges
		const float maxAngle =  0.7f;
	    const float minAngle = -0.7f;
	    const float maxSpeed =  0.7f;
	
		// Specify angles
		angles[0] =  0.5f;
	    angles[1] = -0.5f;
	    angles[2] =  0.5f;
	    angles[3] = -0.5f;
	    angles[4] =  0.5f;
	
	    angles[5] = -0.5f;
	    angles[6] =  0.5f;
	    angles[7] = -0.5f;
	    angles[8] =  0.5f;
	    angles[9] = -0.5f;
	
		// Limit the angle controls to a safe range
	    for ( int i = 0; i < numOfControls; i++ )
	    {
	        if ( angles[i] > maxAngle )
	        {
	            angles[i] = maxAngle;
	        }
	
	        if ( angles[i] < minAngle )
	        {
	            angles[i] = minAngle;
	        }
	    }
	
		// Compute speeds according to specified angles
	    for ( int i = 0; i < numOfControls; i++ )
	    {
	        const float p = 10.0f;
	        speeds[i] = maxSpeed / sqrt( fabs( p * angles[i] ) + 1.0f );
	    }
	}

	// Convert controls to instruction that can be executed
	void genSend( float angle, float speed, char* send )
	{
		send[0] = 0xff;
		send[1] = 0xfe;
	
		// Speed control
		const int *p = (int *)&speed;
		send[5] = (*p>>24)&0xff;
		send[4] = (*p>>16)&0xff;
		send[3] = (*p>>8)&0xff;
		send[2] = (*p>>0)&0xff;
	
		// Angle control
		p = (int *)&angle;
		send[9] = (*p>>24)&0xff;
		send[8] = (*p>>16)&0xff;
		send[7] = (*p>>8)&0xff;
		send[6] = (*p>>0)&0xff;
	}

	// Send the instructions
	int writeToPort( const char *send )
	{
		CSyncSerialComm s0("COM2");
		s0.Open( );
		s0.ConfigPort( CBR_115200, 5 );
		s0.Write( send, 10 );
		s0.Close( );
	
		return 0;
	}

	// This function sends a series of controls to start the car.
	void makeStart( )
	{
		// Controls length
		const int numOfStartControls = 20;
		vector< float > startAngles(numOfStartControls);
		vector< float > startSpeeds(numOfStartControls);
		const DWORD startControlPeriod = 100;
	
		// Specify angles and speeds
		// Send empty controls for a few periods
		startAngles[0] = 0.00f;    startSpeeds[0] = 0.00f;
		startAngles[1] = 0.00f;    startSpeeds[1] = 0.00f;
		startAngles[2] = 0.00f;    startSpeeds[2] = 0.00f;
		startAngles[3] = 0.00f;    startSpeeds[3] = 0.00f;
		startAngles[4] = 0.00f;    startSpeeds[4] = 0.00f;
		startAngles[5] = 0.00f;    startSpeeds[5] = 0.00f;
		startAngles[6] = 0.00f;    startSpeeds[6] = 0.00f;
		startAngles[7] = 0.00f;    startSpeeds[7] = 0.00f;
		startAngles[8] = 0.00f;    startSpeeds[8] = 0.00f;
		startAngles[9] = 0.00f;    startSpeeds[9] = 0.00f;
		
		// Send a few small speed controls to make it go straight ahead
		startAngles[10] = 0.00f;    startSpeeds[10] = 0.00f;
		startAngles[11] = 0.00f;    startSpeeds[11] = 0.00f;
		startAngles[12] = 0.00f;    startSpeeds[12] = 0.00f;
		startAngles[13] = 0.00f;    startSpeeds[13] = 0.00f;
		startAngles[14] = 0.00f;    startSpeeds[14] = 0.00f;
		startAngles[15] = 0.00f;    startSpeeds[15] = 0.00f;
		startAngles[16] = 0.00f;    startSpeeds[16] = 0.00f;
		startAngles[17] = 0.00f;    startSpeeds[17] = 0.00f;
		startAngles[18] = 0.00f;    startSpeeds[18] = 0.00f;
		startAngles[19] = 0.00f;    startSpeeds[19] = 0.00f;
	
		// Send the controls to port
		char send[10];  // Signal to be sent to serial port
	
		for ( int i = 0; i < numOfStartControls; i++ )
		{
			genSend( startAngles[i], startSpeeds[i], send );
			
			writeToPort( send );
	
			//cout << startAngles[i] << ", " << startSpeeds[i] << endl;
			cout << "Starting in " << 20 - i << " seconds..." << endl;
	
			Sleep( startControlPeriod );
		}
	
		cout << "Started!" << endl;
		// When in here, it should have an initial speed and zero angle
	}

	// This function sends a series of controls to stop the car.
	// The changes of controls should be smooth.
	void makeStop( )
	{
		// Controls length
		const int numOfStopControls = 7;
		vector< float > stopAngles(numOfStopControls);
		vector< float > stopSpeeds(numOfStopControls);
	
		const DWORD stopControlPeriod = 1000;
	
		// Specify angles and speeds
		// TODO: a better way is to loop up the current control state,
		// and then to determine a reasonal initial control angle.
		stopAngles[0] = 0.30f;    stopSpeeds[0] = 0.10f;
		stopAngles[1] = 0.25f;    stopSpeeds[1] = 0.05f;
		stopAngles[2] = 0.20f;    stopSpeeds[2] = 0.04f;
		stopAngles[3] = 0.15f;    stopSpeeds[3] = 0.03f;
		stopAngles[4] = 0.10f;    stopSpeeds[4] = 0.02f;
		stopAngles[5] = 0.05f;    stopSpeeds[5] = 0.01f;
		stopAngles[6] = 0.00f;    stopSpeeds[6] = 0.00f;
	
		// Send the controls to port
		
		char send[10];  // Signal to be sent to serial port
	
		for ( int i = 0; i < numOfStopControls; i++ )
		{
			genSend( stopAngles[i], stopSpeeds[i], send );
			
			writeToPort( send );
	
			//cout << stopAngles[i] << ", " << stopSpeeds[i] << endl;
			cout << "Stopping in " << 7 - i << " second..." << endl;
	
			Sleep( stopControlPeriod );
		}
	
		cout << "Stopped!" << endl;
	}

	int chooseDirection( float windowDepthLeft, float windowDepthMiddle, float windowDepthRight )
	{
		int direction = 0;  // Middle the deepest, default

		if ( windowDepthLeft > windowDepthMiddle && windowDepthLeft > windowDepthRight )
		{
			return -1;
		}

		if ( windowDepthMiddle < windowDepthRight && windowDepthLeft < windowDepthRight )
		{
			return 1;
		}
		return 0;
	}
	
	int driveCarByDirection( int direction )
	{
		if ( direction == 0 )
		{
			// Generate controls
			float angle = 0.0f;
			float speed = 0.2f;

			// Signal to be sent to serial port
			char send[10];

			// Executing duration
			const DWORD controlPeriod = 3000;
			genSend( angle, speed, send );

			writeToPort( send );
	
			Sleep( controlPeriod );
		}

		if ( direction == -1 )
		{
			// Generate controls
			float angle = -0.3f;
			float speed = 0.25f;

			// Signal to be sent to serial port
			char send[10];

			// Executing duration
			const DWORD controlPeriod = 3000;
			genSend( angle, speed, send );

			writeToPort( send );
	
			Sleep( controlPeriod );
		}

		if ( direction == 1 )
		{
			// Generate controls
			float angle = 0.3f;
			float speed = 0.2f;

			// Signal to be sent to serial port
			char send[10];

			// Executing duration
			const DWORD controlPeriod = 3000;
			genSend( angle, speed, send );

			writeToPort( send );
	
			Sleep( controlPeriod );
		}
	
		return 0;
	}

	int driveCarByDirection( float safeAngle )
	{
		// Generate controls
		float angle = 0.0f;
		float speed = 0.0f;

		// Normalize the control angle to locate between [-1, +1]
		if ( safeAngle > 10 )
		{
			angle = 0.8f;
		}
		else if ( safeAngle > 5 )
		{
			angle = 0.4f;
		}
		else if ( safeAngle > 0 )
		{
			angle = 0.2f;
		}
		else if ( safeAngle > -5 )
		{
			angle = -0.2f;
		}
		else if ( safeAngle > -10 )
		{
			angle = -0.4f;
		}
		else
		{
			angle = -0.8f;
		}
		
		// Signal to be sent to serial port
		char send[10];
		
		// Executing duration
		//const DWORD controlPeriod = 500;
		genSend( angle, speed, send );
		
		writeToPort( send );
		
		//Sleep( controlPeriod );

		return 0;
	}
}

bb2_wrapper m_camera(640,480);

const int maxDisp = 255;

IplImage* pfL;
IplImage* pfR;
IplImage* pframeL;
IplImage* pframeR;
IplImage* disp8;
IplImage* vdisp;
IplImage* udisp;
IplImage* edgesOfDepth;
IplImage* dilated;

// Function declarations
void miscellaneous::drawInterestPointsOnImage( IplImage* src, int flag );
void miscellaneous::drawInterestLinesOnImage( IplImage* src, int flag );
void miscellaneous::computeUDisparity( IplImage* src, IplImage* dst );
void miscellaneous::computeVDisparity( IplImage* src, IplImage* dst );
void miscellaneous::houghTransform( Mat& src );
void miscellaneous::computeAverageDepth( IplImage* disparity, double& left, double& middle, double& right );
float miscellaneous::sumArtificialForces( vector< PointCloud > pc_seq, GroundPlane groundPlane );
vector< DepthWindow> miscellaneous::computeAvrgDepth( vector< PointCloud > pc_seq, GroundPlane groundPlane );

int main( )
{	
	// Time control
	clock_t start;  // Start time
	clock_t finish;  // finish time is updated at the end of every loop
	double duration = 0;  // Compare this duration with the given one to limit the auto running duration
	
	// Start the car
	start = clock( );  // Start timing when starting the car
	controls::makeStart( );

	pfL = cvCreateImage(cvSize(640,480),8,3);
	pfR = cvCreateImage(cvSize(640,480),8,3);
	disp8 = cvCreateImage(cvSize(640,480),8,1);
	udisp = cvCreateImage( cvSize( 640, maxDisp ), 8, 1 );
	vdisp = cvCreateImage( cvSize( maxDisp, 480 ), 8, 1 );
	edgesOfDepth = cvCreateImage( cvSize( 640, 480 ), 8, 1 );
	dilated = cvCreateImage( cvSize( 640, 480 ), 8, 1 );

	//m_camera.setDispRange( 0, 200 );

	if( !m_camera.StartCamera() )
	{
		cout<<"StartCamera failed!"<<endl;
		return -1;
	}
	else
	{
		//m_camera.showCameraInfo( );
		m_camera.EnableStereoMatch( );
		while ( duration < 60 )  // Autorun 10 seconds
		{
			if(m_camera.AcquireFrame()&&m_camera.StereoMatch())
			{
				pframeL = m_camera.GetRetfImgL();
				pframeR = m_camera.GetRetfImgR();
				m_camera.Four2Three(pframeL,pfL);
				m_camera.Four2Three(pframeR,pfR);
				
				disp8 = m_camera.getStereo( );

				/*CvPoint pt1 = cvPoint( 320, 400 );
				CvPoint pt2 = cvPoint( 300, 410 );
				CvPoint pt3 = cvPoint( 340, 410 );
				int thickness = 5;
				cvLine( disp8, pt1, pt2, cvScalar( 255, 255, 255 ), thickness );
				cvLine( disp8, pt1, pt3, cvScalar( 255, 255, 255 ), thickness );
				cvLine( disp8, pt3, pt2, cvScalar( 255, 255, 255 ), thickness );
				cvLine( pfL, pt1, pt2, cvScalar( 255, 0, 0 ), thickness );
				cvLine( pfL, pt1, pt3, cvScalar( 255, 0, 0 ), thickness );
				cvLine( pfL, pt3, pt2, cvScalar( 255, 0, 0 ), thickness );
				cvLine( pfR, pt1, pt2, cvScalar( 0, 255, 0 ), thickness );
				cvLine( pfR, pt1, pt3, cvScalar( 0, 255, 0 ), thickness );
				cvLine( pfR, pt3, pt2, cvScalar( 0, 255, 0 ), thickness );*/
				//double l, m, r;
				//miscellaneous::computeAverageDepth( disp8, l, m, r );
				//cout << l << ", " << m << ", " << r << endl;

				/*miscellaneous::BwImage disp8_wrapper( disp8 );
				for ( int i = 0; i < 640; i++ )
					for ( int j = 0; j < 480; j++ )
					{
						cout << (int)disp8_wrapper[j][i] << '\t';
					}
				*/
				
				/*
				miscellaneous::computeUDisparity( disp8, udisp );
				miscellaneous::computeVDisparity( disp8, vdisp );
				cvShowImage( "udisp", udisp );
				cvShowImage( "vdisp", vdisp );
				*/

				/*
				Mat iplWrapper( disp8 );
				miscellaneous::houghTransform( iplWrapper );
				imshow( "lines", iplWrapper );
				*/

				//m_camera.showInterestPointsDepth( );

				//m_camera.showInterestPoints3D();

				//m_camera.showAvrgDepth( );

				//vector< CvPoint3D32f > iA( 9 );
				//iA = m_camera.getNinePoints( );

				/*m_camera.showSampledGround( );
				cv::imshow( "ground", m_camera.groundImg );
				cv::Mat lraw( pfL );
				cv::imshow( "mixed", lraw+m_camera.groundImg );
				m_camera.groundImg.setTo( cv::Scalar(0) );*/
				
				//cout << iA[0].x << ", " << iA[0].y << ", " << iA[0].z << endl;

				vector< PointCloud > framePc = m_camera.getFramePointClouds( );
				GroundPlane frameGp = m_camera.getGroundPlane( );
				
				float controlAngle = miscellaneous::sumArtificialForces( framePc, frameGp );
				controls::driveCarByDirection( -controlAngle );
				cout << "\t" << -controlAngle << endl;

				//Sleep( 1000 );

				//cout << "\t" << miscellaneous::sumArtificialForces( framePc, frameGp ) << endl << endl;
				//cout << framePc.size( ) << endl;
				//vector< DepthWindow> dw = miscellaneous::computeAvrgDepth( framePc, frameGp );

				/*int dir = controls::chooseDirection( dw[0].avrgDepth / dw[0].numOfPoints, 
					                               dw[1].avrgDepth / dw[1].numOfPoints,
												   dw[2].avrgDepth / dw[2].numOfPoints );*/

				/*cout << "Average depth: " << '\t' << endl
					 << dw[0].avrgDepth / dw[0].numOfPoints  << ", " 
					 << dw[1].avrgDepth / dw[1].numOfPoints << ", " 
					 << dw[2].avrgDepth / dw[2].numOfPoints << ",Direction:\t"
					 << dir << endl;*/


				//controls::driveCarByDirection( dir );

				//vector< int > randomIndex( 3 );
				//randomIndex = aR.genRandomIndex( );		
				//cout << randomIndex[0] << ": " << iA[randomIndex[0]].x << ", " << iA[randomIndex[0]].y << ", " << iA[randomIndex[0]].z << endl;

				//cvCanny(disp8, edgesOfDepth, 20, 100 );
				//cvDilate( disp8, dilated);

				//miscellaneous::drawInterestPointsOnImage( pfR, 1 );
				//miscellaneous::drawInterestLinesOnImage( pfR, 1 );

				//miscellaneous::drawInterestPointsOnImage( disp8, 0 );
				//miscellaneous::drawInterestLinesOnImage( disp8, 0 );

				//miscellaneous::drawInterestPointsOnImage( pfL, 1 );
				//miscellaneous::drawInterestLinesOnImage( pfL, 1 );

				cvShowImage("Left",pfL);
				//cvShowImage("Right",pfR);
				
				cvShowImage("Disp8", disp8 );
				//cvShowImage("edge", edgesOfDepth );
				//cvShowImage( "dilated", dilated );
				//if(cvWaitKey(20)==27) break;
				cvWaitKey( 5 );
			}

			finish = clock( );

			duration = static_cast< double >( finish - start ) / CLOCKS_PER_SEC;
		}	
	}
	controls::makeStop( );

	m_camera.StopCamera();
	//cvDestroyWindow("Left");
	//vDestroyWindow("Right");

	return 0;
}
#endif

#if 0
// System Inlcudes
#include <vector>
using std::vector;
#include <iostream>
using std::cout;
using std::endl;

#include "serial\syncSerialComm.h"

// Generate controls with some kind of algorithm
void genControls( vector< float >& angles, vector< float >& speeds )
{
	// Controls length
	const int numOfControls = 10;

	// Controls ranges
	const float maxAngle =  0.7f;
    const float minAngle = -0.7f;
    const float maxSpeed =  0.7f;

	// Specify angles
	angles[0] =  0.5f;
    angles[1] = -0.5f;
    angles[2] =  0.5f;
    angles[3] = -0.5f;
    angles[4] =  0.5f;

    angles[5] = -0.5f;
    angles[6] =  0.5f;
    angles[7] = -0.5f;
    angles[8] =  0.5f;
    angles[9] = -0.5f;

	// Limit the angle controls to a safe range
    for ( int i = 0; i < numOfControls; i++ )
    {
        if ( angles[i] > maxAngle )
        {
            angles[i] = maxAngle;
        }

        if ( angles[i] < minAngle )
        {
            angles[i] = minAngle;
        }
    }

	// Compute speeds according to specified angles
    for ( int i = 0; i < numOfControls; i++ )
    {
        const float p = 10.0f;
        speeds[i] = maxSpeed / sqrt( fabs( p * angles[i] ) + 1.0f );
    }
}
// This function sends a series of controls to start the car.

void genSend( float angle, float speed, char* send )
{
	send[0] = 0xff;
	send[1] = 0xfe;

	// Speed control
	const int *p = (int *)&speed;
	send[5] = (*p>>24)&0xff;
	send[4] = (*p>>16)&0xff;
	send[3] = (*p>>8)&0xff;
	send[2] = (*p>>0)&0xff;

	// Angle control
	p = (int *)&angle;
	send[9] = (*p>>24)&0xff;
	send[8] = (*p>>16)&0xff;
	send[7] = (*p>>8)&0xff;
	send[6] = (*p>>0)&0xff;
}

int writeToPort( const char *send )
{
	CSyncSerialComm s0("COM2");
	s0.Open( );
	s0.ConfigPort( CBR_115200, 5 );
	s0.Write( send, 10 );
	s0.Close( );

	return 0;
}

// This function sends a series of controls to start the car.
void makeStart( )
{
	// Controls length
	const int numOfStartControls = 20;
	vector< float > startAngles(numOfStartControls);
	vector< float > startSpeeds(numOfStartControls);
	const DWORD startControlPeriod = 500;

	// Specify angles and speeds
	// Send empty controls for a few periods
	startAngles[0] = 0.00f;    startSpeeds[0] = 0.00f;
	startAngles[1] = 0.00f;    startSpeeds[1] = 0.00f;
	startAngles[2] = 0.00f;    startSpeeds[2] = 0.00f;
	startAngles[3] = 0.00f;    startSpeeds[3] = 0.00f;
	startAngles[4] = 0.00f;    startSpeeds[4] = 0.00f;
	startAngles[5] = 0.00f;    startSpeeds[5] = 0.00f;
	startAngles[6] = 0.00f;    startSpeeds[6] = 0.00f;
	startAngles[7] = 0.00f;    startSpeeds[7] = 0.00f;
	startAngles[8] = 0.00f;    startSpeeds[8] = 0.00f;
	startAngles[9] = 0.00f;    startSpeeds[9] = 0.00f;
	
	// Send a few small speed controls to make it go straight ahead
	startAngles[10] = 0.00f;    startSpeeds[10] = 0.10f;
	startAngles[11] = 0.00f;    startSpeeds[11] = 0.10f;
	startAngles[12] = 0.00f;    startSpeeds[12] = 0.10f;
	startAngles[13] = 0.00f;    startSpeeds[13] = 0.10f;
	startAngles[14] = 0.00f;    startSpeeds[14] = 0.10f;
	startAngles[15] = 0.00f;    startSpeeds[15] = 0.10f;
	startAngles[16] = 0.00f;    startSpeeds[16] = 0.10f;
	startAngles[17] = 0.00f;    startSpeeds[17] = 0.10f;
	startAngles[18] = 0.00f;    startSpeeds[18] = 0.10f;
	startAngles[19] = 0.00f;    startSpeeds[19] = 0.10f;

	// Send the controls to port
	char send[10];  // Signal to be sent to serial port

	for ( int i = 0; i < numOfStartControls; i++ )
	{
		genSend( startAngles[i], startSpeeds[i], send );
		
		writeToPort( send );

		cout << startAngles[i] << ", " << startSpeeds[i] << endl;

		Sleep( startControlPeriod );
	}

	cout << "Started!" << endl;
	// When in here, it should have an initial speed and zero angle
}

// This function sends a series of controls to stop the car.
// The changes of controls should be smooth.
void makeStop( )
{
	// Controls length
	const int numOfStopControls = 7;
	vector< float > stopAngles(numOfStopControls);
	vector< float > stopSpeeds(numOfStopControls);

	const DWORD stopControlPeriod = 500;

	// Specify angles and speeds
	// TODO: a better way is to loop up the current control state,
	// and then to determine a reasonal initial control angle.
	stopAngles[0] = 0.30f;    stopSpeeds[0] = 0.30f;
	stopAngles[1] = 0.25f;    stopSpeeds[1] = 0.25f;
	stopAngles[2] = 0.20f;    stopSpeeds[2] = 0.20f;
	stopAngles[3] = 0.15f;    stopSpeeds[3] = 0.15f;
	stopAngles[4] = 0.10f;    stopSpeeds[4] = 0.10f;
	stopAngles[5] = 0.05f;    stopSpeeds[5] = 0.05f;
	stopAngles[6] = 0.00f;    stopSpeeds[6] = 0.00f;

	// Send the controls to port
	
	char send[10];  // Signal to be sent to serial port

	for ( int i = 0; i < numOfStopControls; i++ )
	{
		genSend( stopAngles[i], stopSpeeds[i], send );
		
		writeToPort( send );

		cout << stopAngles[i] << ", " << stopSpeeds[i] << endl;

		Sleep( stopControlPeriod );
	}

	cout << "Stopped!" << endl;
}

int main(  )
{

	makeStart( );

	// Generate controls
	const int numOfControls( 10 );
	vector< float > angles( numOfControls );
	vector< float > speeds( numOfControls );
	genControls( angles, speeds );

	// Signal to be sent to serial port
	char send[10];

	const DWORD controlPeriod = 1000;
	for ( int i = 0; i < numOfControls; i++ )
	{
		genSend( angles[i], speeds[i], send );
		cout << "angle: " << angles[i] << ",	speed: " << speeds[i] << endl << endl;
		
		writeToPort( send );

		Sleep( controlPeriod );
	}

	makeStop( );

	return 0;
}

#endif
