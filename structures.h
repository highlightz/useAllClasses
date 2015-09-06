#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <opencv2/core/core.hpp>

struct GroundPlanePoints
{
	CvPoint3D32f planePointA;
	int numOfEffectivePointsAourndA;

	CvPoint3D32f planePointB;
	int numOfEffectivePointsAourndB;

	CvPoint3D32f planePointC;
	int numOfEffectivePointsAourndC;
}; 

struct GroundPlane
{
	float coefs_0;
	float coefs_1;
	float coefs_2;
	float coefs_3;
};

// The point cloud refers to camera as a reference coordinate frame.
struct PointCloud
{
	float x, y, z;
	int r, g, b;
	int row, col;
};

// Such a struct object represents a neighborhood of an interest point,
// which serves to determine the ground plane.
// Three of them are needed.
struct DepthWindow
{
	float avrgDepth;  // Average depth of a window
	int numOfPoints;  // Number of points of this window
	int numPixelLessThanFixedDepthThreshold;  // Number of points with depth less than a fixed value
	DepthWindow( )
	{
		avrgDepth = 0.0f;
		numOfPoints = 0;
		numPixelLessThanFixedDepthThreshold = 0;
	}
};

// This struct stores the necessary pose information of a gound vehicle.
struct odometry
{
	double x;  // axis of which points to the right
	double y;  // axis of which points to the earth
	double z;  // axis of which points to the front

	double yaw_rad;  // around y axis, positive when turning right horizontally
};

#endif  // STRUCTURES_H
