#include <GL/glew.h>
#include <gazebo/math/Pose.hh>
#include "pcl_octree_impl.h"

class GLViewer {

typedef struct {
    int width;
	int height;
	char* title;

	float field_of_view_angle;
	float z_near;
	float z_far;
} glutWindow;  
  
public:
  GLViewer();
  void Run(int argc, char** argv);
  void SetLidarPoints(double* range, int nPoints, double angleMin, double angleMax, double angleStepSize, double minRange, double maxRange);
  void SetLidarPose(gazebo::math::Pose p);
    
private:
  
  void Initialize();
  
  glutWindow win;
  static PCL_OCTREE_IMPL *pcl_octree_impl;
  
  
  static void Draw();
  static int numPoints;
  static void Pick(GLint name);
  static void DrawAxes(void);
  static gazebo::math::Pose lidarPose;
  static std::vector<pcl::PointXYZ> points;
  static void GetLidarPointsThread();
};

