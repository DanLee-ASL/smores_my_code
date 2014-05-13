#include "GLViewer.h"
#include "zpr.h"

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

// this is fking stupid. C++ require actual definition for static members.
int GLViewer::numPoints;
gazebo::math::Pose GLViewer::lidarPose;
PCL_OCTREE_IMPL *GLViewer::pcl_octree_impl;
std::vector<pcl::PointXYZ> GLViewer::points;

static pthread_mutex_t lidar_mutex;


GLViewer::GLViewer()
{
  pcl_octree_impl = new PCL_OCTREE_IMPL(500000, 1);
  points.clear();
  
  boost::thread timerThread(&GLViewer::GetLidarPointsThread);
}

void GLViewer::GetLidarPointsThread()
{
  while(true) 
  {
    pcl_octree_impl->DownSample(0.05f);
    points = pcl_octree_impl->GetPoints();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
}

// This function set the GLOBAL pose of the lidar scan
void GLViewer::SetLidarPose(gazebo::math::Pose p)
{
  GLViewer::lidarPose = p;
}

void GLViewer::SetLidarPoints(double* range, int nPoints, double angleMin, double angleMax, double angleStepSize, double minRange, double maxRange){
    
    pthread_mutex_lock(&lidar_mutex);
    double* rangeCopy = (double*)malloc(sizeof(double) * nPoints);
    memcpy(rangeCopy, range, sizeof(double) * nPoints);

    for(int i = 0; i < nPoints; i++)
    {
//       std::cout << i << std::endl;
      double r = rangeCopy[i];
      if(r >= maxRange || r <= minRange)
		continue;
      double angle = angleMin + angleStepSize * i;
      double x = r * cos(angle);
      double y = r * sin(angle);
      gazebo::math::Pose lidarPtPose(x, y, 0, 0, 0, 0);
      gazebo::math::Pose globalLidarPtPose = lidarPtPose + lidarPose;
      pcl_octree_impl->AddPoint(globalLidarPtPose.pos.x, globalLidarPtPose.pos.y, globalLidarPtPose.pos.z);
    }
//     pcl_octree_impl->DownSample();
    delete rangeCopy;
    pthread_mutex_unlock(&lidar_mutex);
    this->numPoints = nPoints;
}

void GLViewer::Draw() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		     // Clear Screen and Depth Buffer

  glBegin(GL_POINTS);
  glColor4f(0.95f, 0.207, 0.031f, 1.0f);
  for(int i = 0; i < points.size(); i++)
  {
    double x = points[i].x;
    double y = points[i].y;
    double z = points[i].z;
    glVertex3d(x, y, z);
  }
  glEnd();
  glFinish();

  DrawAxes();

  glutSwapBuffers();
    
}

void GLViewer::Pick(GLint name)
{
   printf("Pick: %d\n",name);
   fflush(stdout);
}
void GLViewer::DrawAxes(void)
{
  /* Name-stack manipulation for the purpose of
      selection hit processing when mouse button
      is pressed.  Names are ignored in normal
      OpenGL rendering mode.                    */

    glPushMatrix();
			      /* No name for grey sphere */

    glColor3f(0.3,0.3,0.3);
    glutSolidSphere(0.05, 20, 20);

    glPushMatrix();
    glPushName(1);            /* Red cone is 1 */
	glColor3f(1,0,0);
	glRotatef(90,0,1,0);
	glutSolidCone(0.05, 0.5, 20, 20);
    glPopName();
    glPopMatrix();

    glPushMatrix ();
    glPushName(2);            /* Green cone is 2 */
	glColor3f(0,1,0);
	glRotatef(-90,1,0,0);
	glutSolidCone(0.05, 0.5, 20, 20);
    glPopName();
    glPopMatrix();

    glColor3f(0,0,1);         /* Blue cone is 3 */
    glPushName(3);
	glutSolidCone(0.05, 0.5, 20, 20);
    glPopName();

  glPopMatrix();
}

void GLViewer::Initialize () 
{
//     glViewport(0, 0, win.width, win.height);									// set the viewport
//     glLoadIdentity();															// reset projection matrix
//     GLfloat aspect = (GLfloat) win.width / win.height;
//     gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);		// set up a perspective projection matrix
//     glMatrixMode(GL_MODELVIEW);													// specify which matrix is the current matrix
//     glShadeModel( GL_SMOOTH );

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glClearColor(1.0, 1.0, 1.0, 1.0);											// specify clear values for the color buffers								
}

void GLViewer::Run(int argc, char** argv)
{
    win.width = 640;
    win.height = 480;
    win.title = "OpenGL/GLUT Example. Visit http://openglsamples.sf.net ";
    win.field_of_view_angle = 45;
    win.z_near = 1.0f;
    win.z_far = 500.0f;
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(win.width, win.height);
    glutCreateWindow(win.title);
    
    glutDisplayFunc(Draw);
    glutIdleFunc(Draw);
    
    zprInit();
    zprSelectionFunc(DrawAxes);
    zprPickFunc(Pick);
    
    Initialize();
    glutMainLoop();

}
