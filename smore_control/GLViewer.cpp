#include "GLViewer.h"
#include "zpr.h"

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <GL/glut.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/bind.hpp>

// this is fking stupid. C++ require actual definition for static members.
int GLViewer::numPoints;
gazebo::math::Pose GLViewer::lidarPose;
PCL_OCTREE_IMPL *GLViewer::pcl_octree_impl;
std::vector<pcl::PointXYZ> GLViewer::points;
std::vector<octomap::point3d> GLViewer::voxelVector;
float GLViewer::cubeSize;

static pthread_mutex_t lidar_mutex;
static pthread_mutex_t cubePlotMutex;

GLViewer::GLViewer()
{
  pcl_octree_impl = new PCL_OCTREE_IMPL(500000, 1);
  points.clear();
  lidarUpdateCounter = 0;
  getVoxelFrequency = 100;
}

GLViewer::~GLViewer()
{
	pcl_octree_impl->~PCL_OCTREE_IMPL();
}

void GLViewer::GetOccupiedVoxels()
{
    if(++lidarUpdateCounter >= getVoxelFrequency)
    {
        voxelVector = pcl_octree_impl->GetVoxels(16, cubeSize);
        lidarUpdateCounter = 0;
        occupiedCellUpdated(voxelVector);
    }
}

// This function set the GLOBAL pose of the lidar scan
void GLViewer::SetLidarPose(gazebo::math::Pose p)
{
  GLViewer::lidarPose = p;
}

void GLViewer::SetLidarPoints(double* range, int nPoints, double angleMin, double angleMax, double angleStepSize, double minRange, double maxRange){

    if(nPoints <= 0)
	return;
    pthread_mutex_lock(&lidar_mutex);
    double* rangeCopy = (double*)malloc(sizeof(double) * nPoints);
    memcpy(rangeCopy, range, sizeof(double) * nPoints);
	
    std::vector<double> xPts, yPts, zPts;
//     std::cout << nPoints << std::endl;
    for(int i = 0; i < nPoints; i++)
    {
		double r = rangeCopy[i];
		if(r >= (maxRange - 0.5) || r <= (minRange + 0.5) )
			continue;
		double angle = angleMin + angleStepSize * i;
		double x = r * cos(angle);
		double y = r * sin(angle);
// 		printf("x: %f, y:%f\n", x, y);
		gazebo::math::Pose lidarPtPose(x, y, 0, 0, 0, 0);
		gazebo::math::Pose globalLidarPtPose = lidarPtPose + lidarPose;
		
		xPts.push_back(globalLidarPtPose.pos.x);
		yPts.push_back(globalLidarPtPose.pos.y);
		zPts.push_back(globalLidarPtPose.pos.z);
    }
    pcl_octree_impl->AddScan(&xPts[0], &yPts[0], &zPts[0], nPoints, lidarPose.pos.x, lidarPose.pos.y, lidarPose.pos.z);
    pthread_mutex_unlock(&lidar_mutex);
    delete rangeCopy;
    this->numPoints = nPoints;
}

void GLViewer::Draw() 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		     // Clear Screen and Depth Buffer
	VoxelDraw(cubeSize);
}


void GLViewer::VoxelDraw(float cubeSize) 
{
	pthread_mutex_lock(&cubePlotMutex);
	glBegin(GL_POINTS);
	for(int i = 0; i < voxelVector.size(); i++)
	{
		octomap::point3d pt = voxelVector[i];
		// top surface
		glBegin(GL_POLYGON);
		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.5f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glEnd();
		
		// bottom surface
		glBegin(GL_POLYGON);
		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.5f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glEnd();
		
		// front surface
		glBegin(GL_POLYGON);
		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.5f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glEnd();
		
		// back surface
		glBegin(GL_POLYGON);
		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.5f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glEnd();
		
		// right surface
		glBegin(GL_POLYGON);
		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.5f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() + (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glEnd();
		
		// left surface
		glBegin(GL_POLYGON);
		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.5f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() - (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() - cubeSize / 2.0f);
		glVertex3f(pt.x() - (cubeSize / 2.0f), pt.y() + (cubeSize / 2.0f), pt.z() + cubeSize / 2.0f);
		glEnd();
		
		
// 		glColor4f(0, 102 / 255.0f, 204 / 255.0f, 0.1f);
// 		glVertex3f(pt.x(), pt.y(), pt.z());
	}
// 	glEnd();

	glFinish();
	glutSwapBuffers();
	pthread_mutex_unlock(&cubePlotMutex);
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
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.0, 0.0, 0.0, 0.0);											// specify clear values for the color buffers								
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
	pcl_octree_impl->ocGridUpdatedSignal.connect(boost::bind(&GLViewer::GetOccupiedVoxels, this)); // subscribe the listner
    glutMainLoop();

}
