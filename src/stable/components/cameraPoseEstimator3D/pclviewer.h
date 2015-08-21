#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Qt
#include <QMainWindow>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QSizePolicy>

// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "Transform.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer();

public slots:
  void
  savePoseButtonPressed();

  void
  loadImageButtonPressed();

  void
  loadCloudButtonPressed();

  void
  loadKeyframesButtonPressed();

  void
  estimatePoseButtonPressed();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

  // Containers
  cv::Mat target_image;     // Target image to estimate pose
  int keyframe_num;
  std::vector<cv::Mat> keyframe_images;
  std::vector<cv::Mat> keyframe_depths;
  std::vector<rtabmap::Transform> keyframe_poses;
  
  std::vector< cv::Point3d > points3D;
  std::vector< std::vector< cv::Point2d > > pointsImg;
  std::vector< std::vector< int > > visibility;
  std::vector< cv::Mat > cameraMatrix, distCoeffs, R, T; 
       
  int NPOINTS;    // number of 3d points
  int NCAMS;   // number of cameras 

  cv::Mat target_image_matches;
  int max_matches;
  int match_id;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
