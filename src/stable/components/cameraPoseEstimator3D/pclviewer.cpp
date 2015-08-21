#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "myprogeo.h"
#include <cvsba/cvsba.h>

PCLViewer::PCLViewer(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::PCLViewer)
{
  ui->setupUi(this);
  this->setWindowTitle("cameraPoseEstimator3D");

  // Setup the cloud pointer
  cloud.reset(new PointCloudT);
  // The number of keyframes
  keyframe_num = 0;

  // Set up the QVTK window
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
  ui->qvtkWidget->update();

  // Connect button and the function
  connect(ui->pushButton_save_pose,  SIGNAL(clicked()), this, SLOT(savePoseButtonPressed()));
  connect(ui->pushButton_load_cloud, SIGNAL(clicked()), this, SLOT(loadCloudButtonPressed()));
  connect(ui->pushButton_load_keyframes, SIGNAL(clicked()), this, SLOT(loadKeyframesButtonPressed()));
  connect(ui->pushButton_load_image, SIGNAL(clicked()), this, SLOT(loadImageButtonPressed()));
  connect(ui->pushButton_estimate_pose, SIGNAL(clicked()), this, SLOT(estimatePoseButtonPressed()));

  viewer->addPointCloud(cloud, "cloud");
  viewer->resetCamera();
  ui->qvtkWidget->update();
}

void
PCLViewer::savePoseButtonPressed()
{
  // Save the estimated camera pose
}

void
PCLViewer::loadImageButtonPressed()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                     tr("Open File"), QDir::currentPath());
     if (!fileName.isEmpty()) {
         QImage image(fileName);
         if (image.isNull()) {
             QMessageBox::information(this, tr("Load Target Image"),
                                      tr("Cannot load %1.").arg(fileName));
             return;
         }

         // Fill target image
         target_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);

         ui->label_target_image->setPixmap(QPixmap::fromImage(image));
         ui->label_target_image->setScaledContents(true);
         ui->label_target_image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
     }
}

void
PCLViewer::loadCloudButtonPressed()
{
    // You might want to change "/home/" to suit your system
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

    PCL_INFO("File chosen: %s\n", fileName.toStdString().c_str());
    PointCloudT::Ptr cloud_tmp(new PointCloudT);

    if (fileName.isEmpty()) {
        PCL_INFO("File chosen: user cancelled...\n");
        return;
    }

    int return_status;
    if (fileName.endsWith(".pcd", Qt::CaseInsensitive))
      return_status = pcl::io::loadPCDFile(fileName.toStdString(), *cloud_tmp);
    else
      return_status = pcl::io::loadPLYFile(fileName.toStdString(), *cloud_tmp);

    if (return_status != 0)
    {
      PCL_ERROR("Error reading point cloud %s\n", fileName.toStdString().c_str());
      return;
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
      pcl::copyPointCloud(*cloud_tmp, *cloud);
    else
    {
      PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
      std::vector<int> vec;
      pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud, vec);
    }

    viewer->updatePointCloud(cloud, "cloud");
    viewer->resetCamera();
    ui->qvtkWidget->update();    
}

void
PCLViewer::loadKeyframesButtonPressed()
{
    // You might want to change "/home/" to suit your system
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Keyframes data file(*.jde)"));

    PCL_INFO("File chosen: %s\n", fileName.toStdString().c_str());

    if (fileName.isEmpty()) {
        PCL_INFO("File chosen: user cancelled...\n");
        return;
    }

    std::string keyframeDataFileName = "keyframeData.jde";
    std::string basePath = fileName.toStdString();
    basePath.erase(basePath.length() - keyframeDataFileName.length(), keyframeDataFileName.length());

    PCL_INFO("Base path: %s\n", basePath.c_str());

    cv::Mat imageColor, colorDepth, imageDepth;
    rtabmap::Transform cameraTransform;
    std::string line;
    std::string imageFileName, depthFileName, poseFileName;
    std::ifstream keyframeDataFile(fileName.toStdString().c_str());
    if (keyframeDataFile.is_open()) {
        keyframe_images.clear();
        keyframe_depths.clear();
        keyframe_poses.clear();
        keyframe_num = 0;

        while (std::getline(keyframeDataFile, line)) {
            imageFileName = basePath + line + "_color.png";
            depthFileName = basePath + line + "_depth.png";
            poseFileName  = basePath + line + "_pose.txt";

            // Read color image
            //imageColor = cv::imread(imageFileName, CV_LOAD_IMAGE_GRAYSCALE);
            imageColor = cv::imread(imageFileName);

            // Read depth image and color to CV_16UC1
            colorDepth = cv::imread(depthFileName);
            imageDepth = cv::Mat(cv::Size(imageColor.cols, imageColor.rows), CV_16UC1);
            std::vector<cv::Mat> layers;    
            cv::split(colorDepth, layers);  
          
            for (int x = 0; x < layers[1].cols ; x++) {  
                    for (int y = 0; y < layers[1].rows; y++) { 
                        imageDepth.at<unsigned short>(y,x) = ((int)layers[1].at<unsigned char>(y,x)<<8)|(int)layers[2].at<unsigned char>(y,x);
                        //std::cout<<imageDepth.at<float>(y,x)<<" ";
                    }
                    //std::cout<<std::endl;         
            }

            // Read in camera pose
            std::ifstream poseFile(poseFileName.c_str());
            poseFile>>cameraTransform;
            poseFile.close();

            keyframe_images.push_back(imageColor);
            keyframe_depths.push_back(imageDepth);
            keyframe_poses.push_back(cameraTransform);

            keyframe_num++;
        }
    }
    std::cout<<keyframe_num<<" keyframes loaded."<<std::endl;
    keyframeDataFile.close();
}

void
PCLViewer::estimatePoseButtonPressed()
{
    // Step.1 Extract the features of the target image and find the nearest neignbour

    int minHessian = 400;

    // Detector
    cv::SurfFeatureDetector detector( minHessian );

    std::vector<cv::KeyPoint> keypoints_target, keypoints_tmp;
    cv::Mat image_tmp;

    detector.detect( target_image, keypoints_target );

    // Extractor
    cv::SurfDescriptorExtractor extractor;

    cv::Mat descriptors_target, descriptors_tmp;

    extractor.compute( target_image, keypoints_target, descriptors_target );

    max_matches = 0;
    match_id = 0;

    for (int iter = 0; iter < keyframe_num; ++iter) {
        image_tmp = keyframe_images[iter];
        
        //-- Step 1: Detect the keypoints using SURF Detector
        detector.detect( image_tmp, keypoints_tmp );
        
        //-- Step 2: Calculate descriptors (feature vectors)
        extractor.compute( image_tmp, keypoints_tmp, descriptors_tmp );

        //-- Step 3: Matching descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;
        matcher.match( descriptors_target, descriptors_tmp, matches );

        double max_dist = 0; double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors_target.rows; i++ )
        {   double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector< cv::DMatch > good_matches;

        for( int i = 0; i < descriptors_target.rows; i++ )
        {   if( matches[i].distance <= cv::max(2*min_dist, 0.02) )
            { good_matches.push_back( matches[i]); }
        }

        //-- Draw only "good" matches
        cv::Mat img_matches;
        cv::drawMatches( target_image , keypoints_target,  image_tmp, keypoints_tmp,
                     good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Show detected matches
        //cv::imshow( "Good Matches", img_matches );

        /*
        for( int i = 0; i < (int)good_matches.size(); i++ )
        {
            printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
        }
        */

        printf("Found %d good matches between target and image %d/%d.\n", good_matches.size(), iter+1, keyframe_num);

        if (good_matches.size() > max_matches) {
            target_image_matches = img_matches;
            max_matches = good_matches.size();
            match_id = iter;
        }

        //cv::waitKey(0); 
    }
    
    printf("Match image is %d with %d good matches.\n", match_id, max_matches);
    cv::imshow("Match Image", keyframe_images[match_id]);
    cv::imshow("Good Matches", target_image_matches);

    // Step.2 Construct the Bundle Adjustment Problem
    std::vector< cv::Point3d > points3D;
    std::vector< std::vector< cv::Point2d > > pointsImg;
    std::vector< std::vector< int > > visibility;
    std::vector< cv::Mat > cameraMatrix, distCoeffs, R, T;
         
    int NPOINTS = max_matches; 	// number of 3d points
    int NCAMS = 2; 	// number of cameras, best_match and target
 
    // Back project match image to 3D in terms of its pose and depth
    rtabmap::Transform bestMatchImagePose = keyframe_poses[match_id];
    cv::Mat bestMatchImageDepth = keyframe_depths[match_id];

    std::cout<<"Best match image pose:"<<std::endl;
    std::cout<<bestMatchImagePose;

    // Transform camera parameters to myprogeo format
    
    // Use myproeo->mybackproject to get 3D points
   
    // fill 3D points
    points3D.resize(NPOINTS);
    
    // fill image projections
    pointsImg.resize(NCAMS);
    for(int i = 0; i < NCAMS; i++) pointsImg[i].resize(NPOINTS);    
     
    // fill visibility (all points are visible)
    visibility.resize(NCAMS);
    for(int i = 0; i < NCAMS; i++)  {
      visibility[i].resize(NPOINTS);       
      for(int j = 0; j < NPOINTS; j++) visibility[i][j] = 1;
    }

    // fill camera intrinsics (same intrinsics for all cameras)
    cameraMatrix.resize(NCAMS);
    for(int i=0; i<NCAMS; i++) {
      cameraMatrix[i] = cv::Mat::eye(3,3,CV_64FC1);
      cameraMatrix[i].ptr<double>(0)[0]=cameraMatrix[i].ptr<double>(0)[4]=1000.;
      cameraMatrix[i].ptr<double>(0)[2]=600.;cameraMatrix[i].ptr<double>(0)[5]=400.;
    }
   
    // fill distortion (assume no distortion)
    distCoeffs.resize(NCAMS);
    for(int i=0; i<NCAMS; i++) distCoeffs[i] = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0));
     
    // fill rotation (rotation around Z axis)
    R.resize(NCAMS);
    for(int i=0; i<NCAMS; i++) {
      R[i] = cv::Mat(3,1,CV_64FC1,cv::Scalar::all(0));
      R[i].ptr<double>(0)[2] = double(i)*2.0944;
    }     
          
    // fill translation (3 units away in camera Z axis)
    T.resize(NCAMS);
    for(int i=0; i<NCAMS; i++) {
      T[i] = cv::Mat(3,1,CV_64FC1,cv::Scalar::all(0));
      T[i].ptr<double>(0)[2] = 3.;
    }
 
    // Step.3 Solve the BA problem using cvsba
    cvsba::Sba sba;
    cvsba::Sba::Params params;
    sba.setParams(params);
    sba.run(points3D,  pointsImg,  visibility,  cameraMatrix,  R,  T, distCoeffs);
    
    std::cout<<"Initial error="<<sba.getInitialReprjError()<<". Final error="<<sba.getFinalReprjError()<<std::endl;
}

PCLViewer::~PCLViewer()
{
  delete ui;
}
