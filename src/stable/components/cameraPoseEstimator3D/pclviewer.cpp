#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <cvsba/cvsba.h>

#undef EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

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
PCLViewer::estimatePoseButtonPressed() {
    // Step.1 Extract the features of the target image and find the nearest neignbour

    int minHessian = 400;

    // Detector
    cv::SurfFeatureDetector detector(minHessian);

    std::vector <cv::KeyPoint> keypoints_target, keypoints_tmp, keypoints_best_match;
    cv::Mat image_tmp;

    detector.detect(target_image, keypoints_target);

    // Extractor
    cv::SurfDescriptorExtractor extractor;

    cv::Mat descriptors_target, descriptors_tmp, descriptors_best_match;

    extractor.compute(target_image, keypoints_target, descriptors_target);

    std::vector <cv::DMatch> best_matches;
    max_matches = 0;
    match_id = 0;

    for (int iter = 0; iter < keyframe_num; ++iter) {
        image_tmp = keyframe_images[iter];

        //-- Step 1: Detect the keypoints using SURF Detector
        detector.detect(image_tmp, keypoints_tmp);

        //-- Step 2: Calculate descriptors (feature vectors)
        extractor.compute(image_tmp, keypoints_tmp, descriptors_tmp);

        //-- Step 3: Matching descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher;
        std::vector <cv::DMatch> matches;
        matcher.match(descriptors_target, descriptors_tmp, matches);

        double max_dist = 0;
        double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_target.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector <cv::DMatch> good_matches;

        for (int i = 0; i < descriptors_target.rows; i++) {
            if (matches[i].distance <= cv::max(2 * min_dist, 0.02)) { good_matches.push_back(matches[i]); }
        }

        //-- Draw only "good" matches
        cv::Mat img_matches;
        cv::drawMatches(target_image, keypoints_target, image_tmp, keypoints_tmp,
                        good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        //-- Show detected matches
        //cv::imshow( "Good Matches", img_matches );

        /*
        for( int i = 0; i < (int)good_matches.size(); i++ )
        {
            printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
        }
        */

        printf("Found %d good matches between target and image %d/%d.\n", good_matches.size(), iter + 1, keyframe_num);

        if (good_matches.size() > max_matches) {
            target_image_matches = img_matches;
            keypoints_best_match = keypoints_tmp;
            descriptors_best_match = descriptors_tmp;
            max_matches = good_matches.size();
            match_id = iter;
            best_matches = good_matches;
        }
        //cv::waitKey(0); 
    }

    printf("Match image is %d with %d good matches.\n", match_id, max_matches);
    cv::imshow("Match Image", keyframe_images[match_id]);
    cv::imshow("Good Matches", target_image_matches);

    // Step.2 Construct the Bundle Adjustment Problem
    std::vector <cv::Point3d> points3D;
    std::vector <std::vector<cv::Point2d> > pointsImg;
    std::vector <std::vector<int> > visibility;
    std::vector <cv::Mat> cameraMatrix, distCoeffs, R, T;

    int NPOINTS = max_matches;    // number of 3d points
    int NCAMS = 2;    // number of cameras, best_match and target

    // Fill image projections according to good matches
    std::cout << "Filling image points..." << std::endl;
    pointsImg.resize(NCAMS);
    pointsImg[0].clear();
    pointsImg[1].clear();

    std::cout << "Points 2d[0]: " << pointsImg[0].size() << std::endl;
    std::cout << "Points 2d[1]: " << pointsImg[1].size() << std::endl;

    /*for(int i = 0; i < best_matches.size(); ++i) {
        pointsImg[0].push_back(keypoints_target[best_matches[i].queryIdx].pt);
        pointsImg[1].push_back(keypoints_best_match[best_matches[i].trainIdx].pt);
    }*/

    // Back project match image to 3D in terms of its pose and depth
    rtabmap::Transform bestMatchImagePose = keyframe_poses[match_id];
    std::cout << "Best match image pose:" << std::endl;
    std::cout << bestMatchImagePose;

    cv::Mat bestMatchImage = keyframe_images[match_id].clone();
    cv::Mat bestMatchImageDepth = keyframe_depths[match_id].clone();

    // Decode color depth image to get real depth (already done)


    cv::Mat cameraMatrix_IN;
    cameraMatrix_IN = cv::Mat::eye(3, 3, CV_64FC1);
    cameraMatrix_IN.ptr<double>(0)[0] = cameraMatrix_IN.ptr<double>(0)[4] = 5.9421434211923247e+02;
    cameraMatrix_IN.ptr<double>(0)[2] = 3.3930780975300314e+02;
    cameraMatrix_IN.ptr<double>(0)[5] = 2.4273913761751615e+02;

    float roll, pitch, yaw;
    bestMatchImagePose.getEulerAngles(roll, pitch, yaw);
    cv::Mat R_IN = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));
    R_IN.ptr<double>(0)[0] = double(roll);
    R_IN.ptr<double>(0)[1] = double(pitch);
    R_IN.ptr<double>(0)[2] = double(yaw);

    float x, y, z;
    bestMatchImagePose.getTranslation(x, y, z);
    cv::Point3f cameraPosition(x, y, z);
    cv::Mat T_IN = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));
    T_IN.ptr<double>(0)[0] = x;
    T_IN.ptr<double>(0)[1] = y;
    T_IN.ptr<double>(0)[2] = z;


    std::cout << "Fill 3d points" << std::endl;
    // Use myproeo->mybackproject to get 3D points
    for (int iter = 0; iter < best_matches.size(); ++iter) {
        int img_x = (int) keypoints_best_match[best_matches[iter].trainIdx].pt.x;
        int img_y = (int) keypoints_best_match[best_matches[iter].trainIdx].pt.y;

        //cv::imshow("hola",bestMatchImageDepth);
        //cv::waitKey(0);
        unsigned short d = bestMatchImageDepth.at < unsigned
        short > (img_y, img_x);
        if (0 != d) {
            std::cout << "ok" << std::endl;


            cv::Point3f pointBack3D;
            pointBack3D = _backProject(cameraMatrix_IN, R_IN, T_IN,
                                       keypoints_best_match[best_matches[iter].trainIdx].pt);
            pointBack3D = getPointAtDistance(cameraPosition, pointBack3D, d);


            //opencv reprojection
            std::vector <cv::Point2f> repro_points;
            std::vector <cv::Point3f> float_3D_points;
            std::vector <cv::Point2f> float_2D_keypoints;
            float_3D_points.push_back(pointBack3D);


            cv::projectPoints(float_3D_points, R_IN, T_IN, cameraMatrix_IN, cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0)),
                              repro_points);

            std::cout << "-----------------------------" << std::endl;
            std::cout << "2D point from keypoint" << keypoints_best_match[best_matches[iter].trainIdx].pt << std::endl;
            std::cout << "Reprojected 2D (opencv): " << repro_points[0] << std::endl;
            std::cout << "-----------------------------" << std::endl;

            // Fill 3D points
            // points3D.push_back(cv::Point3d(xp, yp, zp));

            points3D.push_back(pointBack3D);

            // Fill 2D points (2D points index must fit with its correcponding 2d projection on each image)
            pointsImg[0].push_back(keypoints_target[best_matches[iter].queryIdx].pt);
            pointsImg[1].push_back(keypoints_best_match[best_matches[iter].trainIdx].pt);

        }
    }
    //remove
    //points3D.resize(30);
    NPOINTS = points3D.size();

    // Fill visibility (all points are visible)
    visibility.resize(NCAMS);
    for (int i = 0; i < NCAMS; i++) {
        visibility[i].resize(NPOINTS);
        for (int j = 0; j < NPOINTS; j++) visibility[i][j] = 1;
    }

    // Fill camera intrinsics (same intrinsics for all cameras)
    for (int i = 0; i < NCAMS; i++) {
        cameraMatrix.push_back(cameraMatrix_IN);
    }

    // Fill distortion (assume no distortion)
    distCoeffs.resize(NCAMS);
    for (int i = 0; i < NCAMS; i++)
        distCoeffs[i] = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

    // Fill rotation
    for (int i = 0; i < NCAMS; i++) {
        if (i == 0) {
            cv::Mat temp = R_IN.clone();
            temp.at<double>(0, 0) = 0;
            temp.at<double>(0, 1) = 0;
            temp.at<double>(0, 2) = 0;
            R.push_back(temp);
        }
        else
            R.push_back(R_IN.clone());
    }

    // Fill translation
    for (int i = 0; i < NCAMS; i++) {
        if (i == 0) {
            cv::Mat temp = T_IN.clone();
            temp.at<double>(0, 0) = 0;
            temp.at<double>(0, 1) = 0;
            temp.at<double>(0, 2) = 0;
            T.push_back(temp);
        }
        else
            T.push_back(T_IN.clone());
    }

    // Step.3 Solve the BA problem using cvsba
    cvsba::Sba sba;
    cvsba::Sba::Params params;
    params.fixedDistortion = 5;
    params.fixedIntrinsics = 5;
    params.type = cvsba::Sba::MOTION;
    sba.setParams(params);

    std::cout << "Points 3d: " << points3D.size() << std::endl;
    std::cout << "Point 3d[0]: " << points3D[0] << std::endl;
    std::cout << "Point 3d[1]: " << points3D[1] << std::endl;

    std::cout << "Points 2d[0]: " << pointsImg[0].size() << std::endl;
    std::cout << "Points 2d[1]: " << pointsImg[1].size() << std::endl;


    std::cout << "P0 C0: " << pointsImg[0][0] << std::endl;
    std::cout << "P1 C0: " << pointsImg[0][1] << std::endl;


    int method; //0 -> sba 1-> pnp
    method = 1;

    if (method == 0) {
        //sba.run(points3D,  pointsImg,  visibility,  cameraMatrix,  R,  T, distCoeffs);
        std::cout << "Initial error=" << sba.getInitialReprjError() << ". Final error=" << sba.getFinalReprjError() <<
        std::endl;

        std::cout << "Estimated pose: T:" << T[1] << std::endl;
        std::cout << "R" << R[1] << std::endl;

        std::cout << "Fixed camera after sba: " << T[0] << endl << R[0] << std::endl;
        std::cout << "Matched image: " << match_id << std::endl;
    }
    else {
        //PNP
        cv::Mat rvec, tvec;
        std::vector<int> inliersIDs;
        cv::solvePnPRansac(points3D, pointsImg[0], cameraMatrix_IN, cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0)), rvec, tvec,
                           true, 1500, 2, 0.5 * pointsImg[0].size(), inliersIDs, CV_EPNP);
        std::cout << "pnp inliers: " << inliersIDs.size() << std::endl;
        R[0] = rvec;
        T[0] = tvec;
    }

    //visual evaluation
    std::vector<cv::Mat> drawImages;
    drawImages.push_back(target_image);
    drawImages.push_back(keyframe_images[match_id].clone());
    for (int i=0; i<NCAMS; i++){
        std::vector<cv::Point2f> repro_points;
        std::vector<cv::Point3f> float_3D_points;
        std::vector<cv::Point2f> float_2D_keypoints;

        for (int p=0; p<points3D.size(); p++){
            float_3D_points.push_back(cv::Point3f(points3D[p].x, points3D[p].y,points3D[p].z));
            float_2D_keypoints.push_back(pointsImg[i][p]);
        }


        cv::projectPoints(float_3D_points,R[i],T[i],cameraMatrix[i],distCoeffs[i],repro_points);
        drawProjectedPoints(drawImages[i],float_2D_keypoints,repro_points);
        std::stringstream ss;
        ss << "image[" << i << "]";
        cv::imshow(ss.str(),drawImages[i]);
    }
}


void PCLViewer::drawProjectedPoints(cv::Mat& img, const std::vector<cv::Point2f>&  pointsRaw, const std::vector<cv::Point2f>& pointsProjected){

    cv::Scalar lineColor= cv::Scalar(255, 0, 0);

    double accumError = 0;
    cv::Point textPosition(50, 50);

    if (pointsRaw.size() != pointsProjected.size()){
    }
    else{
        for (unsigned int i = 0; i < pointsRaw.size(); i++) {
            cv::Point2f p1_1 = pointsRaw[i];
            cv::Point2f p2_1 = pointsProjected[i];

            double dx = (p1_1.x-p2_1.x);
            double dy = (p1_1.y-p2_1.y);
            accumError += std::sqrt( dx*dx +dy*dy );

            cv::circle(img, p1_1, 1, cv::Scalar(0, 0, 255), 1);
            cv::circle(img, p2_1, 1, cv::Scalar(0, 255, 0), 1);
            cv::line(img,p1_1,p2_1,lineColor);

        }
        std::stringstream ss;
        ss<<accumError/ pointsRaw.size();
        cv::putText( img, ss.str(), textPosition, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0 ), 2 );
    }
}

cv::Point3f PCLViewer::_backProject(const cv::Mat&K, const cv::Mat& R, const cv::Mat& T, const cv::Point2f& p2d){

    cv::Mat RMat;
    cv::Rodrigues(R,RMat);

    cv::Mat RT;
    RT = cv::Mat(3, 4, CV_64FC1, cv::Scalar::all(0));
    RT.at<double>(0,0)=RMat.at<double>(0,0);
    RT.at<double>(0,1)=RMat.at<double>(0,1);
    RT.at<double>(0,2)=RMat.at<double>(0,2);
    RT.at<double>(1,0)=RMat.at<double>(1,0);
    RT.at<double>(1,1)=RMat.at<double>(1,1);
    RT.at<double>(1,2)=RMat.at<double>(1,2);
    RT.at<double>(2,0)=RMat.at<double>(2,0);
    RT.at<double>(2,1)=RMat.at<double>(2,1);
    RT.at<double>(2,2)=RMat.at<double>(2,2);
    RT.at<double>(0,3)=T.at<double>(0,0);
    RT.at<double>(1,3)=T.at<double>(0,1);
    RT.at<double>(2,3)=T.at<double>(0,2);


    cv::Mat P =K*RT;
    cv::Mat P_inv = P.clone().inv(cv::DECOMP_SVD);

    cv::Mat p2dMat = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));
    p2dMat.at<double>(0,0)=p2d.x;
    p2dMat.at<double>(0,1)=p2d.y;
    p2dMat.at<double>(0,2)=1;

    cv::Mat p3dMat = cv::Mat(4, 1, CV_64FC1, cv::Scalar::all(0));

    p3dMat = P_inv * p2dMat;


    cv::Point3f pout;
    if (p3dMat.at<double>(0,3) == 0){
        pout.x = p3dMat.at<double>(0, 0);
        pout.y = p3dMat.at<double>(0, 1);
        pout.z = p3dMat.at<double>(0, 2);
    }
    else {
        pout.x = p3dMat.at<double>(0, 0) / p3dMat.at<double>(0, 3);
        pout.y = p3dMat.at<double>(0, 1) / p3dMat.at<double>(0, 3);
        pout.z = p3dMat.at<double>(0, 2) / p3dMat.at<double>(0, 3);
    }

    return  pout;
}


PCLViewer::~PCLViewer()
{
  delete ui;
}


cv::Point3f PCLViewer::getPointAtDistance(const cv::Point3f& cameraPos, const cv::Point3f& backProjectedPoint, const double distance){
    cv::Point3f goodPoint;

    double mod;
    cv::Point3f fx,u;



    mod = sqrt(1/(((cameraPos.x-backProjectedPoint.x)*(cameraPos.x-backProjectedPoint.x))+((cameraPos.y-backProjectedPoint.y)*(cameraPos.y-backProjectedPoint.y))+((cameraPos.z-backProjectedPoint.z)*(cameraPos.z-backProjectedPoint.z))));
    u.x = (backProjectedPoint.x-cameraPos.x)*mod;
    u.y = (backProjectedPoint.y-cameraPos.y)*mod;
    u.z = (backProjectedPoint.z-cameraPos.z)*mod;

    goodPoint.x=distance*u.x+cameraPos.x;
    goodPoint.y=distance*u.y+cameraPos.y;
    goodPoint.z=distance*u.z+cameraPos.z;

    return goodPoint;

}

