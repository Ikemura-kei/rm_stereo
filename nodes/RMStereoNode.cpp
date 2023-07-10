#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace sensor_msgs;
using namespace message_filters;

// -- constant values --
const bool DO_FILTER = false;
const float BASELINE = 0.0604354;
const float FOCAL = 496.98146838;
const float MAX_DEPTH = 0.795;
const float Y_OFFSET = -1.007;

static cv_bridge::CvImagePtr bridgeLeft, bridgeRight;
static bool received = false;
void callback(const ImageConstPtr &imageLeft, const ImageConstPtr &imageRight)
{
    received = true;
    bridgeLeft = cv_bridge::toCvCopy(imageLeft, image_encodings::BGR8);
    bridgeRight = cv_bridge::toCvCopy(imageRight, image_encodings::BGR8);
}

static CameraInfo cameraInfo;
static bool camInfoReceived = false;
void camInfoCallback(const CameraInfoConstPtr &camInfo)
{
    if (!camInfoReceived)
    {
        camInfoReceived = true;
        cameraInfo = *camInfo;
    }
}

// initialize values for StereoSGBM parameters
int numDisparities = 11;
int blockSize = 21;
int preFilterType = 0;
int preFilterSize = 2;
int preFilterCap = 5;
int minDisparity = 2;
int textureThreshold = 10;
int uniquenessRatio = 16;
int speckleRange = 3;
int speckleWindowSize = 6;
int disp12MaxDiff = -1;
int dispType = CV_16S;

// Creating an object of StereoSGBM algorithm
cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

cv::Mat imgL;
cv::Mat imgR;
cv::Mat imgL_gray;
cv::Mat imgR_gray;

// Defining callback functions for the trackbars to update parameter values

static void on_trackbar1(int, void *)
{
    stereo->setNumDisparities(numDisparities * 16);
    numDisparities = numDisparities * 16;
}

static void on_trackbar2(int, void *)
{
    stereo->setBlockSize(blockSize * 2 + 5);
    blockSize = blockSize * 2 + 5;
}

static void on_trackbar3(int, void *)
{
    stereo->setPreFilterType(preFilterType);
}

static void on_trackbar4(int, void *)
{
    stereo->setPreFilterSize(preFilterSize * 2 + 5);
    preFilterSize = preFilterSize * 2 + 5;
}

static void on_trackbar5(int, void *)
{
    stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6(int, void *)
{
    stereo->setTextureThreshold(textureThreshold);
}

static void on_trackbar7(int, void *)
{
    stereo->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8(int, void *)
{
    stereo->setSpeckleRange(speckleRange);
}

static void on_trackbar9(int, void *)
{
    stereo->setSpeckleWindowSize(speckleWindowSize * 2);
    // speckleWindowSize = speckleWindowSize * 2;
}

static void on_trackbar10(int, void *)
{
    stereo->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11(int, void *)
{
    stereo->setMinDisparity(minDisparity);
}

static cv::Mat depthMap; // this callback prints out the depth value of the clicked position
static void onMouse(int event, int x, int y, int, void *)
{
    if (event != CV_EVENT_LBUTTONDOWN)
        return;
    std::cout << std::to_string(depthMap.channels()) << std::endl;
    cv::Point thisPnt(x, y);
    std::cout << "(" << thisPnt << "): " << depthMap.at<float>(y, x) << std::endl;
}

int main(int ac, char **av)
{
    ros::init(ac, av, "rm_stereo_node");

    ros::NodeHandle nh("~");

    // -- create image subscribers --
    // TODO: change the topic name as parameters
    message_filters::Subscriber<Image> leftSub(nh, "/hbv_1780_center/left/image_rect", 1);
    message_filters::Subscriber<Image> rightSub(nh, "/hbv_1780_center/right/image_rect", 1);
    TimeSynchronizer<Image, Image> sync(leftSub, rightSub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // -- create depth publisher --
    sensor_msgs::Image depthMsg;
    std_msgs::Header depthMsgHeader;
    depthMsgHeader.frame_id = "hbv_1780_center"; // TODO: change this to parameters passed externally
    depthMsgHeader.seq = 0;
    depthMsgHeader.stamp = ros::Time::now();
    ros::Publisher depthPub = nh.advertise<sensor_msgs::Image>("/hbv_1780_center/depth/image_rect", 10);

    // -- create camera info subscriber --
    ros::Subscriber camInfoSub = nh.subscribe<sensor_msgs::CameraInfo>("/hbv_1780_center/camera_info", 10, camInfoCallback);

    // -- create camera info publisher --
    ros::Publisher camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/hbv_1780_center/depth/camera_info", 10);

    // -- creating a named window to be linked to the trackbars --
    cv::namedWindow("disparity", cv::WINDOW_NORMAL);
    cv::resizeWindow("disparity", 600, 600);
    cv::namedWindow("overlay", cv::WINDOW_NORMAL);
    cv::setMouseCallback("overlay", onMouse, 0);

    // -- creating trackbars to dynamically update the StereoBM parameters --
    cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
    cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
    cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
    cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
    cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
    cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
    cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
    cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
    cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
    cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
    cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

    // -- here, we manually set a translation on the y-axis to compensate somewhat insuffcient stereo-rectification --
    cv::Mat hackedAffineTransform = (cv::Mat_<double>(2, 3) << 1, 0, 0, 0, 1, Y_OFFSET);
    ros::Rate rate(1000);

    // -- set initial parameters to stereo matcher --
    stereo->setNumDisparities(numDisparities * 16);
    stereo->setBlockSize(blockSize * 2 + 5);
    stereo->setPreFilterType(preFilterType);
    stereo->setPreFilterSize(preFilterSize * 2 + 5);
    stereo->setPreFilterCap(preFilterCap);
    stereo->setTextureThreshold(textureThreshold);
    stereo->setUniquenessRatio(uniquenessRatio);
    stereo->setSpeckleRange(speckleRange);
    stereo->setSpeckleWindowSize(speckleWindowSize * 2);
    stereo->setDisp12MaxDiff(disp12MaxDiff);
    stereo->setMinDisparity(minDisparity);

    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        if (!received) // prevent any further processing before receiving anything
            continue;

        cv::Mat right;
        cv::Mat left;
        bridgeLeft->image.copyTo(left);
        cv::warpAffine(bridgeRight->image, right, hackedAffineTransform, bridgeLeft->image.size());

        const int NUM_STEP = 15;
        int stepSize = bridgeLeft->image.size().height / NUM_STEP;
        cv::Mat leftDrawn, rightDrawn;
        left.copyTo(leftDrawn);
        right.copyTo(rightDrawn);
        for (int i = 0; i < NUM_STEP; i++)
        {
            int yAxis = stepSize * i;
            cv::line(rightDrawn, cv::Point(0, yAxis), cv::Point((bridgeLeft->image.size().width - 1), yAxis), (0, 255, 0), 2);
            cv::line(leftDrawn, cv::Point(0, yAxis), cv::Point((bridgeLeft->image.size().width - 1), yAxis), (0, 255, 0), 2);
        }

        // -- stereo matching requires grayscale images --
        cv::Mat leftGray, rightGray;
        cv::cvtColor(left, leftGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right, rightGray, cv::COLOR_BGR2GRAY);

        // -- compute disparity --
        double min, max;
        cv::Mat disparity;
        stereo->compute(leftGray, rightGray, disparity);
        disparity.convertTo(disparity, CV_32F, 1.0);

        // -- disparity post-filtering --
        if (DO_FILTER)
        {
            cv::Mat origDisp;
            origDisp = disparity.clone();
            cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(stereo);
            cv::Mat filtered_disp, right_disp;
            right_matcher->compute(leftGray, rightGray, right_disp);

            cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
            wls_filter = cv::ximgproc::createDisparityWLSFilter(stereo);

            double lambda = 8000.0;
            double sigma = 0.5;
            wls_filter->setLambda(lambda);
            wls_filter->setSigmaColor(sigma);
            // double filtering_time = (double)getTickCount();
            wls_filter->filter(origDisp, left, filtered_disp, right_disp);
            // filtering_time = ((double)getTickCount() - filtering_time) / getTickFrequency();
            // cv::Mat rawDispVis, filteredDispVis;
            // cv::ximgproc::getDisparityVis(filtered_disp, filteredDispVis, 1.0);
            // cv::imshow("filtered", filteredDispVis);
            disparity = filtered_disp.clone();
        }

        // -- create visualization of disparity --
        cv::Mat disparityVis = disparity.clone();
        disparityVis = disparityVis / 1100.0 * 255;
        disparityVis.convertTo(disparityVis, CV_8UC1, 1.0);
        cv::applyColorMap(disparityVis, disparityVis, cv::COLORMAP_JET);
        cv::imshow("disparity", disparityVis);

        // -- log min & max of disparity --
        cv::minMaxLoc(disparity, &min, &max);
        // std::cout << "Min disparity: " << std::to_string(min) << ", Max disparity: " << std::to_string(max) << std::endl;

        // -- convert disparity to depth --
        cv::Mat depth = disparity.clone() / 16.0f; // note: here, the cloned disparity is of CV_32F (16 is divided as disparity is scaled by 16)
        depth = BASELINE * FOCAL / depth;
        depth.setTo(0, depth > MAX_DEPTH);
        depthMap = depth.clone();

        // -- publish depth map --
        depthMsgHeader.seq += 1;
        depthMsgHeader.stamp = bridgeLeft->header.stamp; // use the image's original time stamp
        cv_bridge::CvImage depthBridge(depthMsgHeader, sensor_msgs::image_encodings::TYPE_32FC1, depthMap);
        depthBridge.toImageMsg(depthMsg);
        depthPub.publish(depthMsg); // publish depth map
        cameraInfo.header.stamp = depthMsgHeader.stamp;
        if (camInfoReceived)
            camInfoPub.publish(cameraInfo); // publish camera info

        // -- log min & max of depth --
        cv::minMaxLoc(depth, &min, &max);
        // std::cout << "Min depth: " << std::to_string(min) << ", Max depth: " << std::to_string(max) << std::endl;

        // -- create visualization of the depth map --
        cv::Mat visualization = depth.clone();
        visualization = visualization / MAX_DEPTH * 255; // normalize to 0-255
        visualization.convertTo(visualization, CV_8UC1, 1.0);
        cv::minMaxLoc(visualization, &min, &max);
        // std::cout << "Min visualization: " << std::to_string(min) << ", Max visualization: " << std::to_string(max) << std::endl;
        cv::applyColorMap(visualization, visualization, cv::COLORMAP_JET);
        visualization.setTo(0, depth <= 0.001);
        cv::Mat pureDepthVis = visualization.clone();
        cv::cvtColor(leftGray, leftGray, cv::COLOR_GRAY2BGR);
        cv::addWeighted(leftGray, 0.45, visualization, 0.55, 0.0, visualization);

        // -- displaying the depth map --
        cv::imshow("overlay", visualization);
        // cv::imshow("depth", pureDepthVis);

        char k = cv::waitKey(1);
        if (k == 'q')
            break;
    }
}