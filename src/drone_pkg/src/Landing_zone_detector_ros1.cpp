#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

using namespace cv;
using namespace std;

class LandingZoneDetector {
public:
    LandingZoneDetector() {
        canny_low = 100;
        canny_high = 200;
        kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilation_iterations = 1;
        base_zone_size = 200;
        min_zone_size = 100;

        rejected_color = Scalar(0, 0, 255);
        accepted_color = Scalar(0, 255, 0);
        current_color = Scalar(255, 0, 0);
        best_zone_color = Scalar(255, 255, 0);

        setUseOptimized(true);
        setNumThreads(4);
    }

    Mat detectObjects(const Mat& image) {
        Mat gray, blurred, edges, dilated;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, blurred, Size(5, 5), 0);
        Canny(blurred, edges, canny_low, canny_high);
        dilate(edges, dilated, kernel, Point(-1, -1), dilation_iterations);
        return dilated;
    }

    int calculateZoneSize(int altitude) {
        int zone_size = base_zone_size - (altitude - 1) * (base_zone_size - min_zone_size) / 4;
        return max(min(zone_size, base_zone_size), min_zone_size);
    }

    bool isZoneClear(const Mat& objects_image, int x, int y, int zone_size) {
        int half = zone_size / 2;
        int x1 = max(0, x - half);
        int y1 = max(0, y - half);
        int x2 = min(objects_image.cols, x + half);
        int y2 = min(objects_image.rows, y + half);
        if (x2 <= x1 || y2 <= y1) return false;
        Mat roi = objects_image(Rect(Point(x1, y1), Point(x2, y2)));
        return countNonZero(roi) == 0;
    }

    tuple<vector<Point>, Point, int> fullImageSearch(const Mat& image, const Mat& objects_image, int altitude) {
        int width = image.cols, height = image.rows;
        int center_x = width / 2, center_y = height / 2;
        int zone_size = calculateZoneSize(altitude);
        int half_size = zone_size / 2;
        int step_size = max(100, zone_size / 5);
        int max_radius = static_cast<int>(sqrt(center_x * center_x + center_y * center_y)) + zone_size;

        vector<Point> accepted_zones;
        Point best_zone(-1, -1);
        double min_distance = DBL_MAX;

        double radius = 0.0, angle = 0.0;
        int iteration = 0;

        while (radius <= max_radius) {
            int x = static_cast<int>(center_x + radius * cos(angle));
            int y = static_cast<int>(center_y + radius * sin(angle));
            if (x >= half_size && x < width - half_size &&
                y >= half_size && y < height - half_size) {
                if (isZoneClear(objects_image, x, y, zone_size)) {
                    accepted_zones.emplace_back(x, y);
                    double dist = sqrt(pow(x - center_x, 2) + pow(y - center_y, 2));
                    if (dist < min_distance) {
                        best_zone = Point(x, y);
                        min_distance = dist;
                    }
                }
            }
            angle += 0.2;
            if (angle >= 2 * CV_PI) {
                angle = 0;
                radius += step_size;
            }
            iteration++;
        }
        return {accepted_zones, best_zone, iteration};
    }

    void drawZones(Mat& image, const vector<Point>& accepted_zones, const Point& best_zone, int zone_size) {
        int half = zone_size / 2;
        for (const auto& pt : accepted_zones)
            rectangle(image, Point(pt.x - half, pt.y - half), Point(pt.x + half, pt.y + half), accepted_color, 1);
        if (best_zone.x != -1 && best_zone.y != -1) {
            rectangle(image, Point(best_zone.x - half, best_zone.y - half), Point(best_zone.x + half, best_zone.y + half), best_zone_color, 3);
            putText(image, "Best Zone", Point(best_zone.x - half, best_zone.y - half - 10),
                    FONT_HERSHEY_SIMPLEX, 0.6, best_zone_color, 2);
        }
    }

    Mat processFrame(const Mat& frame, int altitude, bool visualize, Point& out_best_zone) {
        int zone_size = calculateZoneSize(altitude);
        Mat objects_image = detectObjects(frame);
        vector<Point> accepted_zones;
        Point best_zone;
        int iterations;
        tie(accepted_zones, best_zone, iterations) = fullImageSearch(frame, objects_image, altitude);
        out_best_zone = best_zone;
        if (visualize) {
            Mat annotated = frame.clone();
            drawZones(annotated, accepted_zones, best_zone, zone_size);
            return annotated;
        }
        return frame.clone();
    }

private:
    int canny_low, canny_high;
    int base_zone_size, min_zone_size;
    int dilation_iterations;
    Mat kernel;
    Scalar rejected_color, accepted_color, current_color, best_zone_color;
};

class LZNode {
public:
    LZNode() : nh_(), it_(nh_) {
        // Load altitude parameter (default 3)
        nh_.param("~altitude", altitude_, 3);

        // ROS1 subscriptions and publications
        image_sub_ = it_.subscribe("camera/image_raw", 5,
                        &LZNode::imageCallback, this);
        image_pub_ = it_.advertise("camera_feed/image_annotated", 5);
        point_pub_ = nh_.advertise<geometry_msgs::Point>("landing_zone/center_coords", 10);

        ROS_INFO("Landing Zone Detector Node started.");
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            Point best_zone;
            Mat annotated = detector.processFrame(cv_ptr->image, altitude_, true, best_zone);

            if (best_zone.x != -1 && best_zone.y != -1) {
                geometry_msgs::Point center_msg;
                center_msg.x = best_zone.x;
                center_msg.y = best_zone.y;
                center_msg.z = 0.0;
                point_pub_.publish(center_msg);
            }

            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
            image_pub_.publish(out_msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher point_pub_;
    LandingZoneDetector detector;
    int altitude_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "landing_zone_detector_node");
    LZNode node;
    ros::spin();
    return 0;
}

