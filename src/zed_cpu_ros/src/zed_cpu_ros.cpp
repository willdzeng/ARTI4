#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace arti {

// class RESOLUTION {
// public:
// RESOLUTION(){

// }

// };

class StereoCamera
{

public:

	enum RESOLUTION : int
	{
		HD = 0, // 3840x1080
		SD = 1, // 2560x720
		VGA = 2// 1280x480
	};

	StereoCamera(int resolution, double frame_rate) {

		camera_ = new cv::VideoCapture(0);
		cv::Mat raw;
		cv::Mat left_image;
		cv::Mat right_image;
		setResolution(resolution);
		setFrameRate(frame_rate);

		std::cout << "Stereo Camera Set Resolution: " << camera_->get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camera_->get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
		std::cout << "Stereo Camera Set Frame Rate: " << camera_->get(cv::CAP_PROP_FPS) << std::endl;
	}

	void setResolution(int type) {

		if (type == 0) { width_ = 4416; height_ = 1242;} // 2K
		if (type == 1) { width_ = 3840; height_ = 1080;} // FHD
		if (type == 2) { width_ = 2560; height_ = 720;}  // HD
		if (type == 3) { width_ = 1344; height_ = 376;}  // VGA

		camera_->set(cv::CAP_PROP_FRAME_WIDTH, width_);
		camera_->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
		// make sure that the number set are right
		width_ = camera_->get(cv::CAP_PROP_FRAME_WIDTH);
		height_ = camera_->get(cv::CAP_PROP_FRAME_HEIGHT);

	}

	void setFrameRate(double frame_rate) {
		frame_rate_ = frame_rate;
		camera_->set(cv::CAP_PROP_FPS, frame_rate_);
	}

	bool getImages(cv::Mat& left_image, cv::Mat& right_image) {
		cv::Mat raw;
		if (camera_->grab()) {
			camera_->retrieve(raw);
			cv::Rect left_rect(0, 0, width_ / 2, height_);
			cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
			left_image = raw(left_rect);
			right_image = raw(right_rect);
			return true;
		} else {
			return false;
		}
	}

	~StereoCamera() {
		delete camera_;
	}

private:
	cv::VideoCapture* camera_;
	int width_;
	int height_;
	double frame_rate_;
};

class ZedCameraROS {
public:
	ZedCameraROS(int resolution, double frame_rate) {
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
		// get ros param
		private_nh.param("resolution", resolution_, 1);
		private_nh.param("frame_rate", frame_rate_, 30.0);
		private_nh.param("config_file_location", config_file_location_, std::string("~/SN1267.conf"));


		private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
		private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
		private_nh.param("show_image", show_image_, false);
		// initialize camera
		StereoCamera zed(resolution, frame_rate);

		sensor_msgs::CameraInfoPtr left_cam_info_msg_ptr(new sensor_msgs::CameraInfo());
		sensor_msgs::CameraInfoPtr right_cam_info_msg_ptr(new sensor_msgs::CameraInfo());

		// setup publisher stuff
		image_transport::ImageTransport it(nh);
		image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
		image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

		ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
		ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

		//get camera info
		getCameraInfo(config_file_location_, resolution, left_cam_info_msg_ptr, right_cam_info_msg_ptr);
		ROS_INFO("Left Camera Info as following");
		std::cout << *left_cam_info_msg_ptr << std::endl;
		ROS_INFO("Right Camera Info as following");
		std::cout << *right_cam_info_msg_ptr << std::endl;

		// loop to publish images;
		cv::Mat left_image, right_image;
		while (nh.ok()) {
			ros::Time now = ros::Time::now();
			zed.getImages(left_image, right_image);
			if (show_image_) {
				cv::imshow("left", left_image);
				cv::imshow("right", right_image);
			}

			if (left_image_pub.getNumSubscribers() > 0) {
				publishImage(left_image, left_image_pub, "left_frame", now);
			}
			if (right_image_pub.getNumSubscribers() > 0) {
				publishImage(right_image, right_image_pub, "right_frame", now);
			}
			if (left_cam_info_pub.getNumSubscribers() > 0) {
				publishCamInfo(left_cam_info_pub, left_cam_info_msg_ptr);
			}
			if (right_cam_info_pub.getNumSubscribers() > 0) {
				publishCamInfo(right_cam_info_pub, right_cam_info_msg_ptr);
			}
		}
	}

	void getCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg) {
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini(config_file, pt);
		std::string left_str = "LEFT_CAM_";
		std::string right_str = "RIGHT_CAM_";
		std::string reso_str = "";

		switch (resolution) {
		case 0: reso_str = "2K";
		case 1: reso_str = "FHD";
		case 2: reso_str = "HD";
		case 3: reso_str = "VGA";
		}
		// left value
		double l_cx = pt.get<double>(left_str + reso_str + ".cx");
		double l_cy = pt.get<double>(left_str + reso_str + ".cy");
		double l_fx = pt.get<double>(left_str + reso_str + ".fx");
		double l_fy = pt.get<double>(left_str + reso_str + ".fy");
		double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
		double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
		// right value
		double r_cx = pt.get<double>(right_str + reso_str + ".cx");
		double r_cy = pt.get<double>(right_str + reso_str + ".cy");
		double r_fx = pt.get<double>(right_str + reso_str + ".fx");
		double r_fy = pt.get<double>(right_str + reso_str + ".fy");
		double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
		double r_k2 = pt.get<double>(right_str + reso_str + ".k2");
		double baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
		double p1 = 0, p2 = 0, k3 = 0;

		left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

		left_cam_info_msg->D.resize(5);
		left_cam_info_msg->D[0] = l_k1;
		left_cam_info_msg->D[1] = l_k2;
		left_cam_info_msg->D[2] = k3;
		left_cam_info_msg->D[3] = p1;
		left_cam_info_msg->D[4] = p2;

		right_cam_info_msg->D.resize(5);
		right_cam_info_msg->D[0] = r_k1;
		right_cam_info_msg->D[1] = r_k2;
		right_cam_info_msg->D[2] = k3;
		right_cam_info_msg->D[3] = p1;
		right_cam_info_msg->D[4] = p2;

		left_cam_info_msg->K.fill(0.0);
		left_cam_info_msg->K[0] = l_fx;
		left_cam_info_msg->K[2] = l_cx;
		left_cam_info_msg->K[4] = l_fy;
		left_cam_info_msg->K[5] = l_cy;
		left_cam_info_msg->K[8] = 1.0;

		right_cam_info_msg->K.fill(0.0);
		right_cam_info_msg->K[0] = r_fx;
		right_cam_info_msg->K[2] = r_cx;
		right_cam_info_msg->K[4] = r_fy;
		right_cam_info_msg->K[5] = r_cy;
		right_cam_info_msg->K[8] = 1.0;

		left_cam_info_msg->R.fill(0.0);
		right_cam_info_msg->R.fill(0.0);

		left_cam_info_msg->P.fill(0.0);
		left_cam_info_msg->P[0] = l_fx;
		left_cam_info_msg->P[2] = l_cx;
		left_cam_info_msg->P[5] = l_fy;
		left_cam_info_msg->P[6] = l_cy;
		left_cam_info_msg->P[10] = 1.0;

		right_cam_info_msg->P.fill(0.0);
		right_cam_info_msg->P[0] = r_fx;
		right_cam_info_msg->P[2] = r_cx;
		right_cam_info_msg->P[5] = r_fy;
		right_cam_info_msg->P[6] = r_cy;
		right_cam_info_msg->P[10] = 1.0;
		right_cam_info_msg->P[3] = (-1 * l_fx * baseline);

		left_cam_info_msg->width = right_cam_info_msg->width = width_;
		left_cam_info_msg->height = right_cam_info_msg->height = height_;

		left_cam_info_msg->header.frame_id = left_frame_id_;
		right_cam_info_msg->header.frame_id = right_frame_id_;
	}

	void publishCamInfo(ros::Publisher pub_cam_info, sensor_msgs::CameraInfoPtr cam_info_msg) {
		cam_info_msg->header.stamp = ros::Time::now();
		pub_cam_info.publish(cam_info_msg);
	}

	void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t) {
		cv_bridge::CvImage cv_image;
		cv_image.image = img;
		cv_image.encoding = sensor_msgs::image_encodings::BGR8;
		cv_image.header.frame_id = img_frame_id;
		cv_image.header.stamp = t;
		img_pub.publish(cv_image.toImageMsg());
	}

private:
	int resolution_;
	double frame_rate_;
	bool show_image_;
	double width_, height_;
	std::string left_frame_id_, right_frame_id_;
	std::string config_file_location_;

};

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "zed_camera");
	arti::ZedCameraROS zed_ros(1, 30.0);
	return 0;
}