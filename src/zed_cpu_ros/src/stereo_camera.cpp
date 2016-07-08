#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace arti {

class StereoCamera
{

public:

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

}