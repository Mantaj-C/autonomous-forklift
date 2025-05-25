#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>

#include "Client.h"
//#include "CCharuco.h"
#include "CCamera.h"
#include "CPathing.h"
#include <opencv2/aruco.hpp>


enum servos {BL = 0, BR, ML, MR};

#define WINDOW_NAME "Calibration"
#define WINDOW_SIZE cv::Size(640, 480)


#define server_ip "192.168.0.100"
#define server_port 4012

float timeout_start;

#define IMG "G 1"



void send_command(CClient &client, std::string cmd, cv::Mat &frame)
{
	std::string str;

	client.tx_str(cmd);
	std::cout << "\nClient Tx: " << cmd;


	if (cmd == IMG)
	{
		cv::Mat im;
		if (client.rx_im(im) == true)
		{
			timeout_start = cv::getTickCount();
			if (im.empty() == false)
			{
				std::cout << "\nClient Rx: Image received";
				
				#ifdef UNDISTORT
					cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<  1.8722871502485910e+02, 0., 2.8878842997093369e+02, 0.,  1.7784781070920499e+02, 2.4018022890173202e+02, 0., 0., 1.);
					cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  5.4896083310467022e-03, 2.8609542923694897e-02, -4.2421778012254633e-03, -3.8150653879379264e-02, -8.4360735642799878e-03 );
					std::cout << "\nUndistorting" << std::endl;
					cv::Mat im_undistorted;
					cv::undistort(im, im_undistorted, cameraMatrix, distCoeffs);
					std::cout << "\nUndistortion Complete" << std::endl;
					frame = im_undistorted.clone();
				#else
					frame = im.clone();
				#endif
				std::cout << "\nframe cloned" << std::endl;
				//cv::imshow("rx", im);
				std::cout << "\nImshow" << std::endl;
				cv::waitKey(10);
			}
		}
		else
		{
			if ((cv::getTickCount() - timeout_start) / cv::getTickFrequency() > 1000)
			{
				// No response, disconnect and reconnect
				std::cout << "No Response, Attempting Disconnect --> Reconnect" << std::endl;
				timeout_start = cv::getTickCount();
				client.close_socket();
				client.connect_socket(server_ip, server_port);
			}
		}
	}
	else
	{
		if (client.rx_str(str) == true)
		{
			timeout_start = cv::getTickCount();
			std::cout << "\nClient Rx: " << str;
			str.clear();
		}
		else
		{
			if ((cv::getTickCount() - timeout_start) / cv::getTickFrequency() > 1000)
			{
				// No response, disconnect and reconnect
				timeout_start = cv::getTickCount();
				client.close_socket();
				client.connect_socket(server_ip, server_port);
			}
		}
	}


}

void livestream(CClient& client, CPathing& pathing) {
	int key = 0;
	cv::Mat prevImg;
	int64 timeout_start = cv::getTickCount();
	bool flag = false;

	do {
		// 1) grab a fresh frame into currImg
		cv::Mat currImg;
		client.tx_str(IMG);
		std::cout << "\nClient Tx: " << IMG;

		if (client.rx_im(currImg)) {
			timeout_start = cv::getTickCount();

			if (!currImg.empty()) {
				// first valid frame — seed prevImg
				if (prevImg.empty()) {
					prevImg = currImg.clone();
				}

				// 2) build mask of missing (black) pixels
				cv::Mat mask;
				cv::inRange(currImg, cv::Vec3b(0,0,0), cv::Vec3b(0,0,0), mask);

				// 3) copy from prevImg wherever currImg is missing
				prevImg.copyTo(currImg, mask);

				// 4) show patched result
				std::cout << "\nClient Rx: Image received";
				cv::imshow("rx", currImg);
				//std::cout << "image columns " << currImg.cols << std::endl;

				flag = pathing.AutoRouting(currImg, cv::Point2f(610, 690), CPathing::UPPER);
				cv::imshow("auto:", currImg);

				// 5) save for next iteration
				prevImg = currImg.clone();


			}
		}
		else {

			if (!prevImg.empty()) {
				cv::imshow("rx", prevImg);
			}
			// on receive timeout, reconnect
			double elapsed = (cv::getTickCount() - timeout_start) / cv::getTickFrequency();
			if (elapsed > 1.0) {
				std::cout << "\nNo Response, reconnecting...";
				timeout_start = cv::getTickCount();
				client.close_socket();
				client.connect_socket(server_ip, server_port);
			}
		}

		

		key = cv::waitKey(10);
	} while (key != 'q' && flag == false);
}



int main(int argc, char* argv[])
{

	gpioInitialise();

	//cvui::init(WINDOW_NAME);
	// Create and display a blank image so cv::waitKey() works
	cv::Mat blankImage = cv::Mat::zeros(100, 100, CV_8UC3);
	cv::imshow("", blankImage);

	CClient client;
    
	int cmd = -1;

	timeout_start = cv::getTickCount();
	client.connect_socket(server_ip, server_port);
	CPathing pathing;

	do
	{
		std::cout << "\n (1) Image" << std::endl;
		std::cout << "\n (2) Forward" << std::endl;
		std::cout << "\n (3) Turn Right" << std::endl;
		std::cout << "\n (4) Turn Left" << std::endl;
		std::cout << "\n (5) Move backward" << std::endl;
		std::cout << "\n (6) servos up" << std::endl;
		std::cout << "\n (7) servos down" << std::endl;
    	
		do {
			cmd = cv::waitKey(1);
		} while (cmd == -1);
		std::cout << cmd << std::endl;
	
        cv::Mat im;

		switch (cmd) {

		case '1':	
			livestream(client, pathing);
			break;
		case '2':
			pathing.Move_forward(100);
			break;
		case '3':
			pathing.Turn_right(90);
			break;
		case '4':
			pathing.Turn_left(90);
			break;
		case '5':
			pathing.move_backward(100);
			break;
		case '6':
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLift(BL);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLift(BR);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLift(ML);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLift(MR);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			break;
		case '7':
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLower(BL);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLower(BR);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLower(ML);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			pathing.ServoLower(MR);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			break;
		}
	
	} while (cmd != '0');

	cv::destroyAllWindows();
	gpioTerminate();
}


