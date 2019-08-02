//struct_core_identification main.cpp
//author: Tanja Baumann
//date: 14.08.2019

//additional install: sudo apt-get install libzbar-dev libzbar0
//build: catkin build struct_core_identification
//run: ./catkin_ws/devel/lib/struct_core_identification/struct_core_identification


#include <ST/Utilities.h>
#include <ST/CaptureSession.h>
#include <ST/CameraFrames.h>
#include <iostream>
#include <condition_variable>
#include <mutex>
#include <stdio.h>
#include <vector>
#include <thread>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>

#include <unistd.h>

bool readQRCode(const ST::InfraredFrame& infrared_frame, std::string& qr_string){

	int width = infrared_frame.width(); //get the width of frames of the video
	int height = infrared_frame.height(); //get the height of frames of the video

	// Copy image data to cv mat
	int data_length = width * height;
	std::unique_ptr<uint16_t> image_data(new uint16_t[data_length]);

	for(int i = 0; i < data_length; i++){
		int row = floor(i/width);
		int col = i%width;
		uint16_t value = infrared_frame.data()[row * width + col];
		image_data.get()[i] = value;
	}
	cv::Mat cv_image(height, width, CV_16U, image_data.get());

	//convert 16bit to 8bit image
	cv::Mat cv_image_8bit;
	cv_image.convertTo(cv_image_8bit, CV_8U);

//	//display image for debugging
//	cv::imshow("struct_core image", cv_image_8bit);
//	cv::waitKey(0);

	//convert to zbar image
	uchar *raw = (uchar *)cv_image_8bit.data;
	zbar::Image zbar_image(width, height, "Y800", raw, width * height);


	//scan for QR codes
	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
	scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
	int n = scanner.scan(zbar_image);

	// extract results
	for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
	  qr_string = symbol->get_data();
	  return 1;
	}


	return false;
}


struct SessionDelegate : public ST::CaptureSessionDelegate {

	bool qr_found_ = false;
	std::string qr_string_ = "";
	std::string serial_ = "";

    void captureSessionDidOutputSample (ST::CaptureSession* session, const ST::CaptureSessionSample& sample) override {
        switch (sample.type) {
        case ST::CaptureSessionSample::Type::InfraredFrame:
        	std::cout<<"received infra frame \n";
        	if(readQRCode(sample.infraredFrame, qr_string_)){
        		qr_found_ = true;
        	}
            break;
        };
    }
};

void startCamera(SessionDelegate* delegate, const std::string* serial){

	// configure the options to stream everything needed for tracking
	ST::CaptureSessionSettings sessionConfig;
	ST::CaptureSessionSettings::StructureCoreSettings scConfig;

	scConfig.depthEnabled = false;
	scConfig.infraredEnabled = true;
	scConfig.visibleEnabled = false;
	scConfig.accelerometerEnabled = false;
	scConfig.gyroscopeEnabled = false;
	scConfig.infraredAutoExposureEnabled = true;
	scConfig.infraredFramerate = 15.f;
	scConfig.depthFramerate    = 15.f;
	scConfig.visibleFramerate  = 15.f;
	scConfig.initialProjectorPower = 0.f;
    scConfig.sensorSerial = serial->c_str();

	scConfig.depthResolution = ST::StructureCoreDepthResolution::VGA;
	scConfig.visibleResolution = ST::StructureCoreVisibleResolution::Default;
	scConfig.infraredMode = ST::StructureCoreInfraredMode::LeftCameraOnly;
	scConfig.infraredResolution = ST::StructureCoreInfraredResolution::Default;

	sessionConfig.source = ST::CaptureSessionSourceId::StructureCore;
	sessionConfig.structureCore = scConfig;
    sessionConfig.applyExpensiveCorrection = true;

	// attach the delegate to a CaptureSession
	ST::CaptureSession session;
	delegate->serial_ = *serial;
	session.setDelegate(delegate);

	// apply settings and start streaming!
	session.startMonitoring(sessionConfig);
	session.startStreaming();

	//keep thread from exiting
	while(true){
		sleep(1);
		std::cout<<"thread report: serial written to session: "<<scConfig.sensorSerial<<" serial actually activated: "<<session.sensorInfo().serialNumber<<"\n";
	}
}
bool allCamerasDetected(SessionDelegate* delegates, int num_devices){
	bool all_detected = true;
	for(int i = 0; i < num_devices; i++){
		if(!delegates[i].qr_found_) all_detected = false;
		std::cout<<"camera "<<i<<" serial: "<< delegates[i].serial_<<" detected: "<<delegates[i].qr_found_<<" string: "<<delegates[i].qr_string_<<"\n";
	}
	return all_detected;
}

void printFile(SessionDelegate* delegates, std::string* serial_numbers, int num_devices){
	for(int i = 0; i < num_devices; i++){
		std::cout<<"PRINTING: "<<delegates[i].qr_string_<<" "<<serial_numbers[i]<<"\n";
	}
}


int main()
{

	int num_devices = 10;
	const ST::ConnectedSensorInfo* sensor_info_ptr = nullptr;

	//find connected devices
	ST::enumerateConnectedSensors(&sensor_info_ptr, &num_devices);
	SessionDelegate* delegates = new SessionDelegate[num_devices];
	std::string* serial_numbers = new std::string[num_devices];
	std::vector<std::thread> thread_vector;
	thread_vector.reserve(num_devices);

	for(int i = num_devices -1; i >= 0; i--){
		std::string serial(sensor_info_ptr[i].serial);
		serial_numbers[i] = serial;
		std::thread worker(&startCamera, &delegates[i], &serial_numbers[i]);
		thread_vector.push_back(std::move(worker));
		sleep(5);
	}

	while(!allCamerasDetected(delegates, num_devices)){
		sleep(1);
	}

	printFile(delegates, serial_numbers, num_devices);

	//clean up
    delete[] delegates;
	delete[] serial_numbers;

	std::cout<<"All cameras detected, exiting...\n";
    return 0;
}
