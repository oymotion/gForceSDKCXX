/*
 * Copyright 2017, OYMotion Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */
#include "stdafx.h"
#include "gforce.h"
#include <atomic>
#include "SerialDetect.h"
#include <mutex>
#include <condition_variable>
 //#include "winsparkle.h"
#include <algorithm>
#include <cctype>
//#include "uWS.h"
#include <list>
#include <queue>
#include <fstream>
#include <ctime>
#include <io.h>
#include <direct.h>
#include "conio.h"
#include <future>

using namespace gf;
using namespace std;

#define DEFAULT_RSSI_THRESHOLD 0xD0
#define FILE_NAME_SIZE  80
atomic<bool> bExiting = false;
atomic<bool> bGetNewFile = false;
atomic<bool> bRecording = false;
//save raw data with format
std::ofstream g_file;
char g_filename[FILE_NAME_SIZE];

static void waitInputFileName(void);
static void printSingleFileRecordedBytes(int n);

template <class T>
class Thread_safe_Queue {
public:
	Thread_safe_Queue() {

	}
	~Thread_safe_Queue() {

	}
	void push(T val) {
		std::lock_guard<std::mutex> lgx(mQueueMx);
		mdataQueue.push(val);
		mQueueCv.notify_one();
	}

	T wait_and_pop() {
		std::unique_lock<std::mutex> lk(mQueueMx);
		mQueueCv.wait(lk, [this] {return !mdataQueue.empty(); });
		T retval = mdataQueue.front();
		mdataQueue.pop();
		return retval;
	}
	bool empty() {
		std::lock_guard(mQueueMx);
		return mdataQueue.empty();
	}
private:
	std::queue<T> mdataQueue;
	std::mutex mQueueMx;
	std::condition_variable mQueueCv;
};

struct EMGparameter {
	GF_UINT16 sampleRate;
	GF_UINT8  resolution;
};

const EMGparameter c_emgConfig[2] = {
	{ 650,8 },   ///< sampleRate = 650Hz, ADC resolution = 8bit
	{ 500,12 },  ///< sampleRate = 500Hz, ADC resolution = 12bit
};

//std::list<uWS::WebSocket<uWS::SERVER>*> g_serverlist;
Thread_safe_Queue<gfsPtr<const std::vector<GF_UINT8>>> g_MessageQueue;
volatile unsigned char g_EMGchoice = 0;
atomic<bool> mAppExit = false;

//unsigned char g_
// The GForceHandle implements HubListener,
//     operates gForce device and receives data

class GForceHandle : public HubListener
{
public:
	GForceHandle(gfsPtr<Hub>& pHub)
		: mHub(pHub)
	{
		InitializeCriticalSection(&g_CriticalSection);
	}
	/// This callback is called when the Hub finishes scanning devices.
	virtual void onScanFinished() override
	{
		//cout << __FUNCTION__ << " has been called." << endl;
		if (nullptr == mDevice)
		{
			// if no device found, we do scan again
			std::cout << "[INFO]: Can't find avaliable gForce\n";
			std::cout << "[INFO]: Start to search gForce again" << endl;
			mHub->startScan(DEFAULT_RSSI_THRESHOLD);
		}
		else
		{
			//     in connecting or connected state, try to connect it.
			//std::wcout << "[INFO]: Wow,Found the " << mDevice->getName() << ",try to connect it" << std::endl;
			DeviceConnectionStatus status = mDevice->getConnectionStatus();

			if (DeviceConnectionStatus::Connected != status &&
				DeviceConnectionStatus::Connecting != status)
			{
				if (GF_RET_CODE::GF_SUCCESS != mDevice->connect()) {
					std::cout << "[INFO]:Connect Device failed" << endl;
				}
			}
		}
	}

	/// This callback is called when the state of the hub changed
	virtual void onStateChanged(HubState state) override
	{
		//cout << __FUNCTION__ << " has been called. New state is " << static_cast<GF_UINT32>(state) << endl;
		// if the hub is disconnected (such as unplugged), then set the flag of exiting the app.
		if (HubState::Disconnected == state)
		{
			//g_CVSerialOut.notify_one();
			mLoop = false;
			mDevice.reset();
		}
	}

	/// This callback is called when the hub finds a device.
	virtual void onDeviceFound(SPDEVICE device) override
	{
		// In the sample app, we only connect to one device, so once we got one, we stop scanning.
		//cout << __FUNCTION__ << " has been called." << endl;
		if (nullptr != device)
		{
			// only search the first connected device if we connected it before
			if (nullptr == mDevice || device == mDevice)
			{
				mDevice = device;
				//std::wcout << "[INFO]: Wow,Found the " << mDevice->getName() << ",try to connect it" << std::endl;
				mHub->stopScan();
			}
		}
	}

	/// This callback is called a device has been connected successfully
	virtual void onDeviceConnected(SPDEVICE device) override
	{
		//std::wcout << "[INFO]: Sucessed in connecting " << device->getName() << std::endl;

		if (device)
		{
			auto setting = device->getDeviceSetting();

			if (nullptr != setting)
			{
				setting->enableDataNotification(0);

				this_thread::sleep_for(std::chrono::milliseconds(50));

				int ret = tryConfigEMG(setting, c_emgConfig[g_EMGchoice].sampleRate, c_emgConfig[g_EMGchoice].resolution);

				if (0 != ret) {
					if (0 != g_EMGchoice)
						ret = tryConfigEMG(setting, c_emgConfig[0].sampleRate, c_emgConfig[0].resolution);

					if (0 != ret) {
						std::cout << "[ERROR]: config emg parameters error, application will be quit" << std::endl;
						mAppExit.store(true);
					}
				}

				if (0 == ret) {
					setting->setDataNotifSwitch((DeviceSetting::DataNotifFlags)(DeviceSetting::DNF_OFF | DeviceSetting::DNF_EMG_RAW),
						[this, setting](ResponseResult result) {
							std::string res = (result == ResponseResult::RREST_SUCCESS) ? ("success") : ("failed");
							std::cout << "[INFO]: Open raw data " << res << ", result=" << (uint32_t)result << std::endl;

							if (result == ResponseResult::RREST_SUCCESS) {
								setting->enableDataNotification(1);
								bGetNewFile = true;
							}
						}
					);
				}
			}
		}
	}

	/// This callback is called when a device has been disconnected from
	///                                 connection state or failed to connect to
	virtual void onDeviceDisconnected(SPDEVICE device, GF_UINT8 reason) override
	{
		if (g_file.is_open()) {
			g_file.close();
		}
		// if connection lost, we will try to reconnect again.
		//std::wcout << "[INFO]: Eee, " << device->getName() << " Disconnect" << std::endl;
		mDevice.reset();
		std::cout << "[INFO]: Start to search gForce\n";
		mHub->startScan(DEFAULT_RSSI_THRESHOLD);
	}

	/// This callback is called when the quaternion data is received
	virtual void onOrientationData(SPDEVICE device, const Quaternion& rotation) override
	{
		// print the quaternion data
		//cout << __FUNCTION__ << " has been called. " << rotation.toString() << endl;
	}

	/// This callback is called when the gesture data is recevied
	virtual void onGestureData(SPDEVICE device, Gesture gest) override
	{
		// a gesture event coming.
		string gesture;
		switch (gest)
		{
		case Gesture::Relax:
			gesture = "Relax";
			break;
		case Gesture::Fist:
			gesture = "Fist";
			break;
		case Gesture::SpreadFingers:
			gesture = "SpreadFingers";
			break;
		case Gesture::WaveIn:
			gesture = "WaveIn";
			break;
		case Gesture::WaveOut:
			gesture = "WaveOut";
			break;
		case Gesture::Pinch:
			gesture = "Pinch";
			break;
		case Gesture::Shoot:
			gesture = "Shoot";
			break;
		case Gesture::Undefined:
		default:
		{
			gesture = "Undefined: ";
			string s;
			stringstream ss(s);
			ss << static_cast<int>(gest);
			gesture += ss.str();
		}
		}
		cout << __FUNCTION__ << " has been called. " << gesture << endl;
	}

	/// This callback is called when the button on gForce is pressed by user
	virtual void onDeviceStatusChanged(SPDEVICE device, DeviceStatus status) override
	{
		string devicestatus;
		switch (status)
		{
		case DeviceStatus::ReCenter:
			devicestatus = "ReCenter";
			break;
		case DeviceStatus::UsbPlugged:
			devicestatus = "UsbPlugged";
			break;
		case DeviceStatus::UsbPulled:
			devicestatus = "UsbPulled";
			break;
		case DeviceStatus::Motionless:
			devicestatus = "Motionless";
			break;
		default:
		{
			devicestatus = "Undefined: ";
			string s;
			stringstream ss(s);
			ss << static_cast<int>(status);
			devicestatus += ss.str();
		}
		}
		cout << __FUNCTION__ << " has been called. " << devicestatus << endl;
	}

	/// This callback is called when extended data is received
	virtual void onExtendedDeviceData(SPDEVICE device, DeviceDataType dataType, gfsPtr<const std::vector<GF_UINT8>> data) override
	{
		//cout << __FUNCTION__ << ": datatype = " << (GF_UINT32)dataType << ", datalength = " << data->size()
		//	<< ", first byte: " << hex << (GF_UINT32)((data->size() > 0) ? data->at(0) : 0xFF)
		//	<< ", last byte: " << (GF_UINT32)((data->size() > 0) ? data->at(data->size() - 1) : 0xFF) << dec << endl;

		if (data->size() == 0)
			return;

		switch (dataType) {
		case DeviceDataType::DDT_EMGRAW:
		{
			time_t rawtime;
			time(&rawtime);
			//EnterCriticalSection(&g_CriticalSection);
			//save raw data with format
			if (g_file.is_open() && bRecording) {
				//format data:0x8192 + CheckSum + PID + RawData + Time
				g_file.put((GF_UINT8)0x92);//magic number:0x8192
				g_file.put((GF_UINT8)0x81);//magic number:0x8192
				g_file.put(CheckSum(&(*data)[0], (GF_UINT8)data->size()));
				static GF_UINT8 pid = 0;
				g_file.put(pid);
				pid++;
				g_file.write((const char*)&(*data)[0], data->size());
				ULARGE_INTEGER ltime;
				ltime = GetFileTime();
				g_file.write((const char*)&ltime, sizeof(ltime));
				printSingleFileRecordedBytes((int)g_file.tellp());
			}
			//LeaveCriticalSection(&g_CriticalSection);
			g_MessageQueue.push(data);
		}
		break;
		default:
			break;
		}
	}

	// Indicates if we want to exit app
	bool getLoop()
	{
		return mLoop;
	}

	void setLoop() {
		mLoop = true;
	}

	GF_UINT8 CheckSum(const GF_UINT8* data, GF_UINT8 length)
	{
		GF_UINT8 cs = 0;
		while (length--) {
			cs ^= *data++;
		}
		return cs;
	}

	//get systemtime and convert to ulonglong format
	ULARGE_INTEGER GetFileTime(void)
	{
		SYSTEMTIME utcSystemTime;
		FILETIME utcFileTime;
		ULARGE_INTEGER time;
		GetSystemTime(&utcSystemTime);
		SystemTimeToFileTime(&utcSystemTime, &utcFileTime);
		time.HighPart = utcFileTime.dwHighDateTime;
		time.LowPart = utcFileTime.dwLowDateTime;
		return time;
	}

private:
	CRITICAL_SECTION g_CriticalSection;
	// Indicates if we will keep message polling
	bool mLoop = true;
	// keep a instance of hub.
	gfsPtr<Hub> mHub;
	// keep a device to operate
	gfsPtr<Device> mDevice;

	int tryConfigEMG(gfsPtr<DeviceSetting> setting, GF_UINT16 samRate, GF_UINT8 resol)
	{
		ResponseResult ret = ResponseResult::RREST_FAILED;

		for (int trytime = 0; trytime < 3; ++trytime) {
			std::promise<ResponseResult> emgPromise;
			auto emgFuture = emgPromise.get_future();

			setting->setEMGRawDataConfig(samRate, //sample rate
				(DeviceSetting::EMGRowDataChannels)(0x00FF), //channel 0~7 
				128, //data length
				resol,   //resolution
				[&emgPromise](ResponseResult result) {
					string retLog = (result == ResponseResult::RREST_SUCCESS) ? ("sucess") : ("failed");
					std::cout << "[INFO]: Set Emg Config " << retLog << "(" << (int)result << ")" << std::endl;
					emgPromise.set_value(result);
				});

			if (ResponseResult::RREST_SUCCESS == emgFuture.get()) {
				ret = ResponseResult::RREST_SUCCESS;
				break;
			}
		}

		if (ResponseResult::RREST_SUCCESS == ret)
			return 0;

		return -1;
	}
};


class DetectDiscover :public SerialDetectListener {
public:
	DetectDiscover(std::shared_ptr<SerialDetect> ptr, gfsPtr<gf::Hub> ptrHub)
		:pDetect(ptr), pGforceHub(ptrHub) {

	}
	~DetectDiscover() {

	}

	virtual void HubPlugIn(CString comport) override {
		std::cout << "[INFO]: I find avaliabel gForceDongle\n";
		CString comnumber;
		for (int index = 0; index < comport.GetLength(); ++index) {
			if (comport[index] >= '0' && comport[index] <= '9') {
				comnumber += comport[index];
			}
		}
		int int_comnum = _ttoi(comnumber);
		// Initialize hub. Could be failed in below cases:
		//   1. The hub is not plugged in the USB port.
		//   2. Other apps are connected to the hub already.
		GF_RET_CODE retCode = pGforceHub->init(int_comnum);
		if (GF_RET_CODE::GF_SUCCESS != retCode)
		{
			cout << "hub init failed: " << static_cast<GF_UINT32>(retCode) << endl;
			return;
		}
		pDetect->SetPortOpen(comport);
		// start to scan devices
		retCode = pGforceHub->startScan(DEFAULT_RSSI_THRESHOLD);
		if (GF_RET_CODE::GF_SUCCESS != retCode)
		{
			cout << "scan starting failed: " << static_cast<GF_UINT32>(retCode) << endl;
		}
		else {
			std::cout << "[INFO]: Start to search gForce\n";
		}
	}

private:
	std::shared_ptr<SerialDetect> pDetect;
	gfsPtr<gf::Hub> pGforceHub;
};


class gForceProcessor {
private:
	atomic<bool> mThreadQuitFlag;
	tstring identify;

public:
	std::shared_ptr<SerialDetect> mSerialDetect;
	std::shared_ptr<SerialDetectListener> mDetectListener;
	std::shared_ptr<GForceHandle> mGForceHandle;
	std::shared_ptr<HubListener> mHubListener;
	std::shared_ptr<Hub> mHub;
	std::thread mThread;

	void ProcessHubEvent() {
		bool loop = true;
		while (loop) {
			GF_RET_CODE ret = mHub->registerListener(mHubListener);
			do {
				GF_UINT32 period = 50;
				mHub->run(period);
				if (mThreadQuitFlag.load()) { return; }
			} while (mGForceHandle->getLoop());
			mGForceHandle->setLoop();
			cout << "[WARNING] gForceDongle was pulled out" << endl;
			mHub->unRegisterListener(mHubListener);
			mHub->deinit();
			mSerialDetect->wakeupDetect();
		}
	}

public:
	gForceProcessor() :mThreadQuitFlag(false), identify(_T("EMG_CAPTURE")) {

	}
	~gForceProcessor() {

	}
	gForceProcessor& gForceProcess(gForceProcessor&) = delete;
	gForceProcessor& operator=(gForceProcessor&) = delete;
	void init() {
		mHub = HubManager::getHubInstance(identify);
		mHub->setWorkMode(WorkMode::Polling);
		mGForceHandle = make_shared<GForceHandle>(mHub);
		mHubListener = static_pointer_cast<HubListener>(mGForceHandle);
		mSerialDetect = std::make_shared<SerialDetect>();
		mDetectListener = std::make_shared<DetectDiscover>(mSerialDetect, mHub);
		mSerialDetect->RegisterListener(mDetectListener);
		mSerialDetect->Detect();
	}

	bool run() {
		if (nullptr == mHub) {
			return false;
		}
		mThread = std::thread(&gForceProcessor::ProcessHubEvent, this);
		return true;
	}
	void deint() {
		mThreadQuitFlag.store(true);
		if (mThread.joinable()) {
			mThread.join();
		}
		std::cout << "[INFO] gForceProcessor Thread quit\n";
		if (mHub) {
			mHub->unRegisterListener(mHubListener);
			mHub->deinit();
			mHub = nullptr;
		}
		if (mSerialDetect) {
			mSerialDetect->deinit();
			mSerialDetect = nullptr;
		}
	}
};


BOOL processCtrlHandler(DWORD fdwCtrlType) {
	BOOL fEventHandled = TRUE;
	switch (fdwCtrlType) {
	case CTRL_BREAK_EVENT:
	case CTRL_C_EVENT:
		break;
	case CTRL_CLOSE_EVENT:
	case CTRL_LOGOFF_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		fEventHandled = TRUE;
		mAppExit = true;
		Sleep(5000);
		break;
	default:
		break;
	}
	return fEventHandled;
}


void processInputCommand() {
	string inputCommand;

	while (true) {
		// std::cin >> inputCommand;
		// std::transform(inputCommand.begin(), inputCommand.end(), 
		// 	inputCommand.begin(), std::toupper);
		// if (inputCommand == "UPDATE") {
		// 	win_sparkle_check_update_with_ui();
		// }
		// else if (inputCommand == "EXIT") {
		// 	mAppExit.store(true);
		// 	return;
		// }
		// else {
		// }
		if (bRecording) {
			int in_c = _getch();
			if (in_c == 'Z' || in_c == 'z' || in_c == 'X' || in_c == 'x') {
				// Exit writing to the current file...

				// It takes a couple of seconds to flush some 'delayed' data.
				cout << "\nExiting.......\n"
					<< "Please wait 1 second for buffered data......\n";
				bRecording = false;
				Sleep(1000);
				g_file.close();
				cout << "\nFile " << g_filename << " has been saved successfully :-) \n\n";

				if (in_c == 'X' || in_c == 'x') { // exit the program
					mAppExit.store(true);
					break;
				}
				bGetNewFile = true;
			}
		}
		else {
			if (bGetNewFile) {
				Sleep(200);
				waitInputFileName();
			}
		}
		Sleep(500);
	}
}


static void waitInputFileName(void) {
	std::cout << "\n"
		<< "Please enter the name of the file for recording your EMG data:\n";
	std::string dir = "./data_file/";

	//make folder
	if (_access(dir.c_str(), 0) == -1) {
		_mkdir(dir.c_str());
	}

	while (1)
	{
		std::cin.getline(g_filename, FILE_NAME_SIZE);
		dir.append(g_filename);
		const string suffix[2] = { "_650Hz8Bit","_500Hz12Bit" };
		dir += suffix[g_EMGchoice];
		g_file.open(dir, std::ofstream::binary | ios_base::trunc);
		if (!g_file.is_open()) {
			cout << "Bad file name, please enter a new one:\n";
			continue;
		}
		break;
	}

	std::cout << "\nPress any key to record your EMG data to file " << g_filename << endl;
	_getch();
	std::cout << "Recording to file started..." << std::endl;
	bRecording = true;
	std::cout << "During recording, pressing 'Z' will close the file and promt you to open another file to record, and pressing 'X' will also exit the program.\n\n";
}


static void printSingleFileRecordedBytes(int n) {
	if (bGetNewFile) {
		printf("File %s's recorded bytes:          ", g_filename);
	}

	printf("\b\b\b\b\b\b\b\b%8d", n);
	bGetNewFile = false;
}


unsigned char emg_config_parameters(void)
{
	std::cout << "                 |  EMG PARAMETERS |                    " << "\n";
	std::cout << "+------------------------------------------------------+" << "\n";
	std::cout << "|   index        |    sampleRate   |   ADC resolution  |" << "\n";
	std::cout << "+------------------------------------------------------+" << "\n";
	std::cout << "|    0           |    650Hz        |    8bit           |" << "\n";
	std::cout << "+------------------------------------------------------+" << "\n";
	std::cout << "|    1           |    500Hz        |   12bit           |" << "\n";
	std::cout << "+------------------------------------------------------+" << "\n";
	std::cout << "Please input the index 0/1: ";

	char input[256];
	std::cin.getline(input, 256);
	std::string str(input);

	if (str.size() == 1 && str[0] == '1') {
		std::cout << "[INFO]: config EMG sampleRate: 500Hz, ADC resolution: 12bit" << "\n\n";
		return 0x01;
	}

	std::cout << "[INFO]: config EMG sampleRate: 650Hz, ADC resolution: 8bit" << "\n\n";

	return 0x00;
}


int _tmain(int argc, _TCHAR* argv[])
{
	cout << "===== gForce raw data capturing utility Ver0.3 =====\n\n";
	g_EMGchoice = emg_config_parameters();

	//win_sparkle_set_appcast_url("http://192.168.10.119:8845/appcast.xml");
	//win_sparkle_init();

	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)processCtrlHandler, TRUE)) {
		std::cout << "[ERROR]:Unable to install console control handler\n";
		std::cout << "[INFO] :It will quit after\n";
		system("PAUSE");
		return 0;
	}

	gForceProcessor hub;
	hub.init();
	bool ret = hub.run();

	if (ret == false) {
		std::cout << "[INFO] Run thread failure" << endl;
	}

	std::thread inputThread(processInputCommand);
	inputThread.detach();

	//std::thread webSocketThread = std::thread([]() {
	//	uWS::Hub h;
	//	h.onMessage([](uWS::WebSocket<uWS::SERVER> *ws, char *message, size_t length, uWS::OpCode opCode) {
	//		std::cout << "[INFO] Receive message from " << ws->getAddress().family << std::endl;
	//	});
	//	h.onConnection([](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest request) {
	//		g_serverlist.push_back(ws);
	//		std::cout << "[INFO] The count of client:" << g_serverlist.size() << std::endl;
	//		std::cout << "[INFO] client connect" << std::endl;
	//	});


	//	h.onDisconnection([](uWS::WebSocket<uWS::SERVER> *ws, int opcode, char *message, size_t length) {

	//		g_serverlist.remove(ws);
	//		std::cout << "[INFO] The count of client:" << g_serverlist.size() << std::endl;
	//		std::cout << "[INFO] client dissconnect" << std::endl;
	//	});
	//	if (!h.listen(8888)) {
	//		std::cout << "[ERROR] Filed to listen" << std::endl;
	//	}
	//	h.run();
	//});

	//webSocketThread.detach();

	//std::thread webServer = std::thread([]() {
	//	while (true) {
	//		gfsPtr<const std::vector<GF_UINT8>> value = g_MessageQueue.wait_and_pop();
	//		std::for_each(g_serverlist.begin(), g_serverlist.end(), [&value](uWS::WebSocket<uWS::SERVER> *it) {
	//			it->send((char*)value->data(), value->size(), uWS::OpCode::BINARY);
	//		});
	//	}
	//});

	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		static GF_UINT8 testVal = 0;

		if (mAppExit.load()) {
			hub.deint();
			//win_sparkle_cleanup();

			return 0;
		}
	}
}

