#pragma once

#include <atlstr.h>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

class SerialDetectListener{
public:
	//virtual void findAvaliableHub(CString comport){}
	virtual void HubPlugIn(CString comport){}
	//virtual void HubPullOut(void){}
	virtual ~SerialDetectListener(){}
};


class SerialDetect{
public:
	SerialDetect();
	~SerialDetect();
	SerialDetect(SerialDetect const &) = delete;
	SerialDetect&  operator=(SerialDetect const &) = delete;

	void RegisterListener(std::weak_ptr<SerialDetectListener> listenerPtr);
	void UnRegisterListener(std::weak_ptr<SerialDetectListener> listenerPtr);
	void SetPortOpen(CString comport);
	void wakeupDetect();
	void Detect();
	void deinit();
private:
	const int m_nameLen= 100;
	bool enumSerialPort(CString& comport);
	bool checkSerialPortAvaliable();
	bool setupSerialPort(HANDLE comFile);
	bool ReadEventFromSerialPort(HANDLE comFile);

	CString mComName = "";
	std::atomic<bool> mHasOpenPort = false;
	std::atomic<bool> mQuit = false;
	std::weak_ptr<SerialDetectListener>  mListener;
	std::condition_variable m_conditonVariable;
	std::mutex m_cvMutex;
	std::thread mThread;
	void run();
};