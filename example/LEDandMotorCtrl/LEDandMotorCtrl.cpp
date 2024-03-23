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
#include <functional>
#include <chrono>


using namespace gf;
using namespace std;

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;


atomic<bool> bExiting = false;

BOOL ctrlhandler(DWORD fdwctrltype)
{
	switch (fdwctrltype)
	{
	case CTRL_C_EVENT:
		// handle the ctrl-c signal.
		bExiting = true;
		break;
	case CTRL_CLOSE_EVENT:
		// ctrl-close: confirm that the user wants to exit.
		bExiting = true;
		break;
	case CTRL_BREAK_EVENT:
		// pass other signals to the next handler.
		bExiting = true;
		break;
	case CTRL_LOGOFF_EVENT:
		bExiting = true;
		break;
	case CTRL_SHUTDOWN_EVENT:
		bExiting = true;
		break;
	default:;
	}
	if (bExiting)
		return TRUE;
	else
		return FALSE;
}

// The GForceHandle implements HubListener,
//     operates gForce device and receives data
class GForceHandle : public HubListener
{
public:
	GForceHandle(gfsPtr<Hub>& pHub)
		: mHub(pHub)
	{
	}
	/// This callback is called when the Hub finishes scanning devices.
	virtual void onScanFinished() override
	{
		cout << __FUNCTION__ << " has been called." << endl;
		if (nullptr == mDevice)
		{
			// if no device found, we do scan again
			mHub->startScan();
		}
		else
		{
			// or if there already is a device found and it's not
			//     in connecting or connected state, try to connect it.
			DeviceConnectionStatus status = mDevice->getConnectionStatus();
			if (DeviceConnectionStatus::Connected != status &&
				DeviceConnectionStatus::Connecting != status)
			{
				mDevice->connect();
			}
		}
	}

	/// This callback is called when the state of the hub changed
	virtual void onStateChanged(HubState state) override
	{
		cout << __FUNCTION__ << " has been called. New state is " << static_cast<GF_UINT32>(state) << endl;
		// if the hub is disconnected (such as unplugged), then set the flag of exiting the app.
		if (HubState::Disconnected == state)
		{
			mLoop = false;
		}
	}

	/// This callback is called when the hub finds a device.
	virtual void onDeviceFound(SPDEVICE device) override
	{
		// In the sample app, we only connect to one device, so once we got one, we stop scanning.
		cout << __FUNCTION__ << " has been called." << endl;
		if (nullptr != device)
		{
			// only search the first connected device if we connected it before
			if (nullptr == mDevice || device == mDevice)
			{
				mDevice = device;
				mHub->stopScan();
			}
		}
	}

	/// This callback is called a device has been connected successfully
	virtual void onDeviceConnected(SPDEVICE device) override
	{
		cout << __FUNCTION__ << " has been called." << endl;

		if (nullptr != device)
		{
			gfsPtr<DeviceSetting> ds = device->getDeviceSetting();
			if (nullptr != ds)
			{
				ds->enableDataNotification(0);	// Disable data notificition
			}
		}
	}

	/// This callback is called when a device has been disconnected from
	///                                 connection state or failed to connect to
	virtual void onDeviceDisconnected(SPDEVICE device, GF_UINT8 reason) override
	{
		// if connection lost, we will try to reconnect again.
		cout << __FUNCTION__ << " has been called. reason: " << static_cast<GF_UINT32>(reason) << endl;
		if (nullptr != device && device == mDevice)
		{
			mDevice->connect();
		}
	}

	/// This callback is called when the quaternion data is received
	virtual void onOrientationData(SPDEVICE device, const Quaternion& rotation) override
	{
		// print the quaternion data
		cout << __FUNCTION__ << " has been called. " << rotation.toString() << endl;
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
		cout << __FUNCTION__ << ": datatype = " << (GF_UINT32)dataType << ", datalength = " << data->size()
			<< ", first byte: " << hex << (GF_UINT32)((data->size() > 0) ? data->at(0) : 0xFF)
			<< ", last byte: " << (GF_UINT32)((data->size() > 0) ? data->at(data->size() - 1) : 0xFF) << dec << endl;

		if (data->size() == 0)
			return;

		switch (dataType) {
		case DeviceDataType::DDT_ACCELERATE:
			cout << "DeviceDataType::DDT_ACCELERATE" << endl;
			break;

		case DeviceDataType::DDT_GYROSCOPE:
			break;

		case DeviceDataType::DDT_MAGNETOMETER:
			break;

		case DeviceDataType::DDT_EULERANGLE:
			break;

		case DeviceDataType::DDT_QUATERNION:
			break;

		case DeviceDataType::DDT_ROTATIONMATRIX:
			break;

		case DeviceDataType::DDT_GESTURE:
			break;

		case DeviceDataType::DDT_EMGRAW:
			break;

		case DeviceDataType::DDT_HIDMOUSE:
			break;

		case DeviceDataType::DDT_HIDJOYSTICK:
			break;

		case DeviceDataType::DDT_DEVICESTATUS:
			break;
		}
	}

	// Indicates if we want to exit app
	bool getLoop()
	{
		return mLoop;
	}

	gfsPtr<Device> getDevice()
	{
		return mDevice;
	}

private:
	// Indicates if we will keep message polling
	bool mLoop = true;
	// keep a instance of hub.
	gfsPtr<Hub> mHub;
	// keep a device to operate
	gfsPtr<Device> mDevice;
};

int _tmain(int argc, _TCHAR* argv[])
{
	GF_RET_CODE retCode = GF_RET_CODE::GF_SUCCESS;

	// get the hub instance from hub factory
	tstring identifer(_T("FetchGForceHandle sample app"));
	auto pHub = HubManager::getHubInstance(identifer);

	// set work mode to WorkMode::Polling, then the client has
	//     to call Hub::run to receive callbacks
	pHub->setWorkMode(WorkMode::Polling);

	// create the listener implementation and register to hub
	gfsPtr<GForceHandle> gforceHandle = make_shared<GForceHandle>(pHub);
	gfsPtr<HubListener> listener = static_pointer_cast<HubListener>(gforceHandle);
	retCode = pHub->registerListener(listener);
	cout << "registerListener " << ((retCode == GF_RET_CODE::GF_SUCCESS) ? "SUCCESS" : "FAIL") << endl;

	// Initialize hub. Could be failed in below cases:
	//   1. The hub is not plugged in the USB port.
	//   2. Other apps are connected to the hub already.
	retCode = pHub->init();
	if (GF_RET_CODE::GF_SUCCESS != retCode)
	{
		cout << "hub init failed: " << static_cast<GF_UINT32>(retCode) << endl;
		return 1;
	}

	// start to scan devices
	retCode = pHub->startScan();
	if (GF_RET_CODE::GF_SUCCESS != retCode)
	{
		cout << "scan starting failed: " << static_cast<GF_UINT32>(retCode) << endl;
		goto exit;
	}

	// set console handler to receive Ctrl+C command so that we can exit the app by Ctrl+C.
	if (SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlhandler, TRUE))
	{
		unsigned int loopCnt = 0;
		bool ledOn = false;
		bool motorOn = true;
		long long lastCmdTime = 0;


		do
		{
			// set up 50ms timeout so we can handle console commands
			GF_UINT32 period = 50; // ms

			// Hub::run could be failed in the below cases:
			//   1. other threads have already been launching it.
			//   2. WorkMode is not set to WorkMode::Polling.
			// A return of GF_RET_CODE::GF_ERROR_TIMEOUT means no error but period expired.
			retCode = pHub->run(period);
			if (GF_RET_CODE::GF_SUCCESS != retCode &&
				GF_RET_CODE::GF_ERROR_TIMEOUT != retCode)
			{
				cout << "Method run() failed: " << static_cast<GF_UINT32>(retCode) << endl;
				break;
			}

			auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			if (millisec_since_epoch - lastCmdTime > 2000)	// 2000ms
			{
				auto device = gforceHandle->getDevice();

				if (device != nullptr && device->getConnectionStatus() == DeviceConnectionStatus::Connected)
				{
					auto ds = device->getDeviceSetting();

					if (ds)
					{
						// LED control
						ds->ledControlTest(ledOn ? DeviceSetting::LedControlTestType::On : DeviceSetting::LedControlTestType::Off,
							[](ResponseResult result) {
								string ret = (result == ResponseResult::RREST_SUCCESS) ? ("sucess") : ("failed");
								printf("result of ledControlTest is %s(%d)\n", ret.c_str(), result);

								if (result != ResponseResult::RREST_SUCCESS)
								{
									// Do something

								}
							}
						);

						// Motor control
						ds->vibrateControl(motorOn ? DeviceSetting::VibrateControlType::On : DeviceSetting::VibrateControlType::Off,
							[](ResponseResult result) {
								string ret = (result == ResponseResult::RREST_SUCCESS) ? ("sucess") : ("failed");
								printf("result of vibrateControl is %s(%d)\n", ret.c_str(), result);

								if (result != ResponseResult::RREST_SUCCESS)
								{
									// Do something

								}
							}
						);
					}
				}

				ledOn = !ledOn;
				motorOn = !motorOn;
				lastCmdTime = millisec_since_epoch;
			}
		}
		// Loop ends while the hub is unplugged or Ctrl+C is pressed.
		while (gforceHandle->getLoop() && !bExiting);
	}
	else
	{
		cout << "Error: could not set control handler." << endl;
	}

exit:
	// Clean execution envionment while exiting
	pHub->unRegisterListener(listener);
	pHub->deinit();

	SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlhandler, FALSE);

	return 0;
}

