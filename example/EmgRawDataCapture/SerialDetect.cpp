#include "stdafx.h"
#include "SerialDetect.h"
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>

SerialDetect::SerialDetect(){

}

SerialDetect::~SerialDetect(){

}

bool SerialDetect::setupSerialPort(HANDLE comFile){

	DCB ndcb;
	DCB dcb;
	dcb.BaudRate = 115200;
	dcb.Parity = 0;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	COMMTIMEOUTS timeouts;
	/*set timeouts*/
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 2000;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(comFile, &timeouts);

	/*set com state*/
	if (!GetCommState(comFile, &ndcb)) {
		printf("GetCommState() failed!\n");
		return FALSE;
	}
	ndcb.DCBlength = sizeof(DCB);
	ndcb.BaudRate = dcb.BaudRate;
	ndcb.Parity = dcb.Parity;
	ndcb.ByteSize = dcb.ByteSize;
	ndcb.StopBits = dcb.StopBits;
	ndcb.fRtsControl = RTS_CONTROL_DISABLE;
	ndcb.fDtrControl = DTR_CONTROL_ENABLE;
	ndcb.fOutxCtsFlow = FALSE;
	ndcb.fOutxDsrFlow = FALSE;
	ndcb.fOutX = FALSE;
	ndcb.fInX = FALSE;
	if (!SetCommState(comFile, &ndcb)) {
		printf("SetCommState() failed!\n");
		return FALSE;
	}
	/*clear buffer*/
	PurgeComm(comFile, PURGE_RXCLEAR | PURGE_TXCLEAR);

	/*clear error*/
	DWORD dwError;
	COMSTAT cs;
	if (!ClearCommError(comFile, &dwError, &cs)) {
		printf("ClearCommError() failed");
		return FALSE;
	}

	/*set mask*/
	SetCommMask(comFile, EV_RXCHAR);

	return TRUE;
}

bool SerialDetect::enumSerialPort(CString& comport)
{
	UINT8 result = 0;
	UINT8 data[5] = { 0x01, 0x80, 0xFE, 0x01, 0x00 };

	CString sCom;

	DWORD dwBytesWritten = 0;
	OVERLAPPED osWrite;
	DCB dcb;
	HANDLE serialPortFile;
	memset(&osWrite, 0, sizeof(OVERLAPPED));

	dcb.BaudRate = 115200;
	dcb.Parity = 0;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	HKEY hKey;
	LPCTSTR lpSubKey = (LPCTSTR)"HARDWARE\\DEVICEMAP\\SERIALCOMM\\";

	if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, _T("HARDWARE\\DEVICEMAP\\SERIALCOMM\\"), 0, KEY_READ, &hKey) != ERROR_SUCCESS)
	{
		return false;
	}

	char szValueName[100];
	BYTE szPortName[100];
	LONG status;
	DWORD dwIndex = 0;
	DWORD dwSizeValueName = 100;
	DWORD dwSizeofPortName = 100;
	DWORD Type;
	do
	{
		dwSizeValueName = m_nameLen;
		dwSizeofPortName = m_nameLen;
		status = RegEnumValue(hKey, dwIndex++, (LPWSTR)szValueName, &dwSizeValueName, NULL, &Type,
			szPortName, &dwSizeofPortName);

		if ((status == ERROR_SUCCESS))
		{
			sCom.Format(_T("\\\\.\\%s"), szPortName);
			//wprintf(L"%s found\n", (const char*)sCom.GetBuffer(50));
			serialPortFile = CreateFile(sCom.GetBuffer(50),
				GENERIC_READ | GENERIC_WRITE,
				0,/* do not share*/
				NULL,
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
				NULL);

			if (serialPortFile == INVALID_HANDLE_VALUE)
			{
				if (ERROR_ACCESS_DENIED == GetLastError())
				{
					//printf("SerialPort is opened by others applications!! \n");
				}
				continue;
			}
			else
			{
				//CloseHandle(com_file);
				/*---------------Configure Com port---------------*/
				if (FALSE == setupSerialPort(serialPortFile))
				{
					CloseHandle(serialPortFile);
					serialPortFile = INVALID_HANDLE_VALUE;
					continue;
				}

				if (!WriteFile(serialPortFile, data, 5, &dwBytesWritten, &osWrite))
				{
					if (GetLastError() == ERROR_IO_PENDING) {
						GetOverlappedResult(serialPortFile, &osWrite, &dwBytesWritten, true);
						if (dwBytesWritten != 5){
							//printf("Send Request with length = %d!!! \n", dwBytesWritten);
							CloseHandle(serialPortFile);
							serialPortFile = INVALID_HANDLE_VALUE;
							continue;
						}
						else
						{
							//printf("Send Request with length = %d!!! \n", dwBytesWritten);
							if (TRUE == ReadEventFromSerialPort(serialPortFile))
							{
								comport = sCom;
								CloseHandle(serialPortFile);
								return TRUE;
							}
							else
							{
								CloseHandle(serialPortFile);
								serialPortFile = INVALID_HANDLE_VALUE;
								continue;
							}
						}
					}
					else
					{
						CloseHandle(serialPortFile);
						serialPortFile = INVALID_HANDLE_VALUE;
						continue;
					}
				}
				else
				{
					CloseHandle(serialPortFile);
					serialPortFile = INVALID_HANDLE_VALUE;
					continue;
				}
			}

		}
	} while ((status != ERROR_NO_MORE_ITEMS));

	RegCloseKey(hKey);

	return FALSE;
}

bool SerialDetect::checkSerialPortAvaliable(){
	HANDLE com_file;
	if (mComName == ""){
		return false;
	}
	/*----------------Open Com port----------------*/
	com_file = CreateFile(mComName.GetBuffer(50),
		GENERIC_READ | GENERIC_WRITE,
		0,/* do not share*/
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		NULL);

	if (com_file == INVALID_HANDLE_VALUE)
	{
		if (ERROR_ACCESS_DENIED == GetLastError())
		{
			return FALSE;
		}
		else if (ERROR_FILE_NOT_FOUND == GetLastError())
		{
			std::cout << "[ERROR]: gForceDongle is not avaliable!!!!!\n";
			return TRUE;
		}
	}
	else
	{
		//printf("SerialPort is available now!! \n");
		CloseHandle(com_file);
		return TRUE;
	}
	return FALSE;
}

bool SerialDetect::ReadEventFromSerialPort(HANDLE comFile)
{
	DWORD nLenOut = 0;
	UINT8 event[9] = { 0x00 };
	UINT8 event_expected[9] = { 0x04, 0xFF, 0x06, 0x7F, 0x06, 0x01, 0x80, 0xFE, 0x00 };
	DWORD remaining = 9;
	DWORD offset = 0;
	OVERLAPPED osRead;
	memset(&osRead, 0, sizeof(OVERLAPPED));
	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (comFile == NULL || comFile == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	while (remaining > 0) {
		if (ReadFile(comFile, (PUINT8)event + offset, remaining,
			&nLenOut, &osRead)) {
			remaining -= nLenOut;
			offset += nLenOut;
		}
		else {
			if (GetLastError() == ERROR_IO_PENDING) {
				if (WaitForSingleObject(osRead.hEvent, 1000) != WAIT_OBJECT_0)
				{
					return FALSE;
				}

				GetOverlappedResult(comFile, &osRead, &nLenOut, true);
				if (nLenOut) {
					remaining -= nLenOut;
					offset += nLenOut;
				}
			}
		}

	}

	if (memcmp(event, event_expected, 9) == 0)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
void SerialDetect::SetPortOpen(CString comPort){
	mHasOpenPort = true;
	mComName = comPort;
}

void SerialDetect::wakeupDetect(){
	mHasOpenPort = false;
	m_conditonVariable.notify_one();
}

void SerialDetect::RegisterListener(std::weak_ptr<SerialDetectListener> listenerPtr){
	mListener = listenerPtr;
}

void SerialDetect::UnRegisterListener(std::weak_ptr<SerialDetectListener> listenerPtr){
	
}
void SerialDetect::run(){
	bool fPrintHelpInfo = false;
	while (1){
		CString comName;
		if (enumSerialPort(comName)){
			auto sp = mListener.lock();
			if (nullptr != sp){
				sp->HubPlugIn(comName);
			}
			std::unique_lock<std::mutex> mtx(m_cvMutex);
			m_conditonVariable.wait(mtx, [this]() {return !mHasOpenPort.load(); });
			fPrintHelpInfo = false;
		}
		if (mQuit.load()) {
			return; // eixt the thread
		}
		if (!fPrintHelpInfo) {
			std::cout << "[INFO]: I can't find gForceDongle,Please insert it\n";
			fPrintHelpInfo = true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

void SerialDetect::Detect(){
	mQuit.store(false);
	mThread = std::thread(&SerialDetect::run, this);

}

void SerialDetect::deinit() {
	mQuit.store(true);
	wakeupDetect();
	if (mThread.joinable()) {
		mThread.join();
	}
	std::cout << "[DEBUG] serial detect thread quit" << std::endl;
}
