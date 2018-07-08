# F.A.Q.

> How to get accelerate speed data of the `gForce`  by [gForceSDK](https://github.com/oymotion/gForceSDK/releases)
 
 

1. Implement `HubListener` interface  
```c++
	class HubListenerImpl : public HubListener{...}
```
1. Call `DeviceSetting::setAccelerateConfig(...)` to configure ACC (this step is optional)

2. Call `DeviceSetting::setDataNotifySwitch(...)` to open ACC 

3. Extract the data of accelerate speed  from `HubListenerImpl::onExtendedDeviceData(...)`
```c++
void onExtendedDeviceData(SPDEVICE device, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data) override
{
	switch (dataType) {
		case DeviceDataType::DDT_ACCELERATE:
		//... extract accelerate speed  from 'data'
		//                    Data format accelerate speed 
		//  _________________________________________________________________              
		// |ACC_X(data type = long(4 Byte)) | ACC_Y(...)    | ACC_Z(...)    |
		// |--------------------------------|---------------|---------------|
		// |accelerate speed at X axis      |...            |...            |
		// 
			auto ptr = data->data();
			long acc_x = *(reinterpret_cast<const long*>(ptr));
			long acc_y = *(reinterpret_cast<const long*>(ptr + 4));
			long acc_z = *(reinterpret_cast<const long*>(ptr + 8));
		break;
		case DeviceDataType::DDT_GYROSCOPE:
			//... extract gyroscope data form 'data'
		break;
		default:
		break;
	}
}
```