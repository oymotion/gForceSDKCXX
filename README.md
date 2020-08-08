# gForceSDK

## Brief

This repository include gForce SDK dll files and some simple examples.  

## F.A.Q.

> **How to get accelerate speed data of the `gForce`  by [gForceSDK](https://github.com/oymotion/gForceSDK/releases)?**

1. Implement `HubListener` interface

```c++
  class HubListenerImpl : public HubListener{...}
```

2. Config Accelerate(optional)

Call `DeviceSetting::setAccelerateConfig(...)` to configure ACC

3. Open ACC Data Notification

Call `DeviceSetting::setDataNotifySwitch(...)` to open ACC

4. Extract the data of acceleration

```C++
void onExtendedDeviceData(SPDEVICE device, DeviceDataType dataType, gfsPtr<const vector<GF_UINT8>> data) override
{
  switch (dataType) {
    case DeviceDataType::DDT_ACCELERATE:
    //... extract accelerate speed  from 'data'
    //                    Data format accelerate speed
    // *--------------------------------*---------------*---------------*
    // |ACC_X(data type = long(4 Byte)) | ACC_Y(...)    | ACC_Z(...)    |
    // *--------------------------------*---------------*---------------*
    // |accelerate speed at X axis      |...            |...            |
    // *--------------------------------*---------------*---------------*
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
