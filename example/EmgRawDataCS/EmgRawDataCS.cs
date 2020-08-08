/*
 * Copyright 2020, OYMotion Inc.
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
using System.Threading;
using System.Collections.Generic;

using gf;
using System.Text;

public class GForceHub
{
    public static GForceHub Instance
    {
        get {
            if (instance_ == null)
            {
                // lock
                mutex_.WaitOne();

                if (instance_ == null)
                {
                    instance_ = new GForceHub();
                }

                // unlock
                mutex_.ReleaseMutex();
            }

            return instance_;
        }
    }

    static void Main(string[] args)
    {
        // Display the number of command line arguments.
        // System.Console.WriteLine(args.Length);

        GForceHub.Instance.Reset();

        System.Console.WriteLine("------------------------------");
        System.Console.WriteLine("Working, press return to exit.");
        System.Console.WriteLine("------------------------------");

        System.Console.ReadLine();

        GForceHub.Instance.Exit();
    }

    public bool Reset()
    {
        if (hub_ != null)
        {
            if (connectedDevice_ != null)
            {
                connectedDevice_.disconnect();
                connectedDevice_ = null;
            }

            hub_.Dispose();
            hub_ = null;
        }

        hub_ = Hub.Instance;
        Prepare();

        return true;
    }

    void Exit()
    {
        if (hub_ != null)
        {
            Terminate();
            hub_.Dispose();
            hub_ = null;
        }

        System.Console.WriteLine("GForceHub.OnApplicationQuit");
    }

    void Scan()
    {
        if (needDeviceScan_)
        {
            needDeviceScan_ = false;
            RetCode ret = hub_.startScan();
            System.Console.WriteLine("startScan = {0}", ret);
        }
    }

    private static Mutex mutex_ = new Mutex();
    private static GForceHub instance_ = null;
    private Hub hub_ = null;
    private List<Device> foundDevices = new List<Device>();
    private Device connectedDevice_ = null;
    private bool needDeviceScan_ = true;

    private class Listener : HubListener
    {
        class DeviceComparer : IComparer<Device>
        {
            public int Compare(Device x, Device y)
            {
                if (x == null)
                {
                    if (y == null)
                        return 0;
                    else
                        return 1;
                }
                else
                {
                    if (y == null)
                        return -1;

                    uint xrssi = x.getRssi();
                    uint yrssi = y.getRssi();

                    if (xrssi > yrssi)
                        return -1;
                    else if (xrssi < yrssi)
                        return 1;
                    else
                        return 0;
                }
            }
        }

        private void TryConnect()
        {
            if (0 == gForceHub_.foundDevices.Count)
            {
                // scan for more device
                System.Console.WriteLine("No device found, scan again.");
                gForceHub_.needDeviceScan_ = true;
                gForceHub_.Scan();

                return;
            }

            System.Console.WriteLine("Now try to connect to gForce with largest rssi.");

            // 2. sort devices using rssi by descending
            gForceHub_.foundDevices.Sort(new DeviceComparer());
            int n = 0;

            do
            {
                if (n >= gForceHub_.foundDevices.Count)
                    break;

                if (gForceHub_.foundDevices[n].getConnectionStatus() != Device.ConnectionStatus.Disconnected)
                {
                    n++;
                    continue;
                }

                // try the one with best signal strength
                RetCode ret = gForceHub_.foundDevices[n].connect();

                // if failed to send connect command, try next
                if (ret != RetCode.GF_SUCCESS)
                {
                    System.Console.WriteLine("Connecting failed: {0}, try next.", ret);
                    gForceHub_.foundDevices.RemoveAt(n);
                }
                else
                {
                    // direct quit here if connecting started
                    return;
                }
            } while (gForceHub_.foundDevices.Count > 0);

            // seems connection failed, try scan again
            System.Console.WriteLine("Scan again due to connecting errors.");
            gForceHub_.needDeviceScan_ = true;
            gForceHub_.Scan();
        }

        public override void onScanFinished()
        {
            System.Console.WriteLine("onScanFinished");
            TryConnect();
        }

        public override void onStateChanged(Hub.HubState state)
        {
            System.Console.WriteLine("onStateChanged: {0}", state);
        }

        public override void onDeviceFound(Device device)
        {
            System.Console.WriteLine("onDeviceFound, name = \'{0}\', rssi = {1}",
                device.getName(), device.getRssi());

            gForceHub_.foundDevices.Add(device);
        }

        public override void onDeviceDiscard(Device device)
        {
            System.Console.WriteLine("onDeviceDiscard, handle = name is \'{0}\'", device.getName());

            bool ret = gForceHub_.foundDevices.Remove(device);
            System.Console.WriteLine("gForceHub_.foundDevices.Remove: {0} -> {1}", device.getName(), ret);

            if (device == gForceHub_.connectedDevice_)
            {
                gForceHub_.connectedDevice_ = null;
                TryConnect();
            }
        }

        public override void onDeviceConnected(Device device)
        {
            System.Console.WriteLine("onDeviceConnected, name is \'{0}\'", device.getName());

            if (gForceHub_.connectedDevice_ == null)
            {
                gForceHub_.connectedDevice_ = device;

                if (device != null)
                {
                    RetCode ret;

                    System.Console.WriteLine("setEmgConfig...");
                    ret = device.setEmgConfig(650/*sampleRateHz*/, 0x00FF/*interestedChannels*/, 128/*packageDataLength*/, 8/*adcResolution*/);
                    System.Console.WriteLine("setEmgConfig, ret : {0}", ret);

                    // You may try single attribute one time to determine which attributes are supported
                    DataNotifFlags flags = (DataNotifFlags)
                    (DataNotifFlags.DNF_OFF
                        //| DataNotifFlags.DNF_ACCELERATE
                        //| DataNotifFlags.DNF_GYROSCOPE
                        //| DataNotifFlags.DNF_MAGNETOMETER
                        //| DataNotifFlags.DNF_EULERANGLE
                        //| DataNotifFlags.DNF_QUATERNION
                        //| DataNotifFlags.DNF_ROTATIONMATRIX
                        //| DataNotifFlags.DNF_EMG_GESTURE
                        | DataNotifFlags.DNF_EMG_RAW
                        //| DataNotifFlags.DNF_HID_MOUSE
                        //| DataNotifFlags.DNF_HID_JOYSTICK
                        | DataNotifFlags.DNF_DEVICE_STATUS
                        );

                    // Because there is too much data when EMG data transfer is on,
                    // GF_ERROR_TIMEOUT can be considered as GF_SUCCESS.
                    System.Console.WriteLine("setNotification...");
                    ret = device.setNotification((uint)flags);
                    System.Console.WriteLine("setNotification, ret : {0}", ret);
                }
            }
        }

        public override void onDeviceDisconnected(Device device, int reason)
        {
            System.Console.WriteLine("onDeviceDisconnected, name is \'{0}\', reason is {1}",
                device.getName(), reason);

            if (device == gForceHub_.connectedDevice_)
            {
                System.Console.WriteLine("Need connect");
                gForceHub_.connectedDevice_ = null;

                bool ret = gForceHub_.foundDevices.Remove(device);
                System.Console.WriteLine("gForceHub_.foundDevices.Remove: {0} -> {1}", device.getName(), ret);
            }

            if (gForceHub_.connectedDevice_ == null)
            {
                foreach (Device dev in gForceHub_.foundDevices)
                {
                    if (null == dev)
                        continue;

                    Device.ConnectionStatus status = dev.getConnectionStatus();

                    if (Device.ConnectionStatus.Connecting == status)
                    {
                        // a device is in connecting state, we will do nothing until
                        // connection finished (succeeded or failed)
                        System.Console.WriteLine("A device is in connecting, wait after it done");
                        return;
                    }
                }

                TryConnect();
            }
        }

        public override void onOrientationData(Device device,
            float w, float x, float y, float z)
        {
            if (device == gForceHub_.connectedDevice_)
            {
                System.Console.WriteLine("onOrientationData, orientation(w,x,y,z): ({0}, {1}, {2}, {3})", w, x, y, z);
            }
        }

        public override void onGestureData(Device device, Device.Gesture gest)
        {
            if (device == gForceHub_.connectedDevice_)
            {
                System.Console.WriteLine("onGestureData, gesture: {0}", gest);
            }
        }

        public override void onDeviceStatusChanged(Device device, Device.Status status)
        {
            if (device == gForceHub_.connectedDevice_)
            {
                System.Console.WriteLine("onDeviceStatusChanged, status: {0}", status);
            }
        }

        public override void onExtendedDeviceData(Device device, Device.DataType type, byte[] data)
        {
            if (device == gForceHub_.connectedDevice_)
            {
                System.Console.Write("onExtendedDeviceData, type: {0}, len: {1}, data: ", type, data.Length);

                var sb = new StringBuilder("{");

                foreach (var b in data)
                {
                    sb.Append(b + ",");
                }

                sb.Remove(sb.Length - 1, 1);
                sb.Append("}");

                System.Console.WriteLine(sb.ToString());
            }
        }

        public Listener(GForceHub hub)
        {
            gForceHub_ = hub;
        }

        private GForceHub gForceHub_ = null;
    };

    Listener listener_ = null;

    private volatile bool threadRunning_ = false;

    private Thread runThread_;


    private void Prepare()
    {
        foundDevices.Clear();
        listener_ = new Listener(this);

        RetCode ret = hub_.registerListener(listener_);
        System.Console.WriteLine("registerListener = {0}", ret);
        ret = hub_.init(0);
        System.Console.WriteLine("init = {0}", ret);
        System.Console.WriteLine("Hub status is {0}", hub_.getStatus());
        hub_.setWorkMode(Hub.WorkMode.Polling);
        System.Console.WriteLine("New work mode is {0}", hub_.getWorkMode());
        threadRunning_ = true;
        runThread_ = new Thread(new ThreadStart(RunThreadFn));
        runThread_.Start();

        Scan();
    }

    private void Terminate()
    {
        threadRunning_ = false;
        if (runThread_ != null)
        {
            runThread_.Join();
        }

        hub_.unregisterListener(listener_);
        hub_.deinit();

        foundDevices.Clear();
    }

    private void RunThreadFn()
    {
        int loop = 0;

        while (threadRunning_)
        {
            RetCode ret = hub_.run(50);

            if (RetCode.GF_SUCCESS != ret && RetCode.GF_ERROR_TIMEOUT != ret)
            {
                System.Threading.Thread.Sleep(5);
                continue;
            }

            loop++;

#if DEBUG
            if (loop % 200 == 0)
                System.Console.WriteLine("runThreadFn: {0} seconds elapsed.", loop / 20);
#endif
        }

        System.Console.WriteLine("Leave thread");
    }
}

