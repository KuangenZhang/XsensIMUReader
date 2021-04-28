
#  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#  
#  1.	Redistributions of source code must retain the above copyright notice,
#  	this list of conditions, and the following disclaimer.
#  
#  2.	Redistributions in binary form must reproduce the above copyright notice,
#  	this list of conditions, and the following disclaimer in the documentation
#  	and/or other materials provided with the distribution.
#  
#  3.	Neither the names of the copyright holders nor the names of their contributors
#  	may be used to endorse or promote products derived from this software without
#  	specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
#  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
#  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
#  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
#  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
#  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
#  
import numpy as np
import sys
import xsensdeviceapi as xda
import time
import keyboard
from threading import Lock

init_time = time.time()

class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size = 5):
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self):
        self.m_lock.acquire()
        assert(len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        self.m_lock.release()
        return oldest_packet

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        assert(packet != 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()


def get_port_vec(ref_device_id_vec):
    for i in range(10):
        portInfoArray = xda.XsScanner_scanPorts()
        if len(ref_device_id_vec) == portInfoArray.size():
            break
    imu_num = portInfoArray.size()
    port_vec = np.zeros(imu_num, dtype=np.object)
    device_id_vec = np.zeros(imu_num, dtype=np.object)
    if imu_num == len(ref_device_id_vec):
        for i in range(imu_num):
            # Find an MTi device
            port_vec[i] = xda.XsPortInfo()
            port_vec[i] = portInfoArray[i]
            device_id_vec[i] = str(port_vec[i].deviceId())
            if port_vec[i].empty():
                raise RuntimeError("No MTi device found. Aborting.")
    else:
        raise RuntimeError("Not enough MTi device found. Aborting.")
    sorted_port_vec, sorted_device_id_vec = sort_ports(port_vec, device_id_vec, ref_device_id_vec)
    return sorted_port_vec, sorted_device_id_vec


def sort_ports(port_vec, device_id_vec, ref_device_id_vec):
    '''
    :param port_vec:
    :param device_id_vec:
    :param ref_device_id_vec:
    :return: sorted device based on the order of imu device id.
    '''
    sorted_port_vec = np.copy(port_vec)
    for i in range(len(device_id_vec)):
        sorted_port_vec[i] = port_vec[device_id_vec == ref_device_id_vec[i]][0]
        print('Device ID: {}, reference device ID: {}'.format(sorted_port_vec[i].deviceId(), ref_device_id_vec[i]))
    return sorted_port_vec, ref_device_id_vec


def open_device(port_vec):
    device_vec = np.zeros(len(port_vec), dtype=np.object)
    callback_vec = np.zeros(len(port_vec), dtype=np.object)
    control_vec = np.zeros(len(port_vec), dtype=np.object)
    for i in range(len(port_vec)):
        print("Creating XsControl object...")
        control = xda.XsControl_construct()
        assert (control != 0)
        mtPort = port_vec[i]
        print("Opening port...")
        if not control.openPort(mtPort.portName(), mtPort.baudrate()):
            raise RuntimeError("Could not open port. Aborting.")

        did = mtPort.deviceId()
        print("Found a device with:")
        print(" Device ID: %s" % did.toXsString())
        print(" Port name: %s" % mtPort.portName())

        print("Opening port...")
        if not control.openPort(mtPort.portName(), mtPort.baudrate()):
            raise RuntimeError("Could not open port. Aborting.")

        # Get the device object
        device = control.device(did)
        assert (device != 0)

        print("Device: %s, with ID: %s opened." % (device.productCode(), device.deviceId().toXsString()))

        # Create and attach callback handler to device
        callback = XdaCallback()
        device.addCallbackHandler(callback)

        # Put the device into configuration mode before configuring the device
        print("Putting device into configuration mode...")
        if not device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        print("Configuring the device...")
        configArray = xda.XsOutputConfigurationArray()
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))

        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))

        if not device.setOutputConfiguration(configArray):
            raise RuntimeError("Could not configure the device. Aborting.")

        print("Creating a log file...")
        logFileName = "log/logfile_{}.mtb".format(i)
        if device.createLogFile(logFileName) != xda.XRV_OK:
            raise RuntimeError("Failed to create a log file. Aborting.")
        else:
            print("Created a log file: %s" % logFileName)

        print("Putting device into measurement mode...")
        if not device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        print("Starting recording...")
        if not device.startRecording():
            raise RuntimeError("Failed to start recording. Aborting.")

        device_vec[i] = device
        callback_vec[i] = callback
        control_vec[i] = control
    return device_vec, callback_vec, control_vec

def capture_one_frame(callback_vec):
    '''
    :param device_vec:
    :param callback_vec:
    :return: data_vec
    data_vec: imu on the hip, knee, and ankle, time in second
    each imu: euler, acc, gyr
    each signal: x, y, z
    '''
    data_vec = np.zeros(len(callback_vec) * 9 + 1)
    data_vec[-1] = time.time() - init_time
    for i in range(len(callback_vec)):
        callback = callback_vec[i]
        if callback.packetAvailable():
            # Retrieve a packet
            packet = callback.getNextPacket()
            euler = packet.orientationEuler()
            acc = packet.calibratedAcceleration()
            gyr = packet.calibratedGyroscopeData()
            data_vec[(9 * i): (9 * (i + 1))] = np.array([euler.x(), euler.y(), euler.z(),
                                                         acc[0], acc[1], acc[2],
                                                         gyr[0], gyr[1], gyr[2]])
    return data_vec
            # s = ""
            # # quaternion = packet.orientationQuaternion()
            # # s = "q0: %.2f" % quaternion[0] + ", q1: %.2f" % quaternion[1] + ", q2: %.2f" % quaternion[
            # #     2] + ", q3: %.2f " % quaternion[3]
            # euler = packet.orientationEuler()
            # # s += " |Roll: %.2f" % euler.x() + ", Pitch: %.2f" % euler.y() + ", Yaw: %.2f " % euler.z()
            # acc = packet.calibratedAcceleration()
            # s = "Acc X: %.2f" % acc[0] + ", Acc Y: %.2f" % acc[1] + ", Acc Z: %.2f" % acc[2]
            #
            #
            # gyr = packet.calibratedGyroscopeData()
            # s += " |Gyr X: %.2f" % gyr[0] + ", Gyr Y: %.2f" % gyr[1] + ", Gyr Z: %.2f" % gyr[2]
            #
            # mag = packet.calibratedMagneticField()
            # s += " |Mag X: %.2f" % mag[0] + ", Mag Y: %.2f" % mag[1] + ", Mag Z: %.2f" % mag[2]
            # if packet.containsLatitudeLongitude():
            #     latlon = packet.latitudeLongitude()
            #     s += " |Lat: %7.2f" % latlon[0] + ", Lon: %7.2f " % latlon[1]
            #
            # if packet.containsAltitude():
            #     s += " |Alt: %7.2f " % packet.altitude()
            #
            # if packet.containsVelocity():
            #     vel = packet.velocity(xda.XDI_CoordSysEnu)
            #     s += " |E: %7.2f" % vel[0] + ", N: %7.2f" % vel[1] + ", U: %7.2f " % vel[2]
            # print("%s\r" % s, end="", flush=True)

def close_device(device_vec, control_vec, port_vec, callback_vec):
    for i in range(len(device_vec)):
        device = device_vec[i]
        mtPort = port_vec[i]
        callback = callback_vec[i]
        print("\nStopping recording...")
        if not device.stopRecording():
            raise RuntimeError("Failed to stop recording. Aborting.")
        print("Closing log file...")
        if not device.closeLogFile():
            raise RuntimeError("Failed to close log file. Aborting.")

        print("Removing callback handler...")
        device.removeCallbackHandler(callback)

        print("Closing port...")
        control_vec[i].closePort(mtPort.portName())

        print("Closing XsControl object...")
        control_vec[i].close()


if __name__ == '__main__':
    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    assert(control != 0)

    ref_device_id_vec = np.array(['038819D6', '038818ED', '03881B05'])  # Hip, Knee, Ankle
    port_vec,_ = get_port_vec(ref_device_id_vec)
    device_vec, callback_vec, control_vec = open_device(port_vec)
    for i in range(1000):
        data_vec = capture_one_frame(callback_vec)
        time.sleep(1e-2)
        print(data_vec)
    close_device(device_vec, control_vec, port_vec, callback_vec)