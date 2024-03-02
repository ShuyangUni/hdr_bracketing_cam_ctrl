'''
Author: Shuyang Zhang
Date: 2023-02-08 16:44:29
LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
LastEditTime: 2024-03-02 17:30:09
Description: 

Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved. 
'''

from datetime import datetime
import os
import time
import numpy as np
import camera_attribute_control as cac
import PySpin

# parameters
# save folder
SAVE_FOLDER = '/home/uni/Data/custom/photometric_calibration'

# camera serial
# CAM_SERIAL = '21174973'
CAM_SERIAL = '23274452'
# attribute
GAIN_LOWER = 0.0
GAIN_UPPER = 20.0
GAIN_STEP = 1.0
EXPO_LOWER = 1000
EXPO_UPPER = 20000
EXPO_STEP = 1000
NUM_IMAGE = 3


def capture_images_by_parameters(cam, save_folder, expo, gain):
    cac.gain_change(cam, gain)
    cac.exposure_change(cam, expo)
    b_init = False
    while not b_init:
        image = cam.GetNextImage()
        if not image.IsIncomplete():
            chunk_data = image.GetChunkData()
            expo_re = int(chunk_data.GetExposureTime())
            gain_re = float(f"{chunk_data.GetGain():.2f}")
            if abs(expo_re - expo) < EXPO_STEP / 2 and abs(gain_re - gain) < GAIN_STEP / 2:
                b_init = True

    count = 0
    cac.exposure_change(cam, expo)
    while count < NUM_IMAGE:
        image = cam.GetNextImage()
        if not image.IsIncomplete():
            count = count + 1
            chunk_data = image.GetChunkData()
            expo_re = int(chunk_data.GetExposureTime())
            gain_re = float(f"{chunk_data.GetGain():.2f}")
            image_savepath = os.path.join(
                save_folder, f'{expo_re:05}_{gain_re}_{count:04}.jpg')
            image.Save(image_savepath)


def main():
    """ main """
    # init folder
    data_time = datetime.now()
    dataset_namebase = data_time.strftime("%Y-%m-%d-%H-%M-%S")
    save_folder = os.path.join(SAVE_FOLDER, CAM_SERIAL, dataset_namebase)
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    if cam_list.GetSize() == 0:
        cam_list.Clear()
        system.ReleaseInstance()
        print('not camera detected...')
        return False
    b_get_specified_cam = False
    for cam in cam_list:
        nodemap_tldevice = cam.GetTLDeviceNodeMap()
        node_serial = PySpin.CStringPtr(
            nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_serial) and PySpin.IsReadable(node_serial):
            serial_cam = node_serial.GetValue()
        if serial_cam == CAM_SERIAL:
            b_get_specified_cam = True
            cam_specified = cam
    if not b_get_specified_cam:
        print('specified camera not found...')
        return False
    cam_specified.Init()
    # init setting
    nodemap = cam_specified.GetNodeMap()
    cac.configure_chunk_data(nodemap)
    cac.trigger_mode(cam_specified, 'Off')
    cac.auto_exposure(cam_specified, 'Off')
    cac.auto_gain(cam_specified, 'Off')
    cac.gain_change(cam_specified, GAIN_LOWER)
    cac.exposure_change(cam_specified, EXPO_LOWER)

    cam_specified.BeginAcquisition()

    start_time = time.time()

    expo_list = np.arange(EXPO_LOWER, EXPO_UPPER, EXPO_STEP).tolist()
    expo_list.append(EXPO_UPPER)
    gain_list = np.arange(GAIN_LOWER, GAIN_UPPER, GAIN_STEP).tolist()
    gain_list.append(GAIN_UPPER)
    for expo in expo_list:
        for gain in gain_list:
            capture_images_by_parameters(
                cam_specified, save_folder, expo, gain)
            print(expo, gain)
    end_time = time.time()
    print(f"time comsumption: {end_time - start_time:.2f}s...")

    cam_specified.EndAcquisition()
    cam_specified.DeInit()
    del cam_specified
    del cam
    cam_list.Clear()
    system.ReleaseInstance()
    return True


if __name__ == '__main__':
    main()
