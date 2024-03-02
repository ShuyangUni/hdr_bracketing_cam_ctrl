'''
Author: Shuyang Zhang
Date: 2023-02-08 16:44:29
LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
LastEditTime: 2023-03-04 12:49:24
Description: 

Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved. 
'''

import PySpin

def trigger_mode(cam: PySpin.Camera, setting: str):
    """ trigger mode """
    try:
        # whether accessable
        if cam.TriggerMode.GetAccessMode() != PySpin.RW:
            print('Unable to disable trigger mode (node retrieval). Aborting...')
            return False
        # setting
        if setting == "On":
            cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
        elif setting == "Off":
            cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
        else:
            print(f'Unknown trigger mode: {setting}. Aborting...')
            return False
        print(f'\t\tCamera trigger mode set to {setting}...')
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        return False
    return True

def auto_exposure(cam: PySpin.Camera, setting: str):
    """ auto exposure """
    try:
        # whether accessable
        if cam.ExposureAuto.GetAccessMode() != PySpin.RW:
            print('Unable to set auto exposure. Aborting...')
            return False
        # setting
        if setting == 'Continuous':
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Continuous)
        elif setting == 'Once':
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Once)
        elif setting == 'Off':
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        else:
            print(f'Unknown auto exposure: {setting}. Aborting...')
            return False
        print(f'\t\tCamera auto exposure turned to {setting}...')
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        return False
    return True

def auto_gain(cam: PySpin.Camera, setting: str):
    """ auto gain """
    try:
        # whether accessable
        if cam.GainAuto.GetAccessMode() != PySpin.RW:
            print('Unable to set auto gain. Aborting...')
            return False
        # setting
        if setting == 'Continuous':
            cam.GainAuto.SetValue(PySpin.GainAuto_Continuous)
        elif setting == 'Once':
            cam.GainAuto.SetValue(PySpin.GainAuto_Once)
        elif setting == 'Off':
            cam.GainAuto.SetValue(PySpin.GainAuto_Off)
        else:
            print(f'Unknown auto gain: {setting}. Aborting...')
            return False
        print(f'\t\tCamera auto gain turned to {setting}...')
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        return False
    return True


def pixel_format(cam: PySpin.Camera, setting: str):
    """ pixel format """
    try:
        node_pixel_format = PySpin.CEnumerationPtr(
            cam.GetNodeMap().GetNode('PixelFormat'))
        if not PySpin.IsAvailable(node_pixel_format) \
                or not PySpin.IsWritable(node_pixel_format):
            print(
                f'Unable to set camera pixel format \
                    to {setting} (enum retrieval). Aborting...')
            return False
        node_pixel_format_setting = node_pixel_format.GetEntryByName(setting)
        if not PySpin.IsAvailable(node_pixel_format_setting) \
                or not PySpin.IsReadable(node_pixel_format_setting):
            print(
                f'Unable to set camera pixel format \
                    to {setting} (entry retrieval). Aborting...')
            return False
        pixel_format_setting = node_pixel_format_setting.GetValue()
        node_pixel_format.SetIntValue(pixel_format_setting)
        print(f'\t\tCamera pixel format set to {setting}...')
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        return False
    return True

def configure_chunk_data(nodemap):
    """ configure chunk data """
    try:
        result = True
        chunk_mode_active = PySpin.CBooleanPtr(
            nodemap.GetNode('ChunkModeActive'))
        if PySpin.IsAvailable(chunk_mode_active) and PySpin.IsWritable(chunk_mode_active):
            chunk_mode_active.SetValue(True)
        chunk_selector = PySpin.CEnumerationPtr(
            nodemap.GetNode('ChunkSelector'))
        if not PySpin.IsAvailable(chunk_selector) or not PySpin.IsReadable(chunk_selector):
            print('Unable to retrieve chunk selector. Aborting...\n')
            return False
        entries = [PySpin.CEnumEntryPtr(
            chunk_selector_entry) for chunk_selector_entry in chunk_selector.GetEntries()]
        for chunk_selector_entry in entries:
            if not PySpin.IsAvailable(chunk_selector_entry) \
                    or not PySpin.IsReadable(chunk_selector_entry):
                continue
            chunk_selector.SetIntValue(chunk_selector_entry.GetValue())
            chunk_str = f'\t\t{chunk_selector_entry.GetSymbolic()}:'
            chunk_enable = PySpin.CBooleanPtr(
                nodemap.GetNode('ChunkEnable'))
            if not PySpin.IsAvailable(chunk_enable):
                print(f'\t{chunk_str} not available')
                result = False
            elif chunk_enable.GetValue() is True:
                print(f'\t{chunk_str} enabled')
            elif PySpin.IsWritable(chunk_enable):
                chunk_enable.SetValue(True)
                print(f'\t{chunk_str} enabled')
            else:
                print(f'\t{chunk_str} not writable')
                result = False
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        result = False
    return result


def exposure_change(cam: PySpin.Camera, exp: float):
    """ exposure """
    try:
        if cam.ExposureTime.GetAccessMode() != PySpin.RW:
            print('Unable to set exposure time. Aborting...')
            return False
        cam.ExposureTime.SetValue(int(exp))
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        return False
    return True


def gain_change(cam: PySpin.Camera, gain: float):
    """ gain change """
    try:
        if cam.Gain.GetAccessMode() != PySpin.RW:
            print('Unable to set gain. Aborting...')
            return False
        cam.Gain.SetValue(float(gain))
    except PySpin.SpinnakerException as ex:
        print(f'PySpin.SpinnakerException: {ex}')
        return False
    return True