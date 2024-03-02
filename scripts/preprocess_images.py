'''
Author: Shuyang Zhang
Date: 2023-03-20 13:32:17
LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
LastEditTime: 2024-03-02 16:41:39
Description: 

Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved. 
'''

import os
import cv2
import numpy as np
import re
import pickle
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-i', '--input', required=True,
                    type=str, help='input calibration image folder')
args = parser.parse_args()


def main():
    """ main """

    img_names = os.listdir(args.input)
    img_names.sort()

    img = cv2.imread(os.path.join(
        args.input, img_names[0]), cv2.IMREAD_GRAYSCALE)

    expo_list = np.zeros(len(img_names), np.float32)
    gain_list = np.zeros(len(img_names), np.float32)
    for i, img_name in enumerate(img_names):
        params = [int(num) for num in re.findall(r'\d+', img_name)]
        expo_list[i] = params[0]
        gain_list[i] = round(params[1] + params[2]/10, 2)
    expo_list_uniq = np.unique(expo_list)
    gain_list_uniq = np.unique(gain_list)

    # get intersection
    idx_list = []
    param_list = []
    for expo in expo_list_uniq:
        for gain in gain_list_uniq:
            idx1 = np.where(expo_list == expo)
            idx2 = np.where(gain_list == gain)
            idx = np.intersect1d(idx1, idx2)
            idx_list.append(idx)
            param_list.append([expo, gain])

    # get images
    img_array = np.zeros(
        (img.shape[0] * img.shape[1], len(param_list)), np.uint8)
    expo_array = np.zeros(img.shape[0] * img.shape[1], np.float32)
    gain_array = np.zeros(img.shape[0] * img.shape[1], np.float32)
    for i, param in enumerate(param_list):
        expo = param[0]
        gain = param[1]
        print(expo, gain)
        idx = idx_list[i]
        img_sub = np.zeros((img.shape[0] * img.shape[1], len(idx)), np.uint8)
        for j, k in enumerate(idx):
            img_sub[:, j] = cv2.imread(os.path.join(
                args.input, img_names[k]), cv2.IMREAD_GRAYSCALE).flatten()
        img_array[:, i] = np.mean(img_sub, axis=1).astype(np.uint8)
        expo_array[i] = expo
        gain_array[i] = gain

    with open("avg_image.pickle", "wb+") as f:
        pickle.dump(img_array, f)
    with open("avg_expo.pickle", "wb+") as f:
        pickle.dump(expo_array, f)
    with open("avg_gain.pickle", "wb+") as f:
        pickle.dump(gain_array, f)


if __name__ == '__main__':
    main()
