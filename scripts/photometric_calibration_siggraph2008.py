'''
Author: Shuyang Zhang
Date: 2023-03-20 13:32:17
LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
LastEditTime: 2024-03-02 16:59:57
Description: 

Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved. 
'''

import pickle
import numpy as np
import matplotlib.pyplot as plt
import time
from tqdm import tqdm
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-lb', '--intensity_lower_bound', required=False, default=0,
                    type=int, help='lower bound of the input intensity')
parser.add_argument('-ub', '--intensity_upper_bound', required=False, default=255,
                    type=int, help='upper bound of the input intensity')
parser.add_argument('-n', '--num_seed', required=False, default=100,
                    type=int, help='number of seeds for calibration')
args = parser.parse_args()

intensity_min = args.intensity_lower_bound
intensity_max = args.intensity_upper_bound
num_seed = args.num_seed

# b_weight : 0-no weight, 1-2008 weight


def optimize(items, l, b_weight=1):
    n_func = intensity_max - intensity_min - 1
    n_irradiance = num_seed
    n_item = items.shape[1]

    mid_intensity = np.ceil(
        (intensity_max + intensity_min) / 2).astype(np.uint32)
    A = np.zeros((n_item + n_func + 1, n_func + n_irradiance), np.float32)
    B = np.zeros((n_item + n_func + 1), np.float32)
    W = np.arange(intensity_min + 1, intensity_max, 1)
    W = W - mid_intensity

    print("...generating least square matrices...")
    # item: intensity, idx, expo, snr
    for i in np.arange(n_item):
        intensity = items[0, i].astype(np.uint32) - intensity_min - 1
        irradiance_idx = items[1, i].astype(np.uint32)
        expo = items[2, i]
        snr = items[3, i]
        if b_weight == 1:
            w = W[intensity]
        else:
            w = 1
        A[i, intensity] = w
        A[i, n_func + irradiance_idx] = -w
        B[i] = w * (np.log(expo) + np.log(snr))

    for i in np.arange(0, intensity_max - intensity_min - 3, 1):
        if b_weight == 1:
            A[n_item + i, i] = l * W[i]
            A[n_item + i, i + 1] = -2 * l * W[i]
            A[n_item + i, i + 2] = l * W[i]
        else:
            A[n_item + i, i] = l
            A[n_item + i, i + 1] = -2 * l
            A[n_item + i, i + 2] = l

    A[n_item + n_func, mid_intensity - 1] = 1
    B[n_item + n_func] = 0

    # solve least square
    print(f"...solving linear square...A.shape {A.shape}...")
    ts = time.time()
    x = np.linalg.pinv(A) @ B
    te = time.time()
    print(f"...time comsumption: {te - ts:.2f}s...")

    func = x[:n_func]
    irradiance_map = x[n_func:]

    return func, irradiance_map


def main():
    """ main """
    with open("avg_image.pickle", "rb") as f:
        img_array = pickle.load(f)
    with open("avg_expo.pickle", "rb") as f:
        expo_array = pickle.load(f)
    with open("avg_gain.pickle", "rb") as f:
        gain_array = pickle.load(f)

    idx_seed = np.arange(img_array.shape[0])
    np.random.shuffle(idx_seed)
    idx_seed = idx_seed[:num_seed]
    idx_seed = np.array(idx_seed)
    # generate <intensity, idx, expo, snr> item
    snr_array = np.power(10, gain_array/20)
    items = np.array([], np.float32)
    for i in tqdm(range(img_array.shape[1])):
        img = img_array[idx_seed, i]
        expo = expo_array[i]
        snr = snr_array[i]
        idx_upper = np.where(img < intensity_max)
        idx_lower = np.where(img > intensity_min)
        idx = np.intersect1d(idx_upper, idx_lower)
        idx = np.array(idx)
        tmp_item = np.array([img[idx], idx, np.ones(
            idx.shape, np.float32) * expo, np.ones(idx.shape, np.float32) * snr])
        if items.shape[0] == 0:
            items = tmp_item
        else:
            items = np.append(items, tmp_item, axis=1)

    # # show intensity distribution
    # intensity = items[0,:]
    # plt.hist(intensity, bins=intensity_max-intensity_min -
    #          1, range=(intensity_min+1, intensity_max-1))
    # plt.show()

    # optimization
    l = 1
    func_no_weight, _ = optimize(items, l, 0)
    func_2008, _ = optimize(items, l, 1)

    # fitting for smoothness
    crf_coeff = np.polyfit(
        np.arange(intensity_min+1, intensity_max, 1), func_2008, 10)
    crf_func = np.poly1d(crf_coeff)
    crf_output = crf_func(np.arange(256))

    # result display
    l1 = plt.plot(np.arange(intensity_min+1, intensity_max, 1),
                  func_no_weight, label="no weight")
    l2 = plt.plot(np.arange(intensity_min+1, intensity_max, 1),
                  func_2008, label="2008")
    l3 = plt.plot(np.arange(256),
                  crf_output, label="2008 after smooth")
    lns = l1 + l2 + l3
    labels = [l.get_label() for l in lns]
    plt.legend(lns, labels, loc=0)
    plt.show()

    # save results
    with open('crf_output.txt', 'w') as f:
        for val in crf_output:
            f.write(f"{val:06f}\n")


if __name__ == '__main__':
    main()
