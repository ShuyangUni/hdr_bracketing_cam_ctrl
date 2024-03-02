<!--
 * @Author: Shuyang Zhang
 * @Date: 2024-03-01 19:08:19
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2024-03-02 17:45:00
 * @Description: 
 * 
 * Copyright (c) 2024 by Shuyang Zhang, All Rights Reserved. 
-->
# Photometric Calibration
* The definition of photometric calibration can be found via [2008SIGGRAPH](https://www.pauldebevec.com/Research/HDR/debevec-siggraph97.pdf)
  * The photometric calibrtion aims to reveal the relationship between the scene irradiance and image intensity.
  * It is an internal property of a camera.
  * After the calibration, we can recover the scene irradiance, which means the essential information of the scene after removing the exposure attributes (exposure time and analogy gain)
  * More details can be found in the paper [2008SIGGRAPH](https://www.pauldebevec.com/Research/HDR/debevec-siggraph97.pdf)

* The implementation of the photometric calibration method
  * They provide a MATLAB version at the end of their paper
  * We also provide a Python version in the [scripts](../scripts/) folder

* To run our Python codes for photometric calibration, you need to 
  1. Select a static scene
      * Make sure there are no overly bright or dark pixels
      * Make sure there are no direct light source in the images
  2. Capture images using the code [capture_combination.py](../scripts/capture_combination.py)
      * Install python extension of Spinnaker ([PySpin]((https://www.flir.com/products/spinnaker-sdk/)))
      * Change the serial number to the one of your device
      * Adjust the range and step size of exposure time and analog gain
  3. Preprocess images using the code [preprocess_images.py](../scripts/preprocess_images.py)
      * The code will generate temporary files in pickle format
  4. Run the photometric calibration method using the code [photometric_calibration_siggraph2008.py](../scripts/photometric_calibration_siggraph2008.py)
* After this process, the code will generate a txt file with 256 lines of float values. Please replace the CRF file in [param/coeff](../param/coeff/) folder.
