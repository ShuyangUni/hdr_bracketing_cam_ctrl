<!--
 * @Author: Shuyang Zhang
 * @Date: 2024-03-01 19:08:19
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2024-03-01 20:02:23
 * @Description: 
 * 
 * Copyright (c) 2024 by Shuyang Zhang, All Rights Reserved. 
-->
# Photometric Calibration
* The definition of photometric calibration can be found via [2008SIGGRAPH](https://www.pauldebevec.com/Research/HDR/debevec-siggraph97.pdf)
  * The photometric calibrtion aims to reveal the relationship between the scene irradiance and image intensity.
  * It is an internal property of a camera.
  * After the calibration, we can recover the scene irradiance, which means the essential information of the scene after removing the exposure attributes (exposure time and analogy gain)

* We use the method of [2008SIGGRAPH](https://www.pauldebevec.com/Research/HDR/debevec-siggraph97.pdf)
  * They provide a MATLAB version at the end of their paper