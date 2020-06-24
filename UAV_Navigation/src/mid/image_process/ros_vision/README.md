

# 相机标定

打开相机

```
roslaunch ros_vision usb_cam.launch
```

运行标定脚本

```
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.0245 image:=/usb_cam/image_raw camera:=/usb_cam
```

关于标定板见文件夹

```
dir:ros_vision/doc/calibrationdata
```

size为标点数，square为每个方格宽度(m)，image:=相机话题
标定完保存标定的数据，其中会有ost.yaml文件，把此文件的内容替换到此ros包下的camera_calibration.yaml文件内容。

# Install dependencies

- 二维码识别

	```
  sudo apt-get install ros-melodic-ar-track-alvar*
	```

- 框选物体识别

  ```
  sudo apt-get install ros-melodic-find-object-2d*
  ```

  运行

  ```
  roslaroslaunch ros_vision find_object_2d_camera.launch 
  ```

  

