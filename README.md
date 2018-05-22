# VINS-Stereo

## image_processor:
使用了msckf_vio(https://github.com/KumarRobotics/msckf_vio)前端,加入了outlier剔除。
publish的特征点用unified camera model做了归一化。

## vins_estimator
使用了VINS-Mono()的后端估计，为了适应image_processor，这里调整了一下feature解析格式。


## vins_estimator_stereo
在vins_estimator基础上加入了双目约束。

## others
1.特征点分布均匀时效果更好。
2.使用bmi160(osr4)的效果比mpu9250(dlfp2.9ms)效果要好，bmi160的数据更稳定、噪声更小。
3.双目比单目效果略有提升。
4.加入imu插值后效果反而变差了，可能是vins-mono采用的是mid-point方法所致，但是港科大master分支也加了插值，这个问题待分析。
5.初始化时固定camera-imu之间的位置参数，效果变差，为什么?标定没有问题，mscfk跑的很好。