# ACE电控-视觉解算说明书
接口说明

### 初始化处：

![image-20240116214650576](C:\Users\CAYON2022\AppData\Roaming\Typora\typora-user-images\image-20240116214650576.png)

#### 获取陀螺仪指针

`get_imu_control_point()`

#### 获取pitch轴设定值指针

 `get_Gimbal_pitch_point()`

把你的pitch轴设定值地址传入这里

#### 获取yaw轴设定值指针

`get_Gimbal_yaw_point()`

把你的yaw轴设定值地址传入这里

#### 获取云台当前状态

`get_gimbal_behaviour_point()`

这个是这个

![image-20240116214406261](C:\Users\CAYON2022\AppData\Roaming\Typora\typora-user-images\image-20240116214406261.png)

你可以取一个叫这个名字的，目前还没有用上

### include

![image-20240116214545190](C:\Users\CAYON2022\AppData\Roaming\Typora\typora-user-images\image-20240116214545190.png)

在注释出加入以上指针函数就可以了

# 使用说明

首先初始化时把上面的变量赋值

然后在解算后获取pitch和yaw轴的，视觉传给你pitch和yaw轴设定的目标值

![image-20240116215110475](C:\Users\CAYON2022\AppData\Roaming\Typora\typora-user-images\image-20240116215110475.png)

此处获取云台自瞄控制指针

![image-20240116215141490](C:\Users\CAYON2022\AppData\Roaming\Typora\typora-user-images\image-20240116215141490.png)

在你的云台任务里调用`auto_control_p`的`auto_pitch`和`auto_yaw`即可

# 注意事项

有可能在云台任务初始化时，自瞄控制指针还没被初始化，没地址，云台任务获取的指针没获取到。

我的处理方式是一直等待视觉任务初始化后再让云台任务初始化，还有其他方法，自便

当视觉没瞄到的时候，我注意到：

![image-20240116215637192](C:\Users\CAYON2022\AppData\Roaming\Typora\typora-user-images\image-20240116215637192.png)

这个参数是0，

可以把这个作为判断视觉是否瞄到目标

==注意限位==视觉**没瞄到目标的时候直接解算了个pitch目标值是90**，小心橄榄电机

最好把电机YAW轴使用==LQR==控制算法，因为视觉对跟随性要求较高

# 火控

我的办法是当前设定值和实际值足够小的时候，允许开火，否则不允许开火

差值是要＜0.5f