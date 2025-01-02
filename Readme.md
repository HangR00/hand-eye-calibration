# Hand Eye Calibration

Two calibration method are provided: eye off hand and eye in hand.

Before using this script, another vision script must be provided to get **obj/tag2camera** matrix (4*4).

**obj/tag2camera** matrix is in ROS communication pipeline.

https://github.com/HangR00/hand-eye-calibration.git

# 手眼标定

很多开源的手眼标定程序都是基于某些特定型号的机械臂与相机开发的，在这里我们只关注手眼标定本身，给出一个通用的框架，可以适配任意六轴机械臂和RGBD相机。

在使用之前，开发者必须要明确以下几个概念，这也是手眼标定需要的几个数据（为了简便，在这里变量名字与程序中保持一致）：

* obj2camera：物体在相机坐标系下的变换矩阵
* tag2camera：标定板在相机坐标系下的变换矩阵
* base2end：机械臂基座在机械臂末端坐标系下的变换矩阵
* camera2base：相机在机械臂基座坐标系下的变换矩阵
* 值得一提的是A2B矩阵的逆矩阵即为B2A（其中A、B可以是‘obj’，‘tag’，‘base’，‘end’，‘camera’中的不相同的任意一个）

## 环境要求

Linux（推荐Ubuntu，版本<=20.04），推荐python>=3.8

克隆本仓库：
   - `[pip install -r requirements.txt](https://github.com/HangR00/hand-eye-calibration.git)`

安装必要的库：
   - `pip install -r requirements.txt`

## tag2camera的获取

tag2camera来自于相机的视觉读取进程，由于每位开发者使用的硬件设施不同，在这里无法使用一个通用的程序来实现，所以我们假定你已经可以获得标定板在相机视角下的姿态与位置。你需要将其改为4*4的变换矩阵，然后通过ROS的发布-订阅机制的通信管道完成多进程间的数据通信：

在我们提供的脚本中，有ROS的消息接收语句：

比如接收tag2camera的语句
```python
def camera_callback(rece_tag_trans_mat):
    '''
    ROS callback function: get the transform matrix of tag to camera from camera process
    raw data is a 2D array, but it must be compressed to 1D array when entering the ROS pipeline
    so we need to decode it to 2D array
    @input:
        rece_tag_trans_mat: tag2camera matrix from ROS
    @output:
        None
    '''
    # tag2camera matrix
    global tag_trans_mat
    if rece_tag_trans_mat.data == []:
        pass
    else :
        tag_trans_mat = rece_tag_trans_mat.data
        tag_trans_mat = list(tag_trans_mat)
        tag_trans_mat = [tag_trans_mat[i:i+4] for i in range(0, len(tag_trans_mat), 4)]

rospy.Subscriber('/tag_trans_mat',Float32MultiArray,camera_callback)
```
 
需要注意的是，Float32MultiArray类型数据无法传送二维数组，所以在这里你需要提前将4x4的二维数组转换为1x16的一维数组，然后再送入ROS的通信管道；同样地，在回调函数的接收端要做相应的解码处理，变换为4x4的二维矩阵。

## 标定主程序

主程序分为三个部分，一个是机械臂初始化，一个是数据收集阶段，最后是标定处理阶段。

您需要根据自身硬件做出改动的地方有： 

* init()函数：您需要根据实际的机械臂重新覆写机械臂初始化阶段
* 以tf_eyeoffhand.py为例，第162行：您需要根据实际的机械臂SDK或示教器来获取机械臂末端的数据，通常是一个一维数组[X, Y, Z, RX, RY, RZ]

是的，您只需要根据自身硬件做出这些改动即可开始标定！






