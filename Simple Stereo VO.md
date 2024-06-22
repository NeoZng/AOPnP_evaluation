# Simple Stereo VO

简要介绍程序流程.

start from `rosNodeTest.cpp`

1. 初始化,读取参数文件. 这里没用使用ros1的parameter server, 而是启动结点时从命令行传入参数文件路径, 然后使用`cv::FileStorage` 读取yaml文件.
2. 创建`estimator`, 注册发布者和订阅者.
3. 启动sync_proces, 用于同步左右图像数据.

4. 左右目topic的图像回调函数会将打上时间戳的图像数据放入队列中, 然后`sync_process`会从队列中取出数据, 进行处理.
5. `sync_process`获取同步图像后, 调用`estimator.inputImage`
6. `input_Image` 调用featuretracker的跟踪模块,利用光流更新点的对应关系

7. 光流跟踪点通过`processMeasurements`处理, 这是核心函数
8. 在7中调用`processImage`, 首先添加追踪点(滑窗,因为一个点可以被连续多帧追踪到)
9. 通过`initFramePoseByPnP`计算新帧的位姿(初始化的第一帧不会调用)
10. 通过左目-左目的追踪或单帧左右目的对应关系三角化特征点获取深度,用于下一次PnP

11. 对每个特征点,计算其在所有观测到了该点的帧上的重投影误差,若大于阈值则删除该点
12. 滑窗,删除最老的特征点和位姿等信息