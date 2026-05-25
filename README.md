# vvcam-lib
用于k230的mipi摄像头&isp的用户空间所需程序。需要配合vvcam的内核驱动使用。

安装
```bash
sudo ./install.sh
```
开机时需要先运行 run_isp_media.sh 脚本

## python库
摄像头会存在几种分辨率设置，会影响帧率和画面形变，提供了这个python库来切换摄像头配置

```python
import k230_sensor
sensor = k230_sensor.Sensor(width=1920, height=1080, fps=60)
sensor.run()
```
