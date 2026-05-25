'''
实验名称：YOLO11检测
实验平台：核桃派2B
说明：摄像头采集检测
'''

import cv2,time
import k230_display
import walnutpi_imgxfer as imgxfer

import k230_sensor as sensor
# sensor = sensor.Sensor(1920, 1080)
sensor = sensor.Sensor(1280, 960)
sensor.run()


# 打开摄像头
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置长度

k230_display.init()
ret, img = cap.read()


count=0
pt=0
fps = 0
while True:
    #计算帧率
    count+=1    
    if time.time()-pt >=1 : #超过1秒
        fps=1/((time.time()-pt)/count)#计算帧率
        fps = round(fps, 1)
        count=0
        pt=time.time()
        print("FPS:",fps)
    
    # 摄像头读取一帧图像    
    ret, img = cap.read()
    # bgr转rgb
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    cv2.putText(img, 'FPS: '+str(fps), (10,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2) #图像绘制帧率
    k230_display.show(img)
    imgxfer.push_frame(img)

    
cap .release() # 关闭摄像头
