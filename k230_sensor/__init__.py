
import cv2
from ._SensorSetting import SensorSetting, SensorMode

QQVGA = [320, 240]
QVGA = [320, 240]
VGA = [640, 480]
FHD = [1920, 1080]
HD = [1280, 720]
FRAME_SIZE_INVAILD = [-1, -1]


gc2093 = SensorSetting(
    i2c_addr=0x37,
    sensor="gc2093",
    mode=[
        # SensorMode(1920, 1080, 30, 0),
        SensorMode(1920, 1080, 60, 1),
    ]
)

ov5647 = SensorSetting(
    i2c_addr=0x36,
    sensor="ov5647",
    mode=[
        SensorMode(1920, 1080, 30, 0)
    ]
)

sensor_list = [gc2093, ov5647]


class Sensor(cv2.VideoCapture):
    sensor: SensorSetting
    def __init__(self, index: int, width:int=1920, height:int=1080, fps:int=60) -> None:
        self.fps = fps
        self.sensor = self._scan_sensor()
        super().__init__(index)
        self.set_framesize(width=width, height=height)


    def read(self):
        ret, img = super().read()
        if self.mode.width != self.width or self.mode.height != self.height:
            img = img[0:self.height, 0:self.width]
            img = cv2.resize(img, (self.width, self.height))
        # bgr转rgb，因为cv2默认是bgr格式，而isp默认返回rgb格式
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        return ret, img
    def _scan_sensor(self, i2c_bus: int = 0) -> SensorSetting:
        '''
        扫描I2C总线，检测哪个传感器存在
        '''
        for setting in sensor_list:
            if setting.check_i2c(i2c_bus):
                return setting
        
        # 如果未检测到任何传感器，返回gc2093
        return gc2093

    def set_framesize(self, width:int=1920, height:int=1080):
        self.width = width
        self.height = height
        sensor_mode = self.sensor.get_mode(width, height, self.fps)
        ratio_wh = sensor_mode.width / sensor_mode.height
        w_in_h_same = int(self.height * ratio_wh) + 1
        h_in_w_same = int(self.width / ratio_wh) + 1
        if h_in_w_same > height:
            the_nearest_width = width
            the_nearest_height = h_in_w_same
        elif w_in_h_same > width:
            the_nearest_width = w_in_h_same
            the_nearest_height = height
        else:
            the_nearest_width = sensor_mode.width
            the_nearest_height = sensor_mode.height
        self.set(cv2.CAP_PROP_FRAME_WIDTH, the_nearest_width)  # 设置宽度
        self.set(cv2.CAP_PROP_FRAME_HEIGHT, the_nearest_height)  # 设置长度
        self.mode = self.sensor.get_mode(width, height, self.fps)
        self.sensor.set_mode(self.mode)

    def set_hmirror(self, hmirror:bool=True):
        pass

    def set_vflip(self, vflip:bool=True):
        pass


