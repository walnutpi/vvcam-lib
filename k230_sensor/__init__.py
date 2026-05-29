
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
        SensorMode(1920, 1080, 30, 0),
        SensorMode(1920, 1080, 60, 1),
        SensorMode(1280, 960, 90, 2)
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
        self.set_framesize(width=width, height=height)
        self.sensor.set_mode(self.mode.mode)

        super().__init__(index)
        self.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # 设置宽度
        self.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # 设置长度

    def read(self):
        ret, img = super().read()
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

    def set_framesize(self, framesize = FRAME_SIZE_INVAILD, width:int=1920, height:int=1080):
        self.width = width
        self.height = height
        self.mode = self.sensor.get_mode(width, height, self.fps)

    def set_hmirror(self, hmirror:bool=True):
        pass

    def set_vflip(self, vflip:bool=True):
        pass


