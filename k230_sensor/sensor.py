
import fcntl
import struct
from _SensorSetting import SensorSetting

QQVGA = [320, 240]
QVGA = [320, 240]
VGA = [640, 480]
FHD = [1920, 1080]
HD = [1280, 720]
FRAME_SIZE_INVAILD = [-1, -1]


gc2093 = SensorSetting(
    i2c_addr=0x37,
    sensor="gc2093",
)
gc2093.register_mode(1920, 1080, 30, 0)
gc2093.register_mode(1920, 1080, 60, 1)
gc2093.register_mode(1280, 960, 90, 2)

ov5647 = SensorSetting(
    i2c_addr=0x36,
    sensor="ov5647",
)
ov5647.register_mode(1920, 1080, 30, 0)

sensor_list = [gc2093, ov5647]
class Sensor:
    sensor: SensorSetting
    def __init__(self, width:int=1920, height:int=1080, fps:int=60) -> None:
        self.fps = fps
        self.sensor = self._scan_sensor()
        self.set_framesize(width=width, height=height)

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
        print("输入的宽高是", self.width, self.height)
        print("传感器模式是", self.mode.mode)

    def set_hmirror(self, hmirror:bool=True):
        pass

    def set_vflip(self, vflip:bool=True):
        pass

    def run(self):
        self.sensor.set_mode(self.mode.mode)
