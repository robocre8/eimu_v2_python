import serial
import struct

START_BYTE = 0xBB
READ_QUAT = 0x01
READ_RPY = 0x02
READ_RPY_VAR = 0x03
READ_ACC = 0x05
READ_ACC_VAR = 0x09
READ_GYRO = 0x0B
READ_GYRO_VAR = 0x0F
READ_MAG = 0x11
GET_FILTER_GAIN = 0x1E
SET_FRAME_ID = 0x1F
GET_FRAME_ID = 0x20
READ_QUAT_RPY = 0x22
READ_ACC_GYRO = 0x23
CLEAR_DATA_BUFFER = 0x27

class EIMU_V2:
    def __init__(self, port, baud=921600, timeOut=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeOut)
    
    #------------------------------------------------------------------------
    def send_packet_without_payload(self, cmd):
        length = 0
        packet = bytearray([START_BYTE, cmd, length])
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def send_packet_with_payload(self, cmd, payload_bytes):
        length = len(payload_bytes)
        packet = bytearray([START_BYTE, cmd, length]) + payload_bytes
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def read_packet1(self):
        payload = self.ser.read(4)
        a = struct.unpack('<f', payload)[0]  # little-endian float
        return a
    
    def read_packet3(self):
        payload = self.ser.read(12)
        a, b, c = struct.unpack('<fff', payload)  # little-endian float
        return a, b, c

    def read_packet4(self):
        payload = self.ser.read(16)
        a, b, c, d = struct.unpack('<ffff', payload)  # little-endian float
        return a, b, c, d
    
    def read_packet6(self):
        payload = self.ser.read(24)
        a, b, c, d, e, f = struct.unpack('<ffffff', payload)  # little-endian float
        return a, b, c, d, e, f
    
    def read_packet8(self):
        payload = self.ser.read(32)
        a, b, c, d, e, f, g, h = struct.unpack('<ffffffff', payload)  # little-endian float
        return a, b, c, d, e, f, g, h
    
    #---------------------------------------------------------------------

    def write_data1(self, cmd, pos, val):
        payload = struct.pack('<Bf', pos, val)
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data1(self, cmd, pos):
        payload = struct.pack('<Bf', pos, 0.0)  # big-endian
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val
    
    def write_data3(self, cmd, a, b, c):
        payload = struct.pack('<fff', a,b,c) 
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data3(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c = self.read_packet3()
        return a, b, c

    def write_data4(self, cmd, a, b, c, d):
        payload = struct.pack('<ffff', a,b,c,d) 
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data4(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d = self.read_packet4()
        return a, b, c, d
    
    def read_data6(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d, e, f = self.read_packet6()
        return a, b, c, d, e, f
    
    def read_data8(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d, e, f, g, h = self.read_packet8()
        return a, b, c, d, e, f, g, h
        
    #---------------------------------------------------------------------

    def clearDataBuffer(self):
        res = self.write_data1(CLEAR_DATA_BUFFER, 0, 0.0)
        return int(res)
    
    def setWorldFrameId(self, id):
        res = self.write_data1(SET_FRAME_ID, 0, id)
        return int(res)
    
    def getWorldFrameId(self):
        id = self.read_data1(GET_FRAME_ID, 0)
        return int(id)
    
    def getFilterGain(self):
        gain = self.read_data1(GET_FILTER_GAIN, 0)
        return round(gain,6)
    
    def readQuat(self):
        qw, qx, qy, qz = self.read_data4(READ_QUAT)
        return round(qw,6), round(qx,6), round(qy,6), round(qz,6)
    
    def readRPY(self):
        r, p, y = self.read_data3(READ_RPY)
        return round(r,6), round(p,6), round(y,6)
    
    def readRPYVariance(self):
        r, p, y = self.read_data3(READ_RPY_VAR)
        return round(r,6), round(p,6), round(y,6)
    
    def readAcc(self):
        ax, ay, az = self.read_data3(READ_ACC)
        return round(ax,6), round(ay,6), round(az,6)
    
    def readAccVariance(self):
        ax, ay, az = self.read_data3(READ_ACC_VAR)
        return round(ax,6), round(ay,6), round(az,6)
    
    def readGyro(self):
        gx, gy, gz = self.read_data3(READ_GYRO)
        return round(gx,6), round(gy,6), round(gz,6)
    
    def readGyroVariance(self):
        gx, gy, gz = self.read_data3(READ_GYRO_VAR)
        return round(gx,6), round(gy,6), round(gz,6)
    
    def readMag(self):
        mx, my, mz = self.read_data3(READ_MAG)
        return round(mx,6), round(my,6), round(mz,6)
    
    #---------------------------------------------------------------------

    def readQuatRPY(self):
        qw, qx, qy, qz, r, p, y, _ = self.read_data8(READ_QUAT_RPY)
        return round(qw,6), round(qx,6), round(qy,6), round(qz,6), round(r,6), round(p,6), round(y,6)
    
    def readAccGyro(self):
        ax, ay, az, gx, gy, gz = self.read_data6(READ_ACC_GYRO)
        return round(ax,6), round(ay,6), round(az,6), round(gx,6), round(gy,6), round(gz,6)