from machine import I2C,Pin
import utime as time
import ustruct

class jy61p_i2c():
    i2c_dev = None
    i2c_addr = None
#     i2c = I2C(1, scl=Pin(3), sda=Pin(2))

    def write_data(self, reg, data):
        self.i2c_dev.writeto_mem(self.i2c_addr,reg,bytearray(data), len(data))
        pass

    def read_data(self, reg, length):
        return self.i2c_dev.readfrom_mem(self.i2c_addr,reg,length)

    def init(self, i2c=0, addr=0x50):
        self.i2c_dev = I2C(1, scl=Pin(3), sda=Pin(2))  # 返回i2c对象
        self.i2c_addr = addr
        pass

    def sample_read(self):
        r_data = self.read_data(0x34, 24)        
        
        #print("data is {0}".format(list(r_data)))
        val = ustruct.unpack("hhhhhhhhhhhh", bytearray(r_data))
        msg  = "acc:%.3f,%.3f,%.3f\r\n"%(val[0]/32768.0*16.0, val[1]/32768.0*16.0, val[2]/32768.0*16.0)
        msg += "gyro:%.2f,%.2f,%.2f\r\n"%(val[3]/32768.0*2000.0, val[4]/32768.0*2000.0, val[5]/32768.0*2000.0)
        msg += "mag:%d,%d,%d\r\n"%(val[6], val[7], val[8])
        msg += "angle:%.2f,%.2f,%.2f\r\n"%(val[9]/32768.0*180.0, val[10]/32768.0*180.0, val[11]/32768.0*180.0)
        print(msg)
        
    def read_ang(self):
        r_data = self.read_data(0x34, 24)        
        val = ustruct.unpack("hhhhhhhhhhhh", bytearray(r_data))
        return [val[9]/32768.0*180.0, val[10]/32768.0*180.0, val[11]/32768.0*180.0]
    



# def jy901_i2c_test():
#     jy901_dev = jy61p_i2c()
#     jy901_dev.init()
# 
#     # 测试十次
#     for i in range(50):
#         jy901_dev.sample_read()
#         time.sleep_ms(200)
# 
# 
# if __name__ == "__main__":
#     jy901_dev = jy61p_i2c(I2C(1, scl=Pin(3), sda=Pin(2)))
#     jy901_dev.init()
#     while True:
#         print(float(jy901_dev.read_ang()[2]))
#         time.sleep_ms(200)

