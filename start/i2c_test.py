import smbus
import time

class I2CReader:
    def __init__(self, bus_number=7, device_address=0x55):
        """
        初始化I2C讀取器
        
        參數:
            bus_number: I2C匯流排編號(默認為7)
            device_address: I2C裝置地址(默認為0x55)
        """
        self.bus_number = bus_number
        self.device_address = device_address
        try:
            self.bus = smbus.SMBus(bus_number)
            print(f"成功初始化I2C匯流排 {bus_number}")
        except Exception as e:
            print(f"初始化I2C匯流排失敗: {str(e)}")
            raise
    
    def read_byte(self, register_address):
        """
        讀取單個位元組
        
        參數:
            register_address: 暫存器地址
        返回:
            讀取到的位元組值
        """
        try:
            return self.bus.read_byte_data(self.device_address, register_address)
        except Exception as e:
            print(f"讀取位元組失敗: {str(e)}")
            return None

    def read_word(self, register_address):
        """
        讀取雙位元組(字)
        
        參數:
            register_address: 暫存器地址
        返回:
            讀取到的字值
        """
        try:
            return self.bus.read_word_data(self.device_address, register_address)
        except Exception as e:
            print(f"讀取字值失敗: {str(e)}")
            return None

    def read_remaining_capacity(self):
        """
        讀取電池剩餘容量
        
        返回:
            float: 電池容量(mAh)，發生錯誤時返回None
        """
        try:
            # 讀取兩個位元組的資料
            data = self.bus.read_i2c_block_data(self.device_address, 0x04, 2)
            
            # 組合 MSB 和 LSB
            raw_value = (data[1] << 8) | data[0]
            
            # 轉換成實際容量值 (乘以0.1)
            capacity = raw_value
            
            return capacity
            
        except Exception as e:
            
            print(f"讀取電池容量失敗: {str(e)}")
            return None
        
    def read_state_of_charge(self):
        """
        讀取電池剩餘容量
        
        返回:
            float: 電池容量(mAh)，發生錯誤時返回None
        """
        try:
            # 讀取兩個位元組的資料
            data = self.bus.read_i2c_block_data(self.device_address, 0x06, 2)
            
            # 組合 MSB 和 LSB
            raw_value = (data[1] << 8) | data[0]
            
            # 轉換成實際容量值 (乘以0.1)
            # capacity = raw_value * 0.1
            
            return raw_value
            
        except Exception as e:
            print(f"讀取電池容量失敗: {str(e)}")
            return None

    def read_battery_voltage(self):
        """
        讀取電池電壓
        
        返回:
            float: 電池電壓(V)，發生錯誤時返回None
        """
        try:
            # 讀取兩個位元組的資料
            data = self.bus.read_i2c_block_data(self.device_address, 0x08, 2)
            
            # 組合 MSB 和 LSB
            raw_value = (data[1] << 8) | data[0]
            
            # 轉換成實際電壓值 (乘以0.001)
            voltage = raw_value * 0.001
            
            return voltage
            
        except Exception as e:
            print(f"讀取電池電壓失敗: {str(e)}")
            return None

    def close(self):
        """關閉I2C連接"""
        try:
            self.bus.close()
            print("已關閉I2C連接")
        except Exception as e:
            print(f"關閉I2C連接失敗: {str(e)}")

def main():
    # 使用示例
    try:
        # 創建I2C讀取器實例
        reader = I2CReader()
        
        print("\n開始讀取I2C數據:")
        
        # # 讀取幾個連續的暫存器
        # for register in range(0x00, 0x06):
        #     byte_value = reader.read_byte(register)
        #     if byte_value is not None:
        #         print(f"暫存器 0x{register:02X} 的值: {byte_value}")
            
        #     # 短暫延遲以避免讀取過快
        #     time.sleep(0.1)
        
        # 讀取電池容量
        capacity = reader.read_remaining_capacity()
        if capacity is not None:
            print(f"\n電池剩餘容量: {capacity} mAh")
        
        # 讀取電池電壓
        voltage = reader.read_battery_voltage()
        if voltage is not None:
            print(f"電池電壓: {voltage:.3f} V")

        # 讀取電池電壓
        state = reader.read_state_of_charge()
        if state is not None:
            print(f"電池狀態: {state} V")
        
        # 關閉連接
        reader.close()
        
    except KeyboardInterrupt:
        print("\n程序被用戶中斷")
        if 'reader' in locals():
            reader.close()
    except Exception as e:
        print(f"發生錯誤: {str(e)}")
        if 'reader' in locals():
            reader.close()

if __name__ == "__main__":
    main()