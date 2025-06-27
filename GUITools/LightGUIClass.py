"""
LED控制器GUI類別
提供統一的LED控制介面，作為主GUI與ModbusTCP Server的橋接層

技術規範:
- 基地址: 600-649
- 狀態寄存器: 600-615 (只讀)
- 指令寄存器: 620-624 (讀寫)
- 通道數量: 4 (L1-L4)
- 亮度範圍: 0-511
"""

import time
import threading
from typing import Dict, Any, Optional, Callable
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import logging


class LightGUIClass:
    """LED控制器GUI類別"""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 502, 
                 unit_id: int = 1, timeout: float = 3.0):
        """
        初始化LED控制器GUI類別
        
        Args:
            host: ModbusTCP Server主機位址
            port: ModbusTCP Server端口
            unit_id: Modbus單元ID
            timeout: 連接逾時時間
        """
        # 連接參數
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        
        # ModbusTCP Client
        self.client = None
        self.connected = False
        
        # LED控制器寄存器基地址
        self.base_address = 600
        
        # 狀態數據
        self.status_data = {
            'module_status': 0,      # 模組狀態 (0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤)
            'device_connection': 0,  # 設備連接狀態 (0=斷開, 1=已連接)
            'active_channels': 0,    # 開啟通道數量
            'error_code': 0,         # 錯誤代碼
            'l1_state': 0,          # L1狀態 (0=OFF, 1=ON)
            'l2_state': 0,          # L2狀態
            'l3_state': 0,          # L3狀態
            'l4_state': 0,          # L4狀態
            'l1_brightness': 0,     # L1亮度 (0-511)
            'l2_brightness': 0,     # L2亮度
            'l3_brightness': 0,     # L3亮度
            'l4_brightness': 0,     # L4亮度
            'operation_count': 0,   # 操作計數
            'error_count': 0,       # 錯誤計數
            'timestamp': 0          # 時間戳
        }
        
        # 亮度範圍定義
        self.brightness_min = 0
        self.brightness_max = 511
        
        # 通道定義
        self.channels = [1, 2, 3, 4]
        
        # 狀態更新回調函數
        self.status_callbacks = []
        
        # 自動更新控制
        self.auto_update = False
        self.update_interval = 1.0  # 更新間隔 (秒)
        self.update_thread = None
        self.running = False
        
        # 指令ID計數器
        self.command_id_counter = 1
        
        # 日誌設定
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """連接到ModbusTCP Server"""
        try:
            if self.client:
                self.disconnect()
            
            self.client = ModbusTcpClient(
                host=self.host,
                port=self.port,
                timeout=self.timeout
            )
            
            result = self.client.connect()
            if result:
                self.connected = True
                self.logger.info(f"已連接到LED控制器模組 {self.host}:{self.port}")
                return True
            else:
                self.logger.error("連接LED控制器模組失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        try:
            if self.client and self.connected:
                self.client.close()
            self.connected = False
            self.logger.info("已斷開LED控制器模組連接")
        except Exception as e:
            self.logger.error(f"斷開連接異常: {e}")
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self.connected and self.client is not None
    
    def read_status_registers(self) -> bool:
        """讀取所有狀態寄存器"""
        if not self.is_connected():
            return False
        
        try:
            # PyModbus 3.x 正確的參數傳遞方式
            result = self.client.read_holding_registers(
                self.base_address,  # address
                16,                 # count
                slave=self.unit_id  # 使用slave參數而非unit
            )
            
            if result.isError():
                self.logger.error(f"讀取狀態寄存器失敗: {result}")
                return False
            
            # 更新狀態數據
            registers = result.registers
            self.status_data.update({
                'module_status': registers[0],
                'device_connection': registers[1],
                'active_channels': registers[2],
                'error_code': registers[3],
                'l1_state': registers[4],
                'l2_state': registers[5],
                'l3_state': registers[6],
                'l4_state': registers[7],
                'l1_brightness': registers[8],
                'l2_brightness': registers[9],
                'l3_brightness': registers[10],
                'l4_brightness': registers[11],
                'operation_count': registers[12],
                'error_count': registers[13],
                'timestamp': registers[15]
            })
            
            return True
            
        except Exception as e:
            self.logger.error(f"讀取狀態寄存器異常: {e}")
            return False
    
    def write_command_register(self, command: int, param1: int = 0, 
                              param2: int = 0) -> bool:
        """寫入指令寄存器"""
        if not self.is_connected():
            return False
        
        try:
            # 產生唯一指令ID
            command_id = self.command_id_counter
            self.command_id_counter = (self.command_id_counter + 1) % 65536
            
            # PyModbus 3.x 正確的參數傳遞方式
            values = [command, param1, param2, command_id, 0]
            result = self.client.write_registers(
                self.base_address + 20,  # address
                values,                  # values
                slave=self.unit_id       # 使用slave參數而非unit
            )
            
            if result.isError():
                self.logger.error(f"寫入指令寄存器失敗: {result}")
                return False
            
            self.logger.debug(f"執行指令: cmd={command}, p1={param1}, p2={param2}, id={command_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"寫入指令寄存器異常: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """獲取當前狀態"""
        return self.status_data.copy()
    
    def get_module_status_text(self) -> str:
        """獲取模組狀態文字描述"""
        status_map = {
            0: "離線",
            1: "閒置", 
            2: "執行中",
            3: "初始化",
            4: "錯誤"
        }
        return status_map.get(self.status_data['module_status'], "未知")
    
    def get_channel_states(self) -> Dict[int, bool]:
        """獲取所有通道開關狀態"""
        return {
            1: bool(self.status_data['l1_state']),
            2: bool(self.status_data['l2_state']),
            3: bool(self.status_data['l3_state']),
            4: bool(self.status_data['l4_state'])
        }
    
    def get_channel_brightness(self) -> Dict[int, int]:
        """獲取所有通道亮度"""
        return {
            1: self.status_data['l1_brightness'],
            2: self.status_data['l2_brightness'],
            3: self.status_data['l3_brightness'],
            4: self.status_data['l4_brightness']
        }
    
    def is_device_connected(self) -> bool:
        """檢查LED設備連接狀態"""
        return bool(self.status_data['device_connection'])
    
    def has_error(self) -> bool:
        """檢查是否有錯誤"""
        return self.status_data['error_code'] != 0
    
    def get_error_code(self) -> int:
        """獲取錯誤代碼"""
        return self.status_data['error_code']
    
    # LED控制命令
    def turn_on_all(self) -> bool:
        """全部開啟 (指令1)"""
        return self.write_command_register(1)
    
    def turn_off_all(self) -> bool:
        """全部關閉 (指令2)"""
        return self.write_command_register(2)
    
    def reset_device(self) -> bool:
        """重置設備 (指令3)"""
        return self.write_command_register(3)
    
    def set_channel_brightness(self, channel: int, brightness: int) -> bool:
        """設定通道亮度 (指令4)"""
        if channel not in self.channels:
            self.logger.error(f"無效通道號: {channel}")
            return False
        
        if not (self.brightness_min <= brightness <= self.brightness_max):
            self.logger.error(f"亮度值超出範圍: {brightness} (範圍: {self.brightness_min}-{self.brightness_max})")
            return False
        
        return self.write_command_register(4, channel, brightness)
    
    def turn_on_channel(self, channel: int) -> bool:
        """開啟通道 (指令5)"""
        if channel not in self.channels:
            self.logger.error(f"無效通道號: {channel}")
            return False
        
        return self.write_command_register(5, channel)
    
    def turn_off_channel(self, channel: int) -> bool:
        """關閉通道 (指令6)"""
        if channel not in self.channels:
            self.logger.error(f"無效通道號: {channel}")
            return False
        
        return self.write_command_register(6, channel)
    
    def clear_error(self) -> bool:
        """錯誤重置 (指令7)"""
        return self.write_command_register(7)
    
    # 批量操作
    def set_all_brightness(self, brightness: int) -> bool:
        """設定所有通道亮度"""
        if not (self.brightness_min <= brightness <= self.brightness_max):
            self.logger.error(f"亮度值超出範圍: {brightness}")
            return False
        
        success = True
        for channel in self.channels:
            if not self.set_channel_brightness(channel, brightness):
                success = False
                
        return success
    
    def set_multiple_brightness(self, brightness_dict: Dict[int, int]) -> bool:
        """設定多個通道亮度"""
        success = True
        for channel, brightness in brightness_dict.items():
            if not self.set_channel_brightness(channel, brightness):
                success = False
                
        return success
    
    # 狀態更新機制
    def add_status_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """新增狀態更新回調函數"""
        if callback not in self.status_callbacks:
            self.status_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """移除狀態更新回調函數"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def notify_status_update(self):
        """通知狀態更新"""
        for callback in self.status_callbacks:
            try:
                callback(self.status_data.copy())
            except Exception as e:
                self.logger.error(f"回調函數執行異常: {e}")
    
    def start_auto_update(self, interval: float = 1.0):
        """開始自動狀態更新"""
        if self.auto_update:
            return
        
        self.update_interval = interval
        self.auto_update = True
        self.running = True
        
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        
        self.logger.info(f"開始自動狀態更新，間隔: {interval}秒")
    
    def stop_auto_update(self):
        """停止自動狀態更新"""
        self.auto_update = False
        self.running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        
        self.logger.info("停止自動狀態更新")
    
    def _update_loop(self):
        """狀態更新循環"""
        while self.running and self.auto_update:
            try:
                if self.is_connected():
                    if self.read_status_registers():
                        self.notify_status_update()
                    
            except Exception as e:
                self.logger.error(f"狀態更新異常: {e}")
            
            time.sleep(self.update_interval)
    
    def manual_update(self) -> bool:
        """手動更新狀態"""
        if self.read_status_registers():
            self.notify_status_update()
            return True
        return False
    
    # 資源管理
    def cleanup(self):
        """清理資源"""
        self.stop_auto_update()
        self.disconnect()
        self.status_callbacks.clear()
        self.logger.info("LED控制器GUI類別資源清理完成")
    
    def __enter__(self):
        """上下文管理器進入"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.cleanup()


# 使用範例
if __name__ == "__main__":
    def status_update_handler(status_data):
        """狀態更新處理函數範例"""
        print(f"模組狀態: {status_data['module_status']}")
        print(f"設備連接: {status_data['device_connection']}")
        print(f"L1亮度: {status_data['l1_brightness']}")
    
    # 使用上下文管理器
    with LightGUIClass() as light_gui:
        if light_gui.connect():
            # 註冊狀態回調
            light_gui.add_status_callback(status_update_handler)
            
            # 開始自動更新
            light_gui.start_auto_update(2.0)
            
            # 控制LED
            light_gui.set_channel_brightness(1, 255)
            light_gui.turn_on_channel(2)
            
            time.sleep(5)
        else:
            print("連接失敗")