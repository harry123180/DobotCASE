# -*- coding: utf-8 -*-
"""
VPGUIClass.py - VP震動盤模組GUI控制類別
實現統一機台調適工具的VP震動盤模組控制介面
基於Modbus TCP Client架構，支援完整的震動盤控制功能
"""

import time
import threading
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from pymodbus.client import ModbusTcpClient


@dataclass
class VPStatus:
    """VP震動盤模組狀態"""
    module_status: int = 0          # 0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤
    device_connected: bool = False  # 設備連接狀態
    device_status: bool = False     # 設備狀態 0=OFF, 1=ON
    error_code: int = 0             # 錯誤代碼
    executing_command: bool = False # 指令執行中
    brightness_status: int = 0      # 背光亮度 0-255
    backlight_status: bool = False  # 背光開關
    vibration_status: bool = False  # 震動狀態
    frequency_status: int = 0       # 頻率狀態
    current_action: int = 0         # 當前動作編碼 (32位合併)
    target_action: int = 0          # 目標動作編碼 (32位合併)
    comm_error_count: int = 0       # 通訊錯誤計數
    connected_to_server: bool = False  # Modbus連接狀態


@dataclass
class VPOperationResult:
    """VP操作結果"""
    success: bool = False           # 操作成功標誌
    error_message: str = ""         # 錯誤訊息
    timestamp: str = ""             # 操作時間戳
    operation_type: str = ""        # 操作類型


@dataclass
class VPActionParams:
    """VP動作參數"""
    action: str = "stop"            # 動作名稱
    strength: int = 100             # 強度 0-255
    frequency: int = 100            # 頻率 0-255
    duration: float = 0.0           # 持續時間 (秒, 0表示持續)


class VPGUIClass:
    """VP震動盤GUI控制類別"""
    
    # VP模組基地址
    BASE_ADDRESS = 300
    
    # 動作映射
    ACTION_MAP = {
        'stop': 0, 'up': 1, 'down': 2, 'left': 3, 'right': 4,
        'upleft': 5, 'downleft': 6, 'upright': 7, 'downright': 8,
        'horizontal': 9, 'vertical': 10, 'spread': 11
    }
    
    # 指令碼映射
    COMMAND_MAP = {
        'nop': 0,               # 無操作
        'enable_device': 1,     # 設備啟用 (背光開啟)
        'disable_device': 2,    # 設備停用 (背光關閉)
        'stop_all': 3,          # 停止所有動作
        'set_brightness': 4,    # 設定背光亮度
        'execute_action': 5,    # 執行動作
        'emergency_stop': 6,    # 緊急停止
        'reset_error': 7,       # 錯誤重置
        'set_action_params': 11,    # 設定動作參數
        'toggle_backlight': 12,     # 背光切換
        'execute_with_params': 13,  # 執行動作並設定參數
        'set_frequency': 14,        # 設定頻率
    }
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502, unit_id: int = 1):
        """初始化VP控制器"""
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.unit_id = unit_id
        
        # Modbus TCP Client
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 狀態管理
        self.current_status = VPStatus()
        self.last_command_id = 0
        
        # 監控線程
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self.monitor_lock = threading.Lock()
        
        # 回調函數列表
        self.status_callbacks: List[Callable[[VPStatus], None]] = []
        self.result_callbacks: List[Callable[[VPOperationResult], None]] = []
        
        # 寄存器映射
        self.init_register_mapping()
    
    def init_register_mapping(self):
        """初始化寄存器映射"""
        base = self.BASE_ADDRESS
        
        # 狀態寄存器 (只讀 300-314)
        self.status_registers = {
            'module_status': base + 0,          # 模組狀態
            'device_connection': base + 1,      # 設備連接狀態
            'device_status': base + 2,          # 設備狀態
            'error_code': base + 3,             # 錯誤代碼
            'current_action_low': base + 4,     # 當前動作低位
            'current_action_high': base + 5,    # 當前動作高位
            'target_action_low': base + 6,      # 目標動作低位
            'target_action_high': base + 7,     # 目標動作高位
            'command_status': base + 8,         # 指令執行狀態
            'comm_error_count': base + 9,       # 通訊錯誤計數
            'brightness_status': base + 10,     # 背光亮度狀態
            'backlight_status': base + 11,      # 背光開關狀態
            'vibration_status': base + 12,      # 震動狀態
            'frequency_status': base + 13,      # 頻率狀態
            'timestamp': base + 14              # 時間戳
        }
        
        # 指令寄存器 (讀寫 320-324)
        self.command_registers = {
            'command_code': base + 20,          # 指令代碼 (320)
            'param1': base + 21,                # 參數1 (321)
            'param2': base + 22,                # 參數2 (322)
            'param3': base + 23,                # 參數3 (323)
            'command_id': base + 24,            # 指令ID (324)
        }
        
        # 所有寄存器
        self.all_registers = {**self.status_registers, **self.command_registers}
    
    def connect(self) -> bool:
        """連接到Modbus服務器"""
        try:
            if self.modbus_client:
                self.disconnect()
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.start_monitoring()
                return True
            else:
                self.connected = False
                return False
        
        except Exception as e:
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開連接"""
        self.stop_monitoring()
        
        if self.modbus_client:
            try:
                self.modbus_client.close()
            except:
                pass
        
        self.connected = False
        self.modbus_client = None
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self.connected and self.modbus_client is not None
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if not self.monitoring_active:
            self.monitoring_active = True
            self.monitoring_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitoring_thread.start()
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring_active = False
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=1.0)
    
    def _monitor_loop(self):
        """監控線程主循環"""
        while self.monitoring_active:
            try:
                if self.is_connected():
                    old_status = VPStatus(**self.current_status.__dict__)
                    self._update_status()
                    
                    # 檢查狀態變化
                    if self._status_changed(old_status, self.current_status):
                        self._notify_status_callbacks(self.current_status)
                
                time.sleep(0.2)  # 200ms輪詢頻率
                
            except Exception as e:
                self.connected = False
                time.sleep(1.0)
    
    def _update_status(self):
        """更新狀態資訊"""
        with self.monitor_lock:
            try:
                # 讀取狀態寄存器
                result = self.modbus_client.read_holding_registers(
                    address=self.status_registers['module_status'], 
                    count=15, 
                    unit=self.unit_id
                )
                
                if not result.isError():
                    registers = result.registers
                    
                    # 更新狀態
                    self.current_status.module_status = registers[0]
                    self.current_status.device_connected = bool(registers[1])
                    self.current_status.device_status = bool(registers[2])
                    self.current_status.error_code = registers[3]
                    
                    # 32位動作編碼合併
                    current_low = registers[4]
                    current_high = registers[5]
                    self.current_status.current_action = (current_high << 16) | current_low
                    
                    target_low = registers[6]
                    target_high = registers[7]
                    self.current_status.target_action = (target_high << 16) | target_low
                    
                    self.current_status.executing_command = bool(registers[8])
                    self.current_status.comm_error_count = registers[9]
                    self.current_status.brightness_status = registers[10]
                    self.current_status.backlight_status = bool(registers[11])
                    self.current_status.vibration_status = bool(registers[12])
                    self.current_status.frequency_status = registers[13]
                    
                    self.current_status.connected_to_server = True
                else:
                    self.current_status.connected_to_server = False
                    
            except Exception as e:
                self.current_status.connected_to_server = False
    
    def _status_changed(self, old_status: VPStatus, new_status: VPStatus) -> bool:
        """檢查狀態是否變化"""
        return (old_status.module_status != new_status.module_status or
                old_status.device_connected != new_status.device_connected or
                old_status.executing_command != new_status.executing_command or
                old_status.vibration_status != new_status.vibration_status or
                old_status.backlight_status != new_status.backlight_status or
                old_status.connected_to_server != new_status.connected_to_server)
    
    def get_status(self) -> VPStatus:
        """獲取當前狀態"""
        with self.monitor_lock:
            return VPStatus(**self.current_status.__dict__)
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取單一寄存器"""
        if not self.is_connected() or register_name not in self.all_registers:
            return None
        
        try:
            address = self.all_registers[register_name]
            result = self.modbus_client.read_holding_registers(
                address=address, count=1, unit=self.unit_id
            )
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入單一寄存器"""
        if not self.is_connected() or register_name not in self.all_registers:
            return False
        
        try:
            address = self.all_registers[register_name]
            result = self.modbus_client.write_register(
                address=address, value=value, unit=self.unit_id
            )
            
            return not result.isError()
                
        except Exception as e:
            return False
    
    def _send_command(self, command_code: int, param1: int = 0, param2: int = 0, param3: int = 0) -> bool:
        """發送指令到VP模組"""
        if not self.is_connected():
            return False
        
        try:
            # 生成新的指令ID
            self.last_command_id = (self.last_command_id + 1) % 65536
            
            # 寫入指令寄存器
            success = True
            success &= self.write_register('command_code', command_code)
            success &= self.write_register('param1', param1)
            success &= self.write_register('param2', param2)
            success &= self.write_register('param3', param3)
            success &= self.write_register('command_id', self.last_command_id)
            
            if success:
                # 通知結果回調
                result = VPOperationResult(
                    success=True,
                    operation_type=f"command_{command_code}",
                    timestamp=time.strftime("%H:%M:%S")
                )
                self._notify_result_callbacks(result)
            
            return success
            
        except Exception as e:
            result = VPOperationResult(
                success=False,
                error_message=str(e),
                operation_type=f"command_{command_code}",
                timestamp=time.strftime("%H:%M:%S")
            )
            self._notify_result_callbacks(result)
            return False
    
    # 基本控制操作
    def enable_device(self) -> bool:
        """啟用設備 (開啟背光)"""
        return self._send_command(self.COMMAND_MAP['enable_device'])
    
    def disable_device(self) -> bool:
        """停用設備 (關閉背光)"""
        return self._send_command(self.COMMAND_MAP['disable_device'])
    
    def stop_all_actions(self) -> bool:
        """停止所有動作"""
        return self._send_command(self.COMMAND_MAP['stop_all'])
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        return self._send_command(self.COMMAND_MAP['emergency_stop'])
    
    def reset_error(self) -> bool:
        """錯誤重置"""
        return self._send_command(self.COMMAND_MAP['reset_error'])
    
    # 背光控制
    def set_brightness(self, brightness: int) -> bool:
        """設定背光亮度 (0-255)"""
        if 0 <= brightness <= 255:
            return self._send_command(self.COMMAND_MAP['set_brightness'], brightness)
        return False
    
    def toggle_backlight(self, state: bool) -> bool:
        """切換背光開關"""
        return self._send_command(self.COMMAND_MAP['toggle_backlight'], 1 if state else 0)
    
    # 震動控制
    def execute_action(self, action: str, strength: int = 100, frequency: int = 100) -> bool:
        """執行震動動作"""
        if action not in self.ACTION_MAP:
            return False
        
        if not (0 <= strength <= 255 and 0 <= frequency <= 255):
            return False
        
        action_code = self.ACTION_MAP[action]
        return self._send_command(self.COMMAND_MAP['execute_action'], action_code, strength, frequency)
    
    def execute_action_with_params(self, params: VPActionParams) -> bool:
        """使用參數結構執行動作"""
        return self.execute_action(params.action, params.strength, params.frequency)
    
    def set_action_parameters(self, action: str, strength: int, frequency: int) -> bool:
        """設定動作參數 (不立即執行)"""
        if action not in self.ACTION_MAP:
            return False
        
        action_code = self.ACTION_MAP[action]
        return self._send_command(self.COMMAND_MAP['set_action_params'], action_code, strength, frequency)
    
    def set_frequency(self, frequency: int) -> bool:
        """設定全域頻率"""
        if 0 <= frequency <= 255:
            return self._send_command(self.COMMAND_MAP['set_frequency'], frequency)
        return False
    
    # 便捷操作方法
    def quick_action(self, action: str) -> bool:
        """快速執行動作 (使用預設強度和頻率)"""
        return self.execute_action(action, 150, 120)
    
    def get_available_actions(self) -> List[str]:
        """獲取可用動作列表"""
        return list(self.ACTION_MAP.keys())
    
    def get_action_code(self, action: str) -> Optional[int]:
        """獲取動作編碼"""
        return self.ACTION_MAP.get(action)
    
    def get_status_description(self) -> Dict[str, str]:
        """獲取狀態描述"""
        status = self.get_status()
        
        module_status_desc = {
            0: "離線", 1: "閒置", 2: "執行中", 3: "初始化", 4: "錯誤"
        }
        
        return {
            "模組狀態": module_status_desc.get(status.module_status, "未知"),
            "設備連接": "已連接" if status.device_connected else "斷開",
            "設備狀態": "開啟" if status.device_status else "關閉",
            "背光狀態": "開啟" if status.backlight_status else "關閉",
            "震動狀態": "運行中" if status.vibration_status else "停止",
            "指令執行": "執行中" if status.executing_command else "空閒",
            "錯誤代碼": str(status.error_code) if status.error_code > 0 else "無錯誤",
            "背光亮度": str(status.brightness_status),
            "頻率": str(status.frequency_status),
            "通訊錯誤次數": str(status.comm_error_count)
        }
    
    # 回調管理
    def add_status_callback(self, callback: Callable[[VPStatus], None]):
        """新增狀態變化回調函數"""
        if callback not in self.status_callbacks:
            self.status_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[VPStatus], None]):
        """移除狀態變化回調函數"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def add_result_callback(self, callback: Callable[[VPOperationResult], None]):
        """新增操作結果回調函數"""
        if callback not in self.result_callbacks:
            self.result_callbacks.append(callback)
    
    def remove_result_callback(self, callback: Callable[[VPOperationResult], None]):
        """移除操作結果回調函數"""
        if callback in self.result_callbacks:
            self.result_callbacks.remove(callback)
    
    def _notify_status_callbacks(self, status: VPStatus):
        """通知狀態變化回調函數"""
        for callback in self.status_callbacks:
            try:
                callback(status)
            except Exception as e:
                pass  # 靜默處理回調異常
    
    def _notify_result_callbacks(self, result: VPOperationResult):
        """通知操作結果回調函數"""
        for callback in self.result_callbacks:
            try:
                callback(result)
            except Exception as e:
                pass  # 靜默處理回調異常
    
    # 進階功能
    def wait_for_idle(self, timeout: float = 10.0) -> bool:
        """等待系統進入閒置狀態"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if not status.executing_command and status.module_status == 1:
                return True
            time.sleep(0.1)
        return False
    
    def execute_action_sequence(self, actions: List[VPActionParams], interval: float = 1.0) -> bool:
        """執行動作序列"""
        try:
            for i, action_params in enumerate(actions):
                if not self.wait_for_idle(5.0):
                    return False
                
                if not self.execute_action_with_params(action_params):
                    return False
                
                if i < len(actions) - 1:  # 最後一個動作不需要等待
                    time.sleep(interval)
            
            return True
            
        except Exception as e:
            return False
    
    def get_connection_info(self) -> Dict[str, Any]:
        """獲取連接資訊"""
        return {
            "modbus_host": self.modbus_host,
            "modbus_port": self.modbus_port,
            "unit_id": self.unit_id,
            "connected": self.connected,
            "base_address": self.BASE_ADDRESS,
            "monitoring_active": self.monitoring_active,
            "last_command_id": self.last_command_id
        }