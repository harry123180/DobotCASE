#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_main.py - 機械臂協調控制模組 (分離式設計)
基地址：1200-1299
專注負責：機械臂Flow1/Flow5協調 + AutoFeeding模組交握
"""

import time
import os
import json
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_EXECUTING = 2
    FLOW5_EXECUTING = 3
    ERROR = 4


@dataclass
class AutoFeedingStatus:
    """AutoFeeding模組狀態"""
    module_status: int = 0          # 900: 模組狀態
    feeding_complete: int = 0       # 940: 入料完成標誌
    target_x: float = 0.0          # 941-942: 目標座標X
    target_y: float = 0.0          # 943-944: 目標座標Y
    total_detections: int = 0      # 946: 檢測結果總數
    case_f_count: int = 0          # 947: CASE_F總數


class AutoProgramController:
    """機械臂協調控制模組 (分離式設計)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址配置
        self.BASE_ADDRESS = 1300
        
        # AutoFeeding模組地址
        self.AF_BASE = 900
        self.AF_MODULE_STATUS = 900        # AutoFeeding模組狀態
        self.AF_FEEDING_COMPLETE = 940     # 入料完成標誌
        self.AF_TARGET_X_HIGH = 941        # 目標座標X高位
        self.AF_TARGET_X_LOW = 942         # 目標座標X低位
        self.AF_TARGET_Y_HIGH = 943        # 目標座標Y高位
        self.AF_TARGET_Y_LOW = 944         # 目標座標Y低位
        self.AF_CONFIRM_READ = 945         # AutoProgram確認讀取
        self.AF_TOTAL_DETECTIONS = 946     # 檢測結果總數
        self.AF_CASE_F_COUNT = 947         # CASE_F總數
        
        # AutoFeeding控制地址
        self.AF_RUN_CONTROL = 920          # 啟動/停止控制
        self.AF_PAUSE_CONTROL = 921        # 暫停/恢復控制
        
        # 機械臂模組地址
        self.ROBOT_STATUS = 400            # 機械臂狀態 (實際機械臂狀態)
        self.FLOW1_CONTROL = 1240          # Flow1控制 (Dobot_main)
        self.FLOW1_COMPLETE = 1204         # Flow1完成狀態 (Dobot_main)
        self.FLOW5_COMPLETE = 1206         # Flow5完成狀態 (Dobot_main)
        
        # 載入配置
        self.config = self.load_config()
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 核心狀態變數
        self.prepare_done = False
        
        # 統計資訊
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.feeding_ready_count = 0
        
        print("機械臂協調控制模組初始化完成 (分離式設計)")
        print(f"Modbus服務器: {modbus_host}:{modbus_port}")
        print(f"AutoProgram基地址: {self.BASE_ADDRESS}")
        print(f"AutoFeeding基地址: {self.AF_BASE}")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autoprogram": {
                "coordination_interval": 0.5,      # 協調週期間隔
                "flow1_timeout": 30.0,             # Flow1執行超時
                "flow5_timeout": 60.0,             # Flow5執行超時
                "feeding_confirm_timeout": 2.0,    # 入料確認超時
                "robot_ready_value": 9             # 機械臂Ready狀態值
            },
            "autofeeding_handshake": {
                "max_confirm_attempts": 5,         # 最大確認嘗試次數
                "confirm_retry_delay": 0.2,        # 確認重試間隔
                "coordinate_read_delay": 0.1       # 座標讀取延遲
            },
            "timing": {
                "register_clear_delay": 0.1,       # 寄存器清除延遲
                "status_check_interval": 0.1,      # 狀態檢查間隔
                "command_delay": 0.1               # 指令延遲
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autoprogram_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                print(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # AutoProgram狀態寄存器 (1300-1319)
            self.write_register(1300, SystemStatus.STOPPED.value)  # 系統狀態
            self.write_register(1301, 0)  # 機械臂狀態
            self.write_register(1302, 0)  # prepare_done狀態
            self.write_register(1303, 0)  # feeding_ready狀態
            self.write_register(1304, 0)  # Flow1完成狀態
            self.write_register(1305, 0)  # Flow5完成狀態
            self.write_register(1306, 0)  # 協調週期計數
            self.write_register(1307, 0)  # Flow1觸發次數
            self.write_register(1308, 0)  # Flow5完成次數
            self.write_register(1309, 0)  # 錯誤代碼
            
            # AutoProgram控制寄存器 (1320-1339)
            self.write_register(1320, 0)  # 系統控制
            self.write_register(1321, 0)  # Flow1控制
            self.write_register(1322, 0)  # 錯誤清除
            self.write_register(1323, 0)  # 強制重置
            
            # 機械臂協調寄存器 (1340-1359)
            self.write_register(1340, 0)  # Flow1執行觸發
            self.write_register(1341, 0)  # 目標座標X高位
            self.write_register(1342, 0)  # 目標座標X低位
            self.write_register(1343, 0)  # 目標座標Y高位
            self.write_register(1344, 0)  # 目標座標Y低位
            
            print("AutoProgram系統寄存器初始化完成")
        except Exception as e:
            print(f"系統寄存器初始化失敗: {e}")
    
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def read_32bit_coordinate(self, high_addr: int, low_addr: int) -> float:
        """讀取32位世界座標並轉換為實際值"""
        high_val = self.read_register(high_addr)
        low_val = self.read_register(low_addr)
        
        if high_val is None or low_val is None:
            return 0.0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def check_robot_ready(self) -> bool:
        """檢查機械臂是否Ready"""
        robot_status = self.read_register(400)  # 讀取實際機械臂狀態
        if robot_status is None:
            return False
        
        robot_ready = (robot_status == self.config['autoprogram']['robot_ready_value'])
        
        # 更新到1301寄存器供外部讀取
        self.write_register(1301, robot_status)
        
        return robot_ready
    
    def read_autofeeding_status(self) -> AutoFeedingStatus:
        """讀取AutoFeeding模組狀態"""
        status = AutoFeedingStatus()
        
        status.module_status = self.read_register(self.AF_MODULE_STATUS) or 0
        status.feeding_complete = self.read_register(self.AF_FEEDING_COMPLETE) or 0
        status.total_detections = self.read_register(self.AF_TOTAL_DETECTIONS) or 0
        status.case_f_count = self.read_register(self.AF_CASE_F_COUNT) or 0
        
        # 讀取32位座標
        if status.feeding_complete == 1:
            status.target_x = self.read_32bit_coordinate(self.AF_TARGET_X_HIGH, self.AF_TARGET_X_LOW)
            status.target_y = self.read_32bit_coordinate(self.AF_TARGET_Y_HIGH, self.AF_TARGET_Y_LOW)
        
        return status
    
    def start_autofeeding(self) -> bool:
        """啟動AutoFeeding模組"""
        success = self.write_register(self.AF_RUN_CONTROL, 1)
        if success:
            print("[AutoProgram] 啟動AutoFeeding模組")
        else:
            print("[AutoProgram] ✗ AutoFeeding模組啟動失敗")
        return success
    
    def stop_autofeeding(self) -> bool:
        """停止AutoFeeding模組"""
        success = self.write_register(self.AF_RUN_CONTROL, 0)
        if success:
            print("[AutoProgram] 停止AutoFeeding模組")
        else:
            print("[AutoProgram] ✗ AutoFeeding模組停止失敗")
        return success
    
    def pause_autofeeding(self) -> bool:
        """暫停AutoFeeding檢測 (Flow1執行前)"""
        success = self.write_register(self.AF_PAUSE_CONTROL, 1)
        if success:
            print("[AutoProgram] 暫停AutoFeeding檢測")
        else:
            print("[AutoProgram] ✗ AutoFeeding暫停失敗")
        return success
    
    def resume_autofeeding(self) -> bool:
        """恢復AutoFeeding檢測 (Flow1執行後)"""
        success = self.write_register(self.AF_PAUSE_CONTROL, 0)
        if success:
            print("[AutoProgram] 恢復AutoFeeding檢測")
        else:
            print("[AutoProgram] ✗ AutoFeeding恢復失敗")
        return success
    
    def confirm_feeding_read(self) -> bool:
        """確認已讀取AutoFeeding座標"""
        max_attempts = self.config['autofeeding_handshake']['max_confirm_attempts']
        retry_delay = self.config['autofeeding_handshake']['confirm_retry_delay']
        
        for attempt in range(max_attempts):
            if self.write_register(self.AF_CONFIRM_READ, 1):
                print(f"[AutoProgram] 確認讀取AutoFeeding座標 (嘗試{attempt + 1}/{max_attempts})")
                return True
            else:
                print(f"[AutoProgram] ✗ 確認讀取失敗 (嘗試{attempt + 1}/{max_attempts})")
                time.sleep(retry_delay)
        
        print("[AutoProgram] ✗ 確認讀取最終失敗")
        return False
    
    def copy_target_coordinates(self, target_x: float, target_y: float) -> bool:
        """複製目標座標到AutoProgram寄存器"""
        # 轉換為整數形式(×100)
        x_int = int(target_x * 100)
        y_int = int(target_y * 100)
        
        # 處理負數(補碼)
        if x_int < 0:
            x_int = x_int + 4294967296  # 2^32
        if y_int < 0:
            y_int = y_int + 4294967296  # 2^32
        
        # 分解為高低位
        x_high = (x_int >> 16) & 0xFFFF
        x_low = x_int & 0xFFFF
        y_high = (y_int >> 16) & 0xFFFF
        y_low = y_int & 0xFFFF
        
        # 寫入AutoProgram座標寄存器
        success = True
        success &= self.write_register(1341, x_high)  # 目標座標X高位
        success &= self.write_register(1342, x_low)   # 目標座標X低位
        success &= self.write_register(1343, y_high)  # 目標座標Y高位
        success &= self.write_register(1344, y_low)   # 目標座標Y低位
        
        if success:
            print(f"[AutoProgram] 目標座標已複製: ({target_x:.2f}, {target_y:.2f})")
        else:
            print("[AutoProgram] ✗ 目標座標複製失敗")
        
        return success
    
    def execute_flow1(self) -> bool:
        """執行Flow1取料作業"""
        print("[AutoProgram] === 開始執行Flow1取料作業 ===")
        
        # 觸發Flow1
        if not self.write_register(1340, 1):
            print("[AutoProgram] ✗ Flow1觸發失敗")
            return False
        
        self.flow1_trigger_count += 1
        print("[AutoProgram] Flow1已觸發，等待完成...")
        
        # 等待Flow1完成
        timeout = self.config['autoprogram']['flow1_timeout']
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            flow1_status = self.read_register(self.FLOW1_COMPLETE)
            if flow1_status == 1:
                elapsed = time.time() - start_time
                print(f"[AutoProgram] ✓ Flow1執行完成 (耗時{elapsed:.1f}s)")
                
                # 清除Flow1控制狀態
                self.write_register(1340, 0)
                time.sleep(self.config['timing']['register_clear_delay'])
                
                return True
            
            time.sleep(self.config['timing']['status_check_interval'])
        
        # Flow1超時
        elapsed = time.time() - start_time
        print(f"[AutoProgram] ✗ Flow1執行超時 (耗時{elapsed:.1f}s)")
        self.write_register(1340, 0)  # 清除控制狀態
        return False
    
    def check_flow5_complete(self) -> bool:
        """檢查Flow5是否完成"""
        flow5_status = self.read_register(self.FLOW5_COMPLETE)
        return flow5_status == 1
    
    def clear_flow5_status(self):
        """清除Flow5完成狀態"""
        self.write_register(self.FLOW5_COMPLETE, 0)
        self.flow5_complete_count += 1
        print("[AutoProgram] Flow5完成狀態已清除")
    
    def coordination_cycle(self):
        """機械臂協調控制週期"""
        try:
            self.coordination_cycle_count += 1
            
            # 檢查機械臂Ready狀態
            robot_ready = self.check_robot_ready()
            
            # 讀取AutoFeeding狀態
            af_status = self.read_autofeeding_status()
            
            # 更新狀態寄存器
            self.write_register(1303, 1 if af_status.feeding_complete == 1 else 0)  # feeding_ready狀態
            
            # 核心邏輯：優先確保prepare_done為True
            if not self.prepare_done:
                # prepare_done=False，需要執行Flow1讓機台準備好
                if robot_ready:
                    # 機械臂Ready，啟動AutoFeeding
                    if af_status.module_status != 1:  # AutoFeeding未運行
                        print("[AutoProgram] 機械臂Ready，啟動AutoFeeding模組")
                        self.start_autofeeding()
                    
                    # 檢查是否有入料完成
                    if af_status.feeding_complete == 1:
                        self.feeding_ready_count += 1
                        print(f"[AutoProgram] 檢測到入料完成 (第{self.feeding_ready_count}次)")
                        print(f"[AutoProgram] 目標座標: ({af_status.target_x:.2f}, {af_status.target_y:.2f})")
                        print(f"[AutoProgram] 檢測統計: 總數={af_status.total_detections}, CASE_F={af_status.case_f_count}")
                        
                        # 複製座標到AutoProgram寄存器
                        if self.copy_target_coordinates(af_status.target_x, af_status.target_y):
                            # 確認讀取
                            if self.confirm_feeding_read():
                                # 暫停AutoFeeding避免干擾Flow1
                                if self.pause_autofeeding():
                                    # 執行Flow1
                                    if self.execute_flow1():
                                        # Flow1成功，設置prepare_done=True
                                        self.prepare_done = True
                                        self.write_register(1202, 1)  # 更新prepare_done狀態
                                        print("[AutoProgram] ✓ Flow1完成，prepare_done=True，機台準備就緒")
                                    else:
                                        print("[AutoProgram] ✗ Flow1執行失敗")
                                    
                                    # 恢復AutoFeeding
                                    self.resume_autofeeding()
                                else:
                                    print("[AutoProgram] ✗ AutoFeeding暫停失敗")
                            else:
                                print("[AutoProgram] ✗ 入料完成確認失敗")
                        else:
                            print("[AutoProgram] ✗ 目標座標複製失敗")
                else:
                    # 機械臂未Ready，停止AutoFeeding節省資源
                    if af_status.module_status == 1:  # AutoFeeding正在運行
                        print("[AutoProgram] 機械臂未Ready，停止AutoFeeding模組")
                        self.stop_autofeeding()
            
            else:
                # prepare_done=True，機台已準備好，等待Flow5完成
                if self.check_flow5_complete():
                    print("[AutoProgram] 檢測到Flow5完成，重置系統狀態")
                    
                    # Flow5完成，重置prepare_done=False開始新週期
                    self.prepare_done = False
                    self.write_register(1302, 0)  # 更新prepare_done狀態
                    
                    # 清除Flow5完成狀態
                    self.clear_flow5_status()
                    
                    print("[AutoProgram] prepare_done=False，系統準備新週期")
                
                # 如果機械臂不Ready且AutoFeeding在運行，停止它
                if not robot_ready and af_status.module_status == 1:
                    print("[AutoProgram] 機械臂未Ready，停止AutoFeeding模組")
                    self.stop_autofeeding()
                
                # 如果機械臂Ready且prepare_done=True，確保AutoFeeding在運行以備下次週期
                elif robot_ready and af_status.module_status != 1:
                    print("[AutoProgram] 機械臂Ready且prepare_done=True，確保AutoFeeding運行")
                    self.start_autofeeding()
            
        except Exception as e:
            print(f"[AutoProgram] 協調週期異常: {e}")
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.write_register(1300, self.system_status.value)
            
            # 更新統計資訊
            self.write_register(1306, self.coordination_cycle_count)  # 協調週期計數
            self.write_register(1307, self.flow1_trigger_count)       # Flow1觸發次數
            self.write_register(1308, self.flow5_complete_count)      # Flow5完成次數
            
        except Exception as e:
            print(f"系統寄存器更新失敗: {e}")
    
    def start(self):
        """啟動機械臂協調控制系統"""
        if self.running:
            return
        
        print("[AutoProgram] === 啟動機械臂協調控制系統 ===")
        self.running = True
        self.system_status = SystemStatus.RUNNING
        
        # 重置狀態
        self.prepare_done = False
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.feeding_ready_count = 0
        
        # 更新狀態寄存器
        self.write_register(1302, 0)  # prepare_done=False
        
        self.thread = threading.Thread(target=self._coordination_loop, daemon=True)
        self.thread.start()
        
        print("[AutoProgram] 協調控制系統已啟動，開始機械臂協調控制")
        print("[AutoProgram] 目標：以最快速度讓prepare_done=True")
    
    def stop(self):
        """停止機械臂協調控制系統"""
        if not self.running:
            return
        
        print("[AutoProgram] === 停止機械臂協調控制系統 ===")
        self.running = False
        self.system_status = SystemStatus.STOPPED
        
        # 停止AutoFeeding模組
        self.stop_autofeeding()
        
        # 更新系統寄存器
        self.update_system_registers()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        print("[AutoProgram] 協調控制系統已停止")
        self.print_statistics()
    
    def _coordination_loop(self):
        """協調控制主循環"""
        interval = self.config['autoprogram']['coordination_interval']
        
        while self.running:
            try:
                self.coordination_cycle()
                time.sleep(interval)
                
            except Exception as e:
                print(f"[AutoProgram] 協調循環異常: {e}")
                time.sleep(1.0)
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def print_statistics(self):
        """輸出統計資訊"""
        print(f"\n=== AutoProgram統計資訊 ===")
        print(f"協調週期數: {self.coordination_cycle_count}")
        print(f"Flow1觸發次數: {self.flow1_trigger_count}")
        print(f"Flow5完成次數: {self.flow5_complete_count}")
        print(f"入料完成檢測次數: {self.feeding_ready_count}")
        print(f"當前prepare_done狀態: {self.prepare_done}")
        
        if self.coordination_cycle_count > 0:
            flow1_rate = (self.flow1_trigger_count / self.coordination_cycle_count) * 100
            print(f"Flow1觸發率: {flow1_rate:.1f}%")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        # 讀取AutoFeeding狀態
        af_status = self.read_autofeeding_status()
        
        return {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "running": self.running,
            "prepare_done": self.prepare_done,
            "robot_ready": self.check_robot_ready(),
            "autofeeding_status": {
                "module_status": af_status.module_status,
                "feeding_complete": af_status.feeding_complete,
                "target_coordinates": (af_status.target_x, af_status.target_y),
                "total_detections": af_status.total_detections,
                "case_f_count": af_status.case_f_count
            },
            "statistics": {
                "coordination_cycle_count": self.coordination_cycle_count,
                "flow1_trigger_count": self.flow1_trigger_count,
                "flow5_complete_count": self.flow5_complete_count,
                "feeding_ready_count": self.feeding_ready_count
            }
        }


def main():
    """主程序"""
    print("機械臂協調控制模組啟動 (分離式設計)")
    
    # 創建控制器
    controller = AutoProgramController()
    
    # 連接Modbus
    if not controller.connect():
        print("Modbus連接失敗，程序退出")
        return
    
    try:
        # 啟動協調控制系統
        controller.start()
        
        # 定期更新系統寄存器
        def update_registers():
            while controller.running:
                controller.update_system_registers()
                time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        # 主循環 - 等待用戶操作
        print("\n指令說明:")
        print("  s - 顯示狀態")
        print("  r - 重啟系統")
        print("  start_af - 手動啟動AutoFeeding")
        print("  stop_af - 手動停止AutoFeeding")
        print("  flow1 - 手動觸發Flow1")
        print("  q - 退出程序")
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    status = controller.get_status_info()
                    print(f"\n系統狀態:")
                    for key, value in status.items():
                        if isinstance(value, dict):
                            print(f"  {key}:")
                            for sub_key, sub_value in value.items():
                                print(f"    {sub_key}: {sub_value}")
                        else:
                            print(f"  {key}: {value}")
                elif cmd == 'r':
                    controller.stop()
                    time.sleep(1.0)
                    controller.start()
                elif cmd == 'start_af':
                    controller.start_autofeeding()
                elif cmd == 'stop_af':
                    controller.stop_autofeeding()
                elif cmd == 'flow1':
                    if controller.execute_flow1():
                        controller.prepare_done = True
                        controller.write_register(1202, 1)
                else:
                    print("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        # 清理資源
        controller.stop()
        controller.disconnect()
        print("程序已退出")


if __name__ == "__main__":
    main()