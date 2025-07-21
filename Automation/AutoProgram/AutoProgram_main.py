#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_main.py - 機械臂協調控制模組 (修正版 - 防Flow1重複觸發)
基地址：1300-1399
邏輯修正：
1. 監控Flow5運行狀態(1201=5)和完成狀態(1206)
2. 只有在prepare_done=False且Flow5未運行時才觸發Flow1
3. Flow1觸發後等待完成才設置prepare_done=True
4. Flow5完成後設置prepare_done=False
"""

import time
import os
import json
import threading
import logging
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
from logging.handlers import RotatingFileHandler

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("[ERROR] pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


def setup_logging(module_name: str) -> logging.Logger:
    """統一設置logging配置"""
    # 日誌目錄：執行檔同層目錄下的logs資料夾
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 格式化器
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 文件處理器 (輪替日誌，保存一週)
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=7,          # 保留7個檔案
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    # 控制台處理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    # 配置logger
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_TRIGGERED = 2
    FLOW1_WAITING = 3
    FLOW5_RUNNING = 4
    FLOW5_COMPLETED = 5
    ERROR = 6


class AutoProgramController:
    """機械臂協調控制模組 (修正版 - 防Flow1重複觸發)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        # 設置logger
        self.logger = setup_logging("AutoProgram")
        self.logger.info("AutoProgram控制器初始化開始")
        
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址配置
        self.BASE_ADDRESS = 1300
        
        # AutoFeeding模組地址
        self.AF_CASE_F_AVAILABLE = 940         # AutoFeeding CASE_F可用標誌
        self.AF_TARGET_X_HIGH = 941            # 目標座標X高位
        self.AF_TARGET_X_LOW = 942             # 目標座標X低位
        self.AF_TARGET_Y_HIGH = 943            # 目標座標Y高位
        self.AF_TARGET_Y_LOW = 944             # 目標座標Y低位
        self.AF_COORDS_TAKEN = 945             # 座標已讀取標誌
        
        # Dobot M1Pro地址
        self.DOBOT_CURRENT_MOTION_FLOW = 1201  # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        self.DOBOT_FLOW1_CONTROL = 1240        # Flow1控制
        self.DOBOT_FLOW1_COMPLETE = 1204       # Flow1完成狀態
        self.DOBOT_FLOW5_COMPLETE = 1206       # Flow5完成狀態
        
        # 載入配置
        self.config = self.load_config()
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 核心狀態變數
        self.prepare_done = False
        self.auto_program_enabled = True
        
        # 修正：新增Flow狀態追蹤
        self.flow1_triggered = False    # Flow1是否已觸發（等待完成）
        self.flow5_running = False      # Flow5是否正在運行
        self.last_flow5_complete = False  # 上次Flow5完成狀態
        
        # 統計資訊
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.case_f_taken_count = 0
        self.flow1_repeat_prevention_count = 0  # Flow1重複觸發防護次數
        
        self.logger.info("機械臂協調控制模組初始化完成 (防Flow1重複觸發版)")
        self.logger.info(f"Modbus服務器: {modbus_host}:{modbus_port}")
        self.logger.info(f"AutoProgram基地址: {self.BASE_ADDRESS}")
        self.logger.info("修正邏輯:")
        self.logger.info("  ✓ 監控Flow5運行狀態(1201=5)和完成狀態(1206)")
        self.logger.info("  ✓ 只有在prepare_done=False且Flow5未運行時才觸發Flow1")
        self.logger.info("  ✓ Flow1觸發後等待完成才設置prepare_done=True")
        self.logger.info("  ✓ Flow5完成後設置prepare_done=False")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autoprogram": {
                "coordination_interval": 0.05,      # 協調週期間隔
                "auto_program_enabled": True,      # 自動程序啟用
                "flow1_trigger_delay": 0.1,        # Flow1觸發延遲
                "coords_confirm_delay": 0.1,       # 座標確認延遲
                "flow1_complete_timeout": 30.0,    # Flow1完成超時時間
                "flow5_monitor_interval": 0.02,    # Flow5監控間隔
            },
            "monitoring": {
                "case_f_check_interval": 0.01,      # CASE_F檢查間隔
                "flow5_check_interval": 0.02,       # Flow5檢查間隔
                "status_update_interval": 0.1,     # 狀態更新間隔
            },
            "timing": {
                "register_clear_delay": 0.05,      # 寄存器清除延遲
                "flow1_response_timeout": 10.0,    # Flow1響應超時
            },
            "logging": {
                "debug_interval": 100,              # Debug訊息輸出間隔
                "status_report_interval": 50,      # 狀態報告間隔
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autoprogram_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # 深度合併配置，確保所有必要的key都存在
                    for section_key, section_value in default_config.items():
                        if section_key in loaded_config:
                            if isinstance(section_value, dict):
                                # 合併子字典，保留預設值
                                for sub_key, sub_value in section_value.items():
                                    if sub_key not in loaded_config[section_key]:
                                        loaded_config[section_key][sub_key] = sub_value
                        else:
                            loaded_config[section_key] = section_value
                    default_config = loaded_config
                self.logger.info(f"配置檔案已載入: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"預設配置檔案已創建: {config_path}")
        except Exception as e:
            self.logger.error(f"配置檔案處理失敗: {e}", exc_info=True)
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                self.logger.error("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                self.logger.info(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                self.logger.error(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            self.logger.error(f"Modbus連接異常: {e}", exc_info=True)
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # AutoProgram狀態寄存器 (1300-1319)
            self.write_register(1300, SystemStatus.STOPPED.value)  # 系統狀態
            self.write_register(1301, 0)  # prepare_done狀態
            self.write_register(1302, 1 if self.auto_program_enabled else 0)  # 自動程序啟用狀態
            self.write_register(1303, 0)  # AutoFeeding CASE_F狀態
            self.write_register(1304, 0)  # Flow5運行狀態
            self.write_register(1305, 0)  # Flow5完成狀態
            self.write_register(1306, 0)  # Flow1觸發狀態
            self.write_register(1307, 0)  # 協調週期計數
            self.write_register(1308, 0)  # Flow1觸發次數
            self.write_register(1309, 0)  # Flow5完成次數
            self.write_register(1310, 0)  # CASE_F取得次數
            self.write_register(1311, 0)  # Flow1重複防護次數
            self.write_register(1312, 0)  # 錯誤代碼
            
            # AutoProgram控制寄存器 (1320-1339)
            self.write_register(1320, 0)  # 系統控制
            self.write_register(1321, 1 if self.auto_program_enabled else 0)  # 自動程序啟用控制
            self.write_register(1322, 0)  # 錯誤清除
            self.write_register(1323, 0)  # 強制重置
            
            # AutoFeeding狀態寄存器 (1340-1359)
            self.write_register(1340, 0)  # 目標座標X高位
            self.write_register(1341, 0)  # 目標座標X低位
            self.write_register(1342, 0)  # 目標座標Y高位
            self.write_register(1343, 0)  # 目標座標Y低位
            
            self.logger.info("AutoProgram系統寄存器初始化完成")
        except Exception as e:
            self.logger.error(f"系統寄存器初始化失敗: {e}", exc_info=True)
    
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
    
    def check_flow5_status(self) -> Dict[str, bool]:
        """檢查Flow5運行和完成狀態"""
        try:
            # 讀取當前運動Flow
            current_motion_flow = self.read_register(self.DOBOT_CURRENT_MOTION_FLOW)
            flow5_complete = self.read_register(self.DOBOT_FLOW5_COMPLETE)
            
            # 判斷Flow5是否正在運行
            flow5_running_now = (current_motion_flow == 5)
            flow5_complete_now = (flow5_complete == 1)
            
            # 檢查Flow5狀態變化
            if flow5_running_now != self.flow5_running:
                self.flow5_running = flow5_running_now
                if self.flow5_running:
                    self.logger.info("檢測到Flow5開始運行 (1201=5)")
                    self.system_status = SystemStatus.FLOW5_RUNNING
                else:
                    self.logger.info("檢測到Flow5停止運行 (1201!=5)")
                    if self.system_status == SystemStatus.FLOW5_RUNNING:
                        self.system_status = SystemStatus.RUNNING
            
            # 檢查Flow5完成狀態變化
            if flow5_complete_now != self.last_flow5_complete:
                self.last_flow5_complete = flow5_complete_now
                if flow5_complete_now:
                    self.logger.info("檢測到Flow5完成標誌 (1206=1)")
            
            return {
                'running': flow5_running_now,
                'complete': flow5_complete_now
            }
            
        except Exception as e:
            self.logger.error(f"Flow5狀態檢查失敗: {e}", exc_info=True)
            return {'running': False, 'complete': False}
    
    def get_autofeeding_status(self) -> Dict[str, Any]:
        """獲取AutoFeeding模組狀態"""
        case_f_available = self.read_register(self.AF_CASE_F_AVAILABLE) or 0
        coords_taken = self.read_register(self.AF_COORDS_TAKEN) or 0
        
        target_x = 0.0
        target_y = 0.0
        
        if case_f_available == 1:
            target_x = self.read_32bit_coordinate(self.AF_TARGET_X_HIGH, self.AF_TARGET_X_LOW)
            target_y = self.read_32bit_coordinate(self.AF_TARGET_Y_HIGH, self.AF_TARGET_Y_LOW)
        
        return {
            'case_f_available': bool(case_f_available),
            'coords_taken': bool(coords_taken),
            'target_x': target_x,
            'target_y': target_y
        }
    
    def take_case_f_coordinates(self) -> Optional[tuple]:
        """讀取並確認CASE_F座標"""
        af_status = self.get_autofeeding_status()
        
        if not af_status['case_f_available']:
            self.logger.warning("AutoFeeding無可用CASE_F")
            return None
        
        self.logger.info(f"準備讀取CASE_F座標: ({af_status['target_x']:.2f}, {af_status['target_y']:.2f})")
        
        # 複製座標到AutoProgram寄存器
        x_int = int(af_status['target_x'] * 100)
        y_int = int(af_status['target_y'] * 100)
        
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
        self.logger.debug(f"寫入座標寄存器: X({x_high},{x_low}) Y({y_high},{y_low})")
        
        success = True
        success &= self.write_register(1340, x_high)
        success &= self.write_register(1341, x_low)
        success &= self.write_register(1342, y_high)
        success &= self.write_register(1343, y_low)
        
        if success:
            # 確認已讀取座標
            coords_taken_success = self.write_register(self.AF_COORDS_TAKEN, 1)
            time.sleep(self.config['autoprogram']['coords_confirm_delay'])
            
            if coords_taken_success:
                self.case_f_taken_count += 1
                self.logger.info(f"✓ CASE_F座標已準備完成: ({af_status['target_x']:.2f}, {af_status['target_y']:.2f})")
                self.logger.info(f"✓ AutoProgram座標寄存器(1340-1343)已更新")
                self.logger.info(f"✓ AutoFeeding確認標誌(945)已設置")
                
                return (af_status['target_x'], af_status['target_y'])
            else:
                self.logger.error("✗ AutoFeeding確認標誌(945)寫入失敗")
                return None
        else:
            self.logger.error("✗ AutoProgram座標寄存器(1340-1343)寫入失敗")
            return None
    
    def can_trigger_flow1(self) -> Tuple[bool, str]:
        """
        判斷是否可以觸發Flow1
        
        條件：
        1. prepare_done=False
        2. Flow5未運行
        3. Flow1未被觸發或已完成
        4. AutoFeeding有可用的CASE_F
        
        Returns:
            Tuple[bool, str]: (是否可觸發, 原因說明)
        """
        # 檢查prepare_done狀態
        if self.prepare_done:
            return False, "prepare_done=True，無需觸發Flow1"
        
        # 檢查Flow5狀態
        flow5_status = self.check_flow5_status()
        if flow5_status['running']:
            return False, "Flow5正在運行中，不可觸發Flow1"
        
        # 檢查Flow1是否已觸發且未完成
        if self.flow1_triggered:
            flow1_complete = self.check_flow1_complete()
            if not flow1_complete:
                return False, "Flow1已觸發但未完成，等待完成"
        
        # 檢查AutoFeeding CASE_F狀態
        af_status = self.get_autofeeding_status()
        if not af_status['case_f_available']:
            return False, "AutoFeeding無可用CASE_F"
        
        return True, "所有條件滿足，可觸發Flow1"
    
    def trigger_flow1(self) -> bool:
        """觸發Flow1取料作業"""
        self.logger.info("觸發Flow1取料作業")
        
        # 觸發Flow1控制
        if not self.write_register(self.DOBOT_FLOW1_CONTROL, 1):
            self.logger.error(f"Flow1觸發失敗 (寫入{self.DOBOT_FLOW1_CONTROL}=1失敗)")
            return False
        
        time.sleep(self.config['autoprogram']['flow1_trigger_delay'])
        
        # 清除Flow1控制狀態
        self.write_register(self.DOBOT_FLOW1_CONTROL, 0)
        
        # 更新統計（但不在這裡設置flow1_triggered，因為已經在coordination_cycle中設置）
        self.flow1_trigger_count += 1
        self.system_status = SystemStatus.FLOW1_TRIGGERED
        
        self.logger.info(f"Flow1已觸發 (第{self.flow1_trigger_count}次)")
        return True
    
    def check_flow1_complete(self) -> bool:
        """檢查Flow1是否完成"""
        flow1_complete = self.read_register(self.DOBOT_FLOW1_COMPLETE)
        return flow1_complete == 1
    
    def handle_flow1_complete(self):
        """處理Flow1完成"""
        if not self.flow1_triggered:
            return
        
        self.logger.info("檢測到Flow1完成")
        
        # 清除Flow1完成狀態（可選）
        # self.write_register(self.DOBOT_FLOW1_COMPLETE, 0)
        
        # 設置prepare_done=True
        self.prepare_done = True
        self.flow1_triggered = False  # 重置Flow1觸發狀態
        self.system_status = SystemStatus.RUNNING
        
        self.logger.info("prepare_done=True，機台準備就緒")
    
    def handle_flow5_complete(self):
        """處理Flow5完成"""
        flow5_status = self.check_flow5_status()
        
        if flow5_status['complete']:
            self.logger.info("檢測到Flow5完成，料件已送至組立區")
            
            # 清除Flow5完成狀態
            self.write_register(self.DOBOT_FLOW5_COMPLETE, 0)
            
            # 設置prepare_done=False
            self.prepare_done = False
            self.flow5_complete_count += 1
            self.system_status = SystemStatus.FLOW5_COMPLETED
            
            self.logger.info(f"prepare_done=False，準備新週期 (Flow5完成第{self.flow5_complete_count}次)")
    
    def check_control_registers(self):
        """檢查控制寄存器變更"""
        try:
            # 檢查系統控制寄存器 (1320)
            system_control = self.read_register(1320)
            if system_control == 1 and not self.running:
                self.logger.info("檢測到系統啟動指令 (1320=1)")
                self.start()
            elif system_control == 0 and self.running:
                self.logger.info("檢測到系統停止指令 (1320=0)")
                self.stop()
            
            # 檢查自動程序控制寄存器 (1321)
            auto_control = self.read_register(1321)
            if auto_control is not None:
                if auto_control != (1 if self.auto_program_enabled else 0):
                    self.auto_program_enabled = (auto_control == 1)
                    self.logger.info(f"自動程序啟用狀態更新: {self.auto_program_enabled} (1321={auto_control})")
            
        except Exception as e:
            self.logger.error(f"控制寄存器檢查異常: {e}", exc_info=True)
    
    def coordination_cycle(self):
        """機械臂協調控制週期 (修正版)"""
        try:
            self.coordination_cycle_count += 1
            
            # 定期狀態報告
            status_report_interval = self.config.get('logging', {}).get('status_report_interval', 50)
            if self.coordination_cycle_count % status_report_interval == 0:
                af_status = self.get_autofeeding_status()
                flow5_status = self.check_flow5_status()
                
                self.logger.debug(f"週期{self.coordination_cycle_count}: "
                    f"prepare_done={self.prepare_done}, "
                    f"CASE_F可用={af_status['case_f_available']}, "
                    f"Flow5運行={flow5_status['running']}, "
                    f"Flow1已觸發={self.flow1_triggered}")
            
            # 檢查Flow5狀態（總是檢查）
            self.check_flow5_status()
            
            # 處理Flow5完成
            self.handle_flow5_complete()
            
            # 處理Flow1完成
            if self.flow1_triggered and self.check_flow1_complete():
                self.handle_flow1_complete()
            
            # 主要協調邏輯：觸發Flow1 - 修正：防止重複觸發
            if not self.prepare_done and not self.flow1_triggered:  # 增加 flow1_triggered 檢查
                # 檢查是否可以觸發Flow1
                can_trigger, reason = self.can_trigger_flow1()
                
                if can_trigger:
                    # 先設置觸發標誌，防止重複觸發
                    self.flow1_triggered = True
                    
                    # 讀取座標
                    coords = self.take_case_f_coordinates()
                    if coords:
                        # 觸發Flow1
                        if self.trigger_flow1():
                            self.logger.info(f"Flow1已觸發，座標已準備: {coords}，等待完成...")
                            self.system_status = SystemStatus.FLOW1_WAITING
                        else:
                            self.logger.error("Flow1觸發失敗，重置觸發狀態")
                            self.flow1_triggered = False  # 觸發失敗時重置狀態
                    else:
                        self.logger.error("座標讀取失敗，重置觸發狀態")
                        self.flow1_triggered = False  # 座標讀取失敗時重置狀態
                else:
                    # 定期記錄等待原因
                    debug_interval = self.config.get('logging', {}).get('debug_interval', 100)
                    if self.coordination_cycle_count % debug_interval == 0:
                        self.logger.debug(f"等待觸發Flow1條件: {reason}")
            
        except Exception as e:
            self.logger.error(f"協調週期異常: {e}", exc_info=True)
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.write_register(1300, self.system_status.value)
            self.write_register(1301, 1 if self.prepare_done else 0)
            self.write_register(1302, 1 if self.auto_program_enabled else 0)
            
            # 更新狀態監控
            af_status = self.get_autofeeding_status()
            flow5_status = self.check_flow5_status()
            
            self.write_register(1303, 1 if af_status['case_f_available'] else 0)
            self.write_register(1304, 1 if flow5_status['running'] else 0)
            self.write_register(1305, 1 if flow5_status['complete'] else 0)
            self.write_register(1306, 1 if self.flow1_triggered else 0)
            
            # 更新統計資訊
            self.write_register(1307, self.coordination_cycle_count)
            self.write_register(1308, self.flow1_trigger_count)
            self.write_register(1309, self.flow5_complete_count)
            self.write_register(1310, self.case_f_taken_count)
            self.write_register(1311, self.flow1_repeat_prevention_count)
            
            # 更新座標
            if af_status['case_f_available']:
                x_int = int(af_status['target_x'] * 100)
                y_int = int(af_status['target_y'] * 100)
                
                if x_int < 0:
                    x_int = x_int + 4294967296
                if y_int < 0:
                    y_int = y_int + 4294967296
                
                self.write_register(1340, (x_int >> 16) & 0xFFFF)
                self.write_register(1341, x_int & 0xFFFF)
                self.write_register(1342, (y_int >> 16) & 0xFFFF)
                self.write_register(1343, y_int & 0xFFFF)
            
        except Exception as e:
            self.logger.error(f"系統寄存器更新失敗: {e}", exc_info=True)
    
    def start(self):
        """啟動機械臂協調控制系統"""
        if self.running:
            return
        
        self.logger.info("=== 啟動機械臂協調控制系統 (防Flow1重複觸發版) ===")
        self.running = True
        self.system_status = SystemStatus.RUNNING
        
        # 重置狀態
        self.prepare_done = False
        self.flow1_triggered = False
        self.flow5_running = False
        self.last_flow5_complete = False
        
        # 重置統計
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.case_f_taken_count = 0
        self.flow1_repeat_prevention_count = 0
        
        # 立即更新狀態寄存器
        self.write_register(1300, SystemStatus.RUNNING.value)
        self.write_register(1301, 0)  # prepare_done=False
        
        self.thread = threading.Thread(target=self._coordination_loop, daemon=True)
        self.thread.start()
        
        self.logger.info("協調控制系統已啟動")
        self.logger.info("監控目標:")
        self.logger.info(f"  - AutoFeeding CASE_F可用標誌: {self.AF_CASE_F_AVAILABLE}")
        self.logger.info(f"  - Flow5運行狀態: {self.DOBOT_CURRENT_MOTION_FLOW}=5")
        self.logger.info(f"  - Flow5完成狀態: {self.DOBOT_FLOW5_COMPLETE}")
        self.logger.info("修正邏輯: 防止Flow1重複觸發")
    
    def stop(self):
        """停止機械臂協調控制系統"""
        if not self.running:
            return
        
        self.logger.info("=== 停止機械臂協調控制系統 ===")
        self.running = False
        self.system_status = SystemStatus.STOPPED
        
        # 立即更新狀態寄存器
        self.write_register(1300, SystemStatus.STOPPED.value)
        
        # 更新系統寄存器
        self.update_system_registers()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        self.logger.info("協調控制系統已停止")
        self.print_statistics()
    
    def _coordination_loop(self):
        """協調控制主循環"""
        interval = self.config['autoprogram']['coordination_interval']
        
        self.logger.info("協調控制主循環已啟動")
        
        loop_count = 0
        while True:
            try:
                loop_count += 1
                
                # 定期心跳日誌
                if loop_count % 2000 == 0:  # 每2000次循環輸出一次心跳
                    self.logger.debug(f"控制循環心跳 - 第{loop_count}次, running={self.running}, auto_enabled={self.auto_program_enabled}")
                
                # 總是檢查控制寄存器變更
                self.check_control_registers()
                
                # 只有在系統運行且自動程序啟用時才執行協調邏輯
                if self.running and self.auto_program_enabled:
                    self.coordination_cycle()
                
                time.sleep(interval)
                
            except Exception as e:
                self.logger.error(f"協調循環異常: {e}", exc_info=True)
                time.sleep(1.0)
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            self.logger.info("Modbus連接已斷開")
    
    def print_statistics(self):
        """輸出統計資訊"""
        self.logger.info("=== AutoProgram統計資訊 ===")
        self.logger.info(f"協調週期數: {self.coordination_cycle_count}")
        self.logger.info(f"Flow1觸發次數: {self.flow1_trigger_count}")
        self.logger.info(f"Flow5完成次數: {self.flow5_complete_count}")
        self.logger.info(f"CASE_F取得次數: {self.case_f_taken_count}")
        self.logger.info(f"Flow1重複防護次數: {self.flow1_repeat_prevention_count}")
        self.logger.info(f"當前prepare_done狀態: {self.prepare_done}")
        self.logger.info(f"自動程序啟用: {self.auto_program_enabled}")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        af_status = self.get_autofeeding_status()
        flow5_status = self.check_flow5_status()
        
        return {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "running": self.running,
            "auto_program_enabled": self.auto_program_enabled,
            "prepare_done": self.prepare_done,
            "flow1_triggered": self.flow1_triggered,
            "flow5_running": flow5_status['running'],
            "autofeeding_status": af_status,
            "flow1_complete": self.check_flow1_complete(),
            "flow5_complete": flow5_status['complete'],
            "statistics": {
                "coordination_cycle_count": self.coordination_cycle_count,
                "flow1_trigger_count": self.flow1_trigger_count,
                "flow5_complete_count": self.flow5_complete_count,
                "case_f_taken_count": self.case_f_taken_count,
                "flow1_repeat_prevention_count": self.flow1_repeat_prevention_count
            }
        }


def main():
    """主程序"""
    print("機械臂協調控制模組啟動 (防Flow1重複觸發版)")
    print("修正邏輯: 監控Flow5運行狀態，防止Flow1重複觸發")
    
    # 創建控制器
    controller = AutoProgramController()
    
    # 連接Modbus
    if not controller.connect():
        print("Modbus連接失敗，程序退出")
        return
    
    try:
        # 啟動控制循環
        print("啟動控制循環，等待指令...")
        controller.thread = threading.Thread(target=controller._coordination_loop, daemon=True)
        controller.thread.start()
        
        # 定期更新系統寄存器
        def update_registers():
            while True:
                try:
                    controller.update_system_registers()
                    time.sleep(1.0)
                except Exception as e:
                    print(f"寄存器更新異常: {e}")
                    time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        # 主循環 - 等待用戶操作
        print("\n指令說明:")
        print("  s - 顯示狀態")
        print("  start - 手動啟動系統")
        print("  stop - 手動停止系統")
        print("  enable - 啟用自動程序")
        print("  disable - 停用自動程序")
        print("  flow1 - 手動觸發Flow1")
        print("  check_flow5 - 檢查Flow5狀態")
        print("  clear_f1 - 清除Flow1完成狀態")
        print("  clear_f5 - 清除Flow5完成狀態")
        print("  reset_prepare - 重置prepare_done狀態")
        print("  coords - 手動讀取CASE_F座標")
        print("  check_af - 檢查AutoFeeding狀態")
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
                elif cmd == 'start':
                    controller.write_register(1320, 1)
                    print("系統啟動指令已發送 (1320=1)")
                elif cmd == 'stop':
                    controller.write_register(1320, 0)
                    print("系統停止指令已發送 (1320=0)")
                elif cmd == 'enable':
                    controller.auto_program_enabled = True
                    controller.write_register(1321, 1)
                    print("自動程序已啟用")
                elif cmd == 'disable':
                    controller.auto_program_enabled = False
                    controller.write_register(1321, 0)
                    print("自動程序已停用")
                elif cmd == 'flow1':
                    can_trigger, reason = controller.can_trigger_flow1()
                    if can_trigger:
                        coords = controller.take_case_f_coordinates()
                        if coords and controller.trigger_flow1():
                            print("Flow1已觸發")
                        else:
                            print("Flow1觸發失敗")
                    else:
                        print(f"Flow1無法觸發: {reason}")
                elif cmd == 'check_flow5':
                    flow5_status = controller.check_flow5_status()
                    print(f"Flow5狀態: {flow5_status}")
                elif cmd == 'clear_f1':
                    controller.write_register(controller.DOBOT_FLOW1_COMPLETE, 0)
                    controller.flow1_triggered = False
                    print("Flow1完成狀態已清除")
                elif cmd == 'clear_f5':
                    controller.write_register(controller.DOBOT_FLOW5_COMPLETE, 0)
                    print("Flow5完成狀態已清除")
                elif cmd == 'reset_prepare':
                    controller.prepare_done = False
                    controller.flow1_triggered = False
                    print("prepare_done狀態已重置為False")
                elif cmd == 'coords':
                    coords = controller.take_case_f_coordinates()
                    if coords:
                        print(f"CASE_F座標: {coords}")
                    else:
                        print("無可用的CASE_F座標")
                elif cmd == 'check_af':
                    af_status = controller.get_autofeeding_status()
                    print(f"AutoFeeding狀態: {af_status}")
                else:
                    print("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        # 清理資源
        if controller.running:
            controller.stop()
        controller.disconnect()
        print("程序已退出")


if __name__ == "__main__":
    main()