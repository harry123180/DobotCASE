#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GripperHighLevel_PGE_Enhanced.py - 夾爪高層API模組 (增強PGE支援版)
新增PGE夾爪控制功能，支援PGC和PGE兩種夾爪類型
適用於機械臂流程中import使用，大幅簡化夾爪操作編碼複雜度
"""

import time
import threading
from typing import Optional, Dict, Any
from enum import IntEnum
import logging

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


# ==================== 夾爪指令枚舉 ====================
class GripperCommand(IntEnum):
    """夾爪指令枚舉"""
    NOP = 0             # 無操作
    INITIALIZE = 1      # 初始化/回零
    STOP = 2           # 停止
    MOVE_ABS = 3       # 絕對位置移動
    SET_FORCE = 5      # 設定力道
    SET_SPEED = 6      # 設定速度
    QUICK_OPEN = 7     # 快速開啟
    QUICK_CLOSE = 8    # 快速關閉


# ==================== 夾爪狀態枚舉 ====================
class GripperStatus(IntEnum):
    """夾爪狀態枚舉"""
    MOVING = 0         # 運動中
    REACHED = 1        # 到達位置
    GRIPPED = 2        # 夾住物體
    DROPPED = 3        # 掉落


# ==================== 夾爪類型枚舉 ====================
class GripperType(IntEnum):
    """夾爪類型枚舉"""
    PGC = 1           # PGC夾爪 (原有)
    PGE = 2           # PGE夾爪 (新增)


# ==================== 夾爪高層API類 ====================
class GripperHighLevelAPI:
    """
    夾爪高層API - 支援PGC和PGE兩種夾爪類型
    
    主要功能:
    1. 智能夾取 - 自動判斷夾取成功
    2. 快速指令 - 發了就走不等確認
    3. 確認指令 - 等待動作完成
    4. 位置控制 - 精確位置移動
    5. PGE夾爪專用控制
    """
    
    def __init__(self, gripper_type: GripperType = GripperType.PGC, 
                 modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化夾爪高層API
        
        Args:
            gripper_type: 夾爪類型 (PGC或PGE)
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.gripper_type = gripper_type
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 根據夾爪類型設定寄存器映射
        if gripper_type == GripperType.PGC:
            self._setup_pgc_registers()
        elif gripper_type == GripperType.PGE:
            self._setup_pge_registers()
        else:
            raise ValueError(f"不支援的夾爪類型: {gripper_type}")
        
        # 指令ID計數器
        self.command_id_counter = 1
        
        # 操作超時設定
        self.operation_timeout = 10.0  # 動作超時時間(秒)
        self.quick_timeout = 0.5       # 快速指令超時時間(秒)
        
        # 設置日誌
        self.logger = logging.getLogger(f"GripperHighLevel_{gripper_type.name}")
        self.logger.setLevel(logging.INFO)
        
        # 初始化狀態
        self.initialized = False
        
        # 自動連接
        self.connect()
        
    def _setup_pgc_registers(self):
        """設定PGC夾爪寄存器映射 (基地址520)"""
        self.REGISTERS = {
            # 狀態寄存器 (500-519)
            'MODULE_STATUS': 500,      # 模組狀態
            'CONNECT_STATUS': 501,     # 連接狀態
            'DEVICE_STATUS': 502,      # 設備狀態(初始化狀態)
            'ERROR_COUNT': 503,        # 錯誤計數
            'GRIP_STATUS': 504,        # 夾持狀態
            'CURRENT_POSITION': 505,   # 當前位置
            
            # 指令寄存器 (520-529)
            'COMMAND': 520,            # 指令代碼
            'PARAM1': 521,             # 參數1
            'PARAM2': 522,             # 參數2
            'COMMAND_ID': 523,         # 指令ID
        }
        
    def _setup_pge_registers(self):
        """設定PGE夾爪寄存器映射 (基於Modbus地址表)"""
        self.REGISTERS = {
            # PGE控制寄存器 (基於圖片中的地址表)
            'INITIALIZE': 256,         # 0x0100: 初始化實爪
            'FORCE': 257,              # 0x0101: 力值
            'POSITION': 259,           # 0x0103: 運動到指定位置
            'SPEED': 260,              # 0x0104: 以設定速度運行
            
            # PGE狀態寄存器
            'INIT_STATUS': 512,        # 0x0200: 反饋當前實爪的初始化狀態
            'GRIP_STATUS': 513,        # 0x0201: 反饋當前實爪的夾持狀態
            'CURRENT_POSITION': 514,   # 0x0202: 反饋當前實爪位置信息
        }
        
        # PGE特殊參數
        self.PGE_PARAMS = {
            'MIN_POSITION': 0,         # 最小位置
            'MAX_POSITION': 1000,      # 最大位置
            'DEFAULT_FORCE': 50,       # 預設力值 (20-100百分比)
            'DEFAULT_SPEED': 50,       # 預設速度 (1-100百分比)
            'GRIP_POSITION': 500,      # 夾持位置
            'RELEASE_POSITION': 1000   # 釋放位置
        }
    
    def connect(self) -> bool:
        """連接到Modbus TCP服務器"""
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.logger.info(f"正在連接Modbus TCP服務器: {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3,
                retries=2
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.logger.info(f"✓ {self.gripper_type.name}夾爪Modbus連接成功")
                
                # 連接成功後自動初始化
                if not self.initialized:
                    self.logger.info(f"開始自動初始化{self.gripper_type.name}夾爪...")
                    if self.initialize(wait_completion=True):
                        self.logger.info(f"✓ {self.gripper_type.name}夾爪初始化成功")
                    else:
                        self.logger.warning(f"⚠️ {self.gripper_type.name}夾爪初始化失敗")
                
                return True
            else:
                self.logger.error("Modbus連接失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client:
            self.modbus_client.close()
            self.connected = False
            self.logger.info(f"{self.gripper_type.name}夾爪連接已斷開")
    
    def _write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value)
            
            if not result.isError():
                self.logger.debug(f"寫入寄存器 {register_name}[{address}] = {value}")
                return True
            else:
                self.logger.error(f"寫入寄存器失敗: {result}")
                return False
                
        except Exception as e:
            self.logger.error(f"寫入寄存器異常: {e}")
            return False
    
    def _read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, 1)
            
            if not result.isError() and len(result.registers) > 0:
                value = result.registers[0]
                self.logger.debug(f"讀取寄存器 {register_name}[{address}] = {value}")
                return value
            else:
                self.logger.error(f"讀取寄存器失敗: {result}")
                return None
                
        except Exception as e:
            self.logger.error(f"讀取寄存器異常: {e}")
            return None
    
    # ==================== 初始化API ====================
    
    def initialize(self, wait_completion: bool = True) -> bool:
        """
        初始化夾爪
        
        Args:
            wait_completion: 是否等待初始化完成
            
        Returns:
            bool: 初始化是否成功
        """
        self.logger.info(f"開始初始化{self.gripper_type.name}夾爪")
        
        try:
            if self.gripper_type == GripperType.PGC:
                return self._initialize_pgc(wait_completion)
            elif self.gripper_type == GripperType.PGE:
                return self._initialize_pge(wait_completion)
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"初始化失敗: {e}")
            return False
    
    def _initialize_pgc(self, wait_completion: bool) -> bool:
        """初始化PGC夾爪"""
        if not self._send_command(GripperCommand.INITIALIZE):
            return False
        
        if wait_completion:
            success = self._wait_for_completion(self.operation_timeout)
            if success:
                self.initialized = True
                self.logger.info("✓ PGC夾爪初始化完成")
            return success
        
        return True
    
    def _initialize_pge(self, wait_completion: bool) -> bool:
        """初始化PGE夾爪"""
        # PGE初始化：寫入0x0300到0x0100寄存器，需要Flash存儲
        if not self._write_register('INITIALIZE', 0x0300):
            return False
        
        # 設定預設參數
        time.sleep(0.1)
        if not self._write_register('FORCE', self.PGE_PARAMS['DEFAULT_FORCE']):
            return False
            
        time.sleep(0.1)
        if not self._write_register('SPEED', self.PGE_PARAMS['DEFAULT_SPEED']):
            return False
        
        if wait_completion:
            # 等待初始化完成 (檢查初始化狀態)
            timeout = 10.0
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                init_status = self._read_register('INIT_STATUS')
                if init_status == 1:  # 初始化成功
                    self.initialized = True
                    self.logger.info("✓ PGE夾爪初始化完成")
                    return True
                elif init_status == 2:  # 初始化進行中
                    time.sleep(0.5)
                    continue
                else:
                    break
            
            self.logger.error("PGE夾爪初始化超時")
            return False
        
        return True
    
    def is_initialized(self) -> bool:
        """檢查是否已初始化"""
        if self.gripper_type == GripperType.PGC:
            # 檢查PGC初始化狀態
            status = self._read_register('DEVICE_STATUS')
            return status == 1 if status is not None else False
        elif self.gripper_type == GripperType.PGE:
            # 檢查PGE初始化狀態
            status = self._read_register('INIT_STATUS')
            return status == 1 if status is not None else False
        
        return self.initialized
    
    # ==================== PGE專用API ====================
    
    def pge_smart_grip(self, target_position: int = None, max_attempts: int = 3) -> bool:
        """
        PGE夾爪智能夾取
        
        Args:
            target_position: 目標夾取位置，None使用預設值
            max_attempts: 最大嘗試次數
            
        Returns:
            bool: 夾取是否成功
        """
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if not self.is_initialized():
            self.logger.error("PGE夾爪未初始化")
            return False
        
        target_pos = target_position or self.PGE_PARAMS['GRIP_POSITION']
        self.logger.info(f"PGE智能夾取到位置: {target_pos}")
        
        for attempt in range(max_attempts):
            try:
                # 移動到目標位置
                if not self._write_register('POSITION', target_pos):
                    continue
                
                # 等待運動完成並檢查夾持狀態
                if self._wait_for_pge_grip_completion(timeout=5.0):
                    self.logger.info(f"✓ PGE智能夾取成功 (嘗試 {attempt + 1})")
                    return True
                else:
                    self.logger.warning(f"PGE夾取嘗試 {attempt + 1} 失敗")
                    
            except Exception as e:
                self.logger.error(f"PGE夾取嘗試 {attempt + 1} 異常: {e}")
        
        self.logger.error(f"PGE智能夾取失敗 (已嘗試 {max_attempts} 次)")
        return False
    
    def pge_quick_open(self, release_position: int = None) -> bool:
        """
        PGE夾爪快速開啟
        
        Args:
            release_position: 釋放位置，None使用預設值
            
        Returns:
            bool: 操作是否成功
        """
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if not self.is_initialized():
            self.logger.error("PGE夾爪未初始化")
            return False
        
        release_pos = release_position or self.PGE_PARAMS['RELEASE_POSITION']
        self.logger.info(f"PGE快速開啟到位置: {release_pos}")
        
        return self._write_register('POSITION', release_pos)
    
    def pge_set_force(self, force_percent: int) -> bool:
        """
        設定PGE夾爪力值
        
        Args:
            force_percent: 力值百分比 (20-100)
            
        Returns:
            bool: 設定是否成功
        """
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if not 20 <= force_percent <= 100:
            self.logger.error(f"PGE力值超出範圍: {force_percent} (應為20-100)")
            return False
        
        self.logger.info(f"設定PGE夾爪力值: {force_percent}%")
        return self._write_register('FORCE', force_percent)
    
    def pge_set_speed(self, speed_percent: int) -> bool:
        """
        設定PGE夾爪速度
        
        Args:
            speed_percent: 速度百分比 (1-100)
            
        Returns:
            bool: 設定是否成功
        """
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if not 1 <= speed_percent <= 100:
            self.logger.error(f"PGE速度超出範圍: {speed_percent} (應為1-100)")
            return False
        
        self.logger.info(f"設定PGE夾爪速度: {speed_percent}%")
        return self._write_register('SPEED', speed_percent)
    
    def pge_get_position(self) -> Optional[int]:
        """
        取得PGE夾爪當前位置
        
        Returns:
            Optional[int]: 當前位置，失敗返回None
        """
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return None
        
        return self._read_register('CURRENT_POSITION')
    
    def pge_get_grip_status(self) -> Optional[int]:
        """
        取得PGE夾爪夾持狀態
        
        Returns:
            Optional[int]: 夾持狀態 (0=運動中, 1=到達位置, 2=夾住物體, 3=掉落)
        """
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return None
        
        return self._read_register('GRIP_STATUS')
    
    def _wait_for_pge_grip_completion(self, timeout: float = 5.0) -> bool:
        """等待PGE夾取完成並檢查夾持狀態"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            grip_status = self.pge_get_grip_status()
            
            if grip_status == 2:  # 夾住物體
                return True
            elif grip_status == 1:  # 到達位置但未夾住
                return False
            elif grip_status == 3:  # 掉落
                return False
            
            time.sleep(0.1)
        
        self.logger.warning("PGE夾取完成檢查超時")
        return False
    
    # ==================== 通用API (相容PGC) ====================
    
    def smart_grip(self, target_position: int = 420, max_attempts: int = 3) -> bool:
        """
        智能夾取 - 自動適配夾爪類型
        
        Args:
            target_position: 目標位置
            max_attempts: 最大嘗試次數
            
        Returns:
            bool: 夾取是否成功
        """
        if self.gripper_type == GripperType.PGE:
            return self.pge_smart_grip(target_position, max_attempts)
        else:
            # PGC原有邏輯
            return self._pgc_smart_grip(target_position, max_attempts)
    
    def quick_open(self, position: int = None) -> bool:
        """
        快速開啟 - 自動適配夾爪類型
        
        Args:
            position: 開啟位置
            
        Returns:
            bool: 操作是否成功
        """
        if self.gripper_type == GripperType.PGE:
            return self.pge_quick_open(position)
        else:
            # PGC原有邏輯
            return self._send_command(GripperCommand.QUICK_OPEN)
    
    def quick_close(self) -> bool:
        """快速關閉 - 通用方法"""
        if self.gripper_type == GripperType.PGE:
            return self._write_register('POSITION', self.PGE_PARAMS['GRIP_POSITION'])
        else:
            return self._send_command(GripperCommand.QUICK_CLOSE)
    
    def smart_release(self, release_position: int = 50) -> bool:
        """智能釋放 - 通用方法"""
        if self.gripper_type == GripperType.PGE:
            return self.pge_quick_open(release_position)
        else:
            return self.move_to_and_wait(release_position)
    
    # ==================== PGC相容方法 ====================
    
    def _pgc_smart_grip(self, target_position: int, max_attempts: int) -> bool:
        """PGC智能夾取邏輯 (保持原有功能)"""
        self.logger.info(f"PGC智能夾取到位置: {target_position}")
        
        for attempt in range(max_attempts):
            try:
                # 記錄初始位置
                initial_pos = self.get_current_position()
                if initial_pos is None:
                    self.logger.warning("無法讀取初始位置")
                    initial_pos = 0
                
                # 移動到目標位置
                if not self._send_command(GripperCommand.MOVE_ABS, target_position):
                    continue
                
                # 等待運動完成
                if not self._wait_for_completion(self.operation_timeout):
                    continue
                
                # 檢查是否夾到物體
                final_pos = self.get_current_position()
                if final_pos is None:
                    continue
                
                # 如果位置差異大於閾值，表示夾到物體
                position_diff = abs(target_position - final_pos)
                if position_diff > 20:  # 閾值可調整
                    self.logger.info(f"✓ PGC智能夾取成功 (位置差異: {position_diff})")
                    return True
                
            except Exception as e:
                self.logger.error(f"PGC夾取嘗試 {attempt + 1} 異常: {e}")
        
        return False
    
    def _send_command(self, command: GripperCommand, param1: int = 0, param2: int = 0) -> bool:
        """發送PGC指令 (保持原有邏輯)"""
        if self.gripper_type != GripperType.PGC:
            return False
        
        try:
            command_id = self.command_id_counter
            self.command_id_counter += 1
            
            # 寫入參數
            if param1 != 0:
                self._write_register('PARAM1', param1)
            if param2 != 0:
                self._write_register('PARAM2', param2)
            
            # 寫入指令ID
            self._write_register('COMMAND_ID', command_id)
            
            # 發送指令
            return self._write_register('COMMAND', command)
            
        except Exception as e:
            self.logger.error(f"發送指令失敗: {e}")
            return False
    
    def _wait_for_completion(self, timeout: float) -> bool:
        """等待PGC指令完成"""
        if self.gripper_type != GripperType.PGC:
            return False
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            command = self._read_register('COMMAND')
            if command == 0:  # 指令已清零，表示完成
                return True
            time.sleep(0.1)
        
        return False
    
    def get_current_position(self) -> Optional[int]:
        """取得當前位置 - 通用方法"""
        if self.gripper_type == GripperType.PGE:
            return self.pge_get_position()
        else:
            return self._read_register('CURRENT_POSITION')
    
    def move_to_and_wait(self, position: int, timeout: float = None) -> bool:
        """移動到指定位置並等待完成 - 通用方法"""
        timeout = timeout or self.operation_timeout
        
        if self.gripper_type == GripperType.PGE:
            if not self._write_register('POSITION', position):
                return False
            
            # 等待到達位置
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_pos = self.pge_get_position()
                if current_pos is not None and abs(current_pos - position) <= 5:
                    return True
                time.sleep(0.1)
            return False
        else:
            if not self._send_command(GripperCommand.MOVE_ABS, position):
                return False
            return self._wait_for_completion(timeout)
    
    def get_status(self) -> Dict[str, Any]:
        """取得夾爪狀態 - 通用方法"""
        status = {
            'gripper_type': self.gripper_type.name,
            'connected': self.connected,
            'initialized': self.is_initialized(),
            'current_position': self.get_current_position()
        }
        
        if self.gripper_type == GripperType.PGE:
            status['grip_status'] = self.pge_get_grip_status()
        
        return status


# ==================== 使用範例 ====================
def example_usage():
    """使用範例"""
    print("=== 夾爪高層API使用範例 (PGE支援版) ===")
    
    # 測試PGE夾爪
    print("\n--- PGE夾爪測試 ---")
    pge_gripper = GripperHighLevelAPI(gripper_type=GripperType.PGE)
    
    try:
        # 檢查連接和初始化
        if not pge_gripper.connected:
            print("✗ PGE夾爪連接失敗")
            return
        
        print(f"✓ PGE夾爪連接成功，初始化狀態: {pge_gripper.is_initialized()}")
        
        # 設定參數
        pge_gripper.pge_set_force(60)
        pge_gripper.pge_set_speed(80)
        
        # 智能夾取演示
        print("\n=== PGE智能夾取演示 ===")
        if pge_gripper.smart_grip(target_position=500):
            print("✓ PGE智能夾取成功")
            
            # 檢查狀態
            status = pge_gripper.get_status()
            print(f"當前狀態: {status}")
            
            time.sleep(2)  # 模擬處理時間
            
            # 快速開啟
            if pge_gripper.quick_open(1000):
                print("✓ PGE快速開啟成功")
            else:
                print("✗ PGE快速開啟失敗")
        else:
            print("✗ PGE智能夾取失敗")
        
    finally:
        pge_gripper.disconnect()
        print("\nPGE夾爪連接已斷開")
    
    # 測試PGC夾爪 (向後相容)
    print("\n--- PGC夾爪測試 ---")
    pgc_gripper = GripperHighLevelAPI(gripper_type=GripperType.PGC)
    
    try:
        if pgc_gripper.connected:
            print("✓ PGC夾爪連接成功")
            
            # 使用通用API
            if pgc_gripper.smart_grip(420):
                print("✓ PGC智能夾取成功")
                pgc_gripper.smart_release(50)
                print("✓ PGC智能釋放成功")
        
    finally:
        pgc_gripper.disconnect()


#if __name__ == "__main__":
#    example_usage()