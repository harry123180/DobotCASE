#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (新流程架構版 - 移除模擬代碼修正版)
實現三執行緒併行架構：運動控制、DIO控制、外部模組交握
支援統一指令佇列和優先權機制
禁止任何模擬代碼，全部使用真實API連接
"""

import json
import os
import time
import threading
import traceback
import queue
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum, IntEnum
import logging

# 導入新流程架構模組
from Dobot_Flow1 import Flow1VisionPickExecutor
from Dobot_Flow2 import Flow2UnloadExecutor  
from Dobot_Flow3 import FlowFlipStationExecutor
from Dobot_Flow4 import Flow4VibrationFeedExecutor

# 導入高階API模組
from CCD1HighLevel import CCD1HighLevelAPI
# 使用修正後的GripperHighLevel
from GripperHighLevel import GripperHighLevelAPI, GripperType
from AngleHighLevel import AngleHighLevel

from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove

# 配置常數
DOBOT_BASE_ADDR = 400
CONFIG_FILE = "dobot_config.json"

# ==================== 指令系統 ====================

class CommandType(Enum):
    """指令類型"""
    MOTION = "motion"
    DIO = "dio"
    EXTERNAL = "external"
    EMERGENCY = "emergency"

class CommandPriority(IntEnum):
    """指令優先權 (數值越小優先權越高)"""
    EMERGENCY = 0    # 緊急停止
    MOTION = 1       # 運動指令
    DIO = 2          # DIO指令
    EXTERNAL = 3     # 外部模組指令

@dataclass
class Command:
    """統一指令格式"""
    command_type: CommandType
    command_data: Dict[str, Any]
    priority: CommandPriority
    timestamp: float = field(default_factory=time.time)
    command_id: int = field(default=0)
    callback: Optional[callable] = None

    def __lt__(self, other):
        """優先權比較 (for PriorityQueue)"""
        if self.priority != other.priority:
            return self.priority < other.priority
        return self.timestamp < other.timestamp

class CommandQueue:
    """統一指令佇列"""
    
    def __init__(self, max_size: int = 100):
        self.queue = queue.PriorityQueue(max_size)
        self.command_id_counter = 1
        self._lock = threading.Lock()
        
    def put_command(self, command: Command) -> bool:
        """加入指令到佇列"""
        try:
            with self._lock:
                command.command_id = self.command_id_counter
                self.command_id_counter += 1
                
            self.queue.put_nowait(command)
            return True
        except queue.Full:
            print(f"指令佇列已滿，丟棄指令: {command.command_type}")
            return False
            
    def get_command(self, timeout: Optional[float] = None) -> Optional[Command]:
        """取得指令"""
        try:
            return self.queue.get(timeout=timeout)
        except queue.Empty:
            return None
            
    def size(self) -> int:
        """取得佇列大小"""
        return self.queue.qsize()

# ==================== 機械臂狀態機 ====================

class DobotStateMachine:
    """機械臂狀態機管理"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.base_addr = DOBOT_BASE_ADDR
        
    def _parse_api_response(self, response: str) -> bool:
        """解析API響應，檢查是否成功"""
        if not response:
            return False
        try:
            parts = response.strip().split(',')
            if len(parts) >= 1:
                error_code = int(parts[0])
                return error_code == 0
            return False
        except (ValueError, IndexError):
            return False
        
    def set_ready(self, ready: bool = True):
        """設置Ready狀態 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(address=self.base_addr, count=1)
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取狀態寄存器失敗: {result}")
                return
                
            current = result.registers[0]
            if ready:
                current |= 0x01  # bit0 = 1
            else:
                current &= ~0x01  # bit0 = 0
            self.modbus_client.write_register(address=self.base_addr, value=current)
        except Exception as e:
            print(f"設置Ready狀態失敗: {e}")
            
    def set_running(self, running: bool = True):
        """設置Running狀態 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(address=self.base_addr, count=1)
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取狀態寄存器失敗: {result}")
                return
                
            current = result.registers[0]
            if running:
                current |= 0x02  # bit1 = 1
                current &= ~0x01  # Ready = 0
            else:
                current &= ~0x02  # bit1 = 0
            self.modbus_client.write_register(address=self.base_addr, value=current)
        except Exception as e:
            print(f"設置Running狀態失敗: {e}")
            
    def set_alarm(self, alarm: bool = True):
        """設置Alarm狀態 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(address=self.base_addr, count=1)
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取狀態寄存器失敗: {result}")
                return
                
            current = result.registers[0]
            if alarm:
                current |= 0x04  # bit2 = 1
                current &= ~0x03  # Ready和Running = 0
            else:
                current &= ~0x04  # bit2 = 0
            self.modbus_client.write_register(address=self.base_addr, value=current)
        except Exception as e:
            print(f"設置Alarm狀態失敗: {e}")
            
    def set_current_flow(self, flow_id: int):
        """設置當前流程ID - PyModbus 3.9.2修正版"""
        try:
            self.modbus_client.write_register(address=self.base_addr + 2, value=flow_id)
        except Exception as e:
            print(f"設置流程ID失敗: {e}")
            
    def set_flow1_complete(self, complete: bool = True):
        """設置Flow1完成狀態 - PyModbus 3.9.2修正版"""
        try:
            value = 1 if complete else 0
            self.modbus_client.write_register(address=self.base_addr + 20, value=value)
        except Exception as e:
            print(f"設置Flow1完成狀態失敗: {e}")

# ==================== 真實機械臂控制器 ====================

class RealRobotController:
    """真實機械臂控制器 - 使用dobot_api"""
    
    def __init__(self, ip: str, dashboard_port: int = 29999, move_port: int = 30003):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.is_connected = False
        self.dashboard_api = None
        self.move_api = None
        self.global_speed = 50
        
    def _parse_api_response(self, response: str) -> bool:
        """解析API響應"""
        if not response:
            return False
        try:
            parts = response.strip().split(',')
            if len(parts) >= 1:
                error_code = int(parts[0])
                return error_code == 0
            return False
        except (ValueError, IndexError):
            return False
        
    def initialize(self) -> bool:
        """初始化機械臂連接"""
        try:
            self.dashboard_api = DobotApiDashboard(self.ip, self.dashboard_port)
            self.move_api = DobotApiMove(self.ip, self.move_port)
            
            # 機械臂初始化設置
            clear_result = self.dashboard_api.ClearError()
            if self._parse_api_response(clear_result):
                print("✓ 清除錯誤成功")
            else:
                print(f"清除錯誤失敗: {clear_result}")
                
            enable_result = self.dashboard_api.EnableRobot()
            if self._parse_api_response(enable_result):
                print("✓ 機械臂啟用成功")
            else:
                print(f"機械臂啟用失敗: {enable_result}")
                return False
            
            # 設定初始速度
            if self.set_global_speed(self.global_speed):
                print(f"✓ 初始速度設定成功: {self.global_speed}%")
            else:
                print(f"⚠️ 初始速度設定失敗")
            
            self.is_connected = True
            print(f"✓ 機械臂初始化成功: {self.ip}")
            return True
            
        except Exception as e:
            print(f"機械臂初始化失敗: {e}")
            return False
    
    def set_global_speed(self, speed_percent: int) -> bool:
        """設定全局速度"""
        try:
            if not 1 <= speed_percent <= 100:
                print(f"速度超出範圍: {speed_percent}")
                return False
                
            result = self.dashboard_api.SpeedFactor(speed_percent)
            success = self._parse_api_response(result)
            if success:
                self.global_speed = speed_percent
                print(f"✓ 全局速度設定成功: {speed_percent}%")
            else:
                print(f"全局速度設定失敗: {result}")
            return success
        except Exception as e:
            print(f"設定全局速度異常: {e}")
            return False
    
    def move_j(self, x: float, y: float, z: float, r: float) -> bool:
        """關節運動 - 等待完成並檢查回傳"""
        try:
            result = self.move_api.MovJ(x, y, z, r)
            success = self._parse_api_response(result)
            if success:
                # 等待運動完成 - 檢查機械臂狀態
                if self._wait_for_motion_complete():
                    print(f"✓ MovJ完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                    return True
                else:
                    print(f"✗ MovJ超時: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                    return False
            else:
                print(f"✗ MovJ指令失敗: {result}")
                return False
        except Exception as e:
            print(f"MovJ執行失敗: {e}")
            return False
    
    def move_l(self, x: float, y: float, z: float, r: float) -> bool:
        """直線運動 - 等待完成並檢查回傳"""
        try:
            result = self.move_api.MovL(x, y, z, r)
            success = self._parse_api_response(result)
            if success:
                # 等待運動完成 - 檢查機械臂狀態
                if self._wait_for_motion_complete():
                    print(f"✓ MovL完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                    return True
                else:
                    print(f"✗ MovL超時: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                    return False
            else:
                print(f"✗ MovL指令失敗: {result}")
                return False
        except Exception as e:
            print(f"MovL執行失敗: {e}")
            return False
    
    def _wait_for_motion_complete(self, timeout: float = 30.0) -> bool:
        """等待運動完成 - 檢查機械臂狀態"""
        try:
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # 檢查機械臂狀態
                result = self.dashboard_api.RobotMode()
                if self._parse_api_response(result):
                    # 解析回傳數據，檢查是否完成運動
                    # RobotMode回傳格式通常是 "0,mode,RobotMode;"
                    parts = result.strip().split(',')
                    if len(parts) >= 2:
                        try:
                            mode = int(parts[1])
                            # mode 5 通常表示運動完成(IDLE狀態)
                            if mode == 5:
                                return True
                        except ValueError:
                            pass
                
                time.sleep(0.1)  # 100ms檢查間隔
            
            return False  # 超時
            
        except Exception as e:
            print(f"等待運動完成檢查失敗: {e}")
            return False
    
    def set_do(self, pin: int, value: int) -> bool:
        """設定數位輸出"""
        try:
            result = self.dashboard_api.DOExecute(pin, value)
            return self._parse_api_response(result)
        except Exception as e:
            print(f"設定DO失敗: {e}")
            return False
    
    def get_di(self, pin: int) -> Optional[int]:
        """讀取數位輸入"""
        try:
            result = self.dashboard_api.DI(pin)
            if result:
                parts = result.strip().split(',')
                if len(parts) >= 2:
                    return int(parts[1])
            return None
        except Exception as e:
            print(f"讀取DI失敗: {e}")
            return None
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        try:
            result = self.dashboard_api.EmergencyStop()
            success = self._parse_api_response(result)
            if success:
                print("✓ 緊急停止執行成功")
            else:
                print(f"緊急停止執行失敗: {result}")
            return success
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def disconnect(self) -> bool:
        """斷開機械臂連接"""
        try:
            if self.dashboard_api:
                disable_result = self.dashboard_api.DisableRobot()
                if self._parse_api_response(disable_result):
                    print("✓ 機械臂已停用")
                else:
                    print(f"⚠️ 機械臂停用失敗: {disable_result}")
                self.dashboard_api.close()
            if self.move_api:
                self.move_api.close()
            self.is_connected = False
            return True
        except Exception as e:
            print(f"機械臂斷開連接失敗: {e}")
            return False

# ==================== 執行緒基類 ====================

class BaseFlowThread(threading.Thread):
    """執行緒基類"""
    
    def __init__(self, name: str, command_queue: CommandQueue):
        super().__init__(daemon=True, name=name)
        self.command_queue = command_queue
        self.running = False
        self.status = "停止"
        self.last_error = ""
        self.operation_count = 0
        
    def start_thread(self):
        """啟動執行緒"""
        self.running = True
        self.start()
        
    def stop_thread(self):
        """停止執行緒"""
        self.running = False
        
    def get_status(self) -> Dict[str, Any]:
        """取得執行緒狀態"""
        return {
            'name': self.name,
            'running': self.running,
            'status': self.status,
            'last_error': self.last_error,
            'operation_count': self.operation_count
        }

# ==================== 運動控制執行緒 ====================

class MotionFlowThread(BaseFlowThread):
    """運動控制執行緒"""
    
    def __init__(self, robot: RealRobotController, command_queue: CommandQueue, 
                 state_machine: DobotStateMachine, external_modules: Dict):
        super().__init__("MotionFlow", command_queue)
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
        # 建立Flow執行器
        self.flow_executors = {}
        self.current_flow = None
        
    def initialize_flows(self):
        """初始化Flow執行器 - 使用真實API"""
        try:
            # Flow1: VP視覺抓取
            flow1 = Flow1VisionPickExecutor()
            flow1.initialize(self.robot, self.state_machine, self.external_modules)
            self.flow_executors[1] = flow1
            
            # Flow2: CV出料流程
            flow2 = Flow2UnloadExecutor()
            flow2.initialize(self.robot, self.state_machine, self.external_modules)
            self.flow_executors[2] = flow2
            
            print("✓ Flow執行器初始化完成")
            
        except Exception as e:
            print(f"Flow執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """運動控制執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動")
        
        while self.running:
            try:
                # 取得運動指令
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.MOTION:
                    self._handle_motion_command(command)
                    
            except Exception as e:
                self.last_error = f"運動控制執行緒錯誤: {e}"
                print(self.last_error)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _handle_motion_command(self, command: Command):
        """處理運動指令"""
        try:
            cmd_data = command.command_data
            cmd_type = cmd_data.get('type', '')
            
            if cmd_type == 'flow_vp_vision_pick':
                self._execute_flow1()
            elif cmd_type == 'flow_unload':
                self._execute_flow2()
            else:
                print(f"未知運動指令類型: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"處理運動指令失敗: {e}"
            print(self.last_error)
    
    def _execute_flow1(self):
        """執行Flow1 - VP視覺抓取"""
        try:
            print("開始執行Flow1 - VP視覺抓取")
            self.state_machine.set_running(True)
            self.state_machine.set_current_flow(1)
            
            flow1 = self.flow_executors.get(1)
            if flow1:
                result = flow1.execute()
                
                if result.success:
                    print("✓ Flow1執行成功")
                    self.state_machine.set_flow1_complete(True)
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
                    self.state_machine.set_ready(True)
                else:
                    print(f"✗ Flow1執行失敗: {result.error_message}")
                    self.state_machine.set_alarm(True)
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
            else:
                print("✗ Flow1執行器未初始化")
                self.state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"Flow1執行異常: {e}")
            self.state_machine.set_alarm(True)
            self.state_machine.set_running(False)
            self.state_machine.set_current_flow(0)
    
    def _execute_flow2(self):
        """執行Flow2 - CV出料流程"""
        try:
            print("開始執行Flow2 - CV出料流程")
            self.state_machine.set_running(True)
            self.state_machine.set_current_flow(2)
            
            flow2 = self.flow_executors.get(2)
            if flow2:
                result = flow2.execute()
                
                if result.success:
                    print("✓ Flow2執行成功")
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
                    self.state_machine.set_ready(True)
                else:
                    print(f"✗ Flow2執行失敗: {result.error_message}")
                    self.state_machine.set_alarm(True)
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
            else:
                print("✗ Flow2執行器未初始化")
                self.state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"Flow2執行異常: {e}")
            self.state_machine.set_alarm(True)
            self.state_machine.set_running(False)
            self.state_machine.set_current_flow(0)

# ==================== DIO控制執行緒 ====================

# ==================== DIO控制執行緒 ====================

class DIOControlThread(BaseFlowThread):
    """DIO控制執行緒"""
    
    def __init__(self, robot: RealRobotController, command_queue: CommandQueue):
        super().__init__("DIOControl", command_queue)
        self.robot = robot
        
    def initialize_flows(self):
        """初始化DIO Flow執行器"""
        try:
            # Flow3: 翻轉站控制
            flow3 = FlowFlipStationExecutor()
            flow3.initialize(self.robot, None, {})
            self.flow_executors[3] = flow3
            
            # Flow4: 震動投料控制
            flow4 = Flow4VibrationFeedExecutor()
            flow4.initialize(self.robot, None, {})
            self.flow_executors[4] = flow4
            
            print("✓ DIO Flow執行器初始化完成")
            
        except Exception as e:
            print(f"DIO Flow執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """DIO控制執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動")
        
        while self.running:
            try:
                # 取得DIO指令
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.DIO:
                    self._handle_dio_command(command)
                    
            except Exception as e:
                self.last_error = f"DIO控制執行緒錯誤: {e}"
                print(self.last_error)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _handle_dio_command(self, command: Command):
        """處理DIO指令"""
        try:
            cmd_data = command.command_data
            cmd_type = cmd_data.get('type', '')
            
            print(f"[DIO] 處理指令類型: {cmd_type}")
            
            if cmd_type == 'flow_flip_station':
                self._execute_flip_station()
            elif cmd_type == 'flow_vibration_feed':
                self._execute_vibration_feed()
            else:
                print(f"[DIO] 未知DIO指令類型: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"處理DIO指令失敗: {e}"
            print(f"[DIO] {self.last_error}")
            traceback.print_exc()
    
    def _execute_flip_station(self):
        """執行翻轉站控制 (Flow3)"""
        try:
            print("[DIO] 開始執行翻轉站控制 (Flow3)")
            
            # 修正：使用flow_executors[3]而不是flow_flip_executor
            flow3 = self.flow_executors.get(3)
            if flow3:
                print("[DIO] Flow3執行器已找到，開始執行...")
                result = flow3.execute()
                
                if result.success:
                    print("[DIO] ✓ 翻轉站控制執行成功")
                    print(f"[DIO] 耗時: {result.execution_time:.2f}秒")
                    print(f"[DIO] 完成步驟: {result.steps_completed}/{result.total_steps}")
                else:
                    print(f"[DIO] ✗ 翻轉站控制執行失敗: {result.error_message}")
                    print(f"[DIO] 完成步驟: {result.steps_completed}/{result.total_steps}")
            else:
                print("[DIO] ✗ Flow3翻轉站執行器未初始化")
                print(f"[DIO] 可用的flow_executors: {list(self.flow_executors.keys())}")
                
        except Exception as e:
            print(f"[DIO] 翻轉站控制執行異常: {e}")
            traceback.print_exc()

    def _execute_vibration_feed(self):
        """執行震動投料控制 (Flow4)"""
        try:
            print("[DIO] 開始執行震動投料控制 (Flow4)")
            
            # 使用flow_executors[4]
            flow4 = self.flow_executors.get(4)
            if flow4:
                print("[DIO] Flow4執行器已找到，開始執行...")
                result = flow4.execute()
                
                if result.success:
                    print("[DIO] ✓ 震動投料控制執行成功")
                    print(f"[DIO] 耗時: {result.execution_time:.2f}秒")
                    print(f"[DIO] 完成步驟: {result.steps_completed}/{result.total_steps}")
                else:
                    print(f"[DIO] ✗ 震動投料控制執行失敗: {result.error_message}")
                    print(f"[DIO] 完成步驟: {result.steps_completed}/{result.total_steps}")
            else:
                print("[DIO] ✗ Flow4震動投料執行器未初始化")
                print(f"[DIO] 可用的flow_executors: {list(self.flow_executors.keys())}")
                
        except Exception as e:
            print(f"[DIO] 震動投料控制執行異常: {e}")
            traceback.print_exc()

# ==================== 外部模組執行緒 ====================

class ExternalModuleThread(BaseFlowThread):
    """外部模組交握執行緒"""
    
    def __init__(self, command_queue: CommandQueue, external_modules: Dict):
        super().__init__("ExternalModule", command_queue)
        self.external_modules = external_modules
        
    def run(self):
        """外部模組執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動")
        
        while self.running:
            try:
                # 取得外部模組指令
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.EXTERNAL:
                    self._handle_external_command(command)
                    
            except Exception as e:
                self.last_error = f"外部模組執行緒錯誤: {e}"
                print(self.last_error)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _handle_external_command(self, command: Command):
        """處理外部模組指令"""
        try:
            cmd_data = command.command_data
            module_name = cmd_data.get('module', '')
            operation = cmd_data.get('operation', '')
            params = cmd_data.get('params', {})
            
            if module_name in self.external_modules:
                module = self.external_modules[module_name]
                success = self._handle_module_operation(module, module_name, operation, params)
                
                if success:
                    print(f"{module_name}.{operation} 執行成功")
                else:
                    print(f"{module_name}.{operation} 執行失敗")
                    
                self.operation_count += 1
            else:
                print(f"未知外部模組: {module_name}")
                
        except Exception as e:
            self.last_error = f"執行外部模組指令失敗: {e}"
            print(self.last_error)
            
    def _handle_module_operation(self, module, module_name: str, operation: str, params: Dict) -> bool:
        """處理模組操作"""
        try:
            # 根據模組類型執行相應操作
            if hasattr(module, operation):
                method = getattr(module, operation)
                if callable(method):
                    if params:
                        return method(**params)
                    else:
                        return method()
            return True
        except Exception as e:
            print(f"模組操作執行失敗: {e}")
            return False

# ==================== 主控制器 ====================

class DobotConcurrentController:
    """Dobot併行控制器"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        
        # 核心組件
        self.command_queue = CommandQueue(max_size=100)
        self.robot = None
        self.modbus_client = None
        self.state_machine = None
        
        # 執行緒
        self.motion_thread = None
        self.dio_thread = None
        self.external_thread = None
        self.handshake_thread = None
        
        # 狀態
        self.running = False
        self.external_modules = {}
        
        # 上次控制狀態
        self.last_vp_control = 0
        self.last_unload_control = 0
        self.last_flip_control = 0
        self.last_vibration_feed_control = 0  # 新增：Flow4震動投料控制
        
    def _load_config(self) -> Dict[str, Any]:
        """載入配置"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        # 預設配置
        default_config = {
            "robot": {
                "ip": "192.168.1.6",
                "dashboard_port": 29999,
                "move_port": 30003,
                "default_speed": 50,
                "default_acceleration": 50,
                "enable_collision_detection": True,
                "collision_level": 3
            },
            "modbus": {
                "server_ip": "127.0.0.1",
                "server_port": 502,
                "robot_base_address": 400,
                "timeout": 3.0
            },
            "gripper": {
                "type": "PGE",  # 修正：Flow3需要PGE夾爪
                "enabled": True,
                "base_address": 520,
                "status_address": 500,
                "default_force": 50,
                "default_speed": 80
            },
            "vision": {
                "ccd1_base_address": 200,
                "ccd3_base_address": 800,
                "detection_timeout": 10.0,
                "ccd1_enabled": True,
                "ccd3_enabled": False
            },
            "flows": {
                "flow1_enabled": True,
                "flow2_enabled": True,
                "flow3_enabled": False
            },
            "safety": {
                "enable_emergency_stop": True,
                "max_error_count": 5,
                "auto_recovery": False
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    # 合併配置
                    self._deep_update(default_config, user_config)
            except Exception as e:
                print(f"載入配置檔案失敗，使用預設配置: {e}")
        else:
            # 創建預設配置檔案
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"創建預設配置檔案: {config_path}")
            except Exception as e:
                print(f"創建配置檔案失敗: {e}")
                
        return default_config
    
    def _deep_update(self, base_dict: Dict, update_dict: Dict):
        """深度更新字典"""
        for key, value in update_dict.items():
            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def start(self) -> bool:
        """啟動控制器"""
        print("=== 啟動Dobot併行控制器 ===")
        
        if not self._initialize_robot():
            return False
            
        if not self._initialize_modbus():
            return False
            
        self._initialize_state_machine()
        self._initialize_external_modules()
        
        if not self._initialize_threads():
            return False
            
        self.running = True
        self._start_handshake_loop()
        
        print("✓ Dobot併行控制器啟動成功")
        return True
    
    def _initialize_robot(self) -> bool:
        """初始化機械臂連接 - 使用真實dobot_api"""
        try:
            robot_config = self.config["robot"]
            
            self.robot = RealRobotController(
                robot_config["ip"],
                robot_config["dashboard_port"],
                robot_config["move_port"]
            )
            
            if not self.robot.initialize():
                return False
                
            print("✓ 機械臂控制器初始化成功")
            return True
            
        except Exception as e:
            print(f"✗ 機械臂初始化失敗: {e}")
            return False
    
    def _initialize_modbus(self) -> bool:
        """初始化Modbus連接 - PyModbus 3.9.2修正版"""
        try:
            modbus_config = self.config["modbus"]
            self.modbus_client = ModbusTcpClient(
                host=modbus_config["server_ip"],
                port=modbus_config["server_port"],
                timeout=modbus_config["timeout"]
            )
            
            if self.modbus_client.connect():
                print("✓ Modbus客戶端連接成功")
                return True
            else:
                print("✗ Modbus客戶端連接失敗")
                return False
                
        except Exception as e:
            print(f"✗ Modbus初始化失敗: {e}")
            return False
    
    def _initialize_state_machine(self):
        """初始化狀態機"""
        self.state_machine = DobotStateMachine(self.modbus_client)
        self.state_machine.set_ready(True)
        print("✓ 狀態機初始化完成")
    
    def _initialize_external_modules(self):
        """初始化外部模組 - 修正API參數"""
        try:
            # 初始化CCD1高階API - 使用正確的參數
            if self.config["vision"]["ccd1_enabled"]:
                try:
                    ccd1_api = CCD1HighLevelAPI(
                        modbus_host=self.config["modbus"]["server_ip"],
                        modbus_port=self.config["modbus"]["server_port"]
                    )
                    if ccd1_api.connected:
                        self.external_modules['ccd1'] = ccd1_api
                        print("✓ CCD1高階API連接成功")
                    else:
                        print("⚠️ CCD1高階API連接失敗")
                except Exception as e:
                    print(f"⚠️ CCD1高階API初始化失敗: {e}")
            
            # 初始化夾爪高階API - 使用修正版本
            if self.config["gripper"]["enabled"]:
                try:
                    # 使用修正後的GripperHighLevel模組
                    # 根據配置選擇夾爪類型
                    gripper_type = GripperType.PGE if self.config["gripper"]["type"] == "PGE" else GripperType.PGC
                    
                    gripper_api = GripperHighLevelAPI(
                        gripper_type=gripper_type,
                        modbus_host=self.config["modbus"]["server_ip"],
                        modbus_port=self.config["modbus"]["server_port"]
                    )
                    if gripper_api.connected:
                        self.external_modules['gripper'] = gripper_api
                        print("✓ 夾爪高階API連接成功")
                    else:
                        print("⚠️ 夾爪高階API連接失敗")
                except Exception as e:
                    print(f"⚠️ 夾爪高階API初始化失敗: {e}")
            
            # 初始化角度校正API - 參數正確
            try:
                angle_api = AngleHighLevel(
                    host=self.config["modbus"]["server_ip"],
                    port=self.config["modbus"]["server_port"]
                )
                if angle_api.connect():
                    self.external_modules['angle'] = angle_api
                    print("✓ 角度校正API連接成功")
                else:
                    print("⚠️ 角度校正API連接失敗")
            except Exception as e:
                print(f"⚠️ 角度校正API初始化失敗: {e}")
                
            print("✓ 外部模組初始化完成")
            
        except Exception as e:
            print(f"外部模組初始化異常: {e}")
    
    def _initialize_threads(self) -> bool:
        """初始化執行緒 - 修正DIO執行緒初始化順序"""
        try:
            # 運動控制執行緒
            self.motion_thread = MotionFlowThread(
                self.robot, self.command_queue, self.state_machine, self.external_modules
            )
            self.motion_thread.initialize_flows()
            
            # DIO控制執行緒 - 修正：先創建，再調用initialize_flows
            self.dio_thread = DIOControlThread(self.robot, self.command_queue)
            
            # 確保屬性存在後再初始化Flow
            if not hasattr(self.dio_thread, 'flow_executors'):
                print("警告：DIO執行緒缺少flow_executors屬性，手動添加")
                self.dio_thread.flow_executors = {}
            
            # 調用初始化
            self.dio_thread.initialize_flows()
            
            # 外部模組執行緒
            self.external_thread = ExternalModuleThread(self.command_queue, self.external_modules)
            
            # 啟動執行緒
            self.motion_thread.start_thread()
            self.dio_thread.start_thread()
            self.external_thread.start_thread()
            
            print("✓ 執行緒初始化完成")
            return True
            
        except Exception as e:
            print(f"✗ 執行緒初始化失敗: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _start_handshake_loop(self):
        """啟動握手循環"""
        self.handshake_thread = threading.Thread(target=self._handshake_loop, daemon=True)
        self.handshake_thread.start()
        print("✓ 握手循環啟動")
    
    def _handshake_loop(self):
        """Modbus握手循環"""
        while self.running:
            try:
                self._process_control_registers()
                time.sleep(0.05)  # 50ms循環
                
            except Exception as e:
                print(f"握手循環錯誤: {e}")
                time.sleep(1.0)
    
    def _process_control_registers(self):
        """處理控制寄存器 - PyModbus 3.9.2修正版"""
        try:
            # 讀取控制寄存器 - 修正API調用
            result = self.modbus_client.read_holding_registers(address=DOBOT_BASE_ADDR + 40, count=10)
            
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取控制寄存器失敗: {result}")
                return
                
            registers = result.registers
            
            vp_control = registers[0]           # 440: VP視覺取料控制
            unload_control = registers[1]       # 441: 出料控制
            flip_control = registers[7] if len(registers) > 7 else 0      # 447: 翻轉站控制
            vibration_feed_control = registers[8] if len(registers) > 8 else 0  # 448: 震動投料控制
            
            # 處理VP視覺取料控制 (Flow1 - 運動)
            if vp_control == 1 and self.last_vp_control != vp_control:
                print("收到VP視覺取料指令，分派到運動控制執行緒")
                command = Command(
                    command_type=CommandType.MOTION,
                    command_data={'type': 'flow_vp_vision_pick'},
                    priority=CommandPriority.MOTION
                )
                self.command_queue.put_command(command)
                self.last_vp_control = vp_control
                
            elif vp_control == 0 and self.last_vp_control == 1:
                print("VP視覺取料控制指令已清零")
                self.last_vp_control = 0
                
            # 處理出料控制 (Flow2 - 運動)
            if unload_control == 1 and self.last_unload_control != unload_control:
                print("收到出料指令，分派到運動控制執行緒")
                command = Command(
                    command_type=CommandType.MOTION,
                    command_data={'type': 'flow_unload'},
                    priority=CommandPriority.MOTION
                )
                self.command_queue.put_command(command)
                self.last_unload_control = unload_control
                
            elif unload_control == 0 and self.last_unload_control == 1:
                print("出料控制指令已清零")
                self.last_unload_control = 0
                
            # 處理翻轉站控制 (Flow3 - DIO)
            if flip_control == 1 and self.last_flip_control != flip_control:
                print("收到翻轉站指令，分派到DIO控制執行緒")
                command = Command(
                    command_type=CommandType.DIO,
                    command_data={'type': 'flow_flip_station'},
                    priority=CommandPriority.DIO
                )
                self.command_queue.put_command(command)
                self.last_flip_control = flip_control
                
            elif flip_control == 0 and self.last_flip_control == 1:
                print("翻轉站控制指令已清零")
                self.last_flip_control = 0
            
            # 處理震動投料控制 (Flow4 - DIO)
            if vibration_feed_control == 1 and self.last_vibration_feed_control != vibration_feed_control:
                print("收到震動投料指令，分派到DIO控制執行緒")
                command = Command(
                    command_type=CommandType.DIO,
                    command_data={'type': 'flow_vibration_feed'},
                    priority=CommandPriority.DIO
                )
                self.command_queue.put_command(command)
                self.last_vibration_feed_control = vibration_feed_control
                
            elif vibration_feed_control == 0 and self.last_vibration_feed_control == 1:
                print("震動投料控制指令已清零")
                self.last_vibration_feed_control = 0
                
        except Exception as e:
            print(f"處理控制寄存器失敗: {e}")
    
    def stop(self):
        """停止控制器"""
        print("\n=== 停止Dobot併行控制器 ===")
        
        self.running = False
        
        # 停止執行緒
        if self.motion_thread:
            self.motion_thread.stop_thread()
        if self.dio_thread:
            self.dio_thread.stop_thread()
        if self.external_thread:
            self.external_thread.stop_thread()
            
        # 斷開連接
        if self.robot:
            self.robot.disconnect()
        if self.modbus_client:
            self.modbus_client.close()
            
        # 斷開外部模組
        for name, module in self.external_modules.items():
            try:
                if hasattr(module, 'disconnect'):
                    module.disconnect()
            except Exception as e:
                print(f"斷開{name}失敗: {e}")
        
        print("✓ Dobot併行控制器已停止")
    
    def get_system_status(self) -> Dict[str, Any]:
        """取得系統狀態"""
        return {
            'running': self.running,
            'command_queue_size': self.command_queue.size(),
            'motion_thread': self.motion_thread.get_status() if self.motion_thread else None,
            'dio_thread': self.dio_thread.get_status() if self.dio_thread else None,
            'external_thread': self.external_thread.get_status() if self.external_thread else None
        }

# ==================== 主程序 ====================

def main():
    """主程序"""
    print("="*60)
    print("Dobot M1Pro 併行控制器啟動")
    print("新流程架構：三執行緒併行 + 統一指令佇列")
    print("移除所有模擬代碼，使用真實API連接")
    print("="*60)
    
    controller = DobotConcurrentController()
    
    try:
        if controller.start():
            print("\n系統運行中，按 Ctrl+C 停止...")
            
            # 主循環
            while True:
                time.sleep(1)
                
                # 每10秒輸出一次系統狀態
                if int(time.time()) % 10 == 0:
                    status = controller.get_system_status()
                    print(f"\n系統狀態: 佇列大小={status['command_queue_size']}")
                    
        else:
            print("控制器啟動失敗")
            
    except KeyboardInterrupt:
        print("\n\n收到停止信號...")
    except Exception as e:
        print(f"\n系統錯誤: {e}")
        traceback.print_exc()
    finally:
        controller.stop()
        print("程序結束")

if __name__ == "__main__":
    main()