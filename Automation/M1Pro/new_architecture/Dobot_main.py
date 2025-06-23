#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (新流程架構版)
實現三執行緒併行架構：運動控制、DIO控制、外部模組交握
支援統一指令佇列和優先權機制
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
from Dobot_Flow1_new import Flow1VisionPickExecutor
from Dobot_Flow2_new import Flow2UnloadExecutor  
from Dobot_Flow3_flip import FlowFlipStationExecutor

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

# ==================== Flow狀態 ====================

class FlowStatus(Enum):
    """Flow執行狀態"""
    IDLE = 0
    RUNNING = 1
    COMPLETED = 2
    ERROR = 3
    PAUSED = 4

@dataclass
class FlowResult:
    """Flow執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 0
    flow_data: Dict[str, Any] = None

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
    
    def __init__(self, robot, command_queue, state_machine, external_modules):
        super().__init__("MotionFlow", command_queue)
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
        # 建立Flow執行器
        self.flow_executors = {}
        self.current_flow = None
        
    def initialize_flows(self):
        """初始化Flow執行器"""
        try:
            # Flow1: VP視覺抓取
            flow1 = Flow1VisionPickExecutor()
            flow1.initialize(self.robot, self.state_machine, self.external_modules)
            self.flow_executors[1] = flow1
            
            # Flow2: 出料流程
            flow2 = Flow2UnloadExecutor()
            flow2.initialize(self.robot, self.state_machine, self.external_modules)
            self.flow_executors[2] = flow2
            
            print("運動Flow執行器初始化完成")
            return True
            
        except Exception as e:
            print(f"運動Flow執行器初始化失敗: {e}")
            return False
    
    def run(self):
        """執行緒主循環"""
        print("運動控制執行緒啟動")
        self.status = "運行中"
        
        while self.running:
            try:
                # 取得指令
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.MOTION:
                    self.status = "執行中"
                    self._execute_motion_command(command)
                    self.status = "運行中"
                    
            except Exception as e:
                self.last_error = f"運動控制執行緒錯誤: {e}"
                print(self.last_error)
                time.sleep(0.1)
                
        self.status = "停止"
        print("運動控制執行緒停止")
        
    def _execute_motion_command(self, command: Command):
        """執行運動指令"""
        cmd_data = command.command_data
        cmd_type = cmd_data.get('type')
        
        try:
            if cmd_type.startswith('flow'):
                # 執行Flow
                flow_id = int(cmd_type.replace('flow', ''))
                self._execute_flow(flow_id)
            elif cmd_type == 'move_j':
                self._execute_move_j(cmd_data)
            elif cmd_type == 'move_l':
                self._execute_move_l(cmd_data)
            elif cmd_type == 'set_speed':
                self._execute_set_speed(cmd_data)
            else:
                print(f"未知運動指令: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"執行運動指令失敗: {e}"
            print(self.last_error)
            
    def _execute_flow(self, flow_id: int):
        """執行指定Flow"""
        if flow_id not in self.flow_executors:
            print(f"Flow{flow_id}不存在")
            return
            
        flow = self.flow_executors[flow_id]
        self.current_flow = flow
        
        # 更新狀態機
        if self.state_machine:
            self.state_machine.write_register(2, flow_id)  # CURRENT_FLOW
            self.state_machine.write_register(0, 10)  # STATUS_REGISTER (Running=1)
        
        print(f"開始執行Flow{flow_id}...")
        result = flow.execute()
        
        # 處理結果
        if result.success:
            print(f"Flow{flow_id}執行成功，耗時: {result.execution_time:.2f}秒")
            if flow_id == 1 and self.state_machine:
                self.state_machine.write_register(20, 1)  # FLOW1_COMPLETE
        else:
            print(f"Flow{flow_id}執行失敗: {result.error_message}")
            if self.state_machine:
                self.state_machine.write_register(0, 12)  # STATUS_REGISTER (Alarm=1)
        
        # 清除狀態
        self.current_flow = None
        if self.state_machine:
            self.state_machine.write_register(2, 0)  # 清除CURRENT_FLOW
            self.state_machine.write_register(0, 9)  # STATUS_REGISTER (Ready=1)
            
    def _execute_move_j(self, cmd_data: Dict):
        """執行關節運動"""
        try:
            x = cmd_data.get('x', 0)
            y = cmd_data.get('y', 0)
            z = cmd_data.get('z', 0)
            r = cmd_data.get('r', 0)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.move_api.MovJ(x, y, z, r)
                print(f"MovJ({x}, {y}, {z}, {r}) -> {result}")
                
        except Exception as e:
            print(f"MovJ執行失敗: {e}")
            
    def _execute_move_l(self, cmd_data: Dict):
        """執行直線運動"""
        try:
            x = cmd_data.get('x', 0)
            y = cmd_data.get('y', 0)
            z = cmd_data.get('z', 0)
            r = cmd_data.get('r', 0)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.move_api.MovL(x, y, z, r)
                print(f"MovL({x}, {y}, {z}, {r}) -> {result}")
                
        except Exception as e:
            print(f"MovL執行失敗: {e}")
            
    def _execute_set_speed(self, cmd_data: Dict):
        """設定速度"""
        try:
            speed = cmd_data.get('speed', 50)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.SpeedFactor(speed)
                print(f"設定速度{speed}% -> {result}")
                
        except Exception as e:
            print(f"設定速度失敗: {e}")

# ==================== DIO控制執行緒 ====================

class DIOFlowThread(BaseFlowThread):
    """DIO控制執行緒"""
    
    def __init__(self, robot, command_queue, state_machine):
        super().__init__("DIOFlow", command_queue)
        self.robot = robot
        self.state_machine = state_machine
        
        # 建立翻轉站Flow執行器
        self.flip_station_executor = None
        
    def initialize_flows(self):
        """初始化DIO Flow執行器"""
        try:
            # Flow3: 翻轉站控制
            self.flip_station_executor = FlowFlipStationExecutor()
            self.flip_station_executor.initialize(self.robot, self.state_machine, {})
            
            print("DIO Flow執行器初始化完成")
            return True
            
        except Exception as e:
            print(f"DIO Flow執行器初始化失敗: {e}")
            return False
    
    def run(self):
        """執行緒主循環"""
        print("DIO控制執行緒啟動")
        self.status = "運行中"
        
        while self.running:
            try:
                # 取得指令
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.DIO:
                    self.status = "執行中"
                    self._execute_dio_command(command)
                    self.status = "運行中"
                    
            except Exception as e:
                self.last_error = f"DIO控制執行緒錯誤: {e}"
                print(self.last_error)
                time.sleep(0.1)
                
        self.status = "停止"
        print("DIO控制執行緒停止")
        
    def _execute_dio_command(self, command: Command):
        """執行DIO指令"""
        cmd_data = command.command_data
        cmd_type = cmd_data.get('type')
        
        try:
            if cmd_type == 'flow_flip_station':
                # 執行翻轉站Flow
                self._execute_flip_station_flow()
            elif cmd_type == 'set_do':
                self._execute_set_do(cmd_data)
            elif cmd_type == 'get_di':
                self._execute_get_di(cmd_data)
            elif cmd_type == 'pulse_do':
                self._execute_pulse_do(cmd_data)
            else:
                print(f"未知DIO指令: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"執行DIO指令失敗: {e}"
            print(self.last_error)
            
    def _execute_flip_station_flow(self):
        """執行翻轉站Flow"""
        if not self.flip_station_executor:
            print("翻轉站Flow執行器未初始化")
            return
            
        print("開始執行翻轉站Flow...")
        result = self.flip_station_executor.execute()
        
        if result.success:
            print(f"翻轉站Flow執行成功，耗時: {result.execution_time:.2f}秒")
        else:
            print(f"翻轉站Flow執行失敗: {result.error_message}")
            
    def _execute_set_do(self, cmd_data: Dict):
        """設定數位輸出"""
        try:
            pin = cmd_data.get('pin', 1)
            value = cmd_data.get('value', 0)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.DO(pin, value)
                print(f"DO({pin}, {value}) -> {result}")
                
        except Exception as e:
            print(f"設定DO失敗: {e}")
            
    def _execute_get_di(self, cmd_data: Dict):
        """讀取數位輸入"""
        try:
            pin = cmd_data.get('pin', 1)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.DI(pin)
                print(f"DI({pin}) -> {result}")
                return result
                
        except Exception as e:
            print(f"讀取DI失敗: {e}")
            return None
            
    def _execute_pulse_do(self, cmd_data: Dict):
        """脈衝輸出"""
        try:
            pin = cmd_data.get('pin', 1)
            pulse_width = cmd_data.get('pulse_width', 100)  # 毫秒
            
            if self.robot and self.robot.is_connected:
                # 設定為高
                self.robot.dashboard_api.DO(pin, 1)
                time.sleep(pulse_width / 1000.0)
                # 設定為低
                self.robot.dashboard_api.DO(pin, 0)
                print(f"脈衝DO({pin}, {pulse_width}ms)")
                
        except Exception as e:
            print(f"脈衝輸出失敗: {e}")

# ==================== 外部模組交握執行緒 ====================

class ExternalModuleThread(BaseFlowThread):
    """外部模組交握執行緒"""
    
    def __init__(self, command_queue, state_machine, external_modules):
        super().__init__("ExternalModule", command_queue)
        self.state_machine = state_machine
        self.external_modules = external_modules
        
    def run(self):
        """執行緒主循環"""
        print("外部模組交握執行緒啟動")
        self.status = "運行中"
        
        while self.running:
            try:
                # 取得指令
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.EXTERNAL:
                    self.status = "執行中"
                    self._execute_external_command(command)
                    self.status = "運行中"
                    
            except Exception as e:
                self.last_error = f"外部模組執行緒錯誤: {e}"
                print(self.last_error)
                time.sleep(0.1)
                
        self.status = "停止"
        print("外部模組交握執行緒停止")
        
    def _execute_external_command(self, command: Command):
        """執行外部模組指令"""
        cmd_data = command.command_data
        module_name = cmd_data.get('module')
        operation = cmd_data.get('operation')
        params = cmd_data.get('params', {})
        
        try:
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
        # 這裡可以根據不同模組實現不同的操作
        # 暫時返回True表示操作成功
        return True

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
        
    def _load_config(self) -> Dict[str, Any]:
        """載入配置"""
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        else:
            # 預設配置
            default_config = {
                "robot": {
                    "ip": "192.168.1.6",
                    "dashboard_port": 29999,
                    "move_port": 30003
                },
                "modbus": {
                    "server_ip": "127.0.0.1",
                    "server_port": 502,
                    "base_address": 400
                },
                "command_queue": {
                    "max_size": 100,
                    "timeout": 10.0
                },
                "threading": {
                    "handshake_interval": 0.05,
                    "status_update_interval": 1.0
                }
            }
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
                
            return default_config
    
    def initialize(self) -> bool:
        """初始化控制器"""
        print("初始化Dobot併行控制器...")
        
        # 初始化機械臂連接
        if not self._initialize_robot():
            return False
            
        # 初始化Modbus連接
        if not self._initialize_modbus():
            return False
            
        # 初始化狀態機
        self._initialize_state_machine()
        
        # 初始化外部模組
        self._initialize_external_modules()
        
        # 初始化執行緒
        if not self._initialize_threads():
            return False
            
        print("Dobot併行控制器初始化完成")
        return True
        
    def _initialize_robot(self) -> bool:
        """初始化機械臂連接 - 使用實際dobot_api"""
        try:
            robot_config = self.config["robot"]
            
            from dobot_api import DobotApiDashboard, DobotApiMove
            
            class RealRobot:
                def __init__(self, ip, dashboard_port, move_port):
                    self.ip = ip
                    self.dashboard_port = dashboard_port
                    self.move_port = move_port
                    self.is_connected = False
                    self.dashboard_api = None
                    self.move_api = None
                    
                def initialize(self):
                    """初始化機械臂連接"""
                    try:
                        self.dashboard_api = DobotApiDashboard(self.ip, self.dashboard_port)
                        self.move_api = DobotApiMove(self.ip, self.move_port)
                        
                        # 機械臂初始化設置
                        self.dashboard_api.ClearError()
                        self.dashboard_api.EnableRobot()
                        
                        self.is_connected = True
                        print(f"✓ 機械臂連接成功: {self.ip}")
                        return True
                        
                    except Exception as e:
                        print(f"✗ 機械臂初始化失敗: {e}")
                        self.is_connected = False
                        return False
            
            self.robot = RealRobot(
                robot_config["ip"],
                robot_config["dashboard_port"],
                robot_config["move_port"]
            )
            
            # 初始化機械臂連接
            if self.robot.initialize():
                return True
            else:
                print("機械臂初始化失敗")
                return False
            
        except ImportError as e:
            print(f"✗ dobot_api導入失敗: {e}")
            return False
        except Exception as e:
            print(f"✗ 機械臂初始化異常: {e}")
            return False
            
    def _initialize_modbus(self) -> bool:
        """初始化Modbus連接 - 實際ModbusTCP Client"""
        try:
            modbus_config = self.config["modbus"]
            
            print(f"正在連接實際ModbusTCP服務器: {modbus_config['server_ip']}:{modbus_config['server_port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=modbus_config["server_ip"],
                port=modbus_config["server_port"],
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                print(f"✓ 實際ModbusTCP連接成功: {modbus_config['server_ip']}:{modbus_config['server_port']}")
                return True
            else:
                print(f"✗ ModbusTCP連接失敗: {modbus_config['server_ip']}:{modbus_config['server_port']}")
                return False
            
        except Exception as e:
            print(f"ModbusTCP連接異常: {e}")
            return False
            
    def _initialize_state_machine(self):
        """初始化狀態機 - 實際Modbus操作"""
        class StateMachine:
            def __init__(self, modbus_client, base_address):
                self.modbus_client = modbus_client
                self.base_address = base_address
                
            def read_register(self, offset):
                try:
                    if self.modbus_client and hasattr(self.modbus_client, 'connect'):
                        result = self.modbus_client.read_holding_registers(
                            address=self.base_address + offset, 
                            count=1
                        )
                        if hasattr(result, 'registers') and len(result.registers) > 0:
                            return result.registers[0]
                    return 0
                except Exception as e:
                    print(f"讀取寄存器[{self.base_address + offset}]失敗: {e}")
                    return 0
                
            def write_register(self, offset, value):
                try:
                    if self.modbus_client and hasattr(self.modbus_client, 'connect'):
                        result = self.modbus_client.write_register(
                            address=self.base_address + offset, 
                            value=value
                        )
                        print(f"寫入寄存器[{self.base_address + offset}] = {value}")
                        return not (hasattr(result, 'isError') and result.isError())
                    else:
                        print(f"模擬寫入寄存器[{self.base_address + offset}] = {value}")
                        return True
                except Exception as e:
                    print(f"寫入寄存器[{self.base_address + offset}]失敗: {e}")
                    return False
                
        self.state_machine = StateMachine(
            self.modbus_client,
            self.config["modbus"]["base_address"]
        )
        
    def _initialize_external_modules(self):
        """初始化外部模組"""
        try:
            # 嘗試初始化PGE夾爪
            try:
                from GripperHighLevel_PGE_Enhanced import GripperHighLevelAPI, GripperType
                
                print("正在初始化PGE夾爪模組...")
                pge_gripper = GripperHighLevelAPI(
                    gripper_type=GripperType.PGE,
                    modbus_host="127.0.0.1",
                    modbus_port=502
                )
                
                if pge_gripper.connected and pge_gripper.initialized:
                    self.external_modules['PGE_GRIPPER'] = pge_gripper
                    print("✓ PGE夾爪模組已連接並初始化")
                else:
                    print("⚠️ PGE夾爪連接失敗，將使用模擬模式")
                    self.external_modules['PGE_GRIPPER'] = None
                    
            except ImportError as e:
                print(f"⚠️ PGE夾爪模組導入失敗: {e}")
                self.external_modules['PGE_GRIPPER'] = None
            except Exception as e:
                print(f"⚠️ PGE夾爪初始化異常: {e}")
                self.external_modules['PGE_GRIPPER'] = None
            
            # 其他外部模組 (暫時設為None)
            self.external_modules.update({
                'CCD1': None,
                'VP': None,
                'GRIPPER': None  # 原有夾爪
            })
            
            print(f"外部模組初始化完成: {list(self.external_modules.keys())}")
            
        except Exception as e:
            print(f"外部模組初始化失敗: {e}")
            # 確保至少有基本的外部模組字典
            self.external_modules = {
                'PGE_GRIPPER': None,
                'CCD1': None,
                'VP': None,
                'GRIPPER': None
            }
        
    def _initialize_threads(self) -> bool:
        """初始化執行緒"""
        try:
            # 運動控制執行緒
            self.motion_thread = MotionFlowThread(
                self.robot,
                self.command_queue,
                self.state_machine,
                self.external_modules
            )
            
            # DIO控制執行緒
            self.dio_thread = DIOFlowThread(
                self.robot,
                self.command_queue,
                self.state_machine
            )
            
            # 外部模組交握執行緒
            self.external_thread = ExternalModuleThread(
                self.command_queue,
                self.state_machine,
                self.external_modules
            )
            
            # 初始化Flow執行器
            if not self.motion_thread.initialize_flows():
                return False
                
            if not self.dio_thread.initialize_flows():
                return False
                
            print("所有執行緒初始化完成")
            return True
            
        except Exception as e:
            print(f"執行緒初始化失敗: {e}")
            return False
    
    def start(self) -> bool:
        """啟動控制器"""
        if not self.initialize():
            return False
            
        self.running = True
        
        # 啟動執行緒
        self.motion_thread.start_thread()
        self.dio_thread.start_thread()
        self.external_thread.start_thread()
        
        # 啟動握手循環
        self.handshake_thread = threading.Thread(target=self._handshake_loop, daemon=True)
        self.handshake_thread.start()
        
        print("Dobot併行控制器啟動成功")
        return True
        
    def stop(self):
        """停止控制器"""
        print("停止Dobot併行控制器...")
        
        self.running = False
        
        # 停止執行緒
        if self.motion_thread:
            self.motion_thread.stop_thread()
        if self.dio_thread:
            self.dio_thread.stop_thread()
        if self.external_thread:
            self.external_thread.stop_thread()
            
        print("Dobot併行控制器已停止")
        
    def _handshake_loop(self):
        """主要握手循環"""
        print("握手循環啟動")
        
        while self.running:
            try:
                self._process_control_registers()
                time.sleep(self.config["threading"]["handshake_interval"])
                
            except Exception as e:
                print(f"握手循環錯誤: {e}")
                time.sleep(0.1)
                
        print("握手循環停止")
        
    def _process_control_registers(self):
        """處理控制寄存器"""
        try:
            # 讀取控制寄存器
            vp_control = self.state_machine.read_register(40)      # 440: VP控制
            unload_control = self.state_machine.read_register(41)  # 441: 出料控制
            flip_control = self.state_machine.read_register(48)    # 448: 翻轉站控制
            
            # 處理VP控制 (Flow1)
            if vp_control == 1 and self.last_vp_control != vp_control:
                print("收到VP視覺取料指令，分派到運動控制執行緒")
                command = Command(
                    command_type=CommandType.MOTION,
                    command_data={'type': 'flow1'},
                    priority=CommandPriority.MOTION
                )
                self.command_queue.put_command(command)
                self.last_vp_control = vp_control
                
            elif vp_control == 0 and self.last_vp_control == 1:
                print("VP控制指令已清零")
                self.last_vp_control = 0
                
            # 處理出料控制 (Flow2)
            if unload_control == 1 and self.last_unload_control != unload_control:
                print("收到出料指令，分派到運動控制執行緒")
                command = Command(
                    command_type=CommandType.MOTION,
                    command_data={'type': 'flow2'},
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
                
        except Exception as e:
            print(f"處理控制寄存器失敗: {e}")
            
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