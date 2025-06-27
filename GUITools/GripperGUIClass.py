# -*- coding: utf-8 -*-
"""
GripperGUIClass.py - Gripper夾爪模組GUI控制類別
專為統一機台調適工具設計，提供完整的三款夾爪操作介面
基於ModbusTCP Client架構，支援PGC、PGHL、PGE三種夾爪型號
"""

import time
import threading
from typing import Optional, Dict, Any, List, Callable
from dataclasses import dataclass
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


# ==================== 枚舉定義 ====================
class GripperType(IntEnum):
    """夾爪類型枚舉"""
    PGC = 6      # unit_id=6
    PGHL = 5     # unit_id=5
    PGE = 4      # unit_id=4


class GripperCommand(IntEnum):
    """夾爪控制指令枚舉"""
    NOP = 0              # 無操作
    INITIALIZE = 1       # 初始化/回零
    STOP = 2             # 停止當前動作
    ABSOLUTE_POSITION = 3 # 絕對位置移動
    RELATIVE_POSITION = 4 # 相對位置移動(僅PGHL)
    SET_FORCE = 5        # 設定力道
    SET_SPEED = 6        # 設定速度
    QUICK_OPEN = 7       # 快速開啟
    QUICK_CLOSE = 8      # 快速關閉


class GripperState(IntEnum):
    """夾爪狀態枚舉"""
    OFFLINE = 0          # 離線
    ONLINE = 1           # 在線


class ConnectionState(IntEnum):
    """連接狀態枚舉"""
    DISCONNECTED = 0     # 斷開
    CONNECTED = 1        # 已連接


class InitState(IntEnum):
    """初始化狀態枚舉"""
    NOT_INITIALIZED = 0  # 未初始化
    SUCCESS = 1          # 成功
    IN_PROGRESS = 2      # 進行中


class MotionState(IntEnum):
    """運動狀態枚舉"""
    MOVING = 0           # 運動中
    REACHED = 1          # 到達
    GRIPPED = 2          # 夾住/堵轉
    DROPPED = 3          # 掉落


# ==================== 數據結構 ====================
@dataclass
class GripperStatus:
    """單個夾爪狀態"""
    gripper_type: GripperType
    module_state: int = 0        # 模組狀態
    connection_state: int = 0    # 連接狀態
    device_state: int = 0        # 設備狀態(初始化狀態)
    error_count: int = 0         # 錯誤計數
    motion_state: int = 0        # 夾持/運動狀態
    current_position: int = 0    # 當前位置
    current_value: int = 0       # 電流值(僅PGHL)
    timestamp: int = 0           # 時間戳
    
    # 連接狀態判斷
    @property
    def is_online(self) -> bool:
        return self.module_state == GripperState.ONLINE
    
    @property
    def is_connected(self) -> bool:
        return self.connection_state == ConnectionState.CONNECTED
    
    @property
    def is_initialized(self) -> bool:
        return self.device_state == InitState.SUCCESS
    
    @property
    def is_moving(self) -> bool:
        return self.motion_state == MotionState.MOVING


@dataclass
class GripperCommand_:
    """夾爪指令"""
    command_code: int = 0
    param1: int = 0
    param2: int = 0
    command_id: int = 0


@dataclass
class GripperConfig:
    """夾爪配置"""
    gripper_type: GripperType
    unit_id: int
    enabled: bool = True
    
    # 型號特定配置
    position_range: tuple = (0, 1000)    # 位置範圍
    force_range: tuple = (20, 100)       # 力道範圍
    speed_range: tuple = (1, 100)        # 速度範圍
    has_current_feedback: bool = False   # 是否有電流反饋
    supports_relative_move: bool = False # 是否支援相對移動
    position_precision: float = 1.0     # 位置精度


# ==================== GripperGUI控制類 ====================
class GripperGUIClass:
    """
    Gripper夾爪模組GUI控制類別
    
    功能:
    - Modbus TCP連接管理
    - 三款夾爪統一控制介面
    - 狀態監控
    - 指令執行
    - 參數設置
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化GripperGUI控制類
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        
        # 夾爪寄存器映射 (基地址500-589)
        self.GRIPPER_REGISTERS = {
            GripperType.PGC: {
                'status_base': 500,    # 500-519
                'command_base': 520,   # 520-529
                'unit_id': 6
            },
            GripperType.PGHL: {
                'status_base': 530,    # 530-549
                'command_base': 550,   # 550-559
                'unit_id': 5
            },
            GripperType.PGE: {
                'status_base': 560,    # 560-579
                'command_base': 580,   # 580-589
                'unit_id': 4
            }
        }
        
        # 夾爪配置
        self.gripper_configs = {
            GripperType.PGC: GripperConfig(
                gripper_type=GripperType.PGC,
                unit_id=6,
                position_range=(0, 1000),
                force_range=(20, 100),
                speed_range=(1, 100),
                has_current_feedback=False,
                supports_relative_move=False,
                position_precision=1.0
            ),
            GripperType.PGHL: GripperConfig(
                gripper_type=GripperType.PGHL,
                unit_id=5,
                position_range=(0, 65535),
                force_range=(20, 100),
                speed_range=(50, 100),
                has_current_feedback=True,
                supports_relative_move=True,
                position_precision=0.01
            ),
            GripperType.PGE: GripperConfig(
                gripper_type=GripperType.PGE,
                unit_id=4,
                position_range=(0, 1000),
                force_range=(20, 100),
                speed_range=(1, 100),
                has_current_feedback=False,
                supports_relative_move=False,
                position_precision=1.0
            )
        }
        
        # 狀態回調函數
        self.status_callbacks: List[Callable[[Dict[GripperType, GripperStatus]], None]] = []
        
        # 內部狀態
        self.connected = False
        self.gripper_statuses: Dict[GripperType, GripperStatus] = {}
        self.command_id_counter = 1
        
        # 初始化夾爪狀態
        for gripper_type in GripperType:
            self.gripper_statuses[gripper_type] = GripperStatus(gripper_type=gripper_type)
        
        # 線程控制
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self._lock = threading.Lock()
        
        # 設置日誌
        self.logger = logging.getLogger("GripperGUIClass")
        self.logger.setLevel(logging.INFO)
    
    # ==================== 連接管理 ====================
    def connect(self) -> bool:
        """
        連接到Modbus TCP服務器
        
        Returns:
            bool: 連接是否成功
        """
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
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.logger.info("Modbus TCP連接成功")
                self._start_monitoring()
                return True
            else:
                self.logger.error("Modbus TCP連接失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        self._stop_monitoring()
        
        if self.modbus_client:
            self.modbus_client.close()
            self.modbus_client = None
        
        self.connected = False
        self.logger.info("已斷開Modbus連接")
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self.connected
    
    # ==================== 狀態監控 ====================
    def _start_monitoring(self):
        """開始狀態監控線程"""
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            return
        
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True
        )
        self.monitoring_thread.start()
        self.logger.info("狀態監控線程已啟動")
    
    def _stop_monitoring(self):
        """停止狀態監控線程"""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2.0)
        self.logger.info("狀態監控線程已停止")
    
    def _monitoring_loop(self):
        """狀態監控主循環"""
        while self.monitoring_active:
            try:
                self._update_all_gripper_status()
                time.sleep(0.2)  # 200ms更新間隔
            except Exception as e:
                self.logger.error(f"監控循環異常: {e}")
                time.sleep(1.0)
    
    def _update_all_gripper_status(self):
        """更新所有夾爪狀態"""
        if not self.modbus_client:
            return
        
        status_changed = False
        
        for gripper_type in GripperType:
            try:
                old_status = self.gripper_statuses[gripper_type]
                new_status = self._read_gripper_status(gripper_type)
                
                if new_status:
                    with self._lock:
                        self.gripper_statuses[gripper_type] = new_status
                    
                    # 檢查狀態是否有變化
                    if (old_status.module_state != new_status.module_state or
                        old_status.connection_state != new_status.connection_state or
                        old_status.device_state != new_status.device_state or
                        old_status.motion_state != new_status.motion_state):
                        status_changed = True
                        
            except Exception as e:
                self.logger.error(f"更新{gripper_type.name}狀態失敗: {e}")
        
        # 狀態變化時觸發回調
        if status_changed:
            self._notify_status_change()
    
    def _read_gripper_status(self, gripper_type: GripperType) -> Optional[GripperStatus]:
        """讀取單個夾爪狀態"""
        try:
            reg_info = self.GRIPPER_REGISTERS[gripper_type]
            response = self.modbus_client.read_holding_registers(
                address=reg_info['status_base'],
                count=15  # 讀取前15個狀態寄存器
            )
            
            if response.isError():
                return None
            
            registers = response.registers
            
            status = GripperStatus(
                gripper_type=gripper_type,
                module_state=registers[0],
                connection_state=registers[1],
                device_state=registers[2],
                error_count=registers[3],
                motion_state=registers[4],
                current_position=registers[5],
                current_value=registers[6] if len(registers) > 6 else 0,
                timestamp=registers[14] if len(registers) > 14 else int(time.time())
            )
            
            return status
            
        except Exception as e:
            self.logger.error(f"讀取{gripper_type.name}狀態異常: {e}")
            return None
    
    # ==================== 指令控制 ====================
    def initialize_gripper(self, gripper_type: GripperType) -> bool:
        """
        初始化/回零夾爪
        
        Args:
            gripper_type: 夾爪類型
            
        Returns:
            bool: 指令是否發送成功
        """
        return self._send_gripper_command(gripper_type, GripperCommand.INITIALIZE)
    
    def stop_gripper(self, gripper_type: GripperType) -> bool:
        """
        停止夾爪運動
        
        Args:
            gripper_type: 夾爪類型
            
        Returns:
            bool: 指令是否發送成功
        """
        return self._send_gripper_command(gripper_type, GripperCommand.STOP)
    
    def move_to_position(self, gripper_type: GripperType, position: int) -> bool:
        """
        移動到絕對位置
        
        Args:
            gripper_type: 夾爪類型
            position: 目標位置
            
        Returns:
            bool: 指令是否發送成功
        """
        config = self.gripper_configs[gripper_type]
        
        # 檢查位置範圍
        if not (config.position_range[0] <= position <= config.position_range[1]):
            self.logger.error(f"{gripper_type.name}位置超出範圍: {position}")
            return False
        
        return self._send_gripper_command(gripper_type, GripperCommand.ABSOLUTE_POSITION, position)
    
    def move_relative(self, gripper_type: GripperType, distance: int) -> bool:
        """
        相對位置移動 (僅PGHL支援)
        
        Args:
            gripper_type: 夾爪類型
            distance: 移動距離
            
        Returns:
            bool: 指令是否發送成功
        """
        config = self.gripper_configs[gripper_type]
        
        if not config.supports_relative_move:
            self.logger.error(f"{gripper_type.name}不支援相對移動")
            return False
        
        return self._send_gripper_command(gripper_type, GripperCommand.RELATIVE_POSITION, distance)
    
    def set_force(self, gripper_type: GripperType, force: int) -> bool:
        """
        設定夾持力道
        
        Args:
            gripper_type: 夾爪類型
            force: 力道值
            
        Returns:
            bool: 指令是否發送成功
        """
        config = self.gripper_configs[gripper_type]
        
        # 檢查力道範圍
        if not (config.force_range[0] <= force <= config.force_range[1]):
            self.logger.error(f"{gripper_type.name}力道超出範圍: {force}")
            return False
        
        return self._send_gripper_command(gripper_type, GripperCommand.SET_FORCE, force)
    
    def set_speed(self, gripper_type: GripperType, speed: int) -> bool:
        """
        設定移動速度
        
        Args:
            gripper_type: 夾爪類型
            speed: 速度值
            
        Returns:
            bool: 指令是否發送成功
        """
        config = self.gripper_configs[gripper_type]
        
        # 檢查速度範圍
        if not (config.speed_range[0] <= speed <= config.speed_range[1]):
            self.logger.error(f"{gripper_type.name}速度超出範圍: {speed}")
            return False
        
        return self._send_gripper_command(gripper_type, GripperCommand.SET_SPEED, speed)
    
    def quick_open(self, gripper_type: GripperType) -> bool:
        """
        快速開啟夾爪
        
        Args:
            gripper_type: 夾爪類型
            
        Returns:
            bool: 指令是否發送成功
        """
        return self._send_gripper_command(gripper_type, GripperCommand.QUICK_OPEN)
    
    def quick_close(self, gripper_type: GripperType) -> bool:
        """
        快速關閉夾爪
        
        Args:
            gripper_type: 夾爪類型
            
        Returns:
            bool: 指令是否發送成功
        """
        return self._send_gripper_command(gripper_type, GripperCommand.QUICK_CLOSE)
    
    def _send_gripper_command(self, gripper_type: GripperType, command: GripperCommand, 
                            param1: int = 0, param2: int = 0) -> bool:
        """發送夾爪指令"""
        if not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            reg_info = self.GRIPPER_REGISTERS[gripper_type]
            command_base = reg_info['command_base']
            
            # 生成新的指令ID
            with self._lock:
                self.command_id_counter += 1
                if self.command_id_counter > 65535:
                    self.command_id_counter = 1
                command_id = self.command_id_counter
            
            # 寫入指令寄存器
            values = [command.value, param1, param2, command_id]
            
            response = self.modbus_client.write_multiple_registers(
                address=command_base,
                values=values
            )
            
            if response.isError():
                self.logger.error(f"發送{gripper_type.name}指令失敗: {command}")
                return False
            
            self.logger.info(f"發送{gripper_type.name}指令成功: {command.name}({param1}, {param2})")
            return True
            
        except Exception as e:
            self.logger.error(f"發送{gripper_type.name}指令異常: {e}")
            return False
    
    # ==================== 狀態獲取 ====================
    def get_gripper_status(self, gripper_type: GripperType) -> GripperStatus:
        """獲取單個夾爪狀態"""
        with self._lock:
            return self.gripper_statuses[gripper_type]
    
    def get_all_gripper_status(self) -> Dict[GripperType, GripperStatus]:
        """獲取所有夾爪狀態"""
        with self._lock:
            return self.gripper_statuses.copy()
    
    def get_gripper_config(self, gripper_type: GripperType) -> GripperConfig:
        """獲取夾爪配置"""
        return self.gripper_configs[gripper_type]
    
    def get_all_gripper_configs(self) -> Dict[GripperType, GripperConfig]:
        """獲取所有夾爪配置"""
        return self.gripper_configs.copy()
    
    # ==================== 狀態描述方法 ====================
    def get_state_description(self, state_type: str, state_value: int) -> str:
        """獲取狀態描述"""
        descriptions = {
            'module_state': {
                0: "離線",
                1: "在線"
            },
            'connection_state': {
                0: "斷開",
                1: "已連接"
            },
            'device_state': {
                0: "未初始化",
                1: "初始化成功",
                2: "初始化中"
            },
            'motion_state': {
                0: "運動中",
                1: "到達",
                2: "夾住/堵轉",
                3: "掉落"
            }
        }
        
        return descriptions.get(state_type, {}).get(state_value, f"未知狀態({state_value})")
    
    def get_gripper_type_name(self, gripper_type: GripperType) -> str:
        """獲取夾爪類型名稱"""
        names = {
            GripperType.PGC: "PGC夾爪",
            GripperType.PGHL: "PGHL夾爪",
            GripperType.PGE: "PGE夾爪"
        }
        return names.get(gripper_type, f"未知夾爪({gripper_type})")
    
    def format_position(self, gripper_type: GripperType, position: int) -> str:
        """格式化位置顯示"""
        config = self.gripper_configs[gripper_type]
        if config.position_precision == 0.01:
            return f"{position * 0.01:.2f}mm"
        else:
            return f"{position}"
    
    def format_current(self, current_value: int) -> str:
        """格式化電流顯示"""
        return f"{current_value}mA"
    
    # ==================== 工具方法 ====================
    def is_gripper_ready(self, gripper_type: GripperType) -> bool:
        """檢查夾爪是否準備好操作"""
        status = self.get_gripper_status(gripper_type)
        return (status.is_online and 
                status.is_connected and 
                status.is_initialized and 
                not status.is_moving)
    
    def get_online_grippers(self) -> List[GripperType]:
        """獲取在線的夾爪列表"""
        online_grippers = []
        for gripper_type, status in self.gripper_statuses.items():
            if status.is_online:
                online_grippers.append(gripper_type)
        return online_grippers
    
    def get_ready_grippers(self) -> List[GripperType]:
        """獲取準備好的夾爪列表"""
        ready_grippers = []
        for gripper_type in GripperType:
            if self.is_gripper_ready(gripper_type):
                ready_grippers.append(gripper_type)
        return ready_grippers
    
    # ==================== 回調管理 ====================
    def add_status_callback(self, callback: Callable[[Dict[GripperType, GripperStatus]], None]):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[Dict[GripperType, GripperStatus]], None]):
        """移除狀態變化回調"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def _notify_status_change(self):
        """通知狀態變化"""
        for callback in self.status_callbacks:
            try:
                callback(self.gripper_statuses.copy())
            except Exception as e:
                self.logger.error(f"狀態回調異常: {e}")
    
    # ==================== 批量操作 ====================
    def initialize_all_grippers(self) -> Dict[GripperType, bool]:
        """初始化所有夾爪"""
        results = {}
        for gripper_type in GripperType:
            results[gripper_type] = self.initialize_gripper(gripper_type)
        return results
    
    def stop_all_grippers(self) -> Dict[GripperType, bool]:
        """停止所有夾爪"""
        results = {}
        for gripper_type in GripperType:
            results[gripper_type] = self.stop_gripper(gripper_type)
        return results
    
    def open_all_grippers(self) -> Dict[GripperType, bool]:
        """開啟所有夾爪"""
        results = {}
        for gripper_type in GripperType:
            results[gripper_type] = self.quick_open(gripper_type)
        return results
    
    def close_all_grippers(self) -> Dict[GripperType, bool]:
        """關閉所有夾爪"""
        results = {}
        for gripper_type in GripperType:
            results[gripper_type] = self.quick_close(gripper_type)
        return results
    
    # ==================== 資源清理 ====================
    def __del__(self):
        """析構函數，確保資源釋放"""
        self.disconnect()