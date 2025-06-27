# -*- coding: utf-8 -*-
"""
DobotM1ProGUIClass.py - Dobot M1Pro機械臂GUI控制類別
實現統一機台調適工具的Dobot M1Pro機械臂控制介面
基於新架構混合交握協議，支援運動類Flow和IO類Flow控制
運動類Flow: 基地址1200-1249，狀態機交握，序列化執行
IO類Flow: 地址447-449，專用佇列併行執行
"""

import time
import threading
from typing import Dict, Any, Optional, List, Callable, Tuple
from dataclasses import dataclass
from enum import Enum
from pymodbus.client import ModbusTcpClient


class FlowType(Enum):
    """Flow類型枚舉"""
    MOTION = "motion"   # 運動類Flow (Flow1, Flow2, Flow5)
    IO = "io"          # IO類Flow (Flow3, Flow4)


@dataclass
class DobotMotionStatus:
    """Dobot運動類狀態"""
    ready: bool = False             # Ready狀態 (bit0)
    running: bool = False           # Running狀態 (bit1)
    alarm: bool = False             # Alarm狀態 (bit2)
    initialized: bool = False       # Initialized狀態 (bit3)
    current_motion_flow: int = 0    # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
    motion_progress: int = 0        # 運動進度 (0-100%)
    motion_error_code: int = 0      # 運動錯誤碼
    flow1_complete: bool = False    # Flow1完成且角度校正成功
    flow2_complete: bool = False    # Flow2完成
    flow5_complete: bool = False    # Flow5完成
    motion_op_count: int = 0        # 運動操作計數
    motion_err_count: int = 0       # 運動錯誤計數
    motion_run_time: int = 0        # 運動系統運行時間(分鐘)


@dataclass
class DobotIOStatus:
    """Dobot IO類狀態"""
    flow3_active: bool = False      # Flow3翻轉站執行中
    flow4_active: bool = False      # Flow4震動投料執行中
    flow3_control: int = 0          # Flow3控制寄存器值
    flow4_control: int = 0          # Flow4控制寄存器值


@dataclass
class DobotSystemStatus:
    """Dobot系統整體狀態"""
    motion_status: DobotMotionStatus = None
    io_status: DobotIOStatus = None
    connected_to_server: bool = False
    timestamp: str = ""

    def __post_init__(self):
        if self.motion_status is None:
            self.motion_status = DobotMotionStatus()
        if self.io_status is None:
            self.io_status = DobotIOStatus()


@dataclass
class DobotOperationResult:
    """Dobot操作結果"""
    success: bool = False           # 操作成功標誌
    flow_type: FlowType = None      # Flow類型
    flow_id: int = 0               # Flow ID
    error_message: str = ""         # 錯誤訊息
    timestamp: str = ""             # 操作時間戳
    operation_type: str = ""        # 操作類型


class DobotM1ProGUIClass:
    """Dobot M1Pro機械臂GUI控制類別 - 新架構混合交握協議"""
    
    # 運動類寄存器 (基地址1200-1249) - 修正地址衝突版本
    MOTION_BASE_ADDRESS = 1200
    
    # IO類寄存器 (447-449) - 保持不變
    IO_BASE_ADDRESS = 447
    
    # 運動類寄存器映射
    MOTION_REGISTERS = {
        # 運動狀態寄存器 (1200-1219) - 只讀
        'motion_status': 1200,          # 運動狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
        'current_motion_flow': 1201,    # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        'motion_progress': 1202,        # 運動進度 (0-100百分比)
        'motion_error_code': 1203,      # 運動錯誤碼
        'flow1_complete': 1204,         # Flow1完成狀態 (0=未完成, 1=完成且角度校正成功)
        'flow2_complete': 1205,         # Flow2完成狀態
        'flow5_complete': 1206,         # Flow5完成狀態
        'motion_op_count': 1207,        # 運動操作計數
        'motion_err_count': 1208,       # 運動錯誤計數
        'motion_run_time': 1209,        # 運動系統運行時間(分鐘)
        
        # 運動控制寄存器 (1240-1249) - 讀寫
        'flow1_control': 1240,          # Flow1控制 (0=清空, 1=啟動VP視覺抓取)
        'flow2_control': 1241,          # Flow2控制 (0=清空, 1=啟動出料流程)
        'flow5_control': 1242,          # Flow5控制 (0=清空, 1=啟動機械臂運轉)
        'motion_clear_alarm': 1243,     # 運動清除警報 (0=無動作, 1=清除Alarm)
        'motion_emergency_stop': 1244,  # 運動緊急停止 (0=正常, 1=緊急停止)
    }
    
    # IO類寄存器映射
    IO_REGISTERS = {
        'flow3_control': 447,           # Flow3控制 (0=清空, 1=啟動翻轉站)
        'flow4_control': 448,           # Flow4控制 (0=清空, 1=啟動震動投料)
        'io_reserved': 449,             # 保留IO控制
    }
    
    # Flow功能映射
    FLOW_DESCRIPTIONS = {
        1: "VP視覺抓取流程",
        2: "CV出料流程",
        3: "翻轉站控制",
        4: "震動投料控制",
        5: "機械臂運轉流程"
    }
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502, unit_id: int = 1):
        """初始化Dobot控制器"""
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.unit_id = unit_id
        
        # Modbus TCP Client
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 狀態管理
        self.current_status = DobotSystemStatus()
        
        # 監控線程
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self.monitor_lock = threading.Lock()
        
        # 回調函數列表
        self.status_callbacks: List[Callable[[DobotSystemStatus], None]] = []
        self.result_callbacks: List[Callable[[DobotOperationResult], None]] = []
        
        # 所有寄存器
        self.all_registers = {**self.MOTION_REGISTERS, **self.IO_REGISTERS}
    
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
                    old_status = DobotSystemStatus(
                        motion_status=DobotMotionStatus(**self.current_status.motion_status.__dict__),
                        io_status=DobotIOStatus(**self.current_status.io_status.__dict__),
                        connected_to_server=self.current_status.connected_to_server
                    )
                    
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
                # 更新運動類狀態 (讀取1200-1209)
                motion_result = self.modbus_client.read_holding_registers(
                    address=self.MOTION_REGISTERS['motion_status'], 
                    count=10, 
                    unit=self.unit_id
                )
                
                if not motion_result.isError():
                    registers = motion_result.registers
                    
                    # 解析運動狀態寄存器 (1200)
                    motion_status_reg = registers[0]
                    self.current_status.motion_status.ready = bool(motion_status_reg & 0x01)
                    self.current_status.motion_status.running = bool(motion_status_reg & 0x02)
                    self.current_status.motion_status.alarm = bool(motion_status_reg & 0x04)
                    self.current_status.motion_status.initialized = bool(motion_status_reg & 0x08)
                    
                    # 更新其他運動狀態
                    self.current_status.motion_status.current_motion_flow = registers[1]
                    self.current_status.motion_status.motion_progress = registers[2]
                    self.current_status.motion_status.motion_error_code = registers[3]
                    self.current_status.motion_status.flow1_complete = bool(registers[4])
                    self.current_status.motion_status.flow2_complete = bool(registers[5])
                    self.current_status.motion_status.flow5_complete = bool(registers[6])
                    self.current_status.motion_status.motion_op_count = registers[7]
                    self.current_status.motion_status.motion_err_count = registers[8]
                    self.current_status.motion_status.motion_run_time = registers[9]
                
                # 更新IO類狀態 (讀取447-448)
                io_result = self.modbus_client.read_holding_registers(
                    address=self.IO_REGISTERS['flow3_control'], 
                    count=2, 
                    unit=self.unit_id
                )
                
                if not io_result.isError():
                    io_registers = io_result.registers
                    self.current_status.io_status.flow3_control = io_registers[0]
                    self.current_status.io_status.flow4_control = io_registers[1]
                    self.current_status.io_status.flow3_active = bool(io_registers[0])
                    self.current_status.io_status.flow4_active = bool(io_registers[1])
                
                self.current_status.connected_to_server = True
                self.current_status.timestamp = time.strftime("%H:%M:%S")
                
            except Exception as e:
                self.current_status.connected_to_server = False
    
    def _status_changed(self, old_status: DobotSystemStatus, new_status: DobotSystemStatus) -> bool:
        """檢查狀態是否變化"""
        # 檢查運動狀態變化
        motion_changed = (
            old_status.motion_status.ready != new_status.motion_status.ready or
            old_status.motion_status.running != new_status.motion_status.running or
            old_status.motion_status.alarm != new_status.motion_status.alarm or
            old_status.motion_status.current_motion_flow != new_status.motion_status.current_motion_flow or
            old_status.motion_status.motion_progress != new_status.motion_status.motion_progress or
            old_status.motion_status.flow1_complete != new_status.motion_status.flow1_complete or
            old_status.motion_status.flow2_complete != new_status.motion_status.flow2_complete or
            old_status.motion_status.flow5_complete != new_status.motion_status.flow5_complete
        )
        
        # 檢查IO狀態變化
        io_changed = (
            old_status.io_status.flow3_active != new_status.io_status.flow3_active or
            old_status.io_status.flow4_active != new_status.io_status.flow4_active
        )
        
        # 檢查連接狀態變化
        connection_changed = old_status.connected_to_server != new_status.connected_to_server
        
        return motion_changed or io_changed or connection_changed
    
    def get_status(self) -> DobotSystemStatus:
        """獲取當前狀態"""
        with self.monitor_lock:
            return DobotSystemStatus(
                motion_status=DobotMotionStatus(**self.current_status.motion_status.__dict__),
                io_status=DobotIOStatus(**self.current_status.io_status.__dict__),
                connected_to_server=self.current_status.connected_to_server,
                timestamp=self.current_status.timestamp
            )
    
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
    
    # 運動類Flow控制 (狀態機交握)
    def start_flow1_vp_vision_pick(self) -> bool:
        """啟動Flow1 VP視覺抓取流程 (運動類)"""
        return self._execute_motion_flow(1, "Flow1 VP視覺抓取")
    
    def start_flow2_cv_unload(self) -> bool:
        """啟動Flow2 CV出料流程 (運動類)"""
        return self._execute_motion_flow(2, "Flow2 CV出料")
    
    def start_flow5_assembly(self) -> bool:
        """啟動Flow5 機械臂運轉流程 (運動類)"""
        return self._execute_motion_flow(5, "Flow5 機械臂運轉")
    
    def _execute_motion_flow(self, flow_id: int, flow_name: str) -> bool:
        """執行運動類Flow (狀態機交握)"""
        if not self.is_connected():
            self._notify_result_callbacks(DobotOperationResult(
                success=False,
                flow_type=FlowType.MOTION,
                flow_id=flow_id,
                error_message="未連接到Modbus服務器",
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return False
        
        # 檢查運動系統是否Ready
        status = self.get_status()
        if not status.motion_status.ready or status.motion_status.running:
            self._notify_result_callbacks(DobotOperationResult(
                success=False,
                flow_type=FlowType.MOTION,
                flow_id=flow_id,
                error_message=f"運動系統未準備好 (Ready={status.motion_status.ready}, Running={status.motion_status.running})",
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return False
        
        # 寫入對應的Flow控制寄存器
        control_register = f"flow{flow_id}_control"
        if self.write_register(control_register, 1):
            self._notify_result_callbacks(DobotOperationResult(
                success=True,
                flow_type=FlowType.MOTION,
                flow_id=flow_id,
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return True
        else:
            self._notify_result_callbacks(DobotOperationResult(
                success=False,
                flow_type=FlowType.MOTION,
                flow_id=flow_id,
                error_message="寫入控制寄存器失敗",
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return False
    
    def clear_motion_flow(self, flow_id: int) -> bool:
        """清空運動類Flow控制寄存器"""
        control_register = f"flow{flow_id}_control"
        return self.write_register(control_register, 0)
    
    def clear_motion_alarm(self) -> bool:
        """清除運動類警報"""
        return self.write_register('motion_clear_alarm', 1)
    
    def motion_emergency_stop(self) -> bool:
        """運動類緊急停止"""
        return self.write_register('motion_emergency_stop', 1)
    
    # IO類Flow控制 (併行執行)
    def start_flow3_flip_station(self) -> bool:
        """啟動Flow3 翻轉站控制 (IO類併行)"""
        return self._execute_io_flow(3, "Flow3 翻轉站控制")
    
    def start_flow4_vibration_feed(self) -> bool:
        """啟動Flow4 震動投料控制 (IO類併行)"""
        return self._execute_io_flow(4, "Flow4 震動投料控制")
    
    def _execute_io_flow(self, flow_id: int, flow_name: str) -> bool:
        """執行IO類Flow (併行執行)"""
        if not self.is_connected():
            self._notify_result_callbacks(DobotOperationResult(
                success=False,
                flow_type=FlowType.IO,
                flow_id=flow_id,
                error_message="未連接到Modbus服務器",
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return False
        
        # IO類Flow可以併行執行，直接觸發
        control_register = f"flow{flow_id}_control"
        if self.write_register(control_register, 1):
            self._notify_result_callbacks(DobotOperationResult(
                success=True,
                flow_type=FlowType.IO,
                flow_id=flow_id,
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return True
        else:
            self._notify_result_callbacks(DobotOperationResult(
                success=False,
                flow_type=FlowType.IO,
                flow_id=flow_id,
                error_message="寫入控制寄存器失敗",
                operation_type=flow_name,
                timestamp=time.strftime("%H:%M:%S")
            ))
            return False
    
    # 狀態查詢方法
    def is_motion_ready(self) -> bool:
        """檢查運動系統是否Ready"""
        status = self.get_status()
        return status.motion_status.ready and not status.motion_status.running
    
    def is_motion_running(self) -> bool:
        """檢查運動系統是否執行中"""
        status = self.get_status()
        return status.motion_status.running
    
    def is_motion_alarm(self) -> bool:
        """檢查運動系統是否警報"""
        status = self.get_status()
        return status.motion_status.alarm
    
    def get_current_motion_flow(self) -> Tuple[int, str]:
        """獲取當前運動Flow"""
        status = self.get_status()
        flow_id = status.motion_status.current_motion_flow
        flow_name = self.FLOW_DESCRIPTIONS.get(flow_id, "未知Flow")
        return flow_id, flow_name
    
    def get_motion_progress(self) -> int:
        """獲取運動進度 (0-100%)"""
        status = self.get_status()
        return status.motion_status.motion_progress
    
    def get_flow_completion_status(self) -> Dict[str, bool]:
        """獲取Flow完成狀態"""
        status = self.get_status()
        return {
            "flow1_complete": status.motion_status.flow1_complete,
            "flow2_complete": status.motion_status.flow2_complete,
            "flow5_complete": status.motion_status.flow5_complete
        }
    
    def get_io_flow_status(self) -> Dict[str, bool]:
        """獲取IO類Flow狀態"""
        status = self.get_status()
        return {
            "flow3_active": status.io_status.flow3_active,
            "flow4_active": status.io_status.flow4_active
        }
    
    def get_motion_statistics(self) -> Dict[str, int]:
        """獲取運動類統計資訊"""
        status = self.get_status()
        return {
            "operation_count": status.motion_status.motion_op_count,
            "error_count": status.motion_status.motion_err_count,
            "run_time_minutes": status.motion_status.motion_run_time
        }
    
    def get_status_description(self) -> Dict[str, str]:
        """獲取狀態描述"""
        status = self.get_status()
        
        # 運動狀態描述
        motion_state = "未知"
        if status.motion_status.alarm:
            motion_state = "警報"
        elif status.motion_status.running:
            motion_state = "執行中"
        elif status.motion_status.ready:
            motion_state = "準備就緒"
        elif status.motion_status.initialized:
            motion_state = "已初始化"
        else:
            motion_state = "未初始化"
        
        # 當前Flow描述
        current_flow = "無"
        if status.motion_status.current_motion_flow > 0:
            current_flow = self.FLOW_DESCRIPTIONS.get(
                status.motion_status.current_motion_flow, 
                f"未知Flow{status.motion_status.current_motion_flow}"
            )
        
        return {
            "運動狀態": motion_state,
            "當前運動Flow": current_flow,
            "運動進度": f"{status.motion_status.motion_progress}%",
            "運動錯誤碼": str(status.motion_status.motion_error_code) if status.motion_status.motion_error_code > 0 else "無錯誤",
            "Flow1完成": "是" if status.motion_status.flow1_complete else "否",
            "Flow2完成": "是" if status.motion_status.flow2_complete else "否",
            "Flow5完成": "是" if status.motion_status.flow5_complete else "否",
            "Flow3執行中": "是" if status.io_status.flow3_active else "否",
            "Flow4執行中": "是" if status.io_status.flow4_active else "否",
            "運動操作次數": str(status.motion_status.motion_op_count),
            "運動錯誤次數": str(status.motion_status.motion_err_count),
            "運行時間": f"{status.motion_status.motion_run_time}分鐘"
        }
    
    # 進階功能
    def wait_for_motion_ready(self, timeout: float = 30.0) -> bool:
        """等待運動系統進入Ready狀態"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_motion_ready():
                return True
            time.sleep(0.1)
        return False
    
    def wait_for_motion_complete(self, timeout: float = 300.0) -> bool:
        """等待運動執行完成"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if not status.motion_status.running:
                return True
            time.sleep(0.5)
        return False
    
    def wait_for_flow_complete(self, flow_id: int, timeout: float = 300.0) -> bool:
        """等待特定Flow完成"""
        if flow_id not in [1, 2, 5]:
            return False
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            completion_status = self.get_flow_completion_status()
            if completion_status.get(f"flow{flow_id}_complete", False):
                return True
            time.sleep(0.5)
        return False
    
    def execute_flow_sequence(self, flow_sequence: List[int], clear_after_completion: bool = True) -> bool:
        """執行Flow序列 (僅限運動類Flow)"""
        try:
            for flow_id in flow_sequence:
                if flow_id not in [1, 2, 5]:
                    continue
                
                # 等待運動系統準備就緒
                if not self.wait_for_motion_ready(10.0):
                    return False
                
                # 執行Flow
                if flow_id == 1:
                    if not self.start_flow1_vp_vision_pick():
                        return False
                elif flow_id == 2:
                    if not self.start_flow2_cv_unload():
                        return False
                elif flow_id == 5:
                    if not self.start_flow5_assembly():
                        return False
                
                # 等待完成
                if not self.wait_for_flow_complete(flow_id, 300.0):
                    return False
                
                # 清空控制寄存器
                if clear_after_completion:
                    self.clear_motion_flow(flow_id)
                    time.sleep(0.5)
            
            return True
            
        except Exception as e:
            return False
    
    # 回調管理
    def add_status_callback(self, callback: Callable[[DobotSystemStatus], None]):
        """新增狀態變化回調函數"""
        if callback not in self.status_callbacks:
            self.status_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[DobotSystemStatus], None]):
        """移除狀態變化回調函數"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def add_result_callback(self, callback: Callable[[DobotOperationResult], None]):
        """新增操作結果回調函數"""
        if callback not in self.result_callbacks:
            self.result_callbacks.append(callback)
    
    def remove_result_callback(self, callback: Callable[[DobotOperationResult], None]):
        """移除操作結果回調函數"""
        if callback in self.result_callbacks:
            self.result_callbacks.remove(callback)
    
    def _notify_status_callbacks(self, status: DobotSystemStatus):
        """通知狀態變化回調函數"""
        for callback in self.status_callbacks:
            try:
                callback(status)
            except Exception as e:
                pass  # 靜默處理回調異常
    
    def _notify_result_callbacks(self, result: DobotOperationResult):
        """通知操作結果回調函數"""
        for callback in self.result_callbacks:
            try:
                callback(result)
            except Exception as e:
                pass  # 靜默處理回調異常
    
    def get_connection_info(self) -> Dict[str, Any]:
        """獲取連接資訊"""
        return {
            "modbus_host": self.modbus_host,
            "modbus_port": self.modbus_port,
            "unit_id": self.unit_id,
            "connected": self.connected,
            "motion_base_address": self.MOTION_BASE_ADDRESS,
            "io_base_address": self.IO_BASE_ADDRESS,
            "monitoring_active": self.monitoring_active,
            "architecture": "新架構混合交握協議",
            "motion_flows": "Flow1,2,5 (狀態機交握，序列化執行)",
            "io_flows": "Flow3,4 (專用佇列併行執行)"
        }