# -*- coding: utf-8 -*-
"""
CCD1GUIClass.py - CCD1視覺檢測模組GUI控制類別
專為統一機台調適工具設計，提供完整的CCD1操作介面
基於ModbusTCP Client架構，處理握手協議和寄存器映射
"""

import time
import threading
from typing import Optional, List, Dict, Any, Tuple, Callable
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
class CCD1Command(IntEnum):
    """CCD1控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32


class CCD1StatusBits(IntEnum):
    """CCD1狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 數據結構 ====================
@dataclass
class CCD1Status:
    """CCD1系統狀態"""
    ready: bool = False
    running: bool = False
    alarm: bool = False
    initialized: bool = False
    connected: bool = False
    world_coord_valid: bool = False


@dataclass
class CircleData:
    """圓形檢測結果"""
    pixel_x: int
    pixel_y: int
    radius: int
    world_x: Optional[float] = None
    world_y: Optional[float] = None


@dataclass
class DetectionResult:
    """檢測結果"""
    circle_count: int
    circles: List[CircleData]
    capture_time: int
    process_time: int
    total_time: int
    timestamp: str


@dataclass
class DetectionParams:
    """檢測參數"""
    min_area: int = 50000
    min_roundness: int = 800  # ×1000存儲
    gaussian_kernel: int = 9
    canny_low: int = 20
    canny_high: int = 60


@dataclass
class CalibrationStatus:
    """標定狀態"""
    intrinsic_loaded: bool = False
    extrinsic_loaded: bool = False
    transformer_valid: bool = False
    intrinsic_file: str = ""
    extrinsic_file: str = ""


# ==================== CCD1GUI控制類 ====================
class CCD1GUIClass:
    """
    CCD1視覺檢測模組GUI控制類別
    
    功能:
    - Modbus TCP連接管理
    - 握手協議處理
    - 檢測操作控制
    - 標定檔案管理
    - 狀態監控
    - 參數調整
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化CCD1GUI控制類
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        
        # CCD1寄存器映射 (基地址200)
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,
            'STATUS_REGISTER': 201,
            'MIN_AREA_HIGH': 210,
            'MIN_AREA_LOW': 211,
            'MIN_ROUNDNESS': 212,
            'GAUSSIAN_KERNEL': 213,
            'CANNY_LOW': 214,
            'CANNY_HIGH': 215,
            'CIRCLE_COUNT': 240,
            'WORLD_COORD_VALID': 256,
            'CAPTURE_TIME': 280,
            'PROCESS_TIME': 281,
            'TOTAL_TIME': 282,
            'OP_COUNTER': 283,
            'ERR_COUNTER': 284,
        }
        
        # 狀態回調函數
        self.status_callbacks: List[Callable[[CCD1Status], None]] = []
        self.result_callbacks: List[Callable[[DetectionResult], None]] = []
        
        # 內部狀態
        self.current_status = CCD1Status()
        self.current_result: Optional[DetectionResult] = None
        self.current_params = DetectionParams()
        self.calibration_status = CalibrationStatus()
        
        # 線程控制
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self._lock = threading.Lock()
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1GUIClass")
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
                self.current_status.connected = True
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
        
        self.current_status.connected = False
        self.logger.info("已斷開Modbus連接")
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self.current_status.connected
    
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
                self._update_status()
                self._update_detection_result()
                time.sleep(0.2)  # 200ms更新間隔
            except Exception as e:
                self.logger.error(f"監控循環異常: {e}")
                time.sleep(1.0)
    
    def _update_status(self):
        """更新系統狀態"""
        if not self.modbus_client:
            return
        
        try:
            # 讀取狀態寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['STATUS_REGISTER'],
                count=1
            )
            
            if response.isError():
                return
            
            status_value = response.registers[0]
            
            with self._lock:
                old_status = self.current_status
                self.current_status.ready = bool(status_value & (1 << CCD1StatusBits.READY))
                self.current_status.running = bool(status_value & (1 << CCD1StatusBits.RUNNING))
                self.current_status.alarm = bool(status_value & (1 << CCD1StatusBits.ALARM))
                self.current_status.initialized = bool(status_value & (1 << CCD1StatusBits.INITIALIZED))
                
                # 讀取世界座標有效標誌
                world_response = self.modbus_client.read_holding_registers(
                    address=self.REGISTERS['WORLD_COORD_VALID'],
                    count=1
                )
                if not world_response.isError():
                    self.current_status.world_coord_valid = bool(world_response.registers[0])
                
                # 狀態變化時觸發回調
                if (old_status.ready != self.current_status.ready or
                    old_status.running != self.current_status.running or
                    old_status.alarm != self.current_status.alarm or
                    old_status.initialized != self.current_status.initialized):
                    self._notify_status_change()
                    
        except Exception as e:
            self.logger.error(f"更新狀態失敗: {e}")
    
    def _update_detection_result(self):
        """更新檢測結果"""
        if not self.modbus_client:
            return
        
        try:
            # 讀取檢測結果
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['CIRCLE_COUNT'],
                count=16  # 讀取圓形數量和像素座標
            )
            
            if response.isError():
                return
            
            registers = response.registers
            circle_count = registers[0]
            
            if circle_count > 0:
                circles = []
                for i in range(min(circle_count, 5)):
                    pixel_x = registers[1 + i * 3]
                    pixel_y = registers[2 + i * 3]
                    radius = registers[3 + i * 3]
                    
                    # 讀取世界座標 (如果有效)
                    world_x, world_y = None, None
                    if self.current_status.world_coord_valid:
                        world_x, world_y = self._read_world_coordinates(i)
                    
                    circles.append(CircleData(
                        pixel_x=pixel_x,
                        pixel_y=pixel_y,
                        radius=radius,
                        world_x=world_x,
                        world_y=world_y
                    ))
                
                # 讀取統計資訊
                stats_response = self.modbus_client.read_holding_registers(
                    address=self.REGISTERS['CAPTURE_TIME'],
                    count=3
                )
                
                if not stats_response.isError():
                    capture_time = stats_response.registers[0]
                    process_time = stats_response.registers[1]
                    total_time = stats_response.registers[2]
                    
                    result = DetectionResult(
                        circle_count=circle_count,
                        circles=circles,
                        capture_time=capture_time,
                        process_time=process_time,
                        total_time=total_time,
                        timestamp=time.strftime("%H:%M:%S")
                    )
                    
                    with self._lock:
                        self.current_result = result
                        self._notify_result_change()
                        
        except Exception as e:
            self.logger.error(f"更新檢測結果失敗: {e}")
    
    def _read_world_coordinates(self, circle_index: int) -> Tuple[Optional[float], Optional[float]]:
        """讀取世界座標"""
        try:
            base_addr = 257 + circle_index * 4  # 每個圓形佔用4個寄存器
            response = self.modbus_client.read_holding_registers(
                address=base_addr,
                count=4
            )
            
            if response.isError():
                return None, None
            
            registers = response.registers
            
            # 合併32位世界座標
            world_x_int = (registers[0] << 16) | registers[1]
            world_y_int = (registers[2] << 16) | registers[3]
            
            # 轉換為有符號32位整數並除以100
            if world_x_int & 0x80000000:
                world_x_int -= 0x100000000
            if world_y_int & 0x80000000:
                world_y_int -= 0x100000000
            
            world_x = world_x_int / 100.0
            world_y = world_y_int / 100.0
            
            return world_x, world_y
            
        except Exception as e:
            self.logger.error(f"讀取世界座標失敗: {e}")
            return None, None
    
    # ==================== 操作控制 ====================
    def capture_image(self) -> bool:
        """
        執行拍照操作
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD1Command.CAPTURE)
    
    def capture_and_detect(self) -> bool:
        """
        執行拍照+檢測操作
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD1Command.CAPTURE_DETECT)
    
    def initialize_camera(self) -> bool:
        """
        重新初始化相機
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD1Command.INITIALIZE)
    
    def clear_command(self) -> bool:
        """
        清空控制指令
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD1Command.CLEAR)
    
    def _send_command(self, command: CCD1Command) -> bool:
        """發送控制指令"""
        if not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            response = self.modbus_client.write_single_register(
                address=self.REGISTERS['CONTROL_COMMAND'],
                value=command.value
            )
            
            if response.isError():
                self.logger.error(f"發送指令失敗: {command}")
                return False
            
            self.logger.info(f"發送指令成功: {command}")
            return True
            
        except Exception as e:
            self.logger.error(f"發送指令異常: {e}")
            return False
    
    # ==================== 參數管理 ====================
    def read_detection_params(self) -> DetectionParams:
        """讀取檢測參數"""
        if not self.modbus_client:
            return self.current_params
        
        try:
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['MIN_AREA_HIGH'],
                count=6
            )
            
            if response.isError():
                return self.current_params
            
            registers = response.registers
            
            # 合併32位面積值
            min_area = (registers[0] << 16) | registers[1]
            
            params = DetectionParams(
                min_area=min_area,
                min_roundness=registers[2],
                gaussian_kernel=registers[3],
                canny_low=registers[4],
                canny_high=registers[5]
            )
            
            with self._lock:
                self.current_params = params
            
            return params
            
        except Exception as e:
            self.logger.error(f"讀取檢測參數失敗: {e}")
            return self.current_params
    
    def write_detection_params(self, params: DetectionParams) -> bool:
        """寫入檢測參數"""
        if not self.modbus_client:
            return False
        
        try:
            # 分離32位面積值
            area_high = (params.min_area >> 16) & 0xFFFF
            area_low = params.min_area & 0xFFFF
            
            values = [
                area_high,
                area_low,
                params.min_roundness,
                params.gaussian_kernel,
                params.canny_low,
                params.canny_high
            ]
            
            response = self.modbus_client.write_multiple_registers(
                address=self.REGISTERS['MIN_AREA_HIGH'],
                values=values
            )
            
            if response.isError():
                self.logger.error("寫入檢測參數失敗")
                return False
            
            with self._lock:
                self.current_params = params
            
            self.logger.info("檢測參數更新成功")
            return True
            
        except Exception as e:
            self.logger.error(f"寫入檢測參數異常: {e}")
            return False
    
    # ==================== 標定管理 ====================
    def scan_calibration_files(self) -> Dict[str, Any]:
        """掃描標定檔案 (透過CCD1模組Web API)"""
        # 注意: 這裡需要與CCD1模組的Web API進行通訊
        # 實際實現時需要HTTP請求到CCD1模組的5051端口
        self.logger.info("掃描標定檔案功能需要與CCD1模組Web API通訊")
        return {
            "success": False,
            "message": "需要實現HTTP API通訊"
        }
    
    def load_calibration_data(self, intrinsic_file: str = "", extrinsic_file: str = "") -> Dict[str, Any]:
        """載入標定數據 (透過CCD1模組Web API)"""
        # 注意: 這裡需要與CCD1模組的Web API進行通訊
        # 實際實現時需要HTTP請求到CCD1模組的5051端口
        self.logger.info("載入標定數據功能需要與CCD1模組Web API通訊")
        return {
            "success": False,
            "message": "需要實現HTTP API通訊"
        }
    
    # ==================== 狀態獲取 ====================
    def get_status(self) -> CCD1Status:
        """獲取當前系統狀態"""
        with self._lock:
            return self.current_status
    
    def get_detection_result(self) -> Optional[DetectionResult]:
        """獲取最新檢測結果"""
        with self._lock:
            return self.current_result
    
    def get_detection_params(self) -> DetectionParams:
        """獲取當前檢測參數"""
        with self._lock:
            return self.current_params
    
    def get_calibration_status(self) -> CalibrationStatus:
        """獲取標定狀態"""
        with self._lock:
            return self.calibration_status
    
    # ==================== 回調管理 ====================
    def add_status_callback(self, callback: Callable[[CCD1Status], None]):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def add_result_callback(self, callback: Callable[[DetectionResult], None]):
        """添加檢測結果回調"""
        self.result_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[CCD1Status], None]):
        """移除狀態變化回調"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def remove_result_callback(self, callback: Callable[[DetectionResult], None]):
        """移除檢測結果回調"""
        if callback in self.result_callbacks:
            self.result_callbacks.remove(callback)
    
    def _notify_status_change(self):
        """通知狀態變化"""
        for callback in self.status_callbacks:
            try:
                callback(self.current_status)
            except Exception as e:
                self.logger.error(f"狀態回調異常: {e}")
    
    def _notify_result_change(self):
        """通知檢測結果變化"""
        for callback in self.result_callbacks:
            try:
                callback(self.current_result)
            except Exception as e:
                self.logger.error(f"結果回調異常: {e}")
    
    # ==================== 資源清理 ====================
    def __del__(self):
        """析構函數，確保資源釋放"""
        self.disconnect()