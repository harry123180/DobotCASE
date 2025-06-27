# -*- coding: utf-8 -*-
"""
CCD3GUIClass.py - CCD3角度辨識模組GUI控制類別
專為統一機台調適工具設計，提供完整的CCD3操作介面
基於ModbusTCP Client架構，處理握手協議和角度檢測結果
"""

import time
import threading
from typing import Optional, Dict, Any, Callable
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
class CCD3Command(IntEnum):
    """CCD3控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32


class CCD3StatusBits(IntEnum):
    """CCD3狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


class DetectionMode(IntEnum):
    """檢測模式枚舉"""
    ELLIPSE_FITTING = 0      # 橢圓擬合模式
    MIN_AREA_RECT = 1        # 最小外接矩形模式


class ThresholdMode(IntEnum):
    """閾值處理模式枚舉"""
    OTSU_AUTO = 0            # OTSU自動閾值
    MANUAL = 1               # 手動閾值


class SequenceMode(IntEnum):
    """輪廓選擇模式枚舉"""
    MAX_CONTOUR = 0          # 最大輪廓
    SEQUENCE_CONTOUR = 1     # 序列輪廓


# ==================== 數據結構 ====================
@dataclass
class CCD3Status:
    """CCD3系統狀態"""
    ready: bool = False
    running: bool = False
    alarm: bool = False
    initialized: bool = False
    connected: bool = False


@dataclass
class AngleDetectionParams:
    """角度檢測參數"""
    detection_mode: int = 0          # 檢測模式
    min_area_rate: int = 50          # 最小面積比例(×1000存儲)
    sequence_mode: int = 0           # 輪廓選擇模式
    gaussian_kernel: int = 3         # 高斯模糊核大小
    threshold_mode: int = 0          # 閾值處理模式
    manual_threshold: int = 127      # 手動閾值


@dataclass
class AngleDetectionResult:
    """角度檢測結果"""
    success: bool
    center_x: int
    center_y: int
    angle: float
    major_axis: int          # 長軸長度(橢圓模式)
    minor_axis: int          # 短軸長度(橢圓模式)
    rect_width: int          # 矩形寬度(矩形模式)
    rect_height: int         # 矩形高度(矩形模式)
    contour_area: int        # 輪廓面積
    inner_diameter: Optional[float] = None    # 內徑(預留)
    outer_diameter: Optional[float] = None    # 外徑(預留)
    processing_time: int = 0
    timestamp: str = ""


@dataclass
class StatisticsInfo:
    """統計資訊"""
    capture_time: int = 0
    process_time: int = 0
    total_time: int = 0
    operation_count: int = 0
    error_count: int = 0
    connection_count: int = 0
    software_version_major: int = 3
    software_version_minor: int = 0
    run_time_hours: int = 0
    run_time_minutes: int = 0


# ==================== CCD3GUI控制類 ====================
class CCD3GUIClass:
    """
    CCD3角度辨識模組GUI控制類別
    
    功能:
    - Modbus TCP連接管理
    - 握手協議處理
    - 角度檢測操作控制
    - 檢測參數管理
    - 狀態監控
    - 檢測結果解析
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化CCD3GUI控制類
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        
        # CCD3寄存器映射 (基地址800)
        self.REGISTERS = {
            'CONTROL_COMMAND': 800,
            'STATUS_REGISTER': 801,
            
            # 檢測參數寄存器 (810-819)
            'DETECTION_MODE': 810,
            'MIN_AREA_RATE': 811,
            'SEQUENCE_MODE': 812,
            'GAUSSIAN_KERNEL': 813,
            'THRESHOLD_MODE': 814,
            'MANUAL_THRESHOLD': 815,
            
            # 角度檢測結果寄存器 (840-859)
            'DETECTION_SUCCESS': 840,
            'CENTER_X': 841,
            'CENTER_Y': 842,
            'ANGLE_HIGH': 843,
            'ANGLE_LOW': 844,
            'MAJOR_AXIS': 845,
            'MINOR_AXIS': 846,
            'RECT_WIDTH': 847,
            'RECT_HEIGHT': 848,
            'CONTOUR_AREA': 849,
            'INNER_DIAMETER_HIGH': 850,
            'INNER_DIAMETER_LOW': 851,
            'OUTER_DIAMETER_HIGH': 852,
            'OUTER_DIAMETER_LOW': 853,
            
            # 統計資訊寄存器 (880-899)
            'CAPTURE_TIME': 880,
            'PROCESS_TIME': 881,
            'TOTAL_TIME': 882,
            'OPERATION_COUNT': 883,
            'ERROR_COUNT': 884,
            'CONNECTION_COUNT': 885,
            'SOFTWARE_VERSION_MAJOR': 890,
            'SOFTWARE_VERSION_MINOR': 891,
            'RUN_TIME_HOURS': 892,
            'RUN_TIME_MINUTES': 893,
        }
        
        # 狀態回調函數
        self.status_callbacks: List[Callable[[CCD3Status], None]] = []
        self.result_callbacks: List[Callable[[AngleDetectionResult], None]] = []
        
        # 內部狀態
        self.current_status = CCD3Status()
        self.current_result: Optional[AngleDetectionResult] = None
        self.current_params = AngleDetectionParams()
        self.statistics = StatisticsInfo()
        
        # 線程控制
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self._lock = threading.Lock()
        
        # 設置日誌
        self.logger = logging.getLogger("CCD3GUIClass")
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
                self._update_statistics()
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
                self.current_status.ready = bool(status_value & (1 << CCD3StatusBits.READY))
                self.current_status.running = bool(status_value & (1 << CCD3StatusBits.RUNNING))
                self.current_status.alarm = bool(status_value & (1 << CCD3StatusBits.ALARM))
                self.current_status.initialized = bool(status_value & (1 << CCD3StatusBits.INITIALIZED))
                
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
            # 讀取檢測結果寄存器 (840-853)
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['DETECTION_SUCCESS'],
                count=14
            )
            
            if response.isError():
                return
            
            registers = response.registers
            success = bool(registers[0])
            
            if success:
                center_x = registers[1]
                center_y = registers[2]
                
                # 合併32位角度值
                angle_int = (registers[3] << 16) | registers[4]
                # 轉換為有符號32位角度
                if angle_int & 0x80000000:
                    angle_int -= 0x100000000
                angle = angle_int / 100.0
                
                major_axis = registers[5]
                minor_axis = registers[6]
                rect_width = registers[7]
                rect_height = registers[8]
                contour_area = registers[9]
                
                # 內外徑(預留功能)
                inner_diameter = None
                outer_diameter = None
                if len(registers) >= 14:
                    inner_int = (registers[10] << 16) | registers[11]
                    outer_int = (registers[12] << 16) | registers[13]
                    if inner_int > 0:
                        inner_diameter = inner_int / 100.0
                    if outer_int > 0:
                        outer_diameter = outer_int / 100.0
                
                result = AngleDetectionResult(
                    success=success,
                    center_x=center_x,
                    center_y=center_y,
                    angle=angle,
                    major_axis=major_axis,
                    minor_axis=minor_axis,
                    rect_width=rect_width,
                    rect_height=rect_height,
                    contour_area=contour_area,
                    inner_diameter=inner_diameter,
                    outer_diameter=outer_diameter,
                    timestamp=time.strftime("%H:%M:%S")
                )
                
                with self._lock:
                    self.current_result = result
                    self._notify_result_change()
                        
        except Exception as e:
            self.logger.error(f"更新檢測結果失敗: {e}")
    
    def _update_statistics(self):
        """更新統計資訊"""
        if not self.modbus_client:
            return
        
        try:
            # 讀取統計資訊寄存器 (880-893)
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['CAPTURE_TIME'],
                count=14
            )
            
            if response.isError():
                return
            
            registers = response.registers
            
            with self._lock:
                self.statistics.capture_time = registers[0]
                self.statistics.process_time = registers[1]
                self.statistics.total_time = registers[2]
                self.statistics.operation_count = registers[3]
                self.statistics.error_count = registers[4]
                self.statistics.connection_count = registers[5]
                
                if len(registers) >= 14:
                    self.statistics.software_version_major = registers[10]
                    self.statistics.software_version_minor = registers[11]
                    self.statistics.run_time_hours = registers[12]
                    self.statistics.run_time_minutes = registers[13]
                
        except Exception as e:
            self.logger.error(f"更新統計資訊失敗: {e}")
    
    # ==================== 操作控制 ====================
    def capture_image(self) -> bool:
        """
        執行拍照操作
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD3Command.CAPTURE)
    
    def capture_and_detect_angle(self) -> bool:
        """
        執行拍照+角度檢測操作
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD3Command.CAPTURE_DETECT)
    
    def initialize_camera(self) -> bool:
        """
        重新初始化相機
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD3Command.INITIALIZE)
    
    def clear_command(self) -> bool:
        """
        清空控制指令
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD3Command.CLEAR)
    
    def _send_command(self, command: CCD3Command) -> bool:
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
    def read_detection_params(self) -> AngleDetectionParams:
        """讀取檢測參數"""
        if not self.modbus_client:
            return self.current_params
        
        try:
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['DETECTION_MODE'],
                count=6
            )
            
            if response.isError():
                return self.current_params
            
            registers = response.registers
            
            params = AngleDetectionParams(
                detection_mode=registers[0],
                min_area_rate=registers[1],
                sequence_mode=registers[2],
                gaussian_kernel=registers[3],
                threshold_mode=registers[4],
                manual_threshold=registers[5]
            )
            
            with self._lock:
                self.current_params = params
            
            return params
            
        except Exception as e:
            self.logger.error(f"讀取檢測參數失敗: {e}")
            return self.current_params
    
    def write_detection_params(self, params: AngleDetectionParams) -> bool:
        """寫入檢測參數"""
        if not self.modbus_client:
            return False
        
        try:
            values = [
                params.detection_mode,
                params.min_area_rate,
                params.sequence_mode,
                params.gaussian_kernel,
                params.threshold_mode,
                params.manual_threshold
            ]
            
            response = self.modbus_client.write_multiple_registers(
                address=self.REGISTERS['DETECTION_MODE'],
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
    
    # ==================== 狀態獲取 ====================
    def get_status(self) -> CCD3Status:
        """獲取當前系統狀態"""
        with self._lock:
            return self.current_status
    
    def get_detection_result(self) -> Optional[AngleDetectionResult]:
        """獲取最新檢測結果"""
        with self._lock:
            return self.current_result
    
    def get_detection_params(self) -> AngleDetectionParams:
        """獲取當前檢測參數"""
        with self._lock:
            return self.current_params
    
    def get_statistics(self) -> StatisticsInfo:
        """獲取統計資訊"""
        with self._lock:
            return self.statistics
    
    def get_mode_description(self, mode: int) -> str:
        """獲取檢測模式描述"""
        mode_descriptions = {
            0: "橢圓擬合模式",
            1: "最小外接矩形模式"
        }
        return mode_descriptions.get(mode, f"未知模式({mode})")
    
    def get_threshold_mode_description(self, mode: int) -> str:
        """獲取閾值模式描述"""
        threshold_descriptions = {
            0: "OTSU自動閾值",
            1: "手動閾值"
        }
        return threshold_descriptions.get(mode, f"未知模式({mode})")
    
    def get_sequence_mode_description(self, mode: int) -> str:
        """獲取輪廓選擇模式描述"""
        sequence_descriptions = {
            0: "最大輪廓",
            1: "序列輪廓"
        }
        return sequence_descriptions.get(mode, f"未知模式({mode})")
    
    # ==================== 回調管理 ====================
    def add_status_callback(self, callback: Callable[[CCD3Status], None]):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def add_result_callback(self, callback: Callable[[AngleDetectionResult], None]):
        """添加檢測結果回調"""
        self.result_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[CCD3Status], None]):
        """移除狀態變化回調"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def remove_result_callback(self, callback: Callable[[AngleDetectionResult], None]):
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
    
    # ==================== 工具方法 ====================
    def get_angle_in_range(self, angle: float, min_angle: float = -180.0, max_angle: float = 180.0) -> float:
        """將角度值限制在指定範圍內"""
        while angle > max_angle:
            angle -= 360.0
        while angle < min_angle:
            angle += 360.0
        return angle
    
    def format_angle(self, angle: float, precision: int = 2) -> str:
        """格式化角度值顯示"""
        return f"{angle:.{precision}f}°"
    
    def format_processing_time(self, time_ms: int) -> str:
        """格式化處理時間顯示"""
        if time_ms < 1000:
            return f"{time_ms}ms"
        else:
            return f"{time_ms/1000:.2f}s"
    
    def get_software_version(self) -> str:
        """獲取軟體版本字串"""
        return f"v{self.statistics.software_version_major}.{self.statistics.software_version_minor}"
    
    def get_run_time_string(self) -> str:
        """獲取運行時間字串"""
        hours = self.statistics.run_time_hours
        minutes = self.statistics.run_time_minutes
        return f"{hours}小時{minutes}分鐘"
    
    # ==================== 資源清理 ====================
    def __del__(self):
        """析構函數，確保資源釋放"""
        self.disconnect()