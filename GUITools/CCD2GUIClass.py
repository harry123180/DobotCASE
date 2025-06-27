# -*- coding: utf-8 -*-
"""
CCD2GUIClass.py - CCD2圖像分類模組GUI控制類別
專為統一機台調適工具設計，提供完整的CCD2操作介面
基於ModbusTCP Client架構，處理握手協議和分類結果
"""

import time
import threading
import json
import os
from typing import Optional, List, Dict, Any, Callable
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
class CCD2Command(IntEnum):
    """CCD2控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_CLASSIFY = 16
    INITIALIZE = 32
    RELOAD_CONFIG = 64


class CCD2StatusBits(IntEnum):
    """CCD2狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 數據結構 ====================
@dataclass
class CCD2Status:
    """CCD2系統狀態"""
    ready: bool = False
    running: bool = False
    alarm: bool = False
    initialized: bool = False
    connected: bool = False
    config_loaded: bool = False


@dataclass
class ClassificationResult:
    """分類結果"""
    success: bool
    category_id: int
    confidence: float
    matched_conditions: int
    features: Dict[str, float]
    processing_time: int
    timestamp: str


@dataclass
class FeatureValues:
    """特徵值數據"""
    mean_value: float = 0.0
    std_dev: float = 0.0
    skewness: float = 0.0
    kurtosis: float = 0.0


@dataclass
class ConfigurationInfo:
    """配置檔案資訊"""
    module_name: str = ""
    version: str = ""
    created_date: str = ""
    description: str = ""
    categories_count: int = 0
    loaded_file: str = ""


@dataclass
class ProcessingParams:
    """圖像處理參數"""
    gaussian_kernel: int = 5
    use_otsu: bool = True
    manual_threshold: int = 127
    canny_low: int = 50
    canny_high: int = 150
    lbp_radius: int = 3
    lbp_points: int = 24
    roi_enabled: bool = True
    roi_x: int = 100
    roi_y: int = 100
    roi_width: int = 200
    roi_height: int = 200


@dataclass
class ClassificationCategory:
    """分類類別定義"""
    id: int
    name: str
    enabled: bool
    logic: str  # "AND" or "OR"
    conditions: List[Dict[str, Any]]


# ==================== CCD2GUI控制類 ====================
class CCD2GUIClass:
    """
    CCD2圖像分類模組GUI控制類別
    
    功能:
    - Modbus TCP連接管理
    - 握手協議處理
    - 分類操作控制
    - JSON配置檔案管理
    - 狀態監控
    - 分類結果解析
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化CCD2GUI控制類
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        
        # CCD2寄存器映射 (基地址1100)
        self.REGISTERS = {
            'CONTROL_COMMAND': 1100,
            'STATUS_REGISTER': 1101,
            
            # 分類結果寄存器 (1140-1159)
            'CLASSIFICATION_SUCCESS': 1140,
            'CATEGORY_ID': 1141,
            'CONFIDENCE_HIGH': 1142,
            'CONFIDENCE_LOW': 1143,
            'MATCHED_CONDITIONS': 1144,
            
            # 特徵值寄存器 (1150-1159)
            'MEAN_VALUE_HIGH': 1150,
            'MEAN_VALUE_LOW': 1151,
            'STD_DEV_HIGH': 1152,
            'STD_DEV_LOW': 1153,
            'SKEWNESS_HIGH': 1154,
            'SKEWNESS_LOW': 1155,
            'KURTOSIS_HIGH': 1156,
            'KURTOSIS_LOW': 1157,
            
            # 統計資訊寄存器 (1180-1199)
            'PROCESSING_TIME': 1180,
            'CLASSIFICATION_COUNT': 1181,
            'ERROR_COUNT': 1182,
            'CONFIG_LOAD_COUNT': 1183,
            'SOFTWARE_VERSION_MAJOR': 1190,
            'SOFTWARE_VERSION_MINOR': 1191,
            'RUN_TIME_HOURS': 1192,
            'RUN_TIME_MINUTES': 1193,
        }
        
        # 狀態回調函數
        self.status_callbacks: List[Callable[[CCD2Status], None]] = []
        self.result_callbacks: List[Callable[[ClassificationResult], None]] = []
        
        # 內部狀態
        self.current_status = CCD2Status()
        self.current_result: Optional[ClassificationResult] = None
        self.current_features = FeatureValues()
        self.config_info = ConfigurationInfo()
        self.processing_params = ProcessingParams()
        self.categories: List[ClassificationCategory] = []
        
        # 線程控制
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self._lock = threading.Lock()
        
        # 配置檔案路徑
        self.config_folder = "condition"
        
        # 設置日誌
        self.logger = logging.getLogger("CCD2GUIClass")
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
                self._update_classification_result()
                self._update_feature_values()
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
                self.current_status.ready = bool(status_value & (1 << CCD2StatusBits.READY))
                self.current_status.running = bool(status_value & (1 << CCD2StatusBits.RUNNING))
                self.current_status.alarm = bool(status_value & (1 << CCD2StatusBits.ALARM))
                self.current_status.initialized = bool(status_value & (1 << CCD2StatusBits.INITIALIZED))
                
                # 檢查配置載入狀態 (透過讀取配置載入計數判斷)
                config_response = self.modbus_client.read_holding_registers(
                    address=self.REGISTERS['CONFIG_LOAD_COUNT'],
                    count=1
                )
                if not config_response.isError():
                    config_count = config_response.registers[0]
                    self.current_status.config_loaded = config_count > 0
                
                # 狀態變化時觸發回調
                if (old_status.ready != self.current_status.ready or
                    old_status.running != self.current_status.running or
                    old_status.alarm != self.current_status.alarm or
                    old_status.initialized != self.current_status.initialized):
                    self._notify_status_change()
                    
        except Exception as e:
            self.logger.error(f"更新狀態失敗: {e}")
    
    def _update_classification_result(self):
        """更新分類結果"""
        if not self.modbus_client:
            return
        
        try:
            # 讀取分類結果寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['CLASSIFICATION_SUCCESS'],
                count=5
            )
            
            if response.isError():
                return
            
            registers = response.registers
            success = bool(registers[0])
            
            if success:
                category_id = registers[1]
                
                # 合併32位信心度
                confidence_int = (registers[2] << 16) | registers[3]
                confidence = confidence_int / 100.0
                
                matched_conditions = registers[4]
                
                # 讀取處理時間
                time_response = self.modbus_client.read_holding_registers(
                    address=self.REGISTERS['PROCESSING_TIME'],
                    count=1
                )
                
                processing_time = 0
                if not time_response.isError():
                    processing_time = time_response.registers[0]
                
                result = ClassificationResult(
                    success=success,
                    category_id=category_id,
                    confidence=confidence,
                    matched_conditions=matched_conditions,
                    features=self._features_to_dict(),
                    processing_time=processing_time,
                    timestamp=time.strftime("%H:%M:%S")
                )
                
                with self._lock:
                    self.current_result = result
                    self._notify_result_change()
                        
        except Exception as e:
            self.logger.error(f"更新分類結果失敗: {e}")
    
    def _update_feature_values(self):
        """更新特徵值"""
        if not self.modbus_client:
            return
        
        try:
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['MEAN_VALUE_HIGH'],
                count=8
            )
            
            if response.isError():
                return
            
            registers = response.registers
            
            # 合併32位特徵值
            mean_int = (registers[0] << 16) | registers[1]
            std_int = (registers[2] << 16) | registers[3]
            skew_int = (registers[4] << 16) | registers[5]
            kurt_int = (registers[6] << 16) | registers[7]
            
            # 轉換為有符號值並應用比例係數
            with self._lock:
                self.current_features.mean_value = self._int32_to_float(mean_int, 100)
                self.current_features.std_dev = self._int32_to_float(std_int, 100)
                self.current_features.skewness = self._int32_to_float(skew_int, 1000)
                self.current_features.kurtosis = self._int32_to_float(kurt_int, 1000)
                
        except Exception as e:
            self.logger.error(f"更新特徵值失敗: {e}")
    
    def _int32_to_float(self, value: int, scale: int) -> float:
        """將32位整數轉換為有符號浮點數"""
        if value & 0x80000000:
            value -= 0x100000000
        return value / scale
    
    def _features_to_dict(self) -> Dict[str, float]:
        """將特徵值轉換為字典"""
        return {
            "平均值": self.current_features.mean_value,
            "標準差": self.current_features.std_dev,
            "偏度": self.current_features.skewness,
            "峰度": self.current_features.kurtosis
        }
    
    # ==================== 操作控制 ====================
    def capture_image(self) -> bool:
        """
        執行拍照操作
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD2Command.CAPTURE)
    
    def capture_and_classify(self) -> bool:
        """
        執行拍照+分類操作
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD2Command.CAPTURE_CLASSIFY)
    
    def initialize_camera(self) -> bool:
        """
        重新初始化相機
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD2Command.INITIALIZE)
    
    def reload_configuration(self) -> bool:
        """
        重新載入JSON配置
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD2Command.RELOAD_CONFIG)
    
    def clear_command(self) -> bool:
        """
        清空控制指令
        
        Returns:
            bool: 操作是否成功發送
        """
        return self._send_command(CCD2Command.CLEAR)
    
    def _send_command(self, command: CCD2Command) -> bool:
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
    
    # ==================== 配置檔案管理 ====================
    def scan_configuration_files(self) -> List[str]:
        """
        掃描condition資料夾中的JSON配置檔案
        
        Returns:
            List[str]: 可用的配置檔案列表
        """
        config_files = []
        
        try:
            if not os.path.exists(self.config_folder):
                self.logger.warning(f"配置資料夾不存在: {self.config_folder}")
                return config_files
            
            for filename in os.listdir(self.config_folder):
                if filename.endswith('.json'):
                    file_path = os.path.join(self.config_folder, filename)
                    if self._validate_config_file(file_path):
                        config_files.append(filename)
                    else:
                        self.logger.warning(f"配置檔案格式錯誤: {filename}")
            
            self.logger.info(f"掃描到 {len(config_files)} 個有效配置檔案")
            return sorted(config_files)
            
        except Exception as e:
            self.logger.error(f"掃描配置檔案失敗: {e}")
            return []
    
    def load_configuration_file(self, filename: str) -> bool:
        """
        載入指定的JSON配置檔案
        
        Args:
            filename: 配置檔案名稱
            
        Returns:
            bool: 載入是否成功
        """
        try:
            file_path = os.path.join(self.config_folder, filename)
            
            if not os.path.exists(file_path):
                self.logger.error(f"配置檔案不存在: {filename}")
                return False
            
            with open(file_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # 解析配置檔案
            if self._parse_configuration(config_data, filename):
                self.logger.info(f"配置檔案載入成功: {filename}")
                return True
            else:
                self.logger.error(f"配置檔案解析失敗: {filename}")
                return False
                
        except Exception as e:
            self.logger.error(f"載入配置檔案異常: {e}")
            return False
    
    def _validate_config_file(self, file_path: str) -> bool:
        """驗證配置檔案格式"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # 檢查必要欄位
            required_fields = ['module_info', 'categories', 'processing_parameters']
            for field in required_fields:
                if field not in config_data:
                    return False
            
            # 檢查模組資訊
            module_info = config_data['module_info']
            if not all(key in module_info for key in ['module_name', 'version']):
                return False
            
            # 檢查分類配置
            categories = config_data['categories']
            if not isinstance(categories, list) or len(categories) == 0:
                return False
            
            return True
            
        except Exception:
            return False
    
    def _parse_configuration(self, config_data: Dict[str, Any], filename: str) -> bool:
        """解析配置數據"""
        try:
            with self._lock:
                # 解析模組資訊
                module_info = config_data['module_info']
                self.config_info = ConfigurationInfo(
                    module_name=module_info.get('module_name', ''),
                    version=module_info.get('version', ''),
                    created_date=module_info.get('created_date', ''),
                    description=module_info.get('description', ''),
                    categories_count=len(config_data['categories']),
                    loaded_file=filename
                )
                
                # 解析處理參數
                proc_params = config_data['processing_parameters']
                self.processing_params = ProcessingParams(
                    gaussian_kernel=proc_params.get('gaussian_kernel', 5),
                    use_otsu=proc_params.get('use_otsu', True),
                    manual_threshold=proc_params.get('manual_threshold', 127),
                    canny_low=proc_params.get('canny_low', 50),
                    canny_high=proc_params.get('canny_high', 150),
                    lbp_radius=proc_params.get('lbp_radius', 3),
                    lbp_points=proc_params.get('lbp_points', 24)
                )
                
                # 解析ROI設定
                roi_config = proc_params.get('roi', {})
                if roi_config:
                    self.processing_params.roi_enabled = roi_config.get('enabled', True)
                    self.processing_params.roi_x = roi_config.get('x', 100)
                    self.processing_params.roi_y = roi_config.get('y', 100)
                    self.processing_params.roi_width = roi_config.get('width', 200)
                    self.processing_params.roi_height = roi_config.get('height', 200)
                
                # 解析分類類別
                self.categories = []
                for cat_data in config_data['categories']:
                    category = ClassificationCategory(
                        id=cat_data.get('id', 0),
                        name=cat_data.get('name', ''),
                        enabled=cat_data.get('enabled', True),
                        logic=cat_data.get('logic', 'AND'),
                        conditions=cat_data.get('conditions', [])
                    )
                    self.categories.append(category)
            
            return True
            
        except Exception as e:
            self.logger.error(f"解析配置失敗: {e}")
            return False
    
    # ==================== 狀態獲取 ====================
    def get_status(self) -> CCD2Status:
        """獲取當前系統狀態"""
        with self._lock:
            return self.current_status
    
    def get_classification_result(self) -> Optional[ClassificationResult]:
        """獲取最新分類結果"""
        with self._lock:
            return self.current_result
    
    def get_feature_values(self) -> FeatureValues:
        """獲取當前特徵值"""
        with self._lock:
            return self.current_features
    
    def get_configuration_info(self) -> ConfigurationInfo:
        """獲取配置資訊"""
        with self._lock:
            return self.config_info
    
    def get_processing_params(self) -> ProcessingParams:
        """獲取處理參數"""
        with self._lock:
            return self.processing_params
    
    def get_categories(self) -> List[ClassificationCategory]:
        """獲取分類類別"""
        with self._lock:
            return self.categories.copy()
    
    def get_statistics(self) -> Dict[str, int]:
        """獲取統計資訊"""
        if not self.modbus_client:
            return {}
        
        try:
            response = self.modbus_client.read_holding_registers(
                address=self.REGISTERS['CLASSIFICATION_COUNT'],
                count=4
            )
            
            if response.isError():
                return {}
            
            registers = response.registers
            return {
                "分類次數": registers[0],
                "錯誤次數": registers[1],
                "配置載入次數": registers[2],
                "處理時間": registers[3] if len(registers) > 3 else 0
            }
            
        except Exception as e:
            self.logger.error(f"獲取統計資訊失敗: {e}")
            return {}
    
    # ==================== 回調管理 ====================
    def add_status_callback(self, callback: Callable[[CCD2Status], None]):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def add_result_callback(self, callback: Callable[[ClassificationResult], None]):
        """添加分類結果回調"""
        self.result_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[CCD2Status], None]):
        """移除狀態變化回調"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
    
    def remove_result_callback(self, callback: Callable[[ClassificationResult], None]):
        """移除分類結果回調"""
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
        """通知分類結果變化"""
        for callback in self.result_callbacks:
            try:
                callback(self.current_result)
            except Exception as e:
                self.logger.error(f"結果回調異常: {e}")
    
    # ==================== 資源清理 ====================
    def __del__(self):
        """析構函數，確保資源釋放"""
        self.disconnect()