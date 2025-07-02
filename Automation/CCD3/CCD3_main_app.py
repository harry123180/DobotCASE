import sys
import os
import time
import threading
import json
import logging
import statistics
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import cv2
import numpy as np
import math
import datetime
# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# Flask imports
from flask import Flask, render_template, request, jsonify, send_from_directory ,send_file
from flask_socketio import SocketIO, emit

# Import camera manager
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
from camera_manager import OptimizedCamera, CameraConfig

# 設置logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StatusBits(Enum):
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3

@dataclass
class AngleResult:
    success: bool
    center: Optional[Tuple[int, int]]
    angle: Optional[float]
    major_axis: Optional[float]
    minor_axis: Optional[float]
    rect_width: Optional[float]
    rect_height: Optional[float]
    contour_area: Optional[float]
    processing_time: float
    capture_time: float
    total_time: float
    error_message: Optional[str] = None

class SystemStateMachine:
    def __init__(self):
        self.status_register = 0b0001  # 初始狀態: Ready=1
        self.lock = threading.Lock()
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos.value)
            else:
                self.status_register &= ~(1 << bit_pos.value)
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        with self.lock:
            return bool(self.status_register & (1 << bit_pos.value))
    
    def is_ready(self) -> bool:
        return self.get_bit(StatusBits.READY)
    
    def is_running(self) -> bool:
        return self.get_bit(StatusBits.RUNNING)
    
    def is_alarm(self) -> bool:
        return self.get_bit(StatusBits.ALARM)
    
    def is_initialized(self) -> bool:
        return self.get_bit(StatusBits.INITIALIZED)
    
    def set_ready(self, ready: bool):
        self.set_bit(StatusBits.READY, ready)
    
    def set_running(self, running: bool):
        self.set_bit(StatusBits.RUNNING, running)
    
    def set_alarm(self, alarm: bool):
        self.set_bit(StatusBits.ALARM, alarm)
    
    def set_initialized(self, initialized: bool):
        self.set_bit(StatusBits.INITIALIZED, initialized)
    
    def reset_to_idle(self):
        with self.lock:
            self.status_register = 0b1001  # Ready=1, Initialized=1

class PerformanceMonitor:
    """性能監控類"""
    def __init__(self):
        self.times = []
        self.capture_times = []
        self.process_times = []
        self.lock = threading.Lock()
        
    def add_result(self, result: AngleResult):
        """添加檢測結果用於性能分析"""
        if result.success:
            with self.lock:
                self.times.append(result.total_time)
                self.capture_times.append(result.capture_time)
                self.process_times.append(result.processing_time)
                
                # 保持最近100次記錄
                if len(self.times) > 100:
                    self.times.pop(0)
                    self.capture_times.pop(0)
                    self.process_times.pop(0)
    
    def get_stats(self):
        """獲取性能統計"""
        with self.lock:
            if not self.times:
                return {}
            
            return {
                'avg_total_time': statistics.mean(self.times),
                'avg_capture_time': statistics.mean(self.capture_times),
                'avg_process_time': statistics.mean(self.process_times),
                'min_total_time': min(self.times),
                'max_total_time': max(self.times),
                'sample_count': len(self.times)
            }

class OptimizedAngleDetector:
    """優化版角度檢測器 - 整合簡化CASE算法"""
    def __init__(self):
        self.min_area_rate = 0.05
        self.sequence_mode = True  # CASE模式預設使用序列模式
        self.gaussian_kernel = 3
        self.threshold_mode = 1  # 0=OTSU, 1=Manual
        self.manual_threshold = 150  # 參考新式流程的閾值150
        
        # 性能優化：預編譯快取
        self._kernel_cache = {}
        self._last_image_shape = None
        self._min_area_cache = None
    
    def update_params(self, **kwargs):
        """更新檢測參數"""
        changed = False
        if 'min_area_rate' in kwargs and kwargs['min_area_rate'] != self.min_area_rate * 1000:
            self.min_area_rate = kwargs['min_area_rate'] / 1000.0
            self._min_area_cache = None
            changed = True
        if 'sequence_mode' in kwargs and bool(kwargs['sequence_mode']) != self.sequence_mode:
            self.sequence_mode = bool(kwargs['sequence_mode'])
            changed = True
        if 'gaussian_kernel' in kwargs and kwargs['gaussian_kernel'] != self.gaussian_kernel:
            self.gaussian_kernel = kwargs['gaussian_kernel']
            changed = True
        if 'threshold_mode' in kwargs and kwargs['threshold_mode'] != self.threshold_mode:
            self.threshold_mode = kwargs['threshold_mode']
            changed = True
        if 'manual_threshold' in kwargs and kwargs['manual_threshold'] != self.manual_threshold:
            self.manual_threshold = kwargs['manual_threshold']
            changed = True
        
        if changed:
            print(f"參數已更新：面積比={self.min_area_rate:.3f}, 序列模式={self.sequence_mode}, 高斯核={self.gaussian_kernel}, 閾值模式={self.threshold_mode}, 手動閾值={self.manual_threshold}")
    
    def get_main_contour_optimized(self, image, sequence=True):
        """獲取主要輪廓 - 兼容現有邏輯"""
        min_area_size_rate = self.min_area_rate
        min_area = image.shape[0] * image.shape[1] * min_area_size_rate
        
        contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        if not contours:
            return None
        
        if sequence:
            # CASE模式：選擇最大輪廓（面積排序後第一個）
            contours.sort(key=cv2.contourArea, reverse=True)
            return contours[0]
        else:
            # DR模式：選擇第一個輪廓
            return contours[0]
    
    def get_pre_treatment_image_optimized(self, image):
        """優化版影像前處理 - 參考新式流程"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 高斯模糊
        kernel_size = self.gaussian_kernel
        if kernel_size not in self._kernel_cache:
            if kernel_size <= 0 or kernel_size % 2 == 0:
                kernel_size = 3
            self._kernel_cache[kernel_size] = (kernel_size, kernel_size)
        
        blur = cv2.GaussianBlur(gray, self._kernel_cache[kernel_size], 0)
        
        # 二值化處理 - 參考新式流程使用THRESH_BINARY_INV
        if self.threshold_mode == 0:
            _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            print(f"使用OTSU自動閾值進行反向二值化")
        else:
            _, thresh = cv2.threshold(blur, self.manual_threshold, 255, cv2.THRESH_BINARY_INV)
            print(f"使用手動閾值{self.manual_threshold}進行反向二值化")
        
        return thresh
    
    def get_main_contours(self, image, min_area_size_rate=None):
        """獲取主要輪廓 - 參考新式流程"""
        if min_area_size_rate is None:
            min_area_size_rate = self.min_area_rate
        
        min_area = image.shape[0] * image.shape[1] * min_area_size_rate
        print(f"輪廓檢測參數: 圖像尺寸={image.shape}, 最小面積比率={min_area_size_rate:.3f}, 最小面積={min_area:.0f}")
        
        contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        print(f"檢測到符合面積要求的輪廓數量: {len(contours)}")
        
        if not contours:
            print("警告: 沒有輪廓符合最小面積要求")
            return None
        
        return contours
    
    def _detect_angle_case_mode_optimized(self, contour, original_image):
        """簡化版CASE模式角度檢測 - 跳過橢圓遮罩處理"""
        if len(contour) < 5:
            return None
        
        try:
            print("執行簡化版CASE模式角度檢測 - 直接minAreaRect算法")
            
            # 直接使用minAreaRect獲取角度 - 光源優化後的簡化算法
            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            
            center_int = (int(center[0]), int(center[1]))
            print(f"簡化版CASE模式結果: 中心={center_int}, 角度={angle:.2f}度, 尺寸={size}")
            
            # 為了兼容性，仍提供橢圓擬合資訊（可選）
            if len(contour) >= 5:
                try:
                    ellipse = cv2.fitEllipse(contour)
                    (ex, ey), (MA, ma), ellipse_angle = ellipse
                    extra_data = {
                        'major_axis': MA,
                        'minor_axis': ma,
                        'ellipse_angle': ellipse_angle,
                        'final_angle': angle,
                        'rect_width': size[0],
                        'rect_height': size[1],
                        'rect': rect,  # 添加rect數據用於調試圖像
                        'contour': contour  # 添加輪廓數據用於調試圖像
                    }
                    print(f"兼容性橢圓資訊: 長軸={MA:.2f}, 短軸={ma:.2f}, 橢圓角度={ellipse_angle:.2f}")
                except:
                    extra_data = {
                        'rect_width': size[0],
                        'rect_height': size[1],
                        'final_angle': angle,
                        'rect': rect,
                        'contour': contour
                    }
            else:
                extra_data = {
                    'rect_width': size[0],
                    'rect_height': size[1],
                    'final_angle': angle,
                    'rect': rect,
                    'contour': contour
                }
            
            return center_int, angle, extra_data
            
        except cv2.error as e:
            print(f"簡化版CASE模式檢測錯誤: {e}")
            return None
    
    def _detect_angle_dr_mode(self, contour):
        """DR模式角度檢測 - 保持原有邏輯"""
        print("執行DR模式角度檢測 (mode=1) - 最小外接矩形")
        
        rect = cv2.minAreaRect(contour)
        center, size, angle = rect
        
        print(f"minAreaRect結果: 中心=({center[0]:.2f}, {center[1]:.2f}), 尺寸=({size[0]:.2f}, {size[1]:.2f}), 角度={angle:.2f}")
        
        center_int = (int(center[0]), int(center[1]))
        
        extra_data = {
            'rect_width': size[0],
            'rect_height': size[1],
            'rect': rect,
            'contour': contour
        }
        
        print(f"DR模式最終結果: 中心={center_int}, 角度={angle:.2f}度")
        return center_int, angle, extra_data
    
    def detect_angle(self, image, mode=0) -> AngleResult:
        """優化版角度檢測主函數 - 整合簡化CASE算法"""
        start_time = time.perf_counter()
        
        try:
            # 格式轉換處理
            if len(image.shape) == 2:
                bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 1:
                gray_image = image.squeeze()
                bgr_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 3:
                bgr_image = image
            else:
                raise Exception(f"不支援的圖像格式: {image.shape}")
            
            # 影像前處理 - 使用優化版
            pt_img = self.get_pre_treatment_image_optimized(bgr_image)
            
            # 根據模式選擇不同的輪廓檢測策略
            if mode == 0:
                # CASE模式：使用簡化版算法
                rst_contours = self.get_main_contours(pt_img)
                if rst_contours is None:
                    raise Exception("CASE模式: 未檢測到有效輪廓")
                
                # 選擇最大輪廓（面積排序）
                rst_contours.sort(key=cv2.contourArea, reverse=True)
                rst_contour = rst_contours[0]
            else:
                # DR模式：選擇第一個有效輪廓
                rst_contours = self.get_main_contours(pt_img)
                if rst_contours is None:
                    raise Exception("DR模式: 未檢測到有效輪廓")
                rst_contour = rst_contours[0]
            
            contour_area = cv2.contourArea(rst_contour)
            print(f"檢測到輪廓面積: {contour_area:.0f} 像素")
            
            # 面積檢查
            min_area_threshold = 50
            if contour_area < min_area_threshold:
                return AngleResult(
                    success=False, center=None, angle=None,
                    major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                    contour_area=contour_area, processing_time=0, capture_time=0,
                    total_time=(time.perf_counter() - start_time) * 1000,
                    error_message=f"輪廓面積太小: {contour_area:.0f} < {min_area_threshold}"
                )
            
            # 角度檢測算法選擇
            if mode == 0:
                # CASE模式：使用簡化版算法
                result = self._detect_angle_case_mode_optimized(rst_contour, bgr_image)
            else:
                # DR模式：使用原有算法
                result = self._detect_angle_dr_mode(rst_contour)
            
            if result is None:
                return AngleResult(
                    success=False, center=None, angle=None,
                    major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                    contour_area=contour_area, processing_time=0, capture_time=0,
                    total_time=(time.perf_counter() - start_time) * 1000,
                    error_message="角度計算失敗"
                )
            
            center, angle, extra_data = result
            processing_time = (time.perf_counter() - start_time) * 1000
            
            return AngleResult(
                success=True,
                center=center,
                angle=angle,
                major_axis=extra_data.get('major_axis'),
                minor_axis=extra_data.get('minor_axis'),
                rect_width=extra_data.get('rect_width'),
                rect_height=extra_data.get('rect_height'),
                contour_area=contour_area,
                processing_time=processing_time,
                capture_time=0,
                total_time=processing_time
            )
            
        except Exception as e:
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0,
                total_time=(time.perf_counter() - start_time) * 1000,
                error_message=str(e)
            )

class CCD3AngleDetectionService:
    def __init__(self):
        self.base_address = 800
        self.modbus_client = None
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        
        # 組件初始化 - 使用優化版檢測器
        self.state_machine = SystemStateMachine()
        self.angle_detector = OptimizedAngleDetector()
        self.camera = None
        
        # 性能優化：參數快取和監控
        self._last_params = {}
        self._params_changed = True
        self.perf_monitor = PerformanceMonitor()
        
        # 調試圖像儲存
        self.debug_enabled = True
        # 修改調試圖像儲存目錄
        self.debug_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'debug_images')
        self._ensure_debug_dir()
        self.socketio_instance = None  # 將在main中設置
        # 控制變量
        self.last_control_command = 0
        self.command_processing = False
        self.handshake_thread = None
        self.stop_handshake = False
        
        # 統計資訊
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        
        # 預設檢測參數 - 針對簡化CASE算法優化
        self.default_detection_params = {
            'detection_mode': 0,        # CASE模式0 (簡化版)
            'min_area_rate': 50,        # 0.05 → 50
            'sequence_mode': 1,         # 1=序列輪廓選擇
            'gaussian_kernel': 3,       # 高斯模糊核大小
            'threshold_mode': 1,        # 1=手動閾值（參考新式流程）
            'manual_threshold': 150     # 手動閾值150（參考新式流程）
        }
        
        # 配置檔案
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ccd3_config.json')
        self.load_config()
        
        # 預設參數已寫入標誌
        self.default_params_written = False

    def initialize_camera(self, ip_address: str = "192.168.1.10") -> bool:
        """初始化相機 - 適配新版camera_manager API"""
        try:
            print(f"正在初始化相機，IP地址: {ip_address}")
            
            if self.camera:
                print("關閉現有相機連接...")
                self.camera.disconnect()
                self.camera = None
            
            # 使用新版CameraConfig，增加頻寬控制參數
            config = CameraConfig(
                name="ccd3_camera",
                ip=ip_address,
                exposure_time=20000.0,
                gain=200.0,
                frame_rate=30.0,
                width=2592,
                height=1944,
                bandwidth_limit_mbps=200,  # 新增：200 MB/s頻寬限制
                enable_bandwidth_control=True,  # 新增：啟用頻寬控制
                packet_size=1500,  # 新增：設置包大小
                packet_delay=5000,  # 新增：包間延遲
                buffer_count=1,    # 新增：最小緩存
                use_latest_frame_only=True  # 新增：只保留最新幀
            )
            
            self.camera = OptimizedCamera(config, logger)
            
            if self.camera.connect():
                print(f"CCD3相機已成功連接: {ip_address}")
                print(f"頻寬限制已設置為: {config.bandwidth_limit_mbps} MB/s")
                
                if self.camera.start_streaming():
                    print("相機串流啟動成功")
                    
                    try:
                        # 使用新版API方法名稱
                        test_image = self.camera.capture_latest_frame()
                        if test_image is not None:
                            print(f"相機測試成功，圖像尺寸: {test_image.data.shape}")
                            self.state_machine.set_initialized(True)
                            self.state_machine.set_alarm(False)
                            return True
                        else:
                            print("相機測試失敗: 無法捕獲圖像")
                            self.state_machine.set_alarm(True)
                            return False
                    except Exception as e:
                        print(f"相機測試異常: {e}")
                        self.state_machine.set_alarm(True)
                        return False
                else:
                    print("相機串流啟動失敗")
                    self.state_machine.set_alarm(True)
                    return False
            else:
                print(f"相機連接失敗: {ip_address}")
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            print(f"相機初始化錯誤: {e}")
            self.state_machine.set_alarm(True)
            return False

    def capture_and_detect_angle(self, mode: int = 0) -> AngleResult:
        """修改版拍照並檢測角度 - 適配新版camera_manager API"""
        if not self.camera:
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="Camera not initialized"
            )
        
        if not getattr(self.camera, 'is_streaming', False):
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="Camera streaming not started"
            )
        
        capture_start = time.perf_counter()
        
        try:
            # 使用新版API方法名稱
            frame_data = self.camera.capture_latest_frame()
            
            if frame_data is None:
                raise Exception("Image capture failed")
            
            image = frame_data.data
            capture_time = (time.perf_counter() - capture_start) * 1000
            
            # 參數快取機制
            detection_params = self.read_detection_parameters_cached()
            if detection_params and self._params_changed:
                self.angle_detector.update_params(**detection_params)
                self._params_changed = False
            
            # 執行角度檢測
            result = self.angle_detector.detect_angle(image, mode)
            
            # 準備檢測數據用於調試圖像
            detection_data = {
                'mode': mode,
                'contour': None,
                'rect': None
            }
            
            # 重新獲取輪廓數據用於調試顯示
            if result.success:
                try:
                    pt_img = self.angle_detector.get_pre_treatment_image_optimized(image)
                    
                    if mode == 0:
                        # CASE模式
                        rst_contours = self.angle_detector.get_main_contours(pt_img)
                        if rst_contours:
                            rst_contours.sort(key=cv2.contourArea, reverse=True)
                            detection_data['contour'] = rst_contours[0]
                    else:
                        # DR模式
                        rst_contours = self.angle_detector.get_main_contours(pt_img)
                        if rst_contours:
                            detection_data['contour'] = rst_contours[0]
                    
                    # 計算矩形數據
                    if detection_data['contour'] is not None:
                        detection_data['rect'] = cv2.minAreaRect(detection_data['contour'])
                        print(f"檢測數據準備完成: 輪廓面積={cv2.contourArea(detection_data['contour']):.0f}")
                
                except Exception as contour_e:
                    print(f"檢測數據準備失敗: {contour_e}")
            
            # 使用新的連續儲存方法
            self.save_continuous_debug_images(image, result, detection_data)
            
            result.capture_time = capture_time
            result.total_time = (time.perf_counter() - capture_start) * 1000
            
            # 性能監控
            self.perf_monitor.add_result(result)
            
            if result.success:
                self.operation_count += 1
                print(f"角度檢測成功: 中心{result.center}, 角度{result.angle:.2f}度")
            else:
                self.error_count += 1
                print(f"角度檢測失敗: {result.error_message}")
            
            return result
            
        except Exception as e:
            self.error_count += 1
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0,
                capture_time=(time.perf_counter() - capture_start) * 1000,
                total_time=(time.perf_counter() - capture_start) * 1000,
                error_message=str(e)
            )

    def _execute_command_async(self, command: int):
        """異步執行指令 - 適配新版camera_manager API"""
        try:
            print(f"開始執行指令: {command}")
            
            if command == 8:
                print("執行拍照指令...")
                if self.camera and getattr(self.camera, 'is_streaming', False):
                    # 使用新版API方法名稱
                    frame_data = self.camera.capture_latest_frame()
                    if frame_data is not None:
                        print(f"拍照完成，圖像尺寸: {frame_data.data.shape}")
                    else:
                        print("拍照失敗")
                        self.error_count += 1
                else:
                    print("拍照失敗: 相機未初始化")
                    self.error_count += 1
                        
            elif command == 16:
                print("執行拍照+增強調試版角度檢測指令...")
                
                # 讀取檢測模式
                mode_result = self.modbus_client.read_holding_registers(
                    address=self.base_address + 10, count=1, slave=1
                )
                detection_mode = 0
                if not mode_result.isError():
                    detection_mode = mode_result.registers[0]
                
                print(f"使用檢測模式: {detection_mode} ({'增強調試CASE' if detection_mode == 0 else 'DR'})")
                
                result = self.capture_and_detect_angle(detection_mode)
                self.write_detection_result(result)
                
                if result.success:
                    print(f"增強調試版角度檢測完成: 中心{result.center}, 角度{result.angle:.2f}度")
                    print(f"調試圖像已保存，包含完整的可視化效果")
                else:
                    print(f"增強調試版角度檢測失敗: {result.error_message}")
                    
            elif command == 32:
                print("執行重新初始化指令...")
                success = self.initialize_camera()
                if success:
                    print("重新初始化成功")
                    self.default_params_written = False
                else:
                    print("重新初始化失敗")
            else:
                print(f"未知指令: {command}")
                
        except Exception as e:
            print(f"指令執行異常: {e}")
            self.error_count += 1
            self.state_machine.set_alarm(True)
        
        finally:
            print(f"控制指令 {command} 執行完成")
            self.command_processing = False
            self.state_machine.set_running(False)
            if not self.state_machine.is_alarm():
                self.state_machine.set_ready(True)

    # 其他方法保持不變，繼續沿用原來的實現
    def set_socketio_instance(self, socketio_instance):
        """設置SocketIO實例用於圖像更新通知"""
        self.socketio_instance = socketio_instance

    def create_simplified_result_image(self, original_image, detection_result, detection_data=None):
        """創建簡化的結果圖像 - 只顯示角度指針和格線"""
        result_image = original_image.copy()
        
        if not (detection_result and detection_result.success and detection_data):
            # 檢測失敗時顯示失敗訊息
            img_height, img_width = result_image.shape[:2]
            font_scale = max(1.0, min(img_width, img_height) / 1000)
            cv2.putText(result_image, "DETECTION FAILED", (50, 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), 3)
            return result_image
        
        center = detection_result.center
        angle = detection_result.angle
        contour = detection_data.get('contour')
        
        print(f"創建簡化結果圖像: 中心={center}, 角度={angle:.2f}°")
        
        # 計算輪廓的等效半徑（用於格線繪製）
        if contour is not None:
            contour_area = cv2.contourArea(contour)
            equivalent_radius = math.sqrt(contour_area / math.pi)
        else:
            # 使用預設半徑
            equivalent_radius = 100
        
        print(f"等效半徑: {equivalent_radius:.1f} 像素")
        
        # 1. 繪製X軸基準線（0度基準線）
        x_line_length = int(equivalent_radius * 1.5)
        x_start = (center[0] - x_line_length, center[1])
        x_end = (center[0] + x_line_length, center[1])
        cv2.line(result_image, x_start, x_end, (128, 128, 128), 2)
        
        # 2. 繪製格線指針（每隔15度，半徑0.8-1.3倍）
        inner_radius = equivalent_radius * 0.8
        outer_radius = equivalent_radius * 1.3
        
        for degree in range(0, 360, 15):
            angle_rad = math.radians(degree)
            
            # 計算內圓和外圓上的點
            inner_x = int(center[0] + inner_radius * math.cos(angle_rad))
            inner_y = int(center[1] + inner_radius * math.sin(angle_rad))
            outer_x = int(center[0] + outer_radius * math.cos(angle_rad))
            outer_y = int(center[1] + outer_radius * math.sin(angle_rad))
            
            # 繪製格線（灰色）
            cv2.line(result_image, (inner_x, inner_y), (outer_x, outer_y), (100, 100, 100), 1)
        
        # 3. 繪製圓心標記（小白圓）
        cv2.circle(result_image, center, 3, (255, 255, 255), -1)
        cv2.circle(result_image, center, 5, (0, 0, 0), 1)
        
        # 4. 繪製角度指示線（紅色，長度為1.3倍半徑）
        angle_rad = math.radians(angle)
        indicator_length = outer_radius
        indicator_x = int(center[0] + indicator_length * math.cos(angle_rad))
        indicator_y = int(center[1] + indicator_length * math.sin(angle_rad))
        
        # 繪製角度指示線（粗紅線）
        cv2.line(result_image, center, (indicator_x, indicator_y), (0, 0, 255), 4)
        
        # 在指示線末端繪製箭頭
        arrow_length = 15
        arrow_angle = 20  # 箭頭角度
        
        # 計算箭頭的兩個端點
        arrow_angle1 = angle + 180 - arrow_angle
        arrow_angle2 = angle + 180 + arrow_angle
        
        arrow_rad1 = math.radians(arrow_angle1)
        arrow_rad2 = math.radians(arrow_angle2)
        
        arrow_x1 = int(indicator_x + arrow_length * math.cos(arrow_rad1))
        arrow_y1 = int(indicator_y + arrow_length * math.sin(arrow_rad1))
        arrow_x2 = int(indicator_x + arrow_length * math.cos(arrow_rad2))
        arrow_y2 = int(indicator_y + arrow_length * math.sin(arrow_rad2))
        
        cv2.line(result_image, (indicator_x, indicator_y), (arrow_x1, arrow_y1), (0, 0, 255), 3)
        cv2.line(result_image, (indicator_x, indicator_y), (arrow_x2, arrow_y2), (0, 0, 255), 3)
        
        # 5. 只顯示角度值（右上角）- 優化文字清晰度
        img_height, img_width = result_image.shape[:2]
        
        # 動態調整字體大小，確保清晰度
        base_font_scale = min(img_width, img_height) / 1000.0
        font_scale = max(1.5, base_font_scale * 2.0)  # 增大字體
        text_thickness = max(3, int(font_scale * 2.5))  # 增加厚度
        
        angle_text = f"Angle: {angle:.2f}"
        
        # 使用更清晰的字體
        font = cv2.FONT_HERSHEY_DUPLEX  # 改用更清晰的字體
        text_size = cv2.getTextSize(angle_text, font, font_scale, text_thickness)[0]
        
        # 文字位置（右上角）
        text_x = img_width - text_size[0] - 30
        text_y = 280

        # 繪製角度文字（明亮的黃色，增加對比度）
        cv2.putText(result_image, angle_text, (text_x, text_y), 
                   font, font_scale, (0, 255, 255), text_thickness, cv2.LINE_AA)
        
        # 在圖像左上角添加更大的角度顯示（備用位置）
        large_font_scale = font_scale * 1.5
        large_text_thickness = max(4, int(large_font_scale * 2))
        large_angle_text = f"{angle:.1f}"
        
        large_text_size = cv2.getTextSize(large_angle_text, font, large_font_scale, large_text_thickness)[0]
        large_text_x = 30
        large_text_y = 280
        
        # 大號角度文字（亮綠色）
        cv2.putText(result_image, large_angle_text, (large_text_x, large_text_y), 
                   font, large_font_scale, (0, 255, 0), large_text_thickness, cv2.LINE_AA)
        
        print(f"簡化結果圖像創建完成，格線數量: {360//15}, 指示角度: {angle:.2f}°")
        return result_image

    def _ensure_debug_dir(self):
        """確保調試圖像目錄存在"""
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
            print(f"已創建調試圖像目錄: {self.debug_dir}")

    def get_latest_debug_image_filename(self):
        """獲取最新的Debug圖像檔名"""
        try:
            if not os.path.exists(self.debug_dir):
                return None
            
            files = os.listdir(self.debug_dir)
            result_files = [f for f in files if f.endswith('_result.jpg')]
            
            if not result_files:
                return None
            
            # 按時間戳排序，返回最新的
            result_files.sort(reverse=True)
            return result_files[0]
            
        except Exception as e:
            print(f"獲取最新圖像檔名失敗: {e}")
            return None
    
    def get_debug_images_list(self):
        """獲取Debug圖像列表"""
        try:
            if not os.path.exists(self.debug_dir):
                return []
            
            files = os.listdir(self.debug_dir)
            image_files = [f for f in files if f.endswith(('.jpg', '.png', '.bmp'))]
            
            # 按時間戳排序
            image_files.sort(reverse=True)
            return image_files
            
        except Exception as e:
            print(f"獲取圖像列表失敗: {e}")
            return []

    def save_continuous_debug_images(self, original_image, detection_result, detection_data=None):
        """連續儲存調試圖像 - 使用時間戳格式"""
        if not self.debug_enabled:
            return
        
        try:
            # 生成時間戳檔名
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            result_filename = f"{timestamp}_result.jpg"
            result_path = os.path.join(self.debug_dir, result_filename)
            
            print(f"\n=== CCD3 連續Debug圖像儲存 ===")
            print(f"時間戳: {timestamp}")
            
            # 創建簡化的結果圖像
            if detection_result and detection_data:
                simplified_result_image = self.create_simplified_result_image(
                    original_image, detection_result, detection_data
                )
            else:
                # 檢測失敗時的簡化圖像
                simplified_result_image = self.create_simplified_result_image(
                    original_image, detection_result, None
                )
            
            # 保存結果圖像
            cv2.imwrite(result_path, simplified_result_image)
            print(f"簡化結果圖像已保存: {result_filename}")
            
            if detection_result and detection_result.success:
                print(f"檢測成功 - 角度: {detection_result.angle:.2f}°")
            else:
                print(f"檢測失敗")
            
            print(f"圖像特色:")
            print(f"  - X軸基準線（0度參考）")
            print(f"  - 每15度格線指針")
            print(f"  - 紅色角度指示箭頭") 
            print(f"  - 僅顯示角度數值")
            print(f"=== Debug圖像儲存完成 ===\n")
            
        except Exception as e:
            print(f"連續調試圖像保存失敗: {e}")
            import traceback
            traceback.print_exc()

    # 其他所有方法保持原樣不變
    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "CCD3_Angle_Detection_Enhanced_Debug",
            "camera_config": {
                "name": "ccd3_camera",
                "ip": "192.168.1.10",
                "exposure_time": 20000.0,
                "gain": 200.0,
                "frame_rate": 30.0,
                "width": 2592,
                "height": 1944
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1
            },
            "modbus_mapping": {
                "base_address": 800
            },
            "detection_params": {
                "detection_mode": 0,
                "min_area_rate": 50,
                "sequence_mode": 1,
                "gaussian_kernel": 3,
                "threshold_mode": 1,
                "manual_threshold": 150
            }
        }
        
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
            else:
                config = default_config
                with open(self.config_file, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
            
            self.server_ip = config['tcp_server']['host']
            self.server_port = config['tcp_server']['port']
            self.base_address = config['modbus_mapping']['base_address']
            
        except Exception as e:
            print(f"配置檔案載入錯誤: {e}")

    def connect_modbus(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            print("正在連接Modbus TCP服務器...")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3
            )
            
            if self.modbus_client.connect():
                self.connection_count += 1
                print(f"CCD3角度檢測模組已連接到Modbus服務器: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"Modbus連接失敗: 無法連接到 {self.server_ip}:{self.server_port}")
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            print(f"Modbus連接錯誤: {e}")
            self.state_machine.set_alarm(True)
            return False

    def write_default_detection_params(self) -> bool:
        """寫入簡化版CASE模式預設檢測參數"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                print("無法寫入預設參數: Modbus Client未連接")
                return False
            
            print(f"\n{'='*60}")
            print(f"寫入增強調試版CASE模式預設檢測參數到ModbusTCP Server")
            print(f"{'='*60}")
            
            params_registers = [
                self.default_detection_params['detection_mode'],
                self.default_detection_params['min_area_rate'],
                self.default_detection_params['sequence_mode'],
                self.default_detection_params['gaussian_kernel'],
                self.default_detection_params['threshold_mode'],
                self.default_detection_params['manual_threshold']
            ]
            
            print(f"準備寫入增強調試版CASE模式參數:")
            print(f"   檢測模式 = {params_registers[0]} (增強調試版CASE)")
            print(f"   最小面積比例 = {params_registers[1]} (實際: {params_registers[1]/1000.0:.3f})")
            print(f"   序列模式 = {params_registers[2]} (選擇最大輪廓)")
            print(f"   高斯模糊核 = {params_registers[3]}")
            print(f"   閾值模式 = {params_registers[4]} (手動閾值模式)")
            print(f"   手動閾值 = {params_registers[5]}")
            
            write_result = self.modbus_client.write_registers(
                address=self.base_address + 10, 
                values=params_registers, 
                slave=1
            )
            
            if write_result.isError():
                print(f"預設參數寫入失敗: {write_result}")
                return False
            else:
                print(f"增強調試版CASE模式檢測參數已成功寫入")
                print(f"特色功能: 詳細的result.jpg可視化效果")
                self.default_params_written = True
                
                # 更新本地檢測器參數
                self.angle_detector.update_params(**self.default_detection_params)
                
                print(f"{'='*60}\n")
                return True
                
        except Exception as e:
            print(f"寫入預設參數發生異常: {e}")
            return False

    def read_detection_parameters_cached(self) -> Dict[str, Any]:
        """讀取檢測參數 - 快取機制"""
        params = {}
        try:
            if self.modbus_client and self.modbus_client.connected:
                result = self.modbus_client.read_holding_registers(
                    address=self.base_address + 10, count=6, slave=1
                )
                if not result.isError():
                    registers = result.registers
                    current_params = {
                        'detection_mode': registers[0],
                        'min_area_rate': registers[1],
                        'sequence_mode': registers[2],
                        'gaussian_kernel': registers[3],
                        'threshold_mode': registers[4],
                        'manual_threshold': registers[5]
                    }
                    
                    if current_params != self._last_params:
                        self._last_params = current_params.copy()
                        self._params_changed = True
                        params = current_params
                    
        except Exception as e:
            print(f"讀取檢測參數錯誤: {e}")
        
        return params

    def write_detection_result(self, result: AngleResult):
        """寫入檢測結果"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                print("無法寫入檢測結果: Modbus Client未連接")
                return
            
            all_registers = [0] * 40
            
            print(f"\n{'='*60}")
            print(f"CCD3增強調試版檢測結果寫入")
            print(f"{'='*60}")
            
            if result.success and result.center and result.angle is not None:
                print(f"檢測成功:")
                
                all_registers[0] = 1
                all_registers[1] = int(result.center[0])
                all_registers[2] = int(result.center[1])
                
                # 角度32位存儲
                angle_int = int(result.angle * 100)
                angle_high = (angle_int >> 16) & 0xFFFF
                angle_low = angle_int & 0xFFFF
                all_registers[3] = angle_high
                all_registers[4] = angle_low
                
                print(f"   中心座標: ({int(result.center[0])}, {int(result.center[1])})")
                print(f"   角度: {result.angle:.2f}° (32位存儲值: {angle_int})")
                
                if result.major_axis:
                    all_registers[5] = min(int(result.major_axis), 65535)
                if result.minor_axis:
                    all_registers[6] = min(int(result.minor_axis), 65535)
                if result.rect_width:
                    all_registers[7] = min(int(result.rect_width), 65535)
                if result.rect_height:
                    all_registers[8] = min(int(result.rect_height), 65535)
                if result.contour_area:
                    all_registers[9] = min(int(result.contour_area), 65535)
            else:
                print(f"檢測失敗，寫入失敗標誌")
                all_registers[0] = 0
            
            # 統計資訊
            all_registers[20] = min(int(result.capture_time), 65535)
            all_registers[21] = min(int(result.processing_time), 65535)
            all_registers[22] = min(int(result.total_time), 65535)
            all_registers[23] = self.operation_count & 0xFFFF
            all_registers[24] = min(self.error_count, 65535)
            all_registers[25] = min(self.connection_count, 65535)
            
            all_registers[30] = 3  # 版本號
            all_registers[31] = 4  # 次版本號(增強調試版)
            
            # 批次寫入
            write_result = self.modbus_client.write_registers(
                address=self.base_address + 40, values=all_registers, slave=1
            )
            
            if write_result.isError():
                print(f"寫入失敗: {write_result}")
            else:
                print(f"增強調試版檢測結果已成功寫入")
            
            print(f"{'='*60}\n")
            
        except Exception as e:
            print(f"寫入檢測結果發生異常: {e}")

    def _handshake_sync_loop(self):
        """握手同步循環"""
        print("CCD3握手同步線程啟動 (增強調試版)")
        retry_count = 0
        max_retries = 3
        
        while not self.stop_handshake:
            try:
                if self.modbus_client and self.modbus_client.connected:
                    if not self.default_params_written and retry_count < max_retries:
                        print(f"重試寫入增強調試版參數 (第{retry_count + 1}次)")
                        success = self.write_default_detection_params()
                        if success:
                            print("增強調試版參數重試寫入成功")
                        else:
                            retry_count += 1
                    
                    self._update_status_register()
                    self._process_control_commands()
                
                time.sleep(0.05)
                
            except Exception as e:
                print(f"握手同步錯誤: {e}")
                time.sleep(1)
        
        print("CCD3握手同步線程停止")

    def _update_status_register(self):
        """更新狀態寄存器"""
        try:
            camera_ok = self.camera is not None and getattr(self.camera, 'is_streaming', False)
            modbus_ok = self.modbus_client is not None and self.modbus_client.connected
            
            self.state_machine.set_initialized(camera_ok)
            if not (camera_ok and modbus_ok):
                if not camera_ok:
                    self.state_machine.set_alarm(True)
            
            self.modbus_client.write_register(
                address=self.base_address + 1,
                value=self.state_machine.status_register,
                slave=1
            )
            
        except Exception as e:
            print(f"狀態寄存器更新錯誤: {e}")

    def _process_control_commands(self):
        """處理控制指令"""
        try:
            result = self.modbus_client.read_holding_registers(
                address=self.base_address, count=1, slave=1
            )
            
            if result.isError():
                return
            
            control_command = result.registers[0]
            
            if control_command != self.last_control_command and control_command != 0:
                if not self.command_processing:
                    print(f"\n收到新控制指令: {control_command}")
                    self._handle_control_command(control_command)
                    self.last_control_command = control_command
            elif control_command == 0 and self.last_control_command != 0:
                print(f"PLC已清零指令，恢復Ready狀態")
                self.state_machine.set_ready(True)
                self.last_control_command = 0
                
        except Exception as e:
            print(f"控制指令處理異常: {e}")

    def _handle_control_command(self, command: int):
        """處理控制指令"""
        if not self.state_machine.is_ready():
            print(f"系統未Ready，無法執行指令 {command}")
            return
        
        print(f"開始處理控制指令: {command}")
        
        self.command_processing = True
        self.state_machine.set_ready(False)
        self.state_machine.set_running(True)
        
        threading.Thread(target=self._execute_command_async, args=(command,), daemon=True).start()

    def start_handshake_service(self):
        """啟動握手服務"""
        if not self.handshake_thread or not self.handshake_thread.is_alive():
            self.stop_handshake = False
            self.handshake_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
            self.handshake_thread.start()
            print("握手服務已啟動")
            
            if not self.default_params_written:
                print("自動寫入增強調試版參數...")
                success = self.write_default_detection_params()
                if success:
                    print("增強調試版參數自動寫入成功")
                else:
                    print("增強調試版參數自動寫入失敗，將重試")

    def stop_handshake_service(self):
        """停止握手服務"""
        print("正在停止握手服務...")
        self.stop_handshake = True
        if self.handshake_thread:
            self.handshake_thread.join(timeout=2)

    def disconnect(self):
        """斷開連接"""
        print("正在斷開所有連接...")
        self.stop_handshake_service()
        
        if self.camera:
            print("正在關閉相機連接...")
            if getattr(self.camera, 'is_streaming', False):
                self.camera.stop_streaming()
            self.camera.disconnect()
            self.camera = None
        
        if self.modbus_client:
            print("正在關閉Modbus連接...")
            self.modbus_client.close()
            self.modbus_client = None
        
        print("CCD3角度檢測模組已斷開連接")

# Flask Web應用保持與原版相同
app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'ccd3_angle_detection_enhanced_debug'
socketio = SocketIO(app, cors_allowed_origins="*")

ccd3_service = CCD3AngleDetectionService()
ccd3_service.set_socketio_instance(socketio)
@app.route('/')
def index():
    return render_template('ccd3_angle_detection.html')

@app.route('/api/modbus/set_server', methods=['POST'])
def set_modbus_server():
    data = request.json
    ip = data.get('ip', '127.0.0.1')
    port = data.get('port', 502)
    
    ccd3_service.server_ip = ip
    ccd3_service.server_port = port
    
    return jsonify({'success': True, 'message': f'Modbus服務器設置為 {ip}:{port}'})

@app.route('/api/modbus/connect', methods=['POST'])
def connect_modbus():
    success = ccd3_service.connect_modbus()
    if success:
        ccd3_service.start_handshake_service()
        return jsonify({'success': True, 'message': 'Modbus連接成功，握手服務已啟動'})
    else:
        return jsonify({'success': False, 'message': 'Modbus連接失敗'})

@app.route('/api/initialize', methods=['POST'])
def initialize_camera():
    data = request.json
    ip = data.get('ip', '192.168.1.10')
    
    success = ccd3_service.initialize_camera(ip)
    message = f'相機初始化{"成功" if success else "失敗"}'
    
    return jsonify({'success': success, 'message': message})

@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    data = request.json
    mode = data.get('mode', 0)
    
    result = ccd3_service.capture_and_detect_angle(mode)
    
    response_data = {
        'success': result.success,
        'center': [int(result.center[0]), int(result.center[1])] if result.center else None,
        'angle': float(result.angle) if result.angle is not None else None,
        'major_axis': float(result.major_axis) if result.major_axis else None,
        'minor_axis': float(result.minor_axis) if result.minor_axis else None,
        'rect_width': float(result.rect_width) if result.rect_width else None,
        'rect_height': float(result.rect_height) if result.rect_height else None,
        'contour_area': float(result.contour_area) if result.contour_area else None,
        'processing_time': float(result.processing_time),
        'capture_time': float(result.capture_time),
        'total_time': float(result.total_time)
    }
    
    if not result.success:
        response_data['error'] = result.error_message
    
    return jsonify(response_data)

@app.route('/api/performance_stats', methods=['GET'])
def get_performance_stats():
    """獲取性能統計"""
    stats = ccd3_service.perf_monitor.get_stats()
    return jsonify(stats)

@app.route('/api/debug_images', methods=['GET'])
def get_debug_images():
    """獲取調試圖像列表"""
    debug_dir = ccd3_service.debug_dir
    
    try:
        if os.path.exists(debug_dir):
            files = os.listdir(debug_dir)
            debug_files = [f for f in files if f.endswith(('.jpg', '.png', '.bmp'))]
            return jsonify({
                'images': debug_files,
                'debug_dir': debug_dir,
                'message': f'增強版調試圖像已保存到: {debug_dir}'
            })
        else:
            return jsonify({
                'images': [],
                'debug_dir': debug_dir,
                'message': '調試目錄不存在'
            })
    except Exception as e:
        return jsonify({'images': [], 'error': str(e)})
@app.route('/api/debug_images/latest', methods=['GET'])
def get_latest_debug_image():
    """獲取最新的調試圖像"""
    try:
        latest_filename = ccd3_service.get_latest_debug_image_filename()
        
        if latest_filename:
            return jsonify({
                'filename': latest_filename,
                'url': f'/api/debug_images/{latest_filename}'
            })
        else:
            return jsonify({'filename': None, 'message': '沒有找到調試圖像'})
            
    except Exception as e:
        return jsonify({'error': str(e)}), 500
@app.route('/api/toggle_debug', methods=['POST'])
def toggle_debug():
    """切換調試模式"""
    data = request.json
    enable = data.get('enable', True)
    
    ccd3_service.debug_enabled = enable
    
    status = "已啟用" if enable else "已關閉"
    return jsonify({
        'success': True,
        'message': f'增強版調試圖像保存{status}，圖像將保存到: {ccd3_service.debug_dir}',
        'enabled': enable,
        'debug_dir': ccd3_service.debug_dir
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    perf_stats = ccd3_service.perf_monitor.get_stats()
    
    return jsonify({
        'modbus_connected': ccd3_service.modbus_client and ccd3_service.modbus_client.connected,
        'camera_initialized': ccd3_service.state_machine.is_initialized(),
        'ready': ccd3_service.state_machine.is_ready(),
        'running': ccd3_service.state_machine.is_running(),
        'alarm': ccd3_service.state_machine.is_alarm(),
        'operation_count': ccd3_service.operation_count,
        'error_count': ccd3_service.error_count,
        'connection_count': ccd3_service.connection_count,
        'performance': perf_stats
    })
@app.route('/api/debug_images/<filename>')
def serve_debug_image(filename):
    """提供調試圖像檔案"""
    try:
        # 安全檢查：只允許特定格式的檔案
        if not filename.endswith(('.jpg', '.png', '.bmp')):
            return jsonify({'error': '不支援的檔案格式'}), 400
        
        # 檢查檔案是否存在
        file_path = os.path.join(ccd3_service.debug_dir, filename)
        if not os.path.exists(file_path):
            return jsonify({'error': '檔案不存在'}), 404
        
        return send_file(file_path, mimetype='image/jpeg')
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500
@app.route('/api/modbus/registers', methods=['GET'])
def get_registers():
    """讀取所有寄存器數值"""
    registers = {}
    
    try:
        if ccd3_service.modbus_client and ccd3_service.modbus_client.connected:
            # 讀取握手寄存器 (800-801)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address, count=2, slave=1
            )
            if not result.isError():
                registers['control_command'] = result.registers[0]
                registers['status_register'] = result.registers[1]
            
            # 讀取檢測參數 (810-819)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address + 10, count=10, slave=1
            )
            if not result.isError():
                registers['detection_params'] = result.registers
            
            # 讀取檢測結果 (840-859)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address + 40, count=20, slave=1
            )
            if not result.isError():
                registers['detection_results'] = result.registers
            
            # 讀取統計資訊 (880-899)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address + 80, count=20, slave=1
            )
            if not result.isError():
                registers['statistics'] = result.registers
                
    except Exception as e:
        print(f"寄存器讀取錯誤: {e}")
    
    return jsonify(registers)

@socketio.on('connect')
def handle_connect():
    emit('status_update', {'message': 'CCD3增強調試版角度檢測系統已連接'})

@socketio.on('get_status')
def handle_get_status():
    status = get_status().data
    emit('status_update', status)

def auto_initialize_system():
    print("=== CCD3增強調試版角度檢測系統自動初始化開始 ===")
    
    print("步驟1: 自動連接Modbus服務器...")
    modbus_success = ccd3_service.connect_modbus()
    if modbus_success:
        print("✓ Modbus服務器連接成功")
    else:
        print("✗ Modbus服務器連接失敗")
        return False
    
    print("步驟2: 自動連接相機...")
    camera_success = ccd3_service.initialize_camera("192.168.1.10")
    if camera_success:
        print("✓ 相機連接成功")
    else:
        print("✗ 相機連接失敗")
    
    print("步驟3: 啟動握手服務並寫入增強調試版參數...")
    ccd3_service.start_handshake_service()
    print("✓ 握手服務已啟動")
    
    print("步驟4: 等待增強調試版參數寫入完成...")
    import time
    for i in range(10):
        if ccd3_service.default_params_written:
            print("✓ 增強調試版參數寫入完成")
            break
        time.sleep(0.5)
        print(f"   等待中... ({i+1}/10)")
    
    print("=== CCD3增強調試版角度檢測系統初始化完成 ===")
    print(f"狀態: Ready={ccd3_service.state_machine.is_ready()}")
    print(f"狀態: Initialized={ccd3_service.state_machine.is_initialized()}")
    print("預設模式: 增強調試版CASE模式0")
    print("特色功能: 詳細的result.jpg可視化，包含輪廓、矩形、角度標註")
    print("調試圖像: 3_result.jpg將包含完整的檢測結果可視化")
    
    ccd3_service.state_machine.set_ready(True)
    ccd3_service.state_machine.set_alarm(False)
    print(f"最終狀態: Ready={ccd3_service.state_machine.is_ready()}")
    return True

if __name__ == '__main__':
    print("CCD3增強調試版角度辨識系統啟動中...")
    print(f"系統架構: Modbus TCP Client - 運動控制握手模式")
    print(f"基地址: {ccd3_service.base_address}")
    print(f"檢測模式: 增強調試版CASE模式0")
    print(f"特色功能:")
    print(f"  - 詳細的result.jpg可視化效果")
    print(f"  - 包含輪廓邊界、外接矩形、中心點標註")
    print(f"  - 角度方向箭頭和詳細參數顯示")
    print(f"  - 類似paste-2.txt的專業可視化風格")
    print(f"調試目錄: {ccd3_service.debug_dir}")
    
    auto_success = auto_initialize_system()
    if auto_success:
        print("系統已就緒，等待PLC指令...")
        print("當收到指令16時，將產生增強版調試圖像")
        print("3_result.jpg將包含完整的檢測結果可視化")
    else:
        print("系統初始化失敗，但Web介面仍可使用")
    
    print(f"Web介面啟動中... http://localhost:5056")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5056, debug=False)
    except KeyboardInterrupt:
        print("\n正在關閉CCD3角度檢測系統...")
        ccd3_service.disconnect()
    except Exception as e:
        print(f"系統錯誤: {e}")
        ccd3_service.disconnect()