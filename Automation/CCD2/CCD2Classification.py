# -*- coding: utf-8 -*-
"""
CCD2Classification_Enhanced.py - CCD2圖像分類模組增強版
整合Web介面與圖像分類可視化，參考CCD1架構
基於JSON條件配置的圖像分類系統，採用握手式狀態機控制
"""

import cv2
import numpy as np
import os
import json
import time
import threading
import logging
import sys
from datetime import datetime
from typing import Dict, Any, Optional, List, Tuple
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import traceback
from dataclasses import dataclass
from enum import IntEnum

# Flask Web介面
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# 導入相機管理模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))

try:
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
    print("✅ camera_manager模組導入成功")
except ImportError as e:
    print(f"❌ 無法導入camera_manager模組: {e}")
    CAMERA_MANAGER_AVAILABLE = False

# 引入圖像處理模組
try:
    from skimage.feature import local_binary_pattern
    SKIMAGE_AVAILABLE = True
except ImportError:
    print("⚠️ scikit-image不可用，使用基礎圖像處理")
    SKIMAGE_AVAILABLE = False


class ClassificationResult:
    """分類結果類"""
    def __init__(self):
        self.success = False
        self.category_id = 0
        self.confidence = 0.0
        self.matched_conditions = 0
        self.features = {}
        self.processing_time = 0.0
        self.annotated_image = None  # 新增：可視化圖像


class ModuleState(IntEnum):
    """模組狀態定義"""
    OFFLINE = 0
    READY = 1
    RUNNING = 2
    ERROR = 3


class SystemStateMachine:
    """系統狀態機 - 參考CCD1實現"""
    def __init__(self):
        self.lock = threading.RLock()
        self.ready = False
        self.running = False
        self.alarm = False
        self.initialized = False
        self.config_loaded = False
        
    def get_state_register(self) -> int:
        """獲取狀態寄存器值"""
        with self.lock:
            state = 0
            if self.ready:
                state |= 0x01
            if self.running:
                state |= 0x02
            if self.alarm:
                state |= 0x04
            if self.initialized:
                state |= 0x08
            if self.config_loaded:
                state |= 0x10
            return state
    
    def set_ready(self, ready: bool):
        with self.lock:
            self.ready = ready
            if ready:
                self.running = False
    
    def set_running(self, running: bool):
        with self.lock:
            self.running = running
            if running:
                self.ready = False
    
    def set_alarm(self, alarm: bool):
        with self.lock:
            self.alarm = alarm
            if alarm:
                self.ready = False
                self.running = False
    
    def reset_to_idle(self):
        with self.lock:
            self.running = False
            self.alarm = False
            if self.initialized and self.config_loaded:
                self.ready = True


class CameraManager:
    """相機管理器 - 參考CCD1實現"""
    def __init__(self, camera_ip="192.168.1.9"):
        self.camera_ip = camera_ip
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.camera_name = "ccd2_camera"
        self.is_connected = False
        self.lock = threading.RLock()
        
        # 初始化相機配置
        self.camera_config = CameraConfig(
            name=self.camera_name,
            ip=self.camera_ip,
            exposure_time=20000.0,
            gain=200.0,
            frame_rate=30.0,
            pixel_format=PixelFormat.BAYER_GR8,
            width=2592,
            height=1944,
            trigger_mode=CameraMode.CONTINUOUS,
            auto_reconnect=True
        )
        
    def connect(self) -> bool:
        """連接相機"""
        try:
            if not CAMERA_MANAGER_AVAILABLE:
                print("❌ camera_manager模組不可用")
                return False
                
            with self.lock:
                print(f"正在初始化相機 {self.camera_name} (IP: {self.camera_ip})")
                
                if self.camera_manager:
                    self.camera_manager.shutdown()
                
                self.camera_manager = OptimizedCameraManager()
                
                success = self.camera_manager.add_camera(self.camera_name, self.camera_config)
                if not success:
                    raise Exception("添加相機失敗")
                
                connect_result = self.camera_manager.connect_camera(self.camera_name)
                if not connect_result:
                    raise Exception("相機連接失敗")
                
                stream_result = self.camera_manager.start_streaming([self.camera_name])
                if not stream_result.get(self.camera_name, False):
                    raise Exception("開始串流失敗")
                
                # 設置增益為200
                camera = self.camera_manager.cameras[self.camera_name]
                camera.camera.MV_CC_SetFloatValue("Gain", 200.0)
                
                self.is_connected = True
                print(f"✅ 相機 {self.camera_name} 初始化成功")
                
                return True
                    
        except Exception as e:
            print(f"相機連接異常: {e}")
            self.is_connected = False
            return False
    
    def capture_image(self) -> Optional[np.ndarray]:
        """拍攝圖像"""
        if not self.is_connected or not self.camera_manager:
            print("❌ 相機未連接，無法拍攝")
            return None
        
        try:
            with self.lock:
                frame_data = self.camera_manager.get_image_data(self.camera_name, timeout=3000)
                
                if frame_data is None:
                    print("❌ 圖像拍攝失敗")
                    return None
                
                image_array = frame_data.data
                
                if len(image_array.shape) == 2:
                    display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
                else:
                    display_image = image_array
                
                print(f"✅ 圖像拍攝成功，尺寸: {display_image.shape}")
                return display_image
                    
        except Exception as e:
            print(f"圖像拍攝異常: {e}")
            return None
    
    def disconnect(self):
        """斷開相機連接"""
        try:
            with self.lock:
                if self.camera_manager:
                    self.camera_manager.shutdown()
                    self.camera_manager = None
                self.is_connected = False
                print("✅ 相機已斷開連接")
        except Exception as e:
            print(f"相機斷開失敗: {e}")

    @property
    def connected(self):
        """為了保持兼容性"""
        return self.is_connected


class ImageProcessor:
    """圖像處理器 - 增強版本支援可視化"""
    
    @staticmethod
    def apply_roi_to_image(image: np.ndarray, roi: Tuple[int, int, int, int]) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
        """應用ROI到圖像，返回處理後圖像和實際ROI"""
        if roi is None:
            return image, (0, 0, image.shape[1], image.shape[0])
        
        x, y, w, h = roi
        height, width = image.shape[:2]
        
        # 確保ROI範圍在圖像內
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = min(w, width - x)
        h = min(h, height - y)
        
        actual_roi = (x, y, w, h)
        
        if w > 0 and h > 0:
            return image[y:y+h, x:x+w], actual_roi
        
        return image, actual_roi
    
    @staticmethod
    def analyze_brightness_distribution(image: np.ndarray) -> Dict[str, float]:
        """分析亮度分布"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 計算直方圖
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        
        # 計算統計特徵
        mean_brightness = np.mean(gray)
        std_brightness = np.std(gray)
        
        # 計算亮度分布的偏度和峰度
        flat_pixels = gray.flatten()
        
        # 中心矩
        if std_brightness > 0:
            normalized_pixels = (flat_pixels - mean_brightness) / std_brightness
            # 偏度 (skewness)
            skewness = np.mean(normalized_pixels ** 3)
            # 峰度 (kurtosis)
            kurtosis = np.mean(normalized_pixels ** 4) - 3
        else:
            skewness = 0
            kurtosis = 0
        
        return {
            'histogram': hist,
            'mean': mean_brightness,
            'std': std_brightness,
            'skewness': skewness,
            'kurtosis': kurtosis
        }
    
    @staticmethod
    def create_visualization_image(image: np.ndarray, features: Dict[str, Any], 
                                   roi: Optional[Tuple[int, int, int, int]] = None,
                                   classification_result: Optional[ClassificationResult] = None) -> np.ndarray:
        """創建可視化圖像"""
        vis_image = image.copy()
        
        # 確保圖像是BGR格式
        if len(vis_image.shape) == 2:
            vis_image = cv2.cvtColor(vis_image, cv2.COLOR_GRAY2BGR)
        
        height, width = vis_image.shape[:2]
        
        # 繪製ROI框
        if roi and roi != (0, 0, width, height):
            x, y, w, h = roi
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(vis_image, "ROI", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 繪製特徵資訊
        info_y = 30
        cv2.putText(vis_image, f"Mean: {features.get('mean', 0):.2f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 25
        cv2.putText(vis_image, f"Std: {features.get('std', 0):.2f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 25
        cv2.putText(vis_image, f"Skew: {features.get('skewness', 0):.3f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 25
        cv2.putText(vis_image, f"Kurt: {features.get('kurtosis', 0):.3f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 繪製分類結果
        if classification_result:
            result_y = height - 80
            if classification_result.success:
                color = (0, 255, 0)  # 綠色
                result_text = f"Category: {classification_result.category_id}"
                confidence_text = f"Confidence: {classification_result.confidence:.1f}%"
            else:
                color = (0, 0, 255)  # 紅色
                result_text = "No Match"
                confidence_text = "Confidence: 0.0%"
            
            cv2.putText(vis_image, result_text, (10, result_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(vis_image, confidence_text, (10, result_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return vis_image
    
    @staticmethod
    def process_image_pipeline(image: np.ndarray, params: Dict[str, Any]) -> Dict[str, Any]:
        """圖像處理管線 - 增強版本"""
        try:
            if image is None:
                return {}
            
            results = {}
            original_image = image.copy()
            
            # 轉換為灰階
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            
            # 套用ROI
            roi_enabled = params.get('roi_enabled', False)
            roi = None
            if roi_enabled:
                roi = (params.get('roi_x', 0), params.get('roi_y', 0), 
                      params.get('roi_width', 100), params.get('roi_height', 100))
                gray, actual_roi = ImageProcessor.apply_roi_to_image(gray, roi)
                results['actual_roi'] = actual_roi
            
            # 高斯模糊
            gaussian_kernel = params.get('gaussian_kernel', 5)
            if gaussian_kernel > 1:
                if gaussian_kernel % 2 == 0:
                    gaussian_kernel += 1
                gray_blurred = cv2.GaussianBlur(gray, (gaussian_kernel, gaussian_kernel), 0)
            else:
                gray_blurred = gray
            
            # 二值化處理
            use_otsu = params.get('use_otsu', True)
            if use_otsu:
                _, binary = cv2.threshold(gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            else:
                manual_threshold = params.get('manual_threshold', 127)
                _, binary = cv2.threshold(gray_blurred, manual_threshold, 255, cv2.THRESH_BINARY)
            
            # Canny邊緣檢測
            canny_low = params.get('canny_low', 50)
            canny_high = params.get('canny_high', 150)
            canny = cv2.Canny(gray_blurred, canny_low, canny_high)
            
            # LBP紋理分析
            if SKIMAGE_AVAILABLE:
                lbp_radius = params.get('lbp_radius', 3)
                lbp_points = params.get('lbp_points', 24)
                lbp = local_binary_pattern(gray, lbp_points, lbp_radius, method='uniform')
                results['lbp'] = lbp
            
            # 亮度分布分析
            brightness_stats = ImageProcessor.analyze_brightness_distribution(gray)
            
            results.update({
                'original_image': original_image,
                'gray': gray,
                'binary': binary,
                'canny': canny,
                'brightness_stats': brightness_stats,
                'roi': roi
            })
            
            return results
            
        except Exception as e:
            print(f"圖像處理錯誤: {e}")
            return {}


class ClassificationEngine:
    """分類引擎"""
    
    def __init__(self):
        self.config = None
        self.categories = []
        
    def load_config_from_json(self, json_path: str) -> bool:
        """從JSON檔案載入配置"""
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                self.config = json.load(f)
            
            self.categories = self.config.get('categories', [])
            print(f"成功載入分類配置: {len(self.categories)} 個類別")
            return True
        except Exception as e:
            print(f"載入分類配置失敗: {e}")
            return False
    
    def classify_features(self, features: Dict[str, float]) -> ClassificationResult:
        """基於特徵進行分類"""
        result = ClassificationResult()
        
        if not self.config or not self.categories:
            return result
        
        # 特徵值映射
        feature_map = {
            "平均值": features.get('mean', 0),
            "標準差": features.get('std', 0),
            "偏度": features.get('skewness', 0),
            "峰度": features.get('kurtosis', 0)
        }
        
        result.features = feature_map
        
        # 評估每個類別
        for category in self.categories:
            if not category.get('enabled', True):
                continue
            
            conditions = category.get('conditions', [])
            logic = category.get('logic', 'AND')
            
            if not conditions:
                continue
            
            # 評估條件
            condition_results = []
            matched_count = 0
            
            for condition in conditions:
                if not condition.get('enabled', True):
                    continue
                
                feature_name = condition.get('feature', '')
                operator = condition.get('operator', '>')
                threshold = condition.get('threshold', 0.0)
                
                if feature_name in feature_map:
                    value = feature_map[feature_name]
                    
                    if operator == ">":
                        condition_result = value > threshold
                    elif operator == ">=":
                        condition_result = value >= threshold
                    elif operator == "<":
                        condition_result = value < threshold
                    elif operator == "<=":
                        condition_result = value <= threshold
                    elif operator == "==":
                        condition_result = abs(value - threshold) < 0.001
                    else:
                        condition_result = False
                    
                    condition_results.append(condition_result)
                    if condition_result:
                        matched_count += 1
            
            if condition_results:
                # 根據邏輯運算計算結果
                if logic == "AND":
                    category_matched = all(condition_results)
                else:  # OR
                    category_matched = any(condition_results)
                
                if category_matched:
                    result.success = True
                    result.category_id = category.get('id', 0)
                    result.matched_conditions = matched_count
                    # 計算信心度 (匹配條件數/總條件數)
                    total_conditions = len([c for c in conditions if c.get('enabled', True)])
                    if total_conditions > 0:
                        result.confidence = matched_count / total_conditions * 100.0
                    break  # 找到第一個匹配的類別就停止
        
        return result


class CCD2ClassificationEnhanced:
    """CCD2分類服務增強版 - 整合Web介面"""
    
    def __init__(self):
        # 基本配置
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config = self.load_config()
        self.base_address = self.config['modbus_mapping']['base_address']
        
        # 組件初始化
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.camera_manager = CameraManager(self.config['camera']['ip'])
        self.classification_engine = ClassificationEngine()
        self.state_machine = SystemStateMachine()
        
        # 運行控制
        self.running = False
        self.handshake_thread = None
        self.last_command = 0
        self.command_processing = False
        
        # 統計資訊
        self.start_time = time.time()
        self.classification_count = 0
        self.error_count = 0
        self.json_load_count = 0
        
        # Web介面初始化
        self.init_web_application()
        
        # 日志設置
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # 最後的處理結果（用於Web顯示）
        self.last_classification_result = None
        self.last_processed_image = None
        
        print("CCD2圖像分類模組增強版初始化中...")
    
    def init_web_application(self):
        """初始化Web應用 - 參考CCD1架構"""
        template_dir = os.path.join(self.current_dir, 'templates')
        os.makedirs(template_dir, exist_ok=True)
        
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config['SECRET_KEY'] = 'ccd2_classification_enhanced_2024'
        
        # 初始化SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # 註冊路由和SocketIO事件
        self.register_routes()
        self.register_socketio_events()
    
    def register_routes(self):
        """註冊Flask路由"""
        
        @self.app.route('/')
        def index():
            """主頁面"""
            return render_template('ccd2_classification.html')
        
        @self.app.route('/api/status')
        def get_status():
            """獲取系統狀態"""
            return jsonify(self.get_current_status())
        
        @self.app.route('/api/modbus/connect', methods=['POST'])
        def connect_modbus():
            """連接Modbus服務器"""
            try:
                data = request.get_json() or {}
                if 'host' in data:
                    self.config['tcp_server']['host'] = data['host']
                if 'port' in data:
                    self.config['tcp_server']['port'] = int(data['port'])
                
                result = self.connect_modbus_server()
                return jsonify(result)
            except Exception as e:
                return jsonify({'success': False, 'message': f'連接失敗: {str(e)}'})
        
        @self.app.route('/api/initialize', methods=['POST'])
        def initialize_camera():
            """初始化相機"""
            try:
                success = self.camera_manager.connect()
                if success:
                    self.state_machine.initialized = True
                    self.state_machine.set_ready(True)
                return jsonify({
                    'success': success,
                    'message': '相機初始化成功' if success else '相機初始化失敗'
                })
            except Exception as e:
                return jsonify({'success': False, 'message': f'初始化失敗: {str(e)}'})
        
        @self.app.route('/api/capture_and_classify', methods=['POST'])
        def capture_and_classify():
            """執行拍照分類"""
            try:
                result = self.capture_and_classify()
                return jsonify({
                    'success': result.success,
                    'category_id': result.category_id,
                    'confidence': result.confidence,
                    'matched_conditions': result.matched_conditions,
                    'features': result.features,
                    'processing_time': result.processing_time
                })
            except Exception as e:
                return jsonify({'success': False, 'message': f'分類失敗: {str(e)}'})
        
        @self.app.route('/api/config/scan', methods=['GET'])
        def scan_config_files():
            """掃描條件配置檔案"""
            try:
                files = self.scan_condition_folder()
                return jsonify({'success': True, 'files': files})
            except Exception as e:
                return jsonify({'success': False, 'message': f'掃描失敗: {str(e)}'})
        
        @self.app.route('/api/config/load', methods=['POST'])
        def load_config():
            """載入分類配置"""
            try:
                success = self.load_classification_config()
                if success:
                    self.state_machine.config_loaded = True
                return jsonify({
                    'success': success,
                    'message': '配置載入成功' if success else '配置載入失敗'
                })
            except Exception as e:
                return jsonify({'success': False, 'message': f'載入失敗: {str(e)}'})
    
    def register_socketio_events(self):
        """註冊SocketIO事件"""
        
        @self.socketio.on('connect')
        def on_connect():
            print("Web客戶端已連接")
            emit('status_update', self.get_current_status())
        
        @self.socketio.on('disconnect')
        def on_disconnect():
            print("Web客戶端已斷開")
        
        @self.socketio.on('get_status')
        def on_get_status():
            emit('status_update', self.get_current_status())
    
    def get_current_status(self) -> Dict[str, Any]:
        """獲取當前系統狀態"""
        return {
            'modbus_connected': self.modbus_client is not None and self.modbus_client.is_socket_open(),
            'camera_connected': self.camera_manager.is_connected,
            'config_loaded': self.state_machine.config_loaded,
            'ready': self.state_machine.ready,
            'running': self.state_machine.running,
            'alarm': self.state_machine.alarm,
            'initialized': self.state_machine.initialized,
            'classification_count': self.classification_count,
            'error_count': self.error_count,
            'json_load_count': self.json_load_count,
            'runtime_hours': int((time.time() - self.start_time) // 3600),
            'runtime_minutes': int(((time.time() - self.start_time) % 3600) // 60),
            'last_result': {
                'success': self.last_classification_result.success if self.last_classification_result else False,
                'category_id': self.last_classification_result.category_id if self.last_classification_result else 0,
                'confidence': self.last_classification_result.confidence if self.last_classification_result else 0.0,
                'features': self.last_classification_result.features if self.last_classification_result else {}
            } if self.last_classification_result else None
        }
    
    def connect_modbus_server(self) -> Dict[str, Any]:
        """連接Modbus服務器"""
        try:
            tcp_config = self.config['tcp_server']
            self.modbus_client = ModbusTcpClient(
                host=tcp_config['host'],
                port=tcp_config['port'],
                timeout=tcp_config['timeout']
            )
            
            if self.modbus_client.connect():
                print(f"Modbus TCP連接成功: {tcp_config['host']}:{tcp_config['port']}")
                # 啟動握手服務
                if not self.running:
                    self.start_handshake_service()
                return {'success': True, 'message': 'Modbus連接成功'}
            else:
                return {'success': False, 'message': 'Modbus連接失敗'}
        except Exception as e:
            return {'success': False, 'message': f'Modbus連接異常: {e}'}
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "module_id": "CCD2圖像分類增強模組",
            "camera": {
                "ip": "192.168.1.9",
                "timeout": 5.0
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 1100
            },
            "web_server": {
                "host": "localhost",
                "port": 5052,
                "debug": False
            },
            "classification": {
                "condition_folder": "condition",
                "auto_scan": True,
                "default_params": {
                    "gaussian_kernel": 5,
                    "use_otsu": True,
                    "manual_threshold": 127,
                    "canny_low": 50,
                    "canny_high": 150,
                    "lbp_radius": 3,
                    "lbp_points": 24,
                    "roi_enabled": False,
                    "roi_x": 0,
                    "roi_y": 0,
                    "roi_width": 100,
                    "roi_height": 100
                }
            }
        }
        
        try:
            config_path = os.path.join(self.current_dir, "ccd2_enhanced_config.json")
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"載入配置檔案失敗: {e}")
            
        return default_config
    
    def scan_condition_folder(self) -> list:
        """掃描condition資料夾中的JSON檔案"""
        try:
            condition_dir = os.path.join(self.current_dir, "condition")
            
            if not os.path.exists(condition_dir):
                return []
            
            json_files = []
            for file in os.listdir(condition_dir):
                if file.lower().endswith('.json'):
                    file_path = os.path.join(condition_dir, file)
                    try:
                        # 驗證JSON格式
                        with open(file_path, 'r', encoding='utf-8') as f:
                            json.load(f)
                        json_files.append(file)
                    except:
                        print(f"警告: {file} 不是有效的JSON檔案")
            
            return sorted(json_files)
        except Exception as e:
            print(f"掃描condition資料夾失敗: {e}")
            return []
    
    def load_classification_config(self) -> bool:
        """載入分類配置"""
        try:
            condition_dir = os.path.join(self.current_dir, "condition")
            
            # 確保condition資料夾存在
            if not os.path.exists(condition_dir):
                os.makedirs(condition_dir)
                self.create_default_config(condition_dir)
            
            # 搜尋condition資料夾中的JSON檔案
            json_files = [f for f in os.listdir(condition_dir) if f.lower().endswith('.json')]
            
            if not json_files:
                self.create_default_config(condition_dir)
                json_files = [f for f in os.listdir(condition_dir) if f.lower().endswith('.json')]
            
            # 使用第一個JSON檔案
            json_files.sort()
            selected_config = json_files[0]
            config_path = os.path.join(condition_dir, selected_config)
            
            if self.classification_engine.load_config_from_json(config_path):
                self.json_load_count += 1
                return True
            return False
        except Exception as e:
            print(f"載入分類配置失敗: {e}")
            return False
    
    def create_default_config(self, condition_dir: str):
        """創建預設分類配置檔案"""
        try:
            default_config = {
                "module_info": {
                    "module_name": "CCD2分類模組",
                    "version": "2.0",
                    "created_date": datetime.now().strftime("%Y-%m-%d"),
                    "description": "預設圖像分類配置"
                },
                "categories": [
                    {
                        "id": 1,
                        "name": "類型A",
                        "enabled": True,
                        "logic": "AND",
                        "description": "高亮度物件分類",
                        "conditions": [
                            {
                                "feature": "平均值",
                                "operator": ">",
                                "threshold": 100.0,
                                "enabled": True,
                                "description": "亮度平均值大於100"
                            }
                        ]
                    },
                    {
                        "id": 2,
                        "name": "類型B",
                        "enabled": True,
                        "logic": "AND",
                        "description": "低亮度物件分類",
                        "conditions": [
                            {
                                "feature": "平均值",
                                "operator": "<=",
                                "threshold": 100.0,
                                "enabled": True,
                                "description": "亮度平均值小於等於100"
                            }
                        ]
                    }
                ]
            }
            
            default_path = os.path.join(condition_dir, "default_classification_config.json")
            with open(default_path, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
            
            print(f"✓ 已創建預設配置檔案: default_classification_config.json")
            
        except Exception as e:
            print(f"創建預設配置失敗: {e}")
    
    def read_processing_parameters(self) -> Dict[str, Any]:
        """從寄存器讀取處理參數"""
        params = self.config['classification']['default_params'].copy()
        
        try:
            if not self.modbus_client:
                return params
            
            # 讀取處理參數寄存器 (base+10 到 base+19)
            response = self.modbus_client.read_holding_registers(
                address=self.base_address + 10,
                count=10,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return params
            
            registers = response.registers
            
            params['gaussian_kernel'] = registers[0]
            params['use_otsu'] = bool(registers[1])
            params['manual_threshold'] = registers[2]
            params['canny_low'] = registers[3]
            params['canny_high'] = registers[4]
            params['lbp_radius'] = registers[5]
            params['lbp_points'] = registers[6]
            params['roi_enabled'] = bool(registers[7])
            
            # ROI參數
            if len(registers) > 8:
                params['roi_x'] = registers[8] if len(registers) > 8 else 0
                params['roi_y'] = registers[9] if len(registers) > 9 else 0
            
            # 讀取更多ROI設定
            roi_response = self.modbus_client.read_holding_registers(
                address=self.base_address + 20,
                count=4,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if not roi_response.isError():
                roi_registers = roi_response.registers
                params['roi_width'] = roi_registers[0]
                params['roi_height'] = roi_registers[1]
            
        except Exception as e:
            self.logger.error(f"讀取處理參數失敗: {e}")
        
        return params
    
    def capture_and_classify(self) -> ClassificationResult:
        """拍照並執行分類 - 增強版本"""
        start_time = time.time()
        result = ClassificationResult()
        
        try:
            # 拍攝圖像
            image = self.camera_manager.capture_image()
            if image is None:
                raise Exception("圖像拍攝失敗")
            
            # 讀取處理參數
            params = self.read_processing_parameters()
            
            # 圖像處理
            processed_results = ImageProcessor.process_image_pipeline(image, params)
            if not processed_results:
                raise Exception("圖像處理失敗")
            
            # 提取特徵
            brightness_stats = processed_results.get('brightness_stats', {})
            
            # 執行分類
            result = self.classification_engine.classify_features(brightness_stats)
            result.processing_time = time.time() - start_time
            
            # 創建可視化圖像
            roi = processed_results.get('actual_roi') or processed_results.get('roi')
            result.annotated_image = ImageProcessor.create_visualization_image(
                processed_results.get('original_image', image),
                brightness_stats,
                roi,
                result
            )
            
            # 保存結果用於Web顯示
            self.last_classification_result = result
            self.last_processed_image = result.annotated_image
            
            if result.success:
                self.classification_count += 1
                print(f"分類成功: 類別ID={result.category_id}, 信心度={result.confidence:.2f}%")
            else:
                print("分類失敗: 無匹配類別")
            
            # 寫入結果到寄存器
            self.write_classification_result(result)
            self.update_statistics(result.processing_time)
            
        except Exception as e:
            self.logger.error(f"拍照分類失敗: {e}")
            self.error_count += 1
            result.processing_time = time.time() - start_time
        
        return result
    
    def write_classification_result(self, result: ClassificationResult):
        """寫入分類結果到寄存器"""
        try:
            if not self.modbus_client:
                return
            
            # 分類結果寄存器 (base+40 到 base+59)
            result_registers = [0] * 20
            
            result_registers[0] = 1 if result.success else 0
            result_registers[1] = result.category_id
            
            # 信心度 (32位，保留2位小數)
            confidence_int = int(result.confidence * 100)
            result_registers[2] = (confidence_int >> 16) & 0xFFFF
            result_registers[3] = confidence_int & 0xFFFF
            
            result_registers[4] = result.matched_conditions
            
            # 寫入分類結果寄存器
            self.modbus_client.write_registers(
                address=self.base_address + 40,
                values=result_registers[:10],
                slave=self.config['tcp_server']['unit_id']
            )
            
            # 特徵數值寄存器 (base+50 到 base+59)
            feature_registers = [0] * 10
            
            if result.features:
                # 平均值 (32位)
                mean_int = int(result.features.get('mean', 0.0) * 100)
                feature_registers[0] = (mean_int >> 16) & 0xFFFF
                feature_registers[1] = mean_int & 0xFFFF
                
                # 標準差 (32位)
                std_int = int(result.features.get('std', 0.0) * 100)
                feature_registers[2] = (std_int >> 16) & 0xFFFF
                feature_registers[3] = std_int & 0xFFFF
                
                # 偏度 (32位，保留3位小數)
                skew_int = int(result.features.get('skewness', 0.0) * 1000)
                feature_registers[4] = (skew_int >> 16) & 0xFFFF
                feature_registers[5] = skew_int & 0xFFFF
                
                # 峰度 (32位，保留3位小數)
                kurt_int = int(result.features.get('kurtosis', 0.0) * 1000)
                feature_registers[6] = (kurt_int >> 16) & 0xFFFF
                feature_registers[7] = kurt_int & 0xFFFF
            
            # 寫入特徵數值寄存器
            self.modbus_client.write_registers(
                address=self.base_address + 50,
                values=feature_registers,
                slave=self.config['tcp_server']['unit_id']
            )
            
        except Exception as e:
            self.logger.error(f"寫入分類結果失敗: {e}")
    
    def update_statistics(self, processing_time: float):
        """更新統計資訊"""
        try:
            if not self.modbus_client:
                return
            
            # 統計資訊寄存器 (base+80 到 base+99)
            stats_registers = [0] * 20
            
            stats_registers[0] = int(processing_time * 1000)  # 處理耗時(ms)
            stats_registers[1] = self.classification_count    # 分類計數器
            stats_registers[2] = self.error_count            # 錯誤計數器
            stats_registers[3] = self.json_load_count        # JSON載入次數
            
            stats_registers[10] = 2  # 軟體版本主號 (增強版)
            stats_registers[11] = 0  # 軟體版本次號
            
            # 運行時間
            runtime = int(time.time() - self.start_time)
            stats_registers[12] = runtime // 3600           # 運行時間小時
            stats_registers[13] = (runtime % 3600) // 60    # 運行時間分鐘
            
            # 寫入統計資訊寄存器
            self.modbus_client.write_registers(
                address=self.base_address + 80,
                values=stats_registers,
                slave=self.config['tcp_server']['unit_id']
            )
            
        except Exception as e:
            self.logger.error(f"更新統計資訊失敗: {e}")
    
    def start_handshake_service(self):
        """啟動握手服務"""
        if not self.running:
            self.running = True
            self.handshake_thread = threading.Thread(target=self.handshake_service, daemon=True)
            self.handshake_thread.start()
            print("✓ 握手服務已啟動 (50ms輪詢)")
    
    def handshake_service(self):
        """握手服務線程 - 參考CCD1實現"""
        print("握手服務啟動 - 50ms輪詢間隔")
        
        while self.running:
            try:
                if not self.modbus_client or not self.modbus_client.is_socket_open():
                    time.sleep(0.05)
                    continue
                
                # 更新狀態寄存器
                state_value = self.state_machine.get_state_register()
                self.modbus_client.write_register(
                    self.base_address + 1, state_value,
                    slave=self.config['tcp_server']['unit_id']
                )
                
                # 如果正在處理指令，跳過新指令檢查
                if self.command_processing:
                    time.sleep(0.05)
                    continue
                
                # 讀取控制指令
                response = self.modbus_client.read_holding_registers(
                    address=self.base_address,
                    count=1,
                    slave=self.config['tcp_server']['unit_id']
                )
                
                if response.isError():
                    time.sleep(0.05)
                    continue
                
                command = response.registers[0]
                
                # 檢查新指令
                if command != 0 and command != self.last_command:
                    self.last_command = command
                    self.process_command(command)
                
                # 更新Web介面狀態
                if hasattr(self, 'socketio'):
                    self.socketio.emit('status_update', self.get_current_status())
                
            except Exception as e:
                self.logger.error(f"握手服務異常: {e}")
            
            time.sleep(0.05)  # 50ms輪詢間隔
    
    def process_command(self, command: int):
        """處理控制指令"""
        def command_thread():
            self.command_processing = True
            self.state_machine.set_running(True)
            
            try:
                print(f"收到控制指令: {command}")
                
                if command == 8:  # 拍照
                    image = self.camera_manager.capture_image()
                    if image is None:
                        raise Exception("拍照失敗")
                    print("拍照完成")
                    
                elif command == 16:  # 拍照+分類檢測
                    result = self.capture_and_classify()
                    print(f"分類檢測完成: 類別ID={result.category_id}")
                    
                elif command == 32:  # 重新初始化
                    self.camera_manager.connect()
                    self.load_classification_config()
                    self.state_machine.initialized = True
                    print("重新初始化完成")
                    
                elif command == 64:  # 重新載入JSON配置
                    if self.load_classification_config():
                        print("JSON配置重新載入完成")
                    else:
                        raise Exception("JSON配置載入失敗")
                
            except Exception as e:
                self.logger.error(f"指令執行失敗: {e}")
                self.error_count += 1
                self.state_machine.set_alarm(True)
            
            finally:
                self.command_processing = False
                self.state_machine.reset_to_idle()
                
                # 清除指令寄存器
                try:
                    self.modbus_client.write_register(
                        address=self.base_address,
                        value=0,
                        slave=self.config['tcp_server']['unit_id']
                    )
                except:
                    pass
        
        # 在新線程中執行指令
        thread = threading.Thread(target=command_thread, daemon=True)
        thread.start()
    
    def start_service(self):
        """啟動服務"""
        print("=" * 60)
        print("CCD2圖像分類增強模組啟動中...")
        print("=" * 60)
        print(f"系統架構: Modbus TCP Client + Web介面")
        print(f"基地址: {self.base_address}")
        print(f"相機IP: {self.config['camera']['ip']}")
        print(f"Web介面: http://{self.config['web_server']['host']}:{self.config['web_server']['port']}")
        
        try:
            # 載入分類配置
            if self.load_classification_config():
                self.state_machine.config_loaded = True
                print("✓ 分類配置載入成功")
            
            # 設置初始狀態
            self.state_machine.initialized = True
            self.state_machine.set_ready(True)
            
            print("✓ CCD2圖像分類增強模組啟動完成")
            print("✓ Web介面與API服務已準備就緒")
            
            # 啟動Web應用
            web_config = self.config['web_server']
            self.socketio.run(
                self.app,
                host=web_config['host'],
                port=web_config['port'],
                debug=web_config['debug']
            )
            
        except KeyboardInterrupt:
            print("\n收到中斷信號，正在關閉...")
        except Exception as e:
            print(f"服務啟動失敗: {e}")
        finally:
            self.stop_service()
    
    def stop_service(self):
        """停止服務"""
        print("正在停止CCD2圖像分類增強模組...")
        
        self.running = False
        
        # 斷開相機連接
        if self.camera_manager:
            self.camera_manager.disconnect()
        
        # 斷開Modbus連接
        if self.modbus_client:
            try:
                self.modbus_client.close()
                print("✓ Modbus連接已關閉")
            except:
                pass
        
        print("✓ CCD2圖像分類增強模組已安全關閉")


def main():
    """主函數"""
    service = CCD2ClassificationEnhanced()
    
    try:
        service.start_service()
    except Exception as e:
        print(f"服務運行異常: {e}")
        traceback.print_exc()
    finally:
        service.stop_service()


if __name__ == '__main__':
    main()