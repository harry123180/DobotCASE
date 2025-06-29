# -*- coding: utf-8 -*-
"""
CCD1VisionCode_YOLOv11_Enhanced.py - CCD1視覺控制系統 YOLOv11版本
整合YOLOv11物件檢測功能，支援CASE_B/CASE_F分類檢測
基於Modbus TCP Client架構，實現握手式狀態機控制
適配pymodbus 3.9.2
"""

import sys
import os
import time
import threading
import json
import base64
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import logging
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import IntEnum

# 檢查YOLOv11可用性
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("✅ YOLOv11模組導入成功")
except ImportError as e:
    print(f"❌ YOLOv11模組導入失敗: {e}")
    print("💡 請安裝: pip install ultralytics")
    YOLO_AVAILABLE = False

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    print("✅ Modbus Client模組導入成功 (pymodbus 3.9.2)")
except ImportError as e:
    print(f"❌ Modbus Client模組導入失敗: {e}")
    print("💡 請安裝: pip install pymodbus>=3.0.0")
    MODBUS_AVAILABLE = False

# 導入相機管理模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
try:
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
    print("✅ 相機管理模組導入成功")
except ImportError as e:
    print(f"❌ 無法導入 camera_manager 模組: {e}")
    CAMERA_MANAGER_AVAILABLE = False


# ==================== 枚舉定義 ====================
class ControlCommand(IntEnum):
    """控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32


class StatusBits(IntEnum):
    """狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 數據結構定義 ====================
@dataclass
class YOLODetectionResult:
    """YOLOv11檢測結果 - CASE_B/CASE_F分類"""
    success: bool = False
    case_f_count: int = 0
    case_b_count: int = 0
    case_f_coords: List[Tuple[float, float]] = None
    case_b_coords: List[Tuple[float, float]] = None
    case_f_world_coords: List[Tuple[float, float]] = None  # 新增：CASE_F世界座標
    total_detections: int = 0
    confidence_threshold: float = 0.8
    processing_time: float = 0.0
    capture_time: float = 0.0
    total_time: float = 0.0
    timestamp: str = ""
    error_message: Optional[str] = None

    def __post_init__(self):
        if self.case_f_coords is None:
            self.case_f_coords = []
        if self.case_b_coords is None:
            self.case_b_coords = []
        if self.case_f_world_coords is None:
            self.case_f_world_coords = []
        if not self.timestamp:
            self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")


@dataclass
class CalibrationStatus:
    """標定狀態"""
    intrinsic_loaded: bool = False
    extrinsic_loaded: bool = False
    transformer_valid: bool = False
    intrinsic_file: str = ""
    extrinsic_file: str = ""
    working_dir: str = ""


# ==================== YOLOv11檢測器 ====================
class YOLOv11Detector:
    """YOLOv11物件檢測器 - CASE_B/CASE_F分類"""
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.8):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.class_names = ['CASE_B', 'CASE_F']  # 更新類別名稱
        self.is_loaded = False
        
    def load_model(self) -> bool:
        """載入YOLOv11模型"""
        try:
            if not YOLO_AVAILABLE:
                print("❌ YOLOv11模組不可用")
                return False
                
            if not os.path.exists(self.model_path):
                print(f"❌ 模型檔案不存在: {self.model_path}")
                return False
                
            print(f"🔄 載入YOLOv11模型: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.is_loaded = True
            print("✅ YOLOv11模型載入成功")
            return True
            
        except Exception as e:
            print(f"❌ YOLOv11模型載入失敗: {e}")
            return False
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        print(f"🎯 置信度閾值更新為: {self.confidence_threshold}")
    
    def detect(self, image: np.ndarray) -> YOLODetectionResult:
        """執行YOLOv11檢測 - 修正JSON序列化"""
        start_time = time.time()
        result = YOLODetectionResult()
        result.confidence_threshold = self.confidence_threshold
        
        try:
            if not self.is_loaded or self.model is None:
                result.error_message = "YOLOv11模型未載入"
                return result
            
            # 執行推論
            results = self.model(image, conf=self.confidence_threshold, verbose=False)
            
            # 處理檢測結果
            if results and len(results) > 0:
                detections = results[0]
                
                if detections.boxes is not None and len(detections.boxes) > 0:
                    boxes = detections.boxes.cpu().numpy()
                    
                    for box in boxes:
                        # 獲取類別ID和置信度
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        
                        if confidence >= self.confidence_threshold:
                            # 計算中心點座標
                            x1, y1, x2, y2 = box.xyxy[0]
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            
                            # 轉換為Python原生類型以支援JSON序列化
                            center_x = float(center_x)  # 確保轉換為Python float
                            center_y = float(center_y)  # 確保轉換為Python float
                            
                            # 根據類別分類
                            if class_id == 0:  # CASE_B
                                result.case_b_coords.append((center_x, center_y))
                                result.case_b_count += 1
                            elif class_id == 1:  # CASE_F
                                result.case_f_coords.append((center_x, center_y))
                                result.case_f_count += 1
                    
                    result.total_detections = result.case_b_count + result.case_f_count
                    result.success = True
                else:
                    result.success = True  # 檢測成功但無目標
            else:
                result.success = True  # 檢測成功但無結果
                
        except Exception as e:
            result.error_message = f"YOLOv11檢測失敗: {e}"
            print(f"❌ YOLOv11檢測異常: {e}")
        
        result.processing_time = (time.time() - start_time) * 1000
        return result


# ==================== 座標轉換器 ====================
class CameraCoordinateTransformer:
    """相機座標轉換器"""
    
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.extrinsic_matrix = None
        self.is_valid_flag = False
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str) -> bool:
        """載入標定數據"""
        try:
            print(f"🔄 載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            
            # 載入內參
            try:
                intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
                
                if isinstance(intrinsic_data, dict):
                    self.camera_matrix = intrinsic_data['camera_matrix']
                    self.dist_coeffs = intrinsic_data['dist_coeffs']
                elif hasattr(intrinsic_data, 'item'):
                    dict_data = intrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.camera_matrix = dict_data['camera_matrix']
                        self.dist_coeffs = dict_data['dist_coeffs']
                    else:
                        self.camera_matrix = intrinsic_data
                        self.dist_coeffs = np.zeros((1, 5))
                else:
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))
                    
            except Exception as e1:
                try:
                    self.camera_matrix = np.load(intrinsic_file, allow_pickle=False)
                    self.dist_coeffs = np.zeros((1, 5))
                except Exception as e2:
                    raise e2
            
            # 載入外參
            try:
                self.extrinsic_matrix = np.load(extrinsic_file, allow_pickle=True)
            except Exception as e1:
                self.extrinsic_matrix = np.load(extrinsic_file, allow_pickle=False)
            
            self.is_valid_flag = True
            print(f"✅ 座標轉換器載入成功")
            return True
            
        except Exception as e:
            print(f"❌ 座標轉換器載入失敗: {e}")
            self.is_valid_flag = False
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """像素座標轉世界座標"""
        if not self.is_valid_flag:
            return []
        
        try:
            world_coords = []
            for px, py in pixel_coords:
                # 去畸變
                undistorted = cv2.undistortPoints(
                    np.array([[[px, py]]], dtype=np.float32),
                    self.camera_matrix, self.dist_coeffs
                )
                
                # 歸一化座標
                x_norm, y_norm = undistorted[0][0]
                
                # 簡化的透視投影變換到世界座標（Z=0平面）
                world_x = x_norm * 1000
                world_y = y_norm * 1000
                
                world_coords.append((world_x, world_y))
            
            return world_coords
            
        except Exception as e:
            print(f"❌ 座標轉換失敗: {e}")
            return []
    
    def is_valid(self) -> bool:
        """檢查轉換器是否有效"""
        return self.is_valid_flag


# ==================== 標定管理器 ====================
class CalibrationManager:
    """標定檔案管理器"""
    
    def __init__(self, working_dir: str):
        self.working_dir = working_dir
        self.transformer = CameraCoordinateTransformer()
        self.status = CalibrationStatus()
        self.status.working_dir = working_dir
    
    def scan_calibration_files(self) -> Dict[str, Any]:
        """詳細掃描標定檔案 - 強制要求模式"""
        try:
            print(f"🔍 強制掃描標定檔案目錄: {self.working_dir}")
            
            if not os.path.exists(self.working_dir):
                return {
                    'success': False,
                    'error': f"工作目錄不存在: {self.working_dir}"
                }
            
            all_files = os.listdir(self.working_dir)
            npy_files = [f for f in all_files if f.endswith('.npy')]
            
            print(f"📁 發現 {len(all_files)} 個檔案，其中 {len(npy_files)} 個NPY檔案")
            
            if not npy_files:
                print(f"❌ 目錄中未發現任何NPY檔案")
                return {
                    'success': False,
                    'error': f"執行檔案同層目錄未發現任何NPY檔案: {self.working_dir}"
                }
            
            print(f"\n📋 開始逐一檢查NPY檔案...")
            
            intrinsic_files = []
            extrinsic_files = []
            unknown_files = []
            
            for i, file in enumerate(npy_files, 1):
                file_path = os.path.join(self.working_dir, file)
                file_size = os.path.getsize(file_path)
                mod_time = datetime.fromtimestamp(os.path.getmtime(file_path)).strftime("%Y-%m-%d %H:%M:%S")
                
                print(f"\n{i}. 檢查檔案: {file}")
                print(f"   檔案大小: {file_size} bytes")
                print(f"   修改時間: {mod_time}")
                
                file_type_determined = False
                
                try:
                    # 嘗試載入檔案 (允許pickle)
                    print(f"   🔄 嘗試載入檔案...")
                    data = np.load(file_path, allow_pickle=True)
                    
                    print(f"   數據類型: {type(data)}")
                    if hasattr(data, 'shape'):
                        print(f"   數據形狀: {data.shape}")
                    
                    # 檢查是否為字典格式（內參常見格式）
                    if isinstance(data, dict):
                        print(f"   📊 字典格式檔案")
                        print(f"   字典鍵值: {list(data.keys())}")
                        
                        if 'camera_matrix' in data and 'dist_coeffs' in data:
                            print(f"   ✅ 識別為內參檔案 (包含camera_matrix和dist_coeffs)")
                            intrinsic_files.append(file)
                            file_type_determined = True
                        elif 'camera_matrix' in data:
                            print(f"   ✅ 識別為內參檔案 (包含camera_matrix)")
                            intrinsic_files.append(file)
                            file_type_determined = True
                    
                    elif hasattr(data, 'item') and callable(data.item):
                        print(f"   🔄 嘗試解析為字典項目...")
                        try:
                            dict_data = data.item()
                            if isinstance(dict_data, dict):
                                print(f"   📊 字典項目格式")
                                print(f"   字典鍵值: {list(dict_data.keys())}")
                                
                                if 'camera_matrix' in dict_data and 'dist_coeffs' in dict_data:
                                    print(f"   ✅ 識別為內參檔案 (字典項目包含camera_matrix和dist_coeffs)")
                                    intrinsic_files.append(file)
                                    file_type_determined = True
                                elif 'camera_matrix' in dict_data:
                                    print(f"   ✅ 識別為內參檔案 (字典項目包含camera_matrix)")
                                    intrinsic_files.append(file)
                                    file_type_determined = True
                        except Exception as e:
                            print(f"   ⚠️ 解析字典項目失敗: {e}")
                    
                    # 如果還未確定類型，基於檔案名稱和矩陣大小判斷
                    if not file_type_determined and hasattr(data, 'shape'):
                        file_lower = file.lower()
                        
                        # 檢查檔案名稱關鍵字
                        is_intrinsic_name = any(keyword in file_lower for keyword in 
                                              ['camera_matrix', 'intrinsic', '内参', 'calib', 'camera'])
                        is_extrinsic_name = any(keyword in file_lower for keyword in 
                                              ['extrinsic', '外参', 'external', 'ext'])
                        
                        print(f"   檔案名稱分析: 內參關鍵字={is_intrinsic_name}, 外參關鍵字={is_extrinsic_name}")
                        
                        if data.shape == (3, 3):
                            print(f"   📐 3x3矩陣 - 典型的相機內參矩陣")
                            if is_intrinsic_name or not is_extrinsic_name:
                                print(f"   ✅ 基於形狀和名稱識別為內參檔案")
                                intrinsic_files.append(file)
                                file_type_determined = True
                        
                        elif data.shape in [(4, 4), (3, 4)]:
                            print(f"   📐 {data.shape}矩陣 - 典型的外參變換矩陣")
                            if is_extrinsic_name or not is_intrinsic_name:
                                print(f"   ✅ 基於形狀和名稱識別為外參檔案")
                                extrinsic_files.append(file)
                                file_type_determined = True
                        
                        elif data.shape in [(5,), (6,), (8,), (1, 5), (1, 8)]:
                            print(f"   📐 {data.shape}向量 - 可能是畸變係數")
                            if is_intrinsic_name:
                                print(f"   ✅ 基於形狀和名稱識別為內參相關檔案 (畸變係數)")
                                intrinsic_files.append(file)
                                file_type_determined = True
                        
                        # 如果形狀和名稱都無法確定，嘗試基於名稱判斷
                        if not file_type_determined:
                            if is_intrinsic_name:
                                print(f"   ✅ 基於檔案名稱識別為內參檔案")
                                intrinsic_files.append(file)
                                file_type_determined = True
                            elif is_extrinsic_name:
                                print(f"   ✅ 基於檔案名稱識別為外參檔案")
                                extrinsic_files.append(file)
                                file_type_determined = True
                
                except Exception as e1:
                    print(f"   ⚠️ 允許pickle載入失敗: {e1}")
                    print(f"   🔄 嘗試不允許pickle的載入...")
                    
                    try:
                        data_no_pickle = np.load(file_path, allow_pickle=False)
                        print(f"   📊 無pickle載入成功，形狀: {data_no_pickle.shape}")
                        
                        if data_no_pickle.shape == (3, 3):
                            print(f"   ✅ 識別為內參檔案 (3x3矩陣，無pickle)")
                            intrinsic_files.append(file)
                            file_type_determined = True
                        elif data_no_pickle.shape in [(4, 4), (3, 4)]:
                            print(f"   ✅ 識別為外參檔案 ({data_no_pickle.shape}矩陣，無pickle)")
                            extrinsic_files.append(file)
                            file_type_determined = True
                        elif data_no_pickle.shape in [(5,), (6,), (8,)]:
                            print(f"   ✅ 識別為內參相關檔案 (畸變係數向量，無pickle)")
                            intrinsic_files.append(file)
                            file_type_determined = True
                            
                    except Exception as e2:
                        print(f"   ❌ 無pickle載入也失敗: {e2}")
                
                if not file_type_determined:
                    print(f"   ❓ 無法確定檔案類型")
                    unknown_files.append(file)
            
            # 移除重複項目
            intrinsic_files = list(set(intrinsic_files))
            extrinsic_files = list(set(extrinsic_files))
            
            # 按修改時間排序（最新的在前）
            intrinsic_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.working_dir, x)), reverse=True)
            extrinsic_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.working_dir, x)), reverse=True)
            
            print(f"\n📋 掃描結果統計:")
            print(f"   總檔案數: {len(all_files)}")
            print(f"   NPY檔案數: {len(npy_files)}")
            print(f"   內參檔案: {len(intrinsic_files)}個 - {intrinsic_files}")
            print(f"   外參檔案: {len(extrinsic_files)}個 - {extrinsic_files}")
            print(f"   未知檔案: {len(unknown_files)}個 - {unknown_files}")
            
            # 強制要求檢查
            error_messages = []
            
            if not intrinsic_files:
                error_msg = f"❌ 未發現任何內參檔案！"
                print(error_msg)
                error_messages.append("缺少內參檔案")
                print(f"💡 內參檔案應包含:")
                print(f"   - 檔案名包含關鍵字: camera_matrix, intrinsic, calib")
                print(f"   - 或包含3x3相機矩陣")
                print(f"   - 或字典格式包含camera_matrix鍵值")
            
            if not extrinsic_files:
                error_msg = f"❌ 未發現任何外參檔案！"
                print(error_msg)
                error_messages.append("缺少外參檔案")
                print(f"💡 外參檔案應包含:")
                print(f"   - 檔案名包含關鍵字: extrinsic, external")
                print(f"   - 或包含4x4或3x4變換矩陣")
            
            if error_messages:
                combined_error = f"執行檔案同層目錄標定檔案檢查失敗: {', '.join(error_messages)}"
                return {
                    'success': False,
                    'error': combined_error,
                    'scan_details': {
                        'total_files': len(all_files),
                        'npy_files': len(npy_files),
                        'intrinsic_found': len(intrinsic_files),
                        'extrinsic_found': len(extrinsic_files),
                        'unknown_files': unknown_files
                    }
                }
            
            print(f"✅ 標定檔案掃描成功，發現有效的內外參檔案")
            
            return {
                'success': True,
                'intrinsic_files': intrinsic_files,
                'extrinsic_files': extrinsic_files,
                'working_dir': self.working_dir,
                'total_npy_files': len(npy_files),
                'unknown_files': unknown_files,
                'scan_details': f"掃描了{len(npy_files)}個NPY檔案，發現{len(intrinsic_files)}個內參和{len(extrinsic_files)}個外參檔案"
            }
            
        except Exception as e:
            error_msg = f"掃描標定檔案時發生異常: {e}"
            print(f"❌ {error_msg}")
            import traceback
            print(f"詳細錯誤堆疊: {traceback.format_exc()}")
            return {
                'success': False,
                'error': error_msg
            }
    
    def load_calibration_data(self, intrinsic_file: str = None, extrinsic_file: str = None) -> Dict[str, Any]:
        """載入標定數據"""
        try:
            scan_result = self.scan_calibration_files()
            if not scan_result['success']:
                return scan_result
            
            # 自動選擇檔案
            if not intrinsic_file and scan_result['intrinsic_files']:
                for file in scan_result['intrinsic_files']:
                    if 'camera_matrix' in file:
                        intrinsic_file = file
                        break
                if not intrinsic_file:
                    intrinsic_file = scan_result['intrinsic_files'][0]
            
            if not extrinsic_file and scan_result['extrinsic_files']:
                for file in scan_result['extrinsic_files']:
                    if 'extrinsic' in file:
                        extrinsic_file = file
                        break
                if not extrinsic_file:
                    extrinsic_file = scan_result['extrinsic_files'][0]
            
            if not intrinsic_file or not extrinsic_file:
                return {'success': False, 'error': '缺少內參或外參檔案'}
            
            # 載入數據
            intrinsic_path = os.path.join(self.working_dir, intrinsic_file)
            extrinsic_path = os.path.join(self.working_dir, extrinsic_file)
            
            if self.transformer.load_calibration_data(intrinsic_path, extrinsic_path):
                self.status.intrinsic_loaded = True
                self.status.extrinsic_loaded = True
                self.status.transformer_valid = True
                self.status.intrinsic_file = intrinsic_file
                self.status.extrinsic_file = extrinsic_file
                
                return {
                    'success': True,
                    'message': '標定數據載入成功',
                    'intrinsic_file': intrinsic_file,
                    'extrinsic_file': extrinsic_file
                }
            else:
                return {'success': False, 'error': '標定數據載入失敗'}
                
        except Exception as e:
            return {'success': False, 'error': f"載入標定數據失敗: {e}"}
    
    def get_status(self) -> Dict[str, Any]:
        """獲取標定狀態"""
        return {
            'intrinsic_loaded': self.status.intrinsic_loaded,
            'extrinsic_loaded': self.status.extrinsic_loaded,
            'transformer_valid': self.status.transformer_valid,
            'intrinsic_file': self.status.intrinsic_file,
            'extrinsic_file': self.status.extrinsic_file,
            'working_dir': self.status.working_dir
        }


# ==================== 狀態機 ====================
class SystemStateMachine:
    """系統狀態機"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.status_register = 0b0001  # 初始Ready=1
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        """獲取狀態位"""
        with self.lock:
            return bool(self.status_register & (1 << bit_pos))
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        """設置狀態位"""
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos)
            else:
                self.status_register &= ~(1 << bit_pos)
    
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
        """重置到空閒狀態"""
        with self.lock:
            self.status_register = 0b0001  # Ready=1, 其他=0


# ==================== Modbus TCP Client服務 ====================
class EnhancedModbusTcpClientService:
    """增強版Modbus TCP Client服務 - YOLOv11版本"""
    
    def __init__(self, server_ip="127.0.0.1", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.vision_controller = None
        
        # 同步控制
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.05  # 50ms輪詢
        
        # CCD1 Modbus寄存器映射 (基地址200) - YOLOv11版本
        self.REGISTERS = {
            # 控制寄存器 (200-201)
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            
            # YOLOv11檢測參數寄存器 (210-219)
            'CONFIDENCE_HIGH': 210,        # 置信度閾值高位 (×10000存儲)
            'CONFIDENCE_LOW': 211,         # 置信度閾值低位 (×10000存儲)
            'RESERVED_212': 212,           # 保留參數
            'RESERVED_213': 213,           # 保留參數
            'RESERVED_214': 214,           # 保留參數
            'RESERVED_215': 215,           # 保留參數
            
            # YOLOv11檢測結果寄存器 (240-259)
            'CASE_F_COUNT': 240,           # CASE_F檢測數量
            'CASE_B_COUNT': 241,           # CASE_B檢測數量
            'TOTAL_DETECTIONS': 242,       # 總檢測數量
            'DETECTION_SUCCESS': 243,      # 檢測成功標誌 (0=失敗, 1=成功)
            
            # CASE_F座標寄存器 (244-253) - 最多5個
            'CASE_F_1_X': 244,            # CASE_F 1號 X座標
            'CASE_F_1_Y': 245,            # CASE_F 1號 Y座標
            'CASE_F_2_X': 246,            # CASE_F 2號 X座標
            'CASE_F_2_Y': 247,            # CASE_F 2號 Y座標
            'CASE_F_3_X': 248,            # CASE_F 3號 X座標
            'CASE_F_3_Y': 249,            # CASE_F 3號 Y座標
            'CASE_F_4_X': 250,            # CASE_F 4號 X座標
            'CASE_F_4_Y': 251,            # CASE_F 4號 Y座標
            'CASE_F_5_X': 252,            # CASE_F 5號 X座標
            'CASE_F_5_Y': 253,            # CASE_F 5號 Y座標
            
            # 世界座標寄存器 (260-279) - 擴展區域
            'WORLD_COORD_VALID': 260,      # 世界座標有效標誌
            'CASE_F_1_WORLD_X_HIGH': 261, # CASE_F 1號世界X座標高位
            'CASE_F_1_WORLD_X_LOW': 262,  # CASE_F 1號世界X座標低位
            'CASE_F_1_WORLD_Y_HIGH': 263, # CASE_F 1號世界Y座標高位
            'CASE_F_1_WORLD_Y_LOW': 264,  # CASE_F 1號世界Y座標低位
            'CASE_F_2_WORLD_X_HIGH': 265, # CASE_F 2號世界X座標高位
            'CASE_F_2_WORLD_X_LOW': 266,  # CASE_F 2號世界X座標低位
            'CASE_F_2_WORLD_Y_HIGH': 267, # CASE_F 2號世界Y座標高位
            'CASE_F_2_WORLD_Y_LOW': 268,  # CASE_F 2號世界Y座標低位
            'CASE_F_3_WORLD_X_HIGH': 269, # CASE_F 3號世界X座標高位
            'CASE_F_3_WORLD_X_LOW': 270,  # CASE_F 3號世界X座標低位
            'CASE_F_3_WORLD_Y_HIGH': 271, # CASE_F 3號世界Y座標高位
            'CASE_F_3_WORLD_Y_LOW': 272,  # CASE_F 3號世界Y座標低位
            'CASE_F_4_WORLD_X_HIGH': 273, # CASE_F 4號世界X座標高位
            'CASE_F_4_WORLD_X_LOW': 274,  # CASE_F 4號世界X座標低位
            'CASE_F_4_WORLD_Y_HIGH': 275, # CASE_F 4號世界Y座標高位
            'CASE_F_4_WORLD_Y_LOW': 276,  # CASE_F 4號世界Y座標低位
            'CASE_F_5_WORLD_X_HIGH': 277, # CASE_F 5號世界X座標高位
            'CASE_F_5_WORLD_X_LOW': 278,  # CASE_F 5號世界X座標低位
            'CASE_F_5_WORLD_Y_HIGH': 279, # CASE_F 5號世界Y座標高位
            
            # 統計資訊寄存器 (280-299)
            'LAST_CAPTURE_TIME': 280,     # 最後拍照耗時 (毫秒)
            'LAST_PROCESS_TIME': 281,     # 最後處理耗時 (毫秒)
            'LAST_TOTAL_TIME': 282,       # 最後總耗時 (毫秒)
            'OPERATION_COUNT': 283,       # 操作計數器
            'ERROR_COUNT': 284,           # 錯誤計數器
            'CONNECTION_COUNT': 285,      # 連接計數器
            'VERSION_MAJOR': 290,         # 軟體版本主版號
            'VERSION_MINOR': 291,         # 軟體版本次版號
            'UPTIME_HOURS': 292,          # 系統運行時間 (小時)
            'UPTIME_MINUTES': 293,        # 系統運行時間 (分鐘)
        }
        
        # 狀態追蹤
        self.last_control_command = 0
        self.command_processing = False
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
    
    def set_vision_controller(self, controller):
        """設置視覺控制器引用"""
        self.vision_controller = controller
    
    def connect(self) -> bool:
        """連接到Modbus TCP服務器"""
        if not MODBUS_AVAILABLE:
            print("❌ Modbus Client不可用")
            return False
        
        try:
            if self.client:
                self.client.close()
            
            print(f"🔗 正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            self.client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3.0
            )
            
            if self.client.connect():
                self.connected = True
                self.connection_count += 1
                
                # 寫入初始狀態
                self._write_initial_status()
                
                print(f"✅ Modbus TCP Client連接成功: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"❌ Modbus TCP連接失敗: {self.server_ip}:{self.server_port}")
                self.connected = False
                return False
                
        except Exception as e:
            print(f"❌ Modbus TCP連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        self.stop_sync()
        
        if self.client and self.connected:
            try:
                # 寫入斷線狀態
                self.write_register('STATUS_REGISTER', 0)
                self.client.close()
                print("🔌 Modbus TCP Client已斷開連接")
            except:
                pass
        
        self.connected = False
        self.client = None
    
    def start_sync(self):
        """啟動同步線程"""
        if self.sync_running:
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
        self.sync_thread.start()
        print("✅ Modbus握手同步線程已啟動")
    
    def stop_sync(self):
        """停止同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=2.0)
            print("🛑 Modbus握手同步線程已停止")
    
    def _handshake_sync_loop(self):
        """握手同步循環 - 50ms高頻輪詢"""
        print("🔄 握手同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 更新狀態寄存器到PLC
                self._update_status_register()
                
                # 2. 讀取控制指令並處理握手邏輯
                self._handle_control_command()
                
                # 3. 更新統計資訊
                self._update_statistics()
                
                # 4. 更新世界座標有效性標誌
                self._update_world_coord_status()
                
                # 短暫休眠
                time.sleep(self.sync_interval)
                
            except ConnectionException:
                print("❌ Modbus連接中斷，同步線程退出")
                self.connected = False
                break
                
            except Exception as e:
                print(f"❌ 同步線程錯誤: {e}")
                self.error_count += 1
                time.sleep(1.0)
        
        self.sync_running = False
        print("⏹️ 同步線程已退出")
    
    def _update_status_register(self):
        """更新狀態寄存器到PLC"""
        try:
            if self.vision_controller:
                status_value = self.vision_controller.state_machine.status_register
                self.write_register('STATUS_REGISTER', status_value)
        except:
            pass
    
    def _handle_control_command(self):
        """處理控制指令握手邏輯"""
        try:
            current_command = self.read_register('CONTROL_COMMAND')
            if current_command is None:
                return
            
            # 檢查指令變化和防重複執行
            if (current_command != self.last_control_command and 
                current_command != 0 and 
                not self.command_processing):
                
                print(f"📋 收到新控制指令: {current_command} (上次: {self.last_control_command})")
                self.last_control_command = current_command
                self.command_processing = True
                
                # 異步執行指令
                command_thread = threading.Thread(
                    target=self._execute_command, 
                    args=(ControlCommand(current_command),),
                    daemon=True
                )
                command_thread.start()
                
        except Exception as e:
            print(f"❌ 處理控制指令失敗: {e}")
    
    def _execute_command(self, command: ControlCommand):
        """執行控制指令"""
        try:
            print(f"🚀 開始處理控制指令: {command}")
            
            if not self.vision_controller:
                print("❌ 視覺控制器不存在")
                return
            
            # 檢查Ready狀態
            if not self.vision_controller.state_machine.is_ready():
                print("⚠️ 系統未Ready，忽略指令")
                return
            
            # 設置Running狀態，清除Ready
            self.vision_controller.state_machine.set_running(True)
            self.vision_controller.state_machine.set_ready(False)
            
            if command == ControlCommand.CAPTURE:
                self._handle_capture_command()
            elif command == ControlCommand.CAPTURE_DETECT:
                self._handle_capture_detect_command()
            elif command == ControlCommand.INITIALIZE:
                self._handle_initialize_command()
            else:
                print(f"⚠️ 未知控制指令: {command}")
            
        except Exception as e:
            print(f"❌ 執行控制指令異常: {e}")
            self.error_count += 1
            self.vision_controller.state_machine.set_alarm(True)
        finally:
            # 恢復Ready狀態，清除Running
            self.vision_controller.state_machine.set_running(False)
            self.command_processing = False
            self.operation_count += 1
            print(f"✅ 控制指令 {command} 執行完成")
    
    def _handle_capture_command(self):
        """處理拍照指令"""
        try:
            print("📸 執行拍照指令")
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
                print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
            else:
                print("❌ 拍照失敗")
                self.error_count += 1
                
        except Exception as e:
            print(f"❌ 拍照指令執行失敗: {e}")
            self.error_count += 1
    
    def _handle_capture_detect_command(self):
        """處理拍照+檢測指令"""
        try:
            print("🔍 執行拍照+YOLOv11檢測指令")
            result = self.vision_controller.capture_and_detect()
            
            if result and result.success:
                # 更新檢測結果到PLC
                self.update_detection_results(result)
                print(f"✅ YOLOv11檢測成功，CASE_F={result.case_f_count}, CASE_B={result.case_b_count}")
            else:
                error_msg = result.error_message if result else "檢測結果為空"
                print(f"❌ YOLOv11檢測失敗: {error_msg}")
                self.error_count += 1
                # 清空檢測結果
                self._clear_detection_results()
                
        except Exception as e:
            print(f"❌ 檢測指令執行失敗: {e}")
            self.error_count += 1
            self._clear_detection_results()
    
    def _handle_initialize_command(self):
        """處理初始化指令"""
        try:
            print("🔄 執行系統初始化指令")
            
            # 重新初始化相機
            success = self.vision_controller.initialize_camera()
            
            if success:
                self.vision_controller.state_machine.set_initialized(True)
                self.vision_controller.state_machine.set_alarm(False)
                print("✅ 系統初始化成功")
            else:
                self.vision_controller.state_machine.set_initialized(False)
                self.vision_controller.state_machine.set_alarm(True)
                print("❌ 系統初始化失敗")
                self.error_count += 1
                
        except Exception as e:
            print(f"❌ 初始化指令執行失敗: {e}")
            self.error_count += 1
    
    def _clear_detection_results(self):
        """清空檢測結果寄存器"""
        try:
            self.write_register('CASE_F_COUNT', 0)
            self.write_register('CASE_B_COUNT', 0)
            self.write_register('TOTAL_DETECTIONS', 0)
            self.write_register('DETECTION_SUCCESS', 0)
            
            # 清空CASE_F座標
            for i in range(1, 6):
                self.write_register(f'CASE_F_{i}_X', 0)
                self.write_register(f'CASE_F_{i}_Y', 0)
                
        except Exception as e:
            print(f"❌ 清空檢測結果失敗: {e}")
    
    def update_detection_results(self, result: YOLODetectionResult):
        """更新YOLOv11檢測結果到PLC"""
        try:
            # 寫入檢測數量
            self.write_register('CASE_F_COUNT', result.case_f_count)
            self.write_register('CASE_B_COUNT', result.case_b_count)
            self.write_register('TOTAL_DETECTIONS', result.total_detections)
            self.write_register('DETECTION_SUCCESS', 1 if result.success else 0)
            
            # 寫入CASE_F座標 (最多5個)
            for i in range(5):
                if i < len(result.case_f_coords):
                    x, y = result.case_f_coords[i]
                    # 確保轉換為整數類型用於寄存器寫入
                    self.write_register(f'CASE_F_{i+1}_X', int(float(x)))
                    self.write_register(f'CASE_F_{i+1}_Y', int(float(y)))
                else:
                    # 清空未使用的寄存器
                    self.write_register(f'CASE_F_{i+1}_X', 0)
                    self.write_register(f'CASE_F_{i+1}_Y', 0)
            
            # 寫入時間統計 - 確保為整數類型
            self.write_register('LAST_CAPTURE_TIME', int(float(result.capture_time)))
            self.write_register('LAST_PROCESS_TIME', int(float(result.processing_time)))
            self.write_register('LAST_TOTAL_TIME', int(float(result.total_time)))
            
            # 世界座標轉換（如果可用）
            if (result.success and result.case_f_coords and
                self.vision_controller and 
                self.vision_controller.calibration_manager.transformer.is_valid()):
                
                world_coords = result.case_f_world_coords
                
                if world_coords:
                    self.write_register('WORLD_COORD_VALID', 1)
                    print(f"🌍 世界座標轉換成功，共{len(world_coords)}個CASE_F目標")
                    
                    # 寫入前5個世界座標 (×100存儲，參考原CCD1方式)
                    for i in range(min(5, len(world_coords))):
                        world_x, world_y = world_coords[i]
                        
                        # 轉換為32位整數並分高低位存儲（×100精度）
                        world_x_int = int(float(world_x) * 100)
                        world_y_int = int(float(world_y) * 100)
                        
                        # 處理負數（使用補碼）
                        if world_x_int < 0:
                            world_x_int = world_x_int + 2**32
                        if world_y_int < 0:
                            world_y_int = world_y_int + 2**32
                        
                        self.write_register(f'CASE_F_{i+1}_WORLD_X_HIGH', (world_x_int >> 16) & 0xFFFF)
                        self.write_register(f'CASE_F_{i+1}_WORLD_X_LOW', world_x_int & 0xFFFF)
                        self.write_register(f'CASE_F_{i+1}_WORLD_Y_HIGH', (world_y_int >> 16) & 0xFFFF)
                        self.write_register(f'CASE_F_{i+1}_WORLD_Y_LOW', world_y_int & 0xFFFF)
                        
                        print(f"   CASE_F {i+1} 世界座標寫入: ({world_x:.2f}, {world_y:.2f}) mm")
                    
                    # 清空未使用的世界座標寄存器
                    for i in range(len(world_coords), 5):
                        self.write_register(f'CASE_F_{i+1}_WORLD_X_HIGH', 0)
                        self.write_register(f'CASE_F_{i+1}_WORLD_X_LOW', 0)
                        self.write_register(f'CASE_F_{i+1}_WORLD_Y_HIGH', 0)
                        self.write_register(f'CASE_F_{i+1}_WORLD_Y_LOW', 0)
                else:
                    print(f"❌ 世界座標轉換失敗")
                    self.write_register('WORLD_COORD_VALID', 0)
            else:
                self.write_register('WORLD_COORD_VALID', 0)
            
        except Exception as e:
            print(f"❌ 更新檢測結果到PLC失敗: {e}")
    
    def _update_statistics(self):
        """更新統計資訊"""
        try:
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            # 更新運行時間
            uptime_total_minutes = int((time.time() - self.start_time) / 60)
            uptime_hours = uptime_total_minutes // 60
            uptime_minutes = uptime_total_minutes % 60
            
            self.write_register('UPTIME_HOURS', uptime_hours)
            self.write_register('UPTIME_MINUTES', uptime_minutes)
            
        except:
            pass
    
    def _update_world_coord_status(self):
        """更新世界座標有效性狀態"""
        try:
            if (self.vision_controller and 
                self.vision_controller.calibration_manager.transformer.is_valid()):
                # 如果標定數據有效但還沒設置標誌，設置為有效
                current_status = self.read_register('WORLD_COORD_VALID')
                if current_status is None:
                    self.write_register('WORLD_COORD_VALID', 0)
            else:
                self.write_register('WORLD_COORD_VALID', 0)
        except:
            pass
    
    def _write_initial_status(self):
        """寫入初始狀態到PLC"""
        try:
            # 版本資訊
            self.write_register('VERSION_MAJOR', 5)  # YOLOv11版本
            self.write_register('VERSION_MINOR', 0)
            
            # 初始化置信度閾值為0.8 (×10000存儲)
            confidence_int = int(0.8 * 10000)  # 8000
            self.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
            self.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
            
            # 計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            print("📊 初始狀態已寫入PLC")
            
        except Exception as e:
            print(f"❌ 寫入初始狀態失敗: {e}")
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.write_register(address, value, slave=1)
            
            if not result.isError():
                return True
            else:
                return False
                
        except Exception as e:
            return False
    
    def read_confidence_threshold(self) -> float:
        """讀取置信度閾值"""
        try:
            high = self.read_register('CONFIDENCE_HIGH') or 0
            low = self.read_register('CONFIDENCE_LOW') or 8000
            confidence_int = (high << 16) + low
            return confidence_int / 10000.0
        except:
            return 0.8
    
    def get_connection_status(self) -> Dict[str, Any]:
        """獲取連接狀態"""
        return {
            'connected': self.connected,
            'server_ip': self.server_ip,
            'server_port': self.server_port,
            'sync_running': self.sync_running,
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(time.time() - self.start_time)
        }


# ==================== 主控制器 ====================
class CCD1VisionController:
    """CCD1視覺檢測主控制器 - YOLOv11版本"""
    
    def __init__(self):
        # 基本配置
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.camera_ip = "192.168.1.8"
        
        # 核心組件
        self.state_machine = SystemStateMachine()
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.calibration_manager = CalibrationManager(self.working_dir)
        self.yolo_detector = None
        
        # 圖像緩存
        self.last_image: Optional[np.ndarray] = None
        self.last_result: Optional[YOLODetectionResult] = None
        
        # Modbus客戶端
        self.modbus_client = EnhancedModbusTcpClientService(self.server_ip, self.server_port)
        self.modbus_client.set_vision_controller(self)
        
        # 統計信息
        self.operation_count = 0
        self.error_count = 0
        
        # 檢查並初始化YOLOv11
        self._check_yolo_availability()
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1Vision")
        self.logger.setLevel(logging.INFO)
        
        # 檢查和初始化各組件
        component_status = self._check_and_initialize_components()
        
        # 根據組件狀態執行相應的後續操作
        if component_status['modbus_connected'] and component_status['calibration_loaded']:
            # 嘗試自動初始化相機
            print("📷 嘗試自動初始化相機...")
            try:
                camera_success = self.initialize_camera()
                if camera_success:
                    print("✅ 相機自動初始化成功")
                    self.state_machine.set_initialized(True)
                    self.state_machine.set_ready(True)
                    print("🎯 系統完全就緒，進入Ready狀態")
                else:
                    print("⚠️ 相機自動初始化失敗")
            except Exception as e:
                print(f"⚠️ 相機自動初始化異常: {e}")
        else:
            print("⚠️ 核心組件未完全就緒，跳過相機自動初始化")
    def _check_yolo_availability(self):
        """檢查YOLOv11模型可用性"""
        best_pt_path = os.path.join(self.working_dir, "best.pt")
        
        if not YOLO_AVAILABLE:
            error_msg = "YOLOv11模組不可用，請安裝ultralytics"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        if not os.path.exists(best_pt_path):
            error_msg = f"YOLOv11模型檔案不存在: {best_pt_path}"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        print(f"✅ 發現YOLOv11模型: {best_pt_path}")
        self.yolo_detector = YOLOv11Detector(best_pt_path, 0.8)
        if self.yolo_detector.load_model():
            print("✅ YOLOv11模型載入成功")
        else:
            error_msg = "YOLOv11模型載入失敗"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
    def _check_and_initialize_components(self):
        """檢查和初始化各組件狀態"""
        print("🔍 檢查系統組件狀態...")
        
        # 1. 檢查Modbus客戶端連接狀態
        print("📡 檢查Modbus客戶端連接狀態...")
        modbus_connected = False
        try:
            modbus_connected = self.modbus_client.connect()
            if modbus_connected:
                self.modbus_client.start_sync()
                print("✅ Modbus客戶端已連接並啟動同步")
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                    print(f"🎯 置信度閾值已同步: {confidence}")
            else:
                print("❌ Modbus客戶端連接失敗")
        except Exception as e:
            print(f"❌ Modbus客戶端連接異常: {e}")
        
        # 2. 檢查內外參檔案載入狀態
        print("📐 檢查內外參檔案載入狀態...")
        calibration_loaded = False
        try:
            # 掃描標定檔案
            scan_result = self.calibration_manager.scan_calibration_files()
            if scan_result['success']:
                print(f"✅ 標定檔案掃描成功: {scan_result.get('scan_details', '')}")
                
                # 載入標定數據
                load_result = self.calibration_manager.load_calibration_data()
                if load_result['success']:
                    calibration_loaded = True
                    print("✅ 內外參檔案已成功載入")
                    print(f"   內參檔案: {load_result.get('intrinsic_file', 'N/A')}")
                    print(f"   外參檔案: {load_result.get('extrinsic_file', 'N/A')}")
                    print(f"   座標轉換器: 已啟用")
                else:
                    print(f"❌ 標定數據載入失敗: {load_result['error']}")
            else:
                print(f"❌ 標定檔案掃描失敗: {scan_result['error']}")
                
        except Exception as e:
            print(f"❌ 標定檔案檢查異常: {e}")
        
        # 3. 設置系統初始狀態
        if modbus_connected and calibration_loaded:
            print("🚀 所有核心組件檢查通過")
            self.state_machine.set_alarm(False)
            # 注意：相機初始化將在後續手動或自動執行
        else:
            print("⚠️ 部分核心組件檢查失敗")
            missing_components = []
            if not modbus_connected:
                missing_components.append("Modbus連接")
            if not calibration_loaded:
                missing_components.append("標定檔案")
            
            print(f"   缺失組件: {', '.join(missing_components)}")
            
            # 根據需求決定是否設置Alarm
            if not calibration_loaded:  # 標定檔案是強制要求
                print("❌ 標定檔案是強制要求，設置系統Alarm狀態")
                self.state_machine.set_alarm(True)
            elif not modbus_connected:  # Modbus連接失敗但可以繼續
                print("⚠️ Modbus連接失敗，但系統可以繼續運行")
        
        print("📊 組件檢查完成")
        return {
            'modbus_connected': modbus_connected,
            'calibration_loaded': calibration_loaded
        }
    def _add_world_coordinates_yolo(self, result: YOLODetectionResult):
        """為YOLOv11結果添加世界座標 - 修正JSON序列化"""
        try:
            if not self.calibration_manager.transformer.is_valid():
                print(f"⚠️ 標定數據無效，跳過世界座標轉換")
                return
            
            print(f"🌍 執行YOLOv11結果世界座標轉換...")
            
            # 僅轉換CASE_F座標（作為圓心處理）
            if result.case_f_coords:
                print(f"   轉換{len(result.case_f_coords)}個CASE_F目標座標")
                try:
                    # 使用原CCD1的像素到世界座標轉換方式
                    world_coords = self.calibration_manager.transformer.pixel_to_world(result.case_f_coords)
                    
                    if world_coords:
                        # 將世界座標轉換為Python原生類型並存儲到結果中
                        result.case_f_world_coords = []
                        for wx, wy in world_coords:
                            # 確保轉換為Python原生float類型
                            world_x = float(wx)
                            world_y = float(wy)
                            result.case_f_world_coords.append((world_x, world_y))
                        
                        for i, ((px, py), (wx, wy)) in enumerate(zip(result.case_f_coords, result.case_f_world_coords)):
                            print(f"   CASE_F {i+1}: 像素({px:.1f}, {py:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
                        
                        print(f"✅ 世界座標轉換成功，共轉換{len(world_coords)}個CASE_F目標")
                    else:
                        print(f"❌ 世界座標轉換失敗：轉換結果為空")
                        result.case_f_world_coords = []
                        
                except Exception as e:
                    print(f"   ❌ CASE_F座標轉換失敗: {e}")
                    result.case_f_world_coords = []
            else:
                print(f"   無CASE_F目標需要轉換")
                result.case_f_world_coords = []
                    
        except Exception as e:
            print(f"❌ YOLOv11世界座標轉換失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            result.case_f_world_coords = []
    def connect_modbus(self) -> Dict[str, Any]:
        """連接Modbus TCP服務器"""
        try:
            if self.modbus_client.connect():
                # 連接成功後啟動同步線程
                self.modbus_client.start_sync()
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                
                return {
                    'success': True,
                    'message': f'Modbus TCP連接成功: {self.server_ip}:{self.server_port}',
                    'connection_status': self.modbus_client.get_connection_status()
                }
            else:
                return {
                    'success': False,
                    'message': f'無法連接到Modbus服務器: {self.server_ip}:{self.server_port}'
                }
                
        except Exception as e:
            return {
                'success': False,
                'message': f'Modbus連接異常: {str(e)}'
            }
    
    def disconnect_modbus(self) -> Dict[str, Any]:
        """斷開Modbus連接"""
        try:
            self.modbus_client.disconnect()
            
            return {
                'success': True,
                'message': 'Modbus連接已斷開'
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'斷開Modbus連接失敗: {str(e)}'
            }
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        """初始化相機連接"""
        try:
            if ip_address:
                self.camera_ip = ip_address
            
            # 安全關閉現有相機管理器
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    self.camera_manager = None
            
            # 創建相機配置
            camera_config = CameraConfig(
                name="ccd1_camera",
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
            
            print(f"🔄 初始化相機: {self.camera_ip}")
            self.camera_manager = OptimizedCameraManager()
            
            # 添加相機
            success = self.camera_manager.add_camera("ccd1_camera", camera_config)
            if not success:
                raise Exception("添加相機失敗")
            
            # 連接相機
            connect_result = self.camera_manager.connect_camera("ccd1_camera")
            if not connect_result:
                raise Exception("相機連接失敗")
            
            # 開始串流
            stream_result = self.camera_manager.start_streaming(["ccd1_camera"])
            if not stream_result.get("ccd1_camera", False):
                raise Exception("開始串流失敗")
            
            # 設置增益
            try:
                camera = self.camera_manager.cameras["ccd1_camera"]
                camera.camera.MV_CC_SetFloatValue("Gain", 200.0)
            except Exception as e:
                print(f"⚠️ 設置增益失敗: {e}")
            
            self.state_machine.set_initialized(True)
            self.state_machine.set_alarm(False)
            self.state_machine.set_ready(True)
            print(f"✅ 相機初始化成功: {self.camera_ip}")
            return True
                
        except Exception as e:
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            self.state_machine.set_ready(False)
            print(f"❌ 相機初始化失敗: {e}")
            return False
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        """拍照"""
        if not self.camera_manager:
            return None, 0.0
        
        capture_start = time.time()
        
        try:
            frame_data = self.camera_manager.get_image_data("ccd1_camera", timeout=3000)
            
            if frame_data is None:
                return None, 0.0
            
            capture_time = time.time() - capture_start
            
            image_array = frame_data.data
            
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
            else:
                display_image = image_array
            
            return display_image, capture_time
            
        except Exception as e:
            print(f"❌ 拍照失敗: {e}")
            return None, 0.0
    
    def capture_and_detect(self) -> YOLODetectionResult:
        """拍照並進行YOLOv11檢測"""
        total_start = time.time()
        
        try:
            # 檢查YOLOv11檢測器
            if not self.yolo_detector or not self.yolo_detector.is_loaded:
                result = YOLODetectionResult()
                result.error_message = "YOLOv11檢測器未載入"
                result.total_time = (time.time() - total_start) * 1000
                return result
            
            # 拍照
            image, capture_time = self.capture_image()
            
            if image is None:
                result = YOLODetectionResult()
                result.error_message = "圖像捕獲失敗"
                result.capture_time = capture_time * 1000
                result.total_time = (time.time() - total_start) * 1000
                return result
            
            # YOLOv11檢測
            result = self.yolo_detector.detect(image)
            result.capture_time = capture_time * 1000
            result.total_time = (time.time() - total_start) * 1000
            
            # 世界座標轉換（如果有標定數據）
            if result.success and result.case_f_coords:
                print(f"🌍 開始世界座標轉換...")
                self._add_world_coordinates_yolo(result)
            
            # 創建可視化圖像
            if result.success:
                self._create_yolo_visualization(image, result)
            
            self.last_result = result
            return result
            
        except Exception as e:
            result = YOLODetectionResult()
            result.error_message = f"檢測失敗: {str(e)}"
            result.total_time = (time.time() - total_start) * 1000
            return result
    
    def _create_yolo_visualization(self, image: np.ndarray, result: YOLODetectionResult):
        """創建YOLOv11檢測結果可視化"""
        try:
            vis_image = image.copy()
            
            # 定義顏色 (BGR格式)
            colors = {
                'CASE_B': (255, 0, 0),    # 藍色
                'CASE_F': (0, 255, 0),    # 綠色
            }
            
            # 繪製CASE_F檢測結果
            for i, (x, y) in enumerate(result.case_f_coords):
                # 繪製中心點
                cv2.circle(vis_image, (int(x), int(y)), 15, colors['CASE_F'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 20, (255, 255, 255), 3)
                
                # 繪製標籤
                label = f"CASE_F {i+1}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
                
                # 標籤背景
                cv2.rectangle(vis_image, (int(x-label_size[0]//2-5), int(y-40)), 
                             (int(x+label_size[0]//2+5), int(y-10)), colors['CASE_F'], -1)
                
                # 標籤文字
                cv2.putText(vis_image, label, (int(x-label_size[0]//2), int(y-20)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 繪製CASE_B檢測結果（僅顯示中心點，不需要座標）
            for i, (x, y) in enumerate(result.case_b_coords):
                cv2.circle(vis_image, (int(x), int(y)), 10, colors['CASE_B'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 15, (255, 255, 255), 2)
            
            # 添加檢測統計信息
            stats_text = f"YOLOv11: CASE_F={result.case_f_count}, CASE_B={result.case_b_count}"
            cv2.putText(vis_image, stats_text, (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            
            # 添加置信度閾值信息
            conf_text = f"Confidence >= {result.confidence_threshold:.1f}"
            cv2.putText(vis_image, conf_text, (20, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            
            # 保存可視化圖像
            self.last_image = vis_image
            
        except Exception as e:
            print(f"❌ 創建可視化失敗: {e}")
    
    def get_image_base64(self) -> Optional[str]:
        """獲取當前圖像的base64編碼"""
        if self.last_image is None:
            return None
        
        try:
            height, width = self.last_image.shape[:2]
            if width > 800:
                scale = 800 / width
                new_width = 800
                new_height = int(height * scale)
                display_image = cv2.resize(self.last_image, (new_width, new_height))
            else:
                display_image = self.last_image
            
            _, buffer = cv2.imencode('.jpg', display_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return f"data:image/jpeg;base64,{image_base64}"
            
        except Exception as e:
            self.logger.error(f"圖像編碼失敗: {e}")
            return None
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        if self.yolo_detector:
            self.yolo_detector.update_confidence_threshold(threshold)
            
            # 同步到Modbus寄存器
            if self.modbus_client.connected:
                confidence_int = int(threshold * 10000)
                self.modbus_client.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
                self.modbus_client.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        status = {
            'ready': self.state_machine.is_ready(),
            'running': self.state_machine.is_running(),
            'alarm': self.state_machine.is_alarm(),
            'initialized': self.state_machine.is_initialized(),
            'camera_connected': self.camera_manager is not None and "ccd1_camera" in self.camera_manager.cameras,
            'yolo_loaded': self.yolo_detector is not None and self.yolo_detector.is_loaded,
            'confidence_threshold': self.yolo_detector.confidence_threshold if self.yolo_detector else 0.8,
            'modbus_connected': self.modbus_client.connected,
            'calibration_valid': self.calibration_manager.transformer.is_valid(),
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'last_result': asdict(self.last_result) if self.last_result else None,
            'calibration_status': self.calibration_manager.get_status()
        }
        
        return status
    
    def disconnect(self):
        """斷開所有連接"""
        # 斷開相機連接
        if self.camera_manager:
            self.camera_manager.shutdown()
            self.camera_manager = None
        
        # 斷開Modbus連接
        self.modbus_client.disconnect()
        
        self.logger.info("所有連接已斷開")


# ==================== Flask Web應用 ====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ccd1_yolo_vision_v5'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全局控制器實例
controller = None

def initialize_controller():
    """初始化控制器並自動連接所有組件"""
    global controller
    try:
        print("🚀 正在初始化CCD1視覺控制器 (YOLOv11版本)...")
        controller = CCD1VisionController()
        print("✅ CCD1視覺控制器初始化成功")
        print("🎯 所有組件已自動初始化完成")
        return True
    except Exception as e:
        print(f"❌ CCD1視覺控制器初始化失敗: {e}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
        return False


@app.route('/')
def index():
    """主頁面"""
    return render_template('ccd1_yolo_enhanced.html')


@app.route('/api/status')
def get_status():
    """獲取系統狀態"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        status = controller.get_status()
        return jsonify({'success': True, 'status': status})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/connect_modbus', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.connect_modbus()
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/disconnect_modbus', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.disconnect_modbus()
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/initialize_camera', methods=['POST'])
def initialize_camera():
    """初始化相機"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.get_json() if request.get_json() else {}
        ip = data.get('ip', controller.camera_ip)
        
        success = controller.initialize_camera(ip)
        
        result = {
            'success': success,
            'message': f'相機初始化{"成功" if success else "失敗"}: {ip}',
            'camera_ip': ip
        }
        
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    """拍照並檢測 - 修正JSON序列化"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.capture_and_detect()
        
        # 確保所有數據類型都能序列化為JSON
        def convert_to_serializable(data):
            """轉換數據為JSON可序列化格式"""
            if isinstance(data, (list, tuple)):
                return [convert_to_serializable(item) for item in data]
            elif isinstance(data, dict):
                return {key: convert_to_serializable(value) for key, value in data.items()}
            elif hasattr(data, 'item'):  # NumPy scalar
                return data.item()
            elif isinstance(data, (np.float32, np.float64)):
                return float(data)
            elif isinstance(data, (np.int32, np.int64)):
                return int(data)
            else:
                return data
        
        response = {
            'success': result.success,
            'case_f_count': int(result.case_f_count),
            'case_b_count': int(result.case_b_count),
            'total_detections': int(result.total_detections),
            'case_f_coords': convert_to_serializable(result.case_f_coords),
            'case_b_coords': convert_to_serializable(result.case_b_coords),
            'case_f_world_coords': convert_to_serializable(getattr(result, 'case_f_world_coords', [])),
            'confidence_threshold': float(result.confidence_threshold),
            'capture_time': float(result.capture_time),
            'processing_time': float(result.processing_time),
            'total_time': float(result.total_time),
            'timestamp': str(result.timestamp),
            'image': controller.get_image_base64(),
            'error_message': result.error_message,
            'world_coord_valid': len(getattr(result, 'case_f_world_coords', [])) > 0
        }
        
        socketio.emit('detection_result', response)
        return jsonify(response)
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/update_confidence', methods=['POST'])
def update_confidence():
    """更新置信度閾值"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.json
        threshold = float(data.get('threshold', 0.8))
        
        controller.update_confidence_threshold(threshold)
        
        return jsonify({
            'success': True,
            'confidence_threshold': controller.yolo_detector.confidence_threshold if controller.yolo_detector else threshold,
            'message': f'置信度閾值更新為: {threshold}'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/scan_calibration', methods=['GET'])
def scan_calibration():
    """掃描標定檔案"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.calibration_manager.scan_calibration_files()
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/load_calibration', methods=['POST'])
def load_calibration():
    """載入標定數據"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.get_json() if request.get_json() else {}
        intrinsic_file = data.get('intrinsic_file')
        extrinsic_file = data.get('extrinsic_file')
        
        result = controller.calibration_manager.load_calibration_data(intrinsic_file, extrinsic_file)
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/modbus/registers', methods=['GET'])
def get_modbus_registers():
    """獲取Modbus寄存器狀態"""
    if not controller or not controller.modbus_client.connected:
        return jsonify({'success': False, 'error': 'Modbus未連接'})
    
    try:
        registers = {}
        modbus_client = controller.modbus_client
        
        # 控制寄存器
        registers['200_控制指令'] = modbus_client.read_register('CONTROL_COMMAND')
        registers['201_狀態寄存器'] = modbus_client.read_register('STATUS_REGISTER')
        
        # 置信度閾值
        conf_high = modbus_client.read_register('CONFIDENCE_HIGH') or 0
        conf_low = modbus_client.read_register('CONFIDENCE_LOW') or 8000
        confidence = (conf_high << 16) + conf_low
        registers['210-211_置信度閾值'] = f"{confidence/10000.0:.2f}"
        
        # 檢測結果
        registers['240_CASE_F數量'] = modbus_client.read_register('CASE_F_COUNT')
        registers['241_CASE_B數量'] = modbus_client.read_register('CASE_B_COUNT')
        registers['242_總檢測數量'] = modbus_client.read_register('TOTAL_DETECTIONS')
        registers['243_檢測成功標誌'] = modbus_client.read_register('DETECTION_SUCCESS')
        
        # 世界座標寄存器 (260-279)
        world_coord_valid = modbus_client.read_register('WORLD_COORD_VALID')
        registers['260_世界座標有效'] = world_coord_valid
        
        if world_coord_valid:
            for i in range(1, 6):
                x_high = modbus_client.read_register(f'CASE_F_{i}_WORLD_X_HIGH') or 0
                x_low = modbus_client.read_register(f'CASE_F_{i}_WORLD_X_LOW') or 0
                y_high = modbus_client.read_register(f'CASE_F_{i}_WORLD_Y_HIGH') or 0
                y_low = modbus_client.read_register(f'CASE_F_{i}_WORLD_Y_LOW') or 0
                
                # 組合32位座標值並轉換為實際座標
                world_x_int = (x_high << 16) + x_low
                world_y_int = (y_high << 16) + y_low
                
                # 處理負數（補碼轉換）
                if world_x_int >= 2**31:
                    world_x_int -= 2**32
                if world_y_int >= 2**31:
                    world_y_int -= 2**32
                
                world_x_mm = world_x_int / 100.0
                world_y_mm = world_y_int / 100.0
                
                registers[f'{260+i*4-3}_CASE_F_{i}_世界X'] = f"{world_x_mm:.2f}mm"
                registers[f'{260+i*4-1}_CASE_F_{i}_世界Y'] = f"{world_y_mm:.2f}mm"
        
        # 統計資訊
        registers['280_拍照耗時ms'] = modbus_client.read_register('LAST_CAPTURE_TIME')
        registers['281_處理耗時ms'] = modbus_client.read_register('LAST_PROCESS_TIME')
        registers['282_總耗時ms'] = modbus_client.read_register('LAST_TOTAL_TIME')
        registers['283_操作計數'] = modbus_client.read_register('OPERATION_COUNT')
        registers['284_錯誤計數'] = modbus_client.read_register('ERROR_COUNT')
        
        return jsonify({
            'success': True,
            'registers': registers,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """斷開所有連接"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        controller.disconnect()
        socketio.emit('status_update', controller.get_status())
        return jsonify({'success': True, 'message': '所有連接已斷開'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@socketio.on('connect')
def handle_connect():
    """客戶端連接"""
    if controller:
        emit('status_update', controller.get_status())


@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開"""
    pass


def main():
    """主函數"""
    print("=" * 80)
    print("🚀 CCD1視覺控制系統啟動中 (YOLOv11版本)...")
    print("📊 功能特性:")
    print("   • YOLOv11物件檢測 (CASE_B/CASE_F分類)")
    print("   • 握手式狀態機控制")
    print("   • Modbus TCP Client架構")
    print("   • 50ms高頻輪詢")
    print("   • 世界座標轉換")
    print("   • 置信度閾值動態調整")
    print("=" * 80)
    
    if not CAMERA_MANAGER_AVAILABLE:
        print("❌ 相機管理器不可用，請檢查SDK導入")
        return
    
    try:
        # 設置模板文件夾路徑
        app.template_folder = os.path.join(os.path.dirname(__file__), 'templates')
        
        # 初始化控制器
        if initialize_controller():
            print("🌐 Web介面啟動中...")
            print("📱 訪問地址: http://localhost:5051")
            print("🎯 系統功能:")
            print("   • 相機連接管理 (192.168.1.8)")
            print("   • YOLOv11檢測參數調整")
            print("   • CASE_B/CASE_F分類檢測")
            print("   • Modbus TCP握手協議")
            print("   • 即時狀態監控")
            print("   • 標定檔案管理")
            print("🔗 使用說明:")
            print("   1. 連接到Modbus服務器 (127.0.0.1:502)")
            print("   2. 初始化相機連接")
            print("   3. 調整置信度閾值")
            print("   4. 載入標定檔案 (可選)")
            print("   5. 通過PLC控制拍照和檢測")
            print("📋 寄存器映射:")
            print("   • 控制: 200-201 (指令/狀態)")
            print("   • 參數: 210-219 (置信度等)")
            print("   • 結果: 240-259 (檢測結果)")
            print("   • 世界座標: 260-279 (座標轉換)")
            print("   • 統計: 280-299 (時間/計數)")
            print("=" * 80)
            
            socketio.run(app, host='0.0.0.0', port=5051, debug=False)
            
        else:
            print("❌ 系統初始化失敗，無法啟動")
            print("請檢查以下項目:")
            print("  1. YOLOv11模型檔案 (best.pt) 是否存在")
            print("  2. ultralytics模組是否已安裝")
            print("  3. 相機管理模組是否可用")
            return
        
    except KeyboardInterrupt:
        print("\n🛑 用戶中斷，正在關閉系統...")
    except Exception as e:
        print(f"❌ 系統運行錯誤: {e}")
    finally:
        try:
            if controller:
                controller.disconnect()
        except:
            pass
        print("✅ 系統已安全關閉")


if __name__ == "__main__":
    main()