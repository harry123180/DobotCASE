# -*- coding: utf-8 -*-
"""
CCD1VisionCode_YOLOv11_Enhanced.py - CCD1視覺檢測模組 v5.0
整合YOLOv11物件檢測功能，支援CG_B/CG_F分類檢測
保留圓形檢測功能作為備用模式
基於Modbus TCP Client架構，實現握手式狀態機控制
"""

import os
import sys
import time
import json
import threading
import logging
from typing import Optional, List, Dict, Any, Tuple, Union
from dataclasses import dataclass
from enum import IntEnum
import numpy as np
import cv2
from datetime import datetime

# 檢查YOLOv11可用性
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("YOLOv11模組導入成功")
except ImportError as e:
    print(f"⚠️ YOLOv11模組導入失敗: {e}")
    print("系統將使用圓形檢測模式")

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Modbus Client模組導入失敗: {e}")
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

# Flask Web應用
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit


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


class DetectionMode(IntEnum):
    """檢測模式枚舉"""
    CIRCLE = 0      # 圓形檢測模式
    YOLO = 1        # YOLOv11檢測模式


# ==================== 數據結構定義 ====================
@dataclass
class YOLODetectionResult:
    """YOLOv11檢測結果"""
    success: bool = False
    cg_f_count: int = 0
    cg_b_count: int = 0
    cg_f_coords: List[Tuple[float, float]] = None
    cg_b_coords: List[Tuple[float, float]] = None
    total_detections: int = 0
    confidence_threshold: float = 0.8
    processing_time: float = 0.0
    capture_time: float = 0.0
    total_time: float = 0.0
    error_message: Optional[str] = None

    def __post_init__(self):
        if self.cg_f_coords is None:
            self.cg_f_coords = []
        if self.cg_b_coords is None:
            self.cg_b_coords = []


@dataclass
class CircleDetectionResult:
    """圓形檢測結果（備用模式）"""
    success: bool = False
    circle_count: int = 0
    circles: List[Dict[str, Any]] = None
    processing_time: float = 0.0
    capture_time: float = 0.0
    total_time: float = 0.0
    error_message: Optional[str] = None

    def __post_init__(self):
        if self.circles is None:
            self.circles = []


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
    """YOLOv11物件檢測器"""
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.8):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.class_names = ['CG_B', 'CG_F']
        self.is_loaded = False
        
    def load_model(self) -> bool:
        """載入YOLOv11模型"""
        try:
            if not YOLO_AVAILABLE:
                print("YOLOv11模組不可用")
                return False
                
            if not os.path.exists(self.model_path):
                print(f"模型檔案不存在: {self.model_path}")
                return False
                
            print(f"載入YOLOv11模型: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.is_loaded = True
            print("YOLOv11模型載入成功")
            return True
            
        except Exception as e:
            print(f"YOLOv11模型載入失敗: {e}")
            return False
    
    def detect(self, image: np.ndarray) -> YOLODetectionResult:
        """執行YOLOv11檢測"""
        start_time = time.time()
        result = YOLODetectionResult()
        result.confidence_threshold = self.confidence_threshold
        
        try:
            if not self.is_loaded or self.model is None:
                result.error_message = "模型未載入"
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
                            
                            # 根據類別分類
                            if class_id == 0:  # CG_B
                                result.cg_b_coords.append((float(center_x), float(center_y)))
                                result.cg_b_count += 1
                            elif class_id == 1:  # CG_F
                                result.cg_f_coords.append((float(center_x), float(center_y)))
                                result.cg_f_count += 1
                    
                    result.total_detections = result.cg_b_count + result.cg_f_count
                    result.success = True
                else:
                    result.success = True  # 檢測成功但無目標
            else:
                result.success = True  # 檢測成功但無結果
                
        except Exception as e:
            result.error_message = f"YOLOv11檢測失敗: {e}"
            print(f"YOLOv11檢測異常: {e}")
        
        result.processing_time = (time.time() - start_time) * 1000
        return result


# ==================== 圓形檢測器（備用模式）====================
class CircleDetector:
    """圓形檢測器（備用模式）"""
    
    def __init__(self):
        self.min_area = 50000.0
        self.min_roundness = 0.8
        self.gaussian_kernel_size = 9
        self.canny_low = 20
        self.canny_high = 60
    
    def detect_circles(self, image: np.ndarray) -> CircleDetectionResult:
        """執行圓形檢測"""
        start_time = time.time()
        result = CircleDetectionResult()
        
        try:
            # 轉為灰階
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image.copy()
            
            # 高斯濾波
            blurred = cv2.GaussianBlur(gray, (self.gaussian_kernel_size, self.gaussian_kernel_size), 0)
            
            # Canny邊緣檢測
            edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
            
            # 尋找輪廓
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 分析輪廓
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                
                # 計算圓度
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                
                roundness = 4 * np.pi * area / (perimeter * perimeter)
                if roundness < self.min_roundness:
                    continue
                
                # 計算中心和半徑
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                circle_data = {
                    'center': (int(x), int(y)),
                    'radius': int(radius),
                    'area': area,
                    'roundness': roundness
                }
                result.circles.append(circle_data)
            
            result.circle_count = len(result.circles)
            result.success = True
            
        except Exception as e:
            result.error_message = f"圓形檢測失敗: {e}"
            print(f"圓形檢測異常: {e}")
        
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
        """載入標定數據 - 兼容原CCD1的檔案格式並處理pickle問題"""
        try:
            print(f"🔄 載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            
            # 載入內參 - 支援多種格式並處理pickle問題
            print(f"📖 載入內參檔案...")
            try:
                # 首先嘗試允許pickle的載入（原CCD1格式）
                intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
                print(f"   載入成功，數據類型: {type(intrinsic_data)}")
                
                # 檢查是否為字典格式（原CCD1格式）
                if isinstance(intrinsic_data, dict):
                    print(f"   字典格式，鍵值: {list(intrinsic_data.keys())}")
                    self.camera_matrix = intrinsic_data['camera_matrix']
                    self.dist_coeffs = intrinsic_data['dist_coeffs']
                    print(f"   ✅ 從字典載入內參: camera_matrix {self.camera_matrix.shape}, dist_coeffs {self.dist_coeffs.shape}")
                    
                elif hasattr(intrinsic_data, 'item'):
                    # 嘗試轉換為字典
                    try:
                        dict_data = intrinsic_data.item()
                        if isinstance(dict_data, dict):
                            print(f"   字典項目格式，鍵值: {list(dict_data.keys())}")
                            self.camera_matrix = dict_data['camera_matrix']
                            self.dist_coeffs = dict_data['dist_coeffs']
                            print(f"   ✅ 從字典項目載入內參: camera_matrix {self.camera_matrix.shape}, dist_coeffs {self.dist_coeffs.shape}")
                        else:
                            raise ValueError("無法解析為字典")
                    except:
                        # 如果不是字典，當作直接的矩陣處理
                        self.camera_matrix = intrinsic_data
                        # 查找對應的畸變係數檔案
                        dist_file = intrinsic_file.replace('camera_matrix', 'dist_coeffs')
                        if os.path.exists(dist_file):
                            self.dist_coeffs = np.load(dist_file, allow_pickle=True)
                            print(f"   ✅ 載入配對的畸變係數檔案: {dist_file}")
                        else:
                            self.dist_coeffs = np.zeros((1, 5))  # 預設無畸變
                            print(f"   ⚠️ 未找到畸變係數檔案，使用預設值")
                        print(f"   ✅ 直接載入矩陣: camera_matrix {self.camera_matrix.shape}, dist_coeffs {self.dist_coeffs.shape}")
                        
                else:
                    # 直接使用載入的數據作為相機矩陣
                    self.camera_matrix = intrinsic_data
                    # 查找對應的畸變係數檔案
                    dist_file = intrinsic_file.replace('camera_matrix', 'dist_coeffs')
                    if os.path.exists(dist_file):
                        self.dist_coeffs = np.load(dist_file, allow_pickle=True)
                        print(f"   ✅ 載入配對的畸變係數檔案: {dist_file}")
                    else:
                        self.dist_coeffs = np.zeros((1, 5))  # 預設無畸變
                        print(f"   ⚠️ 未找到畸變係數檔案，使用預設值")
                    print(f"   ✅ 直接載入矩陣: camera_matrix {self.camera_matrix.shape}, dist_coeffs {self.dist_coeffs.shape}")
                    
            except Exception as e1:
                print(f"   ⚠️ 允許pickle載入失敗: {e1}")
                print(f"   🔄 嘗試不允許pickle的載入...")
                try:
                    # 嘗試不允許pickle的載入
                    self.camera_matrix = np.load(intrinsic_file, allow_pickle=False)
                    print(f"   ✅ 無pickle載入成功: camera_matrix {self.camera_matrix.shape}")
                    
                    # 查找對應的畸變係數檔案
                    dist_file = intrinsic_file.replace('camera_matrix', 'dist_coeffs')
                    if os.path.exists(dist_file):
                        try:
                            self.dist_coeffs = np.load(dist_file, allow_pickle=False)
                            print(f"   ✅ 載入畸變係數檔案: {dist_file}, 形狀: {self.dist_coeffs.shape}")
                        except:
                            self.dist_coeffs = np.zeros((1, 5))
                            print(f"   ⚠️ 畸變係數檔案載入失敗，使用預設值")
                    else:
                        self.dist_coeffs = np.zeros((1, 5))  # 預設無畸變
                        print(f"   ⚠️ 未找到畸變係數檔案，使用預設值")
                        
                except Exception as e2:
                    print(f"   ❌ 無pickle載入也失敗: {e2}")
                    raise e2
            
            # 載入外參
            print(f"📖 載入外參檔案...")
            try:
                # 先嘗試允許pickle
                self.extrinsic_matrix = np.load(extrinsic_file, allow_pickle=True)
                print(f"   ✅ 允許pickle載入成功: {self.extrinsic_matrix.shape}")
            except Exception as e1:
                print(f"   ⚠️ 允許pickle載入失敗: {e1}")
                try:
                    # 嘗試不允許pickle
                    self.extrinsic_matrix = np.load(extrinsic_file, allow_pickle=False)
                    print(f"   ✅ 無pickle載入成功: {self.extrinsic_matrix.shape}")
                except Exception as e2:
                    print(f"   ❌ 無pickle載入也失敗: {e2}")
                    raise e2
            
            # 驗證載入的數據
            print(f"🔍 驗證載入的標定數據...")
            print(f"   相機矩陣形狀: {self.camera_matrix.shape}")
            print(f"   畸變係數形狀: {self.dist_coeffs.shape}")
            print(f"   外參矩陣形狀: {self.extrinsic_matrix.shape}")
            
            # 基本驗證
            if self.camera_matrix.shape != (3, 3):
                print(f"   ⚠️ 相機矩陣形狀異常，期望(3,3)，實際{self.camera_matrix.shape}")
            
            self.is_valid_flag = True
            print(f"✅ 座標轉換器載入成功")
            print(f"   內參檔案: {os.path.basename(intrinsic_file)}")
            print(f"   外參檔案: {os.path.basename(extrinsic_file)}")
            return True
            
        except Exception as e:
            print(f"❌ 座標轉換器載入失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            self.is_valid_flag = False
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """像素座標轉世界座標"""
        if not self.is_valid_flag:
            print(f"⚠️ 座標轉換器無效")
            return []
        
        try:
            world_coords = []
            print(f"🔄 開始轉換{len(pixel_coords)}個像素座標...")
            
            for i, (px, py) in enumerate(pixel_coords):
                try:
                    # 去畸變
                    undistorted = cv2.undistortPoints(
                        np.array([[[px, py]]], dtype=np.float32),
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    # 歸一化座標
                    x_norm, y_norm = undistorted[0][0]
                    
                    # 簡化的透視投影變換到世界座標（Z=0平面）
                    # 這裡使用簡化的比例轉換，實際項目中可能需要更複雜的變換
                    world_x = x_norm * 1000  # 假設比例係數
                    world_y = y_norm * 1000
                    
                    world_coords.append((world_x, world_y))
                    print(f"   {i+1}. 像素({px:.1f}, {py:.1f}) → 世界({world_x:.2f}, {world_y:.2f}) mm")
                    
                except Exception as e:
                    print(f"   ❌ 第{i+1}個座標轉換失敗: {e}")
                    world_coords.append((0.0, 0.0))  # 失敗時使用預設值
            
            print(f"✅ 座標轉換完成，成功轉換{len(world_coords)}個座標")
            return world_coords
            
        except Exception as e:
            print(f"❌ 座標轉換失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
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
        """掃描標定檔案 - 兼容原CCD1命名模式並詳細檢查"""
        try:
            print(f"🔍 掃描標定檔案目錄: {self.working_dir}")
            
            if not os.path.exists(self.working_dir):
                return {
                    'success': False,
                    'error': f"工作目錄不存在: {self.working_dir}"
                }
            
            files = os.listdir(self.working_dir)
            npy_files = [f for f in files if f.endswith('.npy')]
            
            print(f"📁 發現 {len(npy_files)} 個NPY檔案:")
            for i, file in enumerate(npy_files, 1):
                file_path = os.path.join(self.working_dir, file)
                file_size = os.path.getsize(file_path)
                mod_time = datetime.fromtimestamp(os.path.getmtime(file_path)).strftime("%Y-%m-%d %H:%M:%S")
                print(f"   {i}. {file} ({file_size} bytes, 修改時間: {mod_time})")
            
            if not npy_files:
                return {
                    'success': False,
                    'error': f"未發現任何NPY檔案在目錄: {self.working_dir}"
                }
            
            intrinsic_files = []
            extrinsic_files = []
            
            print(f"\n🔬 逐一檢查NPY檔案內容...")
            
            for file in npy_files:
                file_path = os.path.join(self.working_dir, file)
                file_lower = file.lower()
                
                print(f"\n檢查檔案: {file}")
                
                try:
                    # 嘗試載入檔案並檢查內容
                    data = np.load(file_path, allow_pickle=True)
                    
                    # 檢查數據類型和形狀
                    print(f"   數據類型: {type(data)}")
                    print(f"   數據形狀: {data.shape if hasattr(data, 'shape') else '非陣列'}")
                    
                    # 檢查是否為字典格式（內參常見格式）
                    if isinstance(data, dict) or (hasattr(data, 'item') and callable(data.item)):
                        try:
                            if hasattr(data, 'item'):
                                dict_data = data.item()
                            else:
                                dict_data = data
                                
                            if isinstance(dict_data, dict):
                                print(f"   字典鍵值: {list(dict_data.keys())}")
                                if 'camera_matrix' in dict_data and 'dist_coeffs' in dict_data:
                                    print(f"   ✅ 識別為內參檔案 (包含camera_matrix和dist_coeffs)")
                                    intrinsic_files.append(file)
                                    continue
                        except Exception as e:
                            print(f"   ⚠️ 解析字典失敗: {e}")
                    
                    # 檢查檔案名稱模式
                    is_intrinsic_name = any(keyword in file_lower for keyword in 
                                          ['camera_matrix', 'intrinsic', '内参', 'calib'])
                    is_extrinsic_name = any(keyword in file_lower for keyword in 
                                          ['extrinsic', '外参', 'external'])
                    
                    # 檢查矩陣大小（內參通常是3x3，外參可能是4x4或3x4）
                    if hasattr(data, 'shape'):
                        if data.shape == (3, 3):
                            print(f"   📐 3x3矩陣 - 可能是相機內參矩陣")
                            if is_intrinsic_name or not is_extrinsic_name:
                                print(f"   ✅ 識別為內參檔案 (3x3矩陣)")
                                intrinsic_files.append(file)
                                continue
                        elif data.shape in [(4, 4), (3, 4), (6,), (5,)]:
                            if data.shape == (5,) or data.shape == (6,):
                                print(f"   📐 畸變係數向量 - 可能是畸變參數")
                                # 畸變係數通常配對內參使用
                                if is_intrinsic_name:
                                    print(f"   ✅ 識別為內參相關檔案 (畸變係數)")
                                    intrinsic_files.append(file)
                                    continue
                            else:
                                print(f"   📐 {data.shape}矩陣 - 可能是外參矩陣")
                                if is_extrinsic_name or not is_intrinsic_name:
                                    print(f"   ✅ 識別為外參檔案 ({data.shape}矩陣)")
                                    extrinsic_files.append(file)
                                    continue
                    
                    # 基於檔案名稱的備用判斷
                    if is_intrinsic_name:
                        print(f"   ✅ 基於檔案名稱識別為內參檔案")
                        intrinsic_files.append(file)
                    elif is_extrinsic_name:
                        print(f"   ✅ 基於檔案名稱識別為外參檔案")
                        extrinsic_files.append(file)
                    else:
                        print(f"   ❓ 無法確定檔案類型")
                        
                except Exception as e:
                    print(f"   ❌ 載入檔案失敗: {e}")
                    # 嘗試不允許pickle的載入方式
                    try:
                        data_no_pickle = np.load(file_path, allow_pickle=False)
                        print(f"   📊 無pickle載入成功，形狀: {data_no_pickle.shape}")
                        
                        if data_no_pickle.shape == (3, 3):
                            print(f"   ✅ 識別為內參檔案 (3x3矩陣，無pickle)")
                            intrinsic_files.append(file)
                        elif data_no_pickle.shape in [(4, 4), (3, 4)]:
                            print(f"   ✅ 識別為外參檔案 ({data_no_pickle.shape}矩陣，無pickle)")
                            extrinsic_files.append(file)
                            
                    except Exception as e2:
                        print(f"   ❌ 無pickle載入也失敗: {e2}")
            
            # 按修改時間排序（最新的在前）
            intrinsic_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.working_dir, x)), reverse=True)
            extrinsic_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.working_dir, x)), reverse=True)
            
            print(f"\n📋 掃描結果:")
            print(f"   內參檔案 ({len(intrinsic_files)}個): {intrinsic_files}")
            print(f"   外參檔案 ({len(extrinsic_files)}個): {extrinsic_files}")
            
            if not intrinsic_files:
                print(f"❌ 未發現內參檔案")
            if not extrinsic_files:
                print(f"❌ 未發現外參檔案")
            
            return {
                'success': True,
                'intrinsic_files': intrinsic_files,
                'extrinsic_files': extrinsic_files,
                'working_dir': self.working_dir,
                'total_npy_files': len(npy_files),
                'scan_details': f"掃描了{len(npy_files)}個NPY檔案"
            }
            
        except Exception as e:
            error_msg = f"掃描檔案失敗: {e}"
            print(f"❌ {error_msg}")
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
            
            # 自動選擇檔案 - 按照原CCD1的檔案命名模式
            if not intrinsic_file and scan_result['intrinsic_files']:
                # 優先選擇camera_matrix開頭的檔案
                for file in scan_result['intrinsic_files']:
                    if 'camera_matrix' in file:
                        intrinsic_file = file
                        break
                if not intrinsic_file:
                    intrinsic_file = scan_result['intrinsic_files'][0]
            
            if not extrinsic_file and scan_result['extrinsic_files']:
                # 選擇extrinsic開頭的檔案
                for file in scan_result['extrinsic_files']:
                    if 'extrinsic' in file:
                        extrinsic_file = file
                        break
                if not extrinsic_file:
                    extrinsic_file = scan_result['extrinsic_files'][0]
            
            if not intrinsic_file or not extrinsic_file:
                return {
                    'success': False,
                    'error': '缺少內參或外參檔案'
                }
            
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
                return {
                    'success': False,
                    'error': '標定數據載入失敗'
                }
                
        except Exception as e:
            return {
                'success': False,
                'error': f"載入標定數據失敗: {e}"
            }
    
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


# ==================== 主控制器 ====================
class CCD1VisionController:
    """CCD1視覺檢測主控制器"""
    
    def __init__(self):
        # 基本配置
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.base_address = 200
        self.camera_ip = "192.168.1.8"
        
        # 檢測模式和閾值
        self.detection_mode = DetectionMode.CIRCLE
        self.confidence_threshold = 0.8
        
        # 核心組件
        self.state_machine = SystemStateMachine()
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.calibration_manager = CalibrationManager(self.working_dir)
        self.yolo_detector = None
        self.circle_detector = CircleDetector()
        
        # Modbus客戶端
        self.modbus_client = None
        self.last_control_command = 0
        self.command_processing = False
        
        # 統計信息
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        
        # 檢查並初始化YOLOv11
        self._check_yolo_availability()
        
        # 強制要求
        self._enforce_requirements()
    
    def _check_yolo_availability(self):
        """檢查YOLOv11模型可用性"""
        best_pt_path = os.path.join(self.working_dir, "best.pt")
        
        if os.path.exists(best_pt_path) and YOLO_AVAILABLE:
            print(f"發現YOLOv11模型: {best_pt_path}")
            self.yolo_detector = YOLOv11Detector(best_pt_path, self.confidence_threshold)
            if self.yolo_detector.load_model():
                self.detection_mode = DetectionMode.YOLO
                print("系統切換到AI檢測模式（YOLOv11）")
            else:
                print("YOLOv11模型載入失敗，使用圓形檢測模式")
                self.detection_mode = DetectionMode.CIRCLE
        else:
            print("未發現best.pt或YOLOv11不可用，使用圓形檢測模式")
            self.detection_mode = DetectionMode.CIRCLE
    
    def _enforce_requirements(self):
        """強制要求檢查"""
        # 1. 強制Modbus連接到127.0.0.1:502
        if self.server_ip != "127.0.0.1" or self.server_port != 502:
            print("強制設置Modbus服務器為127.0.0.1:502")
            self.server_ip = "127.0.0.1"
            self.server_port = 502
        
        # 2. 強制要求標定檔案
        print(f"🔍 檢查標定檔案要求...")
        scan_result = self.calibration_manager.scan_calibration_files()
        
        if not scan_result['success']:
            error_msg = f"CCD1模組初始化失敗: {scan_result['error']}"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        if not scan_result['intrinsic_files']:
            error_msg = "CCD1模組初始化失敗: 缺少內參標定檔案"
            print(f"❌ {error_msg}")
            print(f"💡 請確保目錄中包含以下格式的內參檔案:")
            print(f"   - camera_matrix_*.npy")
            print(f"   - intrinsic_*.npy") 
            print(f"   - 包含camera_matrix和dist_coeffs鍵值的字典格式NPY檔案")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        if not scan_result['extrinsic_files']:
            error_msg = "CCD1模組初始化失敗: 缺少外參標定檔案"
            print(f"❌ {error_msg}")
            print(f"💡 請確保目錄中包含以下格式的外參檔案:")
            print(f"   - extrinsic_*.npy")
            print(f"   - 外參矩陣NPY檔案")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        # 自動載入標定檔案
        print(f"🔄 自動載入標定檔案...")
        load_result = self.calibration_manager.load_calibration_data()
        if not load_result['success']:
            error_msg = f"CCD1模組初始化失敗: {load_result['error']}"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        print("✅ 標定檔案載入成功")
        print(f"   內參檔案: {load_result.get('intrinsic_file', 'N/A')}")
        print(f"   外參檔案: {load_result.get('extrinsic_file', 'N/A')}")
    
    def connect_modbus(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("❌ Modbus模組不可用")
                return False
            
            # 強制連接到指定地址
            if self.server_ip != "127.0.0.1" or self.server_port != 502:
                error_msg = "Modbus服務器必須連接到127.0.0.1:502"
                print(f"❌ {error_msg}")
                self.state_machine.set_alarm(True)
                return False  # 改為返回False而不是拋出異常
            
            print(f"🔗 正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            # 創建新的Modbus客戶端
            if hasattr(self, 'modbus_client') and self.modbus_client:
                try:
                    if hasattr(self.modbus_client, 'close'):
                        self.modbus_client.close()
                except:
                    pass
            
            from pymodbus.client import ModbusTcpClient
            self.modbus_client = ModbusTcpClient(host=self.server_ip, port=self.server_port)
            
            if self.modbus_client.connect():
                print(f"✅ Modbus連接成功: {self.server_ip}:{self.server_port}")
                return True
            else:
                error_msg = f"Modbus連接失敗: {self.server_ip}:{self.server_port}"
                print(f"❌ {error_msg}")
                return False
                
        except Exception as e:
            error_msg = f"Modbus連接異常: {e}"
            print(f"❌ {error_msg}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            return False
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        """初始化相機 - 使用原CCD1的相機管理架構"""
        try:
            if not CAMERA_MANAGER_AVAILABLE:
                error_msg = "相機管理模組不可用"
                print(f"❌ {error_msg}")
                self.state_machine.set_alarm(True)
                raise RuntimeError(error_msg)
            
            if ip_address:
                self.camera_ip = ip_address
            
            # 安全關閉現有相機管理器
            if self.camera_manager:
                try:
                    print(f"🔄 關閉現有相機連接...")
                    # 修復：兼容舊版本Python的shutdown方法
                    self.camera_manager.shutdown()
                except Exception as e:
                    print(f"⚠️ 關閉現有相機時出現警告: {e}")
                    # 即使關閉失敗也繼續初始化
                finally:
                    self.camera_manager = None
            
            # 強制要求相機連接成功
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
            
            print(f"🔄 創建新的相機管理器...")
            # 使用原CCD1的相機管理器架構
            self.camera_manager = OptimizedCameraManager()
            
            # 添加相機
            print(f"📷 添加相機配置...")
            success = self.camera_manager.add_camera("ccd1_camera", camera_config)
            if not success:
                error_msg = f"添加相機失敗: {self.camera_ip}"
                print(f"❌ {error_msg}")
                self.state_machine.set_alarm(True)
                self.state_machine.set_initialized(False)
                raise RuntimeError(error_msg)
            
            # 連接相機
            print(f"🔗 連接相機...")
            connect_result = self.camera_manager.connect_camera("ccd1_camera")
            if not connect_result:
                error_msg = f"相機連接失敗: {self.camera_ip}"
                print(f"❌ {error_msg}")
                self.state_machine.set_alarm(True)
                self.state_machine.set_initialized(False)
                raise RuntimeError(error_msg)
            
            # 開始串流
            print(f"📺 開始相機串流...")
            stream_result = self.camera_manager.start_streaming(["ccd1_camera"])
            if not stream_result.get("ccd1_camera", False):
                error_msg = f"開始串流失敗: {self.camera_ip}"
                print(f"❌ {error_msg}")
                self.state_machine.set_alarm(True)
                self.state_machine.set_initialized(False)
                raise RuntimeError(error_msg)
            
            # 設置增益為200
            try:
                camera = self.camera_manager.cameras["ccd1_camera"]
                camera.camera.MV_CC_SetFloatValue("Gain", 200.0)
                print(f"✅ 相機增益設置為: 200.0")
            except Exception as e:
                print(f"⚠️ 設置增益失敗: {e}")
            
            self.state_machine.set_initialized(True)
            self.state_machine.set_alarm(False)
            print(f"✅ 相機初始化成功: {self.camera_ip}")
            return True
                
        except Exception as e:
            error_msg = f"相機初始化異常: {e}"
            print(f"❌ {error_msg}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            return False  # 改為返回False而不是拋出異常
    
    def capture_image(self) -> Optional[np.ndarray]:
        """拍照 - 使用原CCD1的相機管理架構"""
        if not self.camera_manager:
            print("❌ 相機未初始化")
            return None
        
        try:
            # 使用原CCD1的圖像捕獲方式
            frame_data = self.camera_manager.get_image_data("ccd1_camera", timeout=3000)
            
            if frame_data is None:
                print("❌ 拍照失敗: 未獲取到圖像數據")
                return None
            
            image_array = frame_data.data
            
            # 確保圖像格式正確
            if len(image_array.shape) == 2:
                # 灰階圖像轉為BGR
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
            else:
                display_image = image_array
            
            print("✅ 拍照成功")
            return display_image
                
        except Exception as e:
            print(f"❌ 拍照異常: {e}")
            return None
    
    def capture_and_detect(self) -> Union[YOLODetectionResult, CircleDetectionResult]:
        """拍照並檢測"""
        capture_start = time.time()
        
        try:
            print(f"🚀 開始拍照和檢測，模式: {self.detection_mode.name}")
            
            # 檢查相機管理器
            if not self.camera_manager:
                error_msg = "相機管理器未初始化"
                print(f"❌ {error_msg}")
                if self.detection_mode == DetectionMode.YOLO:
                    result = YOLODetectionResult()
                    result.error_message = error_msg
                    return result
                else:
                    result = CircleDetectionResult()
                    result.error_message = error_msg
                    return result
            
            # 檢查相機是否存在（簡化檢查）
            try:
                if "ccd1_camera" not in self.camera_manager.cameras:
                    error_msg = "相機未找到在管理器中"
                    print(f"❌ {error_msg}")
                    print(f"   可用相機: {list(self.camera_manager.cameras.keys())}")
                    if self.detection_mode == DetectionMode.YOLO:
                        result = YOLODetectionResult()
                        result.error_message = error_msg
                        return result
                    else:
                        result = CircleDetectionResult()
                        result.error_message = error_msg
                        return result
                else:
                    print(f"✅ 相機檢查通過: ccd1_camera 存在於管理器中")
            except Exception as e:
                error_msg = f"檢查相機狀態失敗: {e}"
                print(f"❌ {error_msg}")
                if self.detection_mode == DetectionMode.YOLO:
                    result = YOLODetectionResult()
                    result.error_message = error_msg
                    return result
                else:
                    result = CircleDetectionResult()
                    result.error_message = error_msg
                    return result
            
            # 拍照
            print(f"📸 開始拍照...")
            image = self.capture_image()
            if image is None:
                error_msg = "拍照失敗，未獲取到圖像"
                print(f"❌ {error_msg}")
                if self.detection_mode == DetectionMode.YOLO:
                    result = YOLODetectionResult()
                    result.capture_time = (time.time() - capture_start) * 1000
                    result.error_message = error_msg
                    return result
                else:
                    result = CircleDetectionResult()
                    result.capture_time = (time.time() - capture_start) * 1000
                    result.error_message = error_msg
                    return result
            
            capture_time = (time.time() - capture_start) * 1000
            print(f"✅ 拍照成功，耗時: {capture_time:.2f}ms")
            
            # 根據模式執行檢測
            if self.detection_mode == DetectionMode.YOLO and self.yolo_detector:
                print(f"🤖 執行YOLOv11檢測...")
                detect_start = time.time()
                
                try:
                    result = self.yolo_detector.detect(image)
                    result.capture_time = capture_time
                    result.total_time = (time.time() - capture_start) * 1000
                    
                    print(f"✅ YOLOv11檢測完成: CG_F={result.cg_f_count}, CG_B={result.cg_b_count}")
                    
                    # 世界座標轉換
                    print(f"🌍 開始世界座標轉換...")
                    self._add_world_coordinates_yolo(result)
                    
                except Exception as e:
                    error_msg = f"YOLOv11檢測失敗: {e}"
                    print(f"❌ {error_msg}")
                    import traceback
                    print(f"詳細錯誤: {traceback.format_exc()}")
                    
                    result = YOLODetectionResult()
                    result.capture_time = capture_time
                    result.total_time = (time.time() - capture_start) * 1000
                    result.error_message = error_msg
                    return result
            else:
                print(f"🔵 執行圓形檢測...")
                detect_start = time.time()
                
                try:
                    result = self.circle_detector.detect_circles(image)
                    result.capture_time = capture_time
                    result.total_time = (time.time() - capture_start) * 1000
                    
                    print(f"✅ 圓形檢測完成: 找到{result.circle_count}個圓形")
                    
                    # 世界座標轉換
                    print(f"🌍 開始世界座標轉換...")
                    self._add_world_coordinates_circle(result)
                    
                except Exception as e:
                    error_msg = f"圓形檢測失敗: {e}"
                    print(f"❌ {error_msg}")
                    import traceback
                    print(f"詳細錯誤: {traceback.format_exc()}")
                    
                    result = CircleDetectionResult()
                    result.capture_time = capture_time
                    result.total_time = (time.time() - capture_start) * 1000
                    result.error_message = error_msg
                    return result
            
            print(f"🎯 檢測流程完成，總耗時: {result.total_time:.2f}ms")
            return result
            
        except Exception as e:
            error_msg = f"檢測流程異常: {e}"
            print(f"❌ {error_msg}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            
            total_time = (time.time() - capture_start) * 1000
            
            if self.detection_mode == DetectionMode.YOLO:
                result = YOLODetectionResult()
                result.total_time = total_time
                result.error_message = error_msg
                return result
            else:
                result = CircleDetectionResult()
                result.total_time = total_time
                result.error_message = error_msg
                return result
    
    def _add_world_coordinates_yolo(self, result: YOLODetectionResult):
        """為YOLOv11結果添加世界座標"""
        try:
            if not self.calibration_manager.transformer.is_valid():
                print(f"⚠️ 標定數據無效，跳過世界座標轉換")
                return
            
            print(f"🌍 執行YOLOv11結果世界座標轉換...")
            
            # 轉換CG_F座標
            if result.cg_f_coords:
                print(f"   轉換{len(result.cg_f_coords)}個CG_F目標座標")
                try:
                    world_coords = self.calibration_manager.transformer.pixel_to_world(result.cg_f_coords)
                    for i, (wx, wy) in enumerate(world_coords):
                        print(f"   CG_F {i+1}: 像素({result.cg_f_coords[i][0]:.1f}, {result.cg_f_coords[i][1]:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
                        # 更新結果中的世界座標（如果需要存儲的話）
                        # result.cg_f_coords[i] = (result.cg_f_coords[i][0], result.cg_f_coords[i][1], wx, wy)
                except Exception as e:
                    print(f"   ❌ CG_F座標轉換失敗: {e}")
            
            # 轉換CG_B座標
            if result.cg_b_coords:
                print(f"   轉換{len(result.cg_b_coords)}個CG_B目標座標")
                try:
                    world_coords = self.calibration_manager.transformer.pixel_to_world(result.cg_b_coords)
                    for i, (wx, wy) in enumerate(world_coords):
                        print(f"   CG_B {i+1}: 像素({result.cg_b_coords[i][0]:.1f}, {result.cg_b_coords[i][1]:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
                except Exception as e:
                    print(f"   ❌ CG_B座標轉換失敗: {e}")
                    
        except Exception as e:
            print(f"❌ YOLOv11世界座標轉換失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
    
    def _add_world_coordinates_circle(self, result: CircleDetectionResult):
        """為圓形檢測結果添加世界座標"""
        try:
            if not self.calibration_manager.transformer.is_valid():
                print(f"⚠️ 標定數據無效，跳過世界座標轉換")
                return
            
            print(f"🌍 執行圓形檢測結果世界座標轉換...")
            
            if result.circles:
                print(f"   轉換{len(result.circles)}個圓形目標座標")
                try:
                    pixel_coords = [(circle['center'][0], circle['center'][1]) for circle in result.circles]
                    world_coords = self.calibration_manager.transformer.pixel_to_world(pixel_coords)
                    
                    for i, ((wx, wy), circle) in enumerate(zip(world_coords, result.circles)):
                        circle['world_coords'] = (wx, wy)
                        center = circle['center']
                        print(f"   圓形 {i+1}: 像素({center[0]}, {center[1]}) → 世界({wx:.2f}, {wy:.2f}) mm")
                except Exception as e:
                    print(f"   ❌ 圓形座標轉換失敗: {e}")
                    
        except Exception as e:
            print(f"❌ 圓形世界座標轉換失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        if self.yolo_detector:
            self.yolo_detector.confidence_threshold = self.confidence_threshold
        print(f"置信度閾值更新為: {self.confidence_threshold}")
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        camera_connected = False
        try:
            if self.camera_manager:
                # 根據原始代碼，檢查相機是否在cameras字典中
                if "ccd1_camera" in self.camera_manager.cameras:
                    # 使用控制器的is_connected屬性（如原始代碼）
                    # 或者檢查相機管理器的連接狀態
                    camera_connected = True  # 如果相機在字典中，表示已連接
                    print(f"🔍 狀態檢查: 相機存在={True}, 認定為已連接")
                else:
                    print(f"🔍 狀態檢查: 相機不存在於cameras字典中")
                    print(f"   可用相機: {list(self.camera_manager.cameras.keys()) if self.camera_manager.cameras else '無'}")
            else:
                print(f"🔍 狀態檢查: 相機管理器為None")
        except Exception as e:
            print(f"⚠️ 獲取相機狀態時異常: {e}")
            camera_connected = False
        
        # 檢查系統是否初始化（相機連接 + 標定有效）
        calibration_valid = self.calibration_manager.transformer.is_valid()
        initialized = camera_connected and calibration_valid
        
        status = {
            'ready': self.state_machine.is_ready(),
            'running': self.state_machine.is_running(),
            'alarm': self.state_machine.is_alarm(),
            'initialized': initialized,
            'detection_mode': self.detection_mode.name,
            'confidence_threshold': self.confidence_threshold,
            'modbus_connected': self.modbus_client is not None and hasattr(self.modbus_client, 'connected') and self.modbus_client.connected,
            'camera_connected': camera_connected,
            'calibration_valid': calibration_valid,
            'operation_count': self.operation_count,
            'error_count': self.error_count
        }
        
        print(f"📊 系統狀態: camera_connected={camera_connected}, initialized={initialized}, calibration_valid={calibration_valid}")
        return status


# ==================== Web應用 ====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ccd1_yolo_vision_v5'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全局控制器實例
controller = None

def initialize_controller():
    """初始化控制器並自動連接相機和Modbus"""
    global controller
    try:
        print("🚀 正在初始化CCD1視覺控制器...")
        controller = CCD1VisionController()
        print("✅ CCD1視覺控制器初始化成功")
        
        # 自動初始化相機
        print("📷 自動初始化相機...")
        try:
            camera_success = controller.initialize_camera()
            if camera_success:
                print("✅ 相機自動初始化成功")
            else:
                print("⚠️ 相機自動初始化失敗，但系統可繼續運行")
        except Exception as e:
            print(f"⚠️ 相機自動初始化異常: {e}")
        
        # 嘗試自動連接Modbus（可選）
        print("📡 嘗試自動連接Modbus...")
        try:
            modbus_success = controller.connect_modbus()
            if modbus_success:
                print("✅ Modbus自動連接成功")
            else:
                print("⚠️ Modbus自動連接失敗，但不影響檢測功能")
        except Exception as e:
            print(f"⚠️ Modbus自動連接異常: {e}")
        
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


@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取系統狀態"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        status = controller.get_system_status()
        calibration_status = controller.calibration_manager.get_status()
        
        return jsonify({
            'success': True,
            'status': status,
            'calibration': calibration_status
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/connect_modbus', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        print(f"📡 收到Modbus連接請求")
        
        # 檢查是否有Modbus客戶端
        if not hasattr(controller, 'modbus_client') or not controller.modbus_client:
            return jsonify({
                'success': False,
                'message': 'Modbus客戶端未初始化'
            })
        
        # 強制設置為127.0.0.1:502（根據需求）
        controller.server_ip = "127.0.0.1"
        controller.server_port = 502
        
        print(f"🔗 嘗試連接Modbus服務器: {controller.server_ip}:{controller.server_port}")
        
        # 嘗試連接
        success = controller.connect_modbus()
        
        if success:
            print(f"✅ Modbus連接成功")
            return jsonify({
                'success': True,
                'message': f'Modbus TCP連接成功: {controller.server_ip}:{controller.server_port}',
                'server_ip': controller.server_ip,
                'server_port': controller.server_port
            })
        else:
            print(f"❌ Modbus連接失敗")
            return jsonify({
                'success': False,
                'message': f'無法連接到Modbus服務器: {controller.server_ip}:{controller.server_port}'
            })
            
    except Exception as e:
        error_msg = f'Modbus連接異常: {str(e)}'
        print(f"❌ {error_msg}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
        
        return jsonify({
            'success': False,
            'message': error_msg,
            'error': str(e)
        })


@app.route('/api/initialize_camera', methods=['POST'])
def initialize_camera():
    """初始化相機"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.get_json() if request.get_json() else {}
        ip = data.get('ip', controller.camera_ip)
        
        print(f"📷 收到相機初始化請求: {ip}")
        
        success = controller.initialize_camera(ip)
        
        response = {
            'success': success,
            'message': f'相機初始化{"成功" if success else "失敗"}: {ip}',
            'camera_ip': ip
        }
        
        if success:
            print(f"✅ 相機初始化API成功: {ip}")
        else:
            print(f"❌ 相機初始化API失敗: {ip}")
            
        return jsonify(response)
        
    except Exception as e:
        error_msg = f'相機初始化異常: {str(e)}'
        print(f"❌ {error_msg}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
        
        return jsonify({
            'success': False, 
            'error': error_msg,
            'message': error_msg
        })


@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    """執行拍照和檢測"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        print(f"📸 收到拍照+檢測請求")
        
        # 檢查標定數據
        if not controller.calibration_manager.transformer.is_valid():
            return jsonify({
                'success': False,
                'error': '標定數據無效'
            })
        
        # 檢查相機管理器
        if not controller.camera_manager:
            return jsonify({
                'success': False,
                'error': '相機管理器未初始化'
            })
        
        # 簡化相機連接檢查 - 如果相機在cameras字典中就認為已連接
        camera_connected = False
        try:
            if "ccd1_camera" in controller.camera_manager.cameras:
                camera_connected = True
                print(f"✅ 相機連接檢查通過: 相機存在於管理器中")
            else:
                print(f"❌ 相機連接檢查失敗: 相機不存在於管理器中")
                print(f"   可用相機: {list(controller.camera_manager.cameras.keys())}")
        except Exception as e:
            print(f"⚠️ 檢查相機狀態時異常: {e}")
        
        if not camera_connected:
            return jsonify({
                'success': False, 
                'error': '相機未連接或未找到',
                'camera_debug': {
                    'camera_manager_exists': controller.camera_manager is not None,
                    'cameras_list': list(controller.camera_manager.cameras.keys()) if controller.camera_manager else [],
                    'target_camera': 'ccd1_camera'
                }
            })
        
        print(f"✅ 所有檢查通過，開始執行檢測")
        print(f"🔄 檢測模式: {controller.detection_mode.name}")
        
        result = controller.capture_and_detect()
        
        response_data = {
            'success': result.success,
            'detection_mode': controller.detection_mode.name,
            'capture_time': getattr(result, 'capture_time', 0),
            'processing_time': getattr(result, 'processing_time', 0),
            'total_time': getattr(result, 'total_time', 0),
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        if controller.detection_mode == DetectionMode.YOLO:
            # YOLOv11模式
            response_data.update({
                'cg_f_count': getattr(result, 'cg_f_count', 0),
                'cg_b_count': getattr(result, 'cg_b_count', 0),
                'total_detections': getattr(result, 'total_detections', 0),
                'confidence_threshold': getattr(result, 'confidence_threshold', controller.confidence_threshold),
                'cg_f_coords': getattr(result, 'cg_f_coords', []),
                'cg_b_coords': getattr(result, 'cg_b_coords', [])
            })
            
            if result.success:
                print(f"✅ YOLOv11檢測成功: CG_F={response_data['cg_f_count']}, CG_B={response_data['cg_b_count']}")
            else:
                print(f"❌ YOLOv11檢測失敗: {getattr(result, 'error_message', '未知錯誤')}")
        else:
            # 圓形檢測模式
            response_data.update({
                'circle_count': getattr(result, 'circle_count', 0),
                'circles': getattr(result, 'circles', [])
            })
            
            if result.success:
                print(f"✅ 圓形檢測成功: 找到{response_data['circle_count']}個圓形")
            else:
                print(f"❌ 圓形檢測失敗: {getattr(result, 'error_message', '未知錯誤')}")
        
        if not result.success:
            response_data['error'] = getattr(result, 'error_message', '檢測失敗')
        
        print(f"📊 檢測完成: success={response_data['success']}")
        return jsonify(response_data)
        
    except Exception as e:
        error_msg = f"檢測過程異常: {str(e)}"
        print(f"❌ {error_msg}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
        
        return jsonify({
            'success': False,
            'error': error_msg,
            'detection_mode': controller.detection_mode.name if controller else 'unknown',
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })


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
            'confidence_threshold': controller.confidence_threshold,
            'message': f'置信度閾值更新為: {controller.confidence_threshold}'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


if __name__ == '__main__':
    print("=== CCD1視覺檢測模組 v5.0 啟動 ===")
    print("整合功能: YOLOv11物件檢測 + 圓形檢測備用模式")
    print("支援功能: CG_B/CG_F分類檢測 + 世界座標轉換")
    print("握手協議: Modbus TCP Client + 50ms輪詢")
    print("強制要求: 127.0.0.1:502 + 標定檔案 + 相機連接")
    
    # 初始化控制器
    if initialize_controller():
        print("Web介面啟動中...")
        print("訪問地址: http://localhost:5051")
        socketio.run(app, host='localhost', port=5051, debug=False)
    else:
        print("❌ 系統初始化失敗，無法啟動")
        sys.exit(1)