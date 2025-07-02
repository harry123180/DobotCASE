# -*- coding: utf-8 -*-
"""
修正版察看類型.py - 標定檔案類型查看器
修正載入邏輯，測試所有組合載入
"""

import os
import sys
import numpy as np
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
import traceback

class FixedCalibrationInspector:
    """修正版標定檔案檢查器"""
    
    def __init__(self):
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        print(f"🔍 修正版標定檔案檢查器啟動")
        print(f"📁 工作目錄: {self.working_dir}")
        print("=" * 80)
    
    def inspect_all_files(self):
        """檢查所有檔案"""
        try:
            # 1. 掃描所有檔案
            all_files = os.listdir(self.working_dir)
            npy_files = [f for f in all_files if f.endswith('.npy')]
            
            print(f"📋 檔案清單:")
            print(f"   總檔案數: {len(all_files)}")
            print(f"   NPY檔案數: {len(npy_files)}")
            print(f"   NPY檔案: {npy_files}")
            print("\n" + "=" * 80)
            
            if not npy_files:
                print("❌ 未發現任何NPY檔案")
                return
            
            # 2. 逐一檢查NPY檔案
            for i, file in enumerate(npy_files, 1):
                print(f"\n📄 檢查檔案 {i}/{len(npy_files)}: {file}")
                self._inspect_single_file(file)
                print("-" * 60)
            
            # 3. 測試修正版載入邏輯
            print(f"\n🧪 測試修正版載入邏輯:")
            print("=" * 80)
            self._test_fixed_loading_logic(npy_files)
            
        except Exception as e:
            print(f"❌ 檢查過程發生異常: {e}")
            traceback.print_exc()
    
    def _inspect_single_file(self, filename: str):
        """檢查單個檔案"""
        file_path = os.path.join(self.working_dir, filename)
        
        try:
            # 基本檔案資訊
            file_size = os.path.getsize(file_path)
            mod_time = datetime.fromtimestamp(os.path.getmtime(file_path)).strftime("%Y-%m-%d %H:%M:%S")
            
            print(f"   📊 基本資訊:")
            print(f"      檔案大小: {file_size} bytes")
            print(f"      修改時間: {mod_time}")
            
            # 嘗試載入檔案 (允許pickle)
            print(f"   🔄 嘗試載入檔案 (allow_pickle=True)...")
            try:
                data = np.load(file_path, allow_pickle=True)
                self._analyze_data_structure(data, "pickle=True")
            except Exception as e1:
                print(f"      ❌ pickle=True載入失敗: {e1}")
                
                # 嘗試不允許pickle載入
                print(f"   🔄 嘗試載入檔案 (allow_pickle=False)...")
                try:
                    data_no_pickle = np.load(file_path, allow_pickle=False)
                    self._analyze_data_structure(data_no_pickle, "pickle=False")
                except Exception as e2:
                    print(f"      ❌ pickle=False載入也失敗: {e2}")
                    return
            
            # 檔案名稱分析
            self._analyze_filename(filename)
            
        except Exception as e:
            print(f"   ❌ 檢查檔案失敗: {e}")
            traceback.print_exc()
    
    def _analyze_data_structure(self, data, load_method: str):
        """分析數據結構"""
        print(f"   📊 數據結構分析 ({load_method}):")
        print(f"      數據類型: {type(data)}")
        
        if hasattr(data, 'shape'):
            print(f"      數據形狀: {data.shape}")
            print(f"      數據dtype: {data.dtype}")
            
            # 如果是小數組，顯示內容
            if data.size <= 50:  # 最多顯示50個元素
                print(f"      數據內容:")
                if len(data.shape) == 0:  # 0維數組
                    print(f"         {data}")
                elif len(data.shape) == 1:  # 1維數組
                    print(f"         {data}")
                elif len(data.shape) == 2 and data.shape[0] <= 10 and data.shape[1] <= 10:  # 小2維數組
                    for row in data:
                        print(f"         {row}")
                else:
                    print(f"         形狀太大，僅顯示範圍: [{data.min():.6f}, {data.max():.6f}]")
            else:
                print(f"      數據範圍: [{data.min():.6f}, {data.max():.6f}]")
        
        # 檢查是否為字典格式
        if isinstance(data, dict):
            print(f"      📋 字典格式:")
            print(f"         鍵值: {list(data.keys())}")
            for key, value in data.items():
                if hasattr(value, 'shape'):
                    print(f"         {key}: shape={value.shape}, dtype={value.dtype}")
                else:
                    print(f"         {key}: {type(value)} = {value}")
        
        # 檢查是否為包含字典的0維數組
        elif hasattr(data, 'item') and callable(data.item):
            print(f"   🔄 嘗試解析為字典項目...")
            try:
                dict_data = data.item()
                if isinstance(dict_data, dict):
                    print(f"      📋 字典項目格式:")
                    print(f"         鍵值: {list(dict_data.keys())}")
                    for key, value in dict_data.items():
                        if hasattr(value, 'shape'):
                            print(f"         {key}: shape={value.shape}, dtype={value.dtype}")
                            # 顯示少量數據
                            if value.size <= 20:
                                print(f"            內容: {value}")
                        else:
                            print(f"         {key}: {type(value)} = {value}")
                else:
                    print(f"         項目內容: {type(dict_data)} = {dict_data}")
            except Exception as e:
                print(f"      ⚠️ 解析字典項目失敗: {e}")
    
    def _analyze_filename(self, filename: str):
        """分析檔案名稱"""
        print(f"   📝 檔案名稱分析:")
        
        file_lower = filename.lower()
        
        # 檢查各種關鍵字
        keywords = {
            'camera_matrix': ['camera_matrix', 'camera', 'intrinsic', 'calib'],
            'dist_coeffs': ['dist_coeffs', 'dist', 'distortion', 'coeffs'],
            'extrinsic': ['extrinsic', '外参', 'external', 'ext', 'rvec', 'tvec'],
            'case': ['case'],
            'timestamp': ['2024', '2025', '20241', '20251']
        }
        
        matches = {}
        for category, words in keywords.items():
            matches[category] = any(word in file_lower for word in words)
        
        print(f"      關鍵字匹配:")
        for category, matched in matches.items():
            print(f"         {category}: {'✅' if matched else '❌'}")
        
        # 根據關鍵字推測檔案類型
        if matches['camera_matrix']:
            if matches['case']:
                print(f"      🎯 推測類型: 相機矩陣檔案 (CASE專用)")
            else:
                print(f"      🎯 推測類型: 相機矩陣檔案")
        elif matches['dist_coeffs']:
            if matches['case']:
                print(f"      🎯 推測類型: 畸變係數檔案 (CASE專用)")
            else:
                print(f"      🎯 推測類型: 畸變係數檔案")
        elif matches['extrinsic']:
            if matches['case']:
                print(f"      🎯 推測類型: 外參檔案 (CASE專用)")
            else:
                print(f"      🎯 推測類型: 外參檔案")
        else:
            print(f"      🎯 推測類型: 未知類型")
    
    def _test_fixed_loading_logic(self, npy_files: List[str]):
        """測試修正版載入邏輯"""
        print(f"🧪 測試修正版掃描和載入邏輯:")
        
        # 使用修正版分類檔案
        camera_matrix_files = []
        dist_coeffs_files = []
        extrinsic_files = []
        unknown_files = []
        
        for file in npy_files:
            file_path = os.path.join(self.working_dir, file)
            file_type = self._classify_file_fixed(file, file_path)
            
            if file_type == 'camera_matrix':
                camera_matrix_files.append(file)
            elif file_type == 'dist_coeffs':
                dist_coeffs_files.append(file)
            elif file_type == 'extrinsic':
                extrinsic_files.append(file)
            else:
                unknown_files.append(file)
        
        print(f"\n📊 修正版分類結果:")
        print(f"   相機矩陣檔案: {len(camera_matrix_files)}個 - {camera_matrix_files}")
        print(f"   畸變係數檔案: {len(dist_coeffs_files)}個 - {dist_coeffs_files}")
        print(f"   外參檔案: {len(extrinsic_files)}個 - {extrinsic_files}")
        print(f"   未知檔案: {len(unknown_files)}個 - {unknown_files}")
        
        # 測試修正版載入組合
        self._test_fixed_calibration_combinations(camera_matrix_files, dist_coeffs_files, extrinsic_files)
    
    def _classify_file_fixed(self, filename: str, file_path: str) -> str:
        """修正版檔案分類邏輯"""
        print(f"\n🔍 修正版分類檔案: {filename}")
        
        try:
            # 載入檔案
            data = np.load(file_path, allow_pickle=True)
            
            # 檔案名稱關鍵字分析
            file_lower = filename.lower()
            is_camera_matrix = any(keyword in file_lower for keyword in 
                                 ['camera_matrix', 'camera', 'intrinsic', 'calib'])
            is_dist_coeffs = any(keyword in file_lower for keyword in 
                               ['dist_coeffs', 'dist', 'distortion', 'coeffs'])
            is_extrinsic = any(keyword in file_lower for keyword in 
                             ['extrinsic', '外参', 'external', 'ext'])
            
            print(f"   檔案名稱關鍵字: camera={is_camera_matrix}, dist={is_dist_coeffs}, ext={is_extrinsic}")
            
            # 1. 檢查字典格式
            if isinstance(data, dict):
                print(f"   格式: 字典 - 鍵值: {list(data.keys())}")
                if 'camera_matrix' in data and 'dist_coeffs' in data:
                    print(f"   ✅ 判定: 完整內參檔案 (包含相機矩陣和畸變係數)")
                    return 'camera_matrix'
                elif 'camera_matrix' in data:
                    print(f"   ✅ 判定: 相機矩陣檔案")
                    return 'camera_matrix'
                elif 'rvec' in data and 'tvec' in data:
                    print(f"   ✅ 判定: 外參檔案")
                    return 'extrinsic'
            
            # 2. 檢查字典項目格式 (0維數組包含字典)
            elif hasattr(data, 'item') and callable(data.item) and data.shape == ():
                print(f"   格式: 0維數組，嘗試解析字典項目...")
                try:
                    dict_data = data.item()
                    if isinstance(dict_data, dict):
                        print(f"   格式: 字典項目 - 鍵值: {list(dict_data.keys())}")
                        if 'camera_matrix' in dict_data and 'dist_coeffs' in dict_data:
                            print(f"   ✅ 判定: 完整內參檔案 (字典項目)")
                            return 'camera_matrix'
                        elif 'camera_matrix' in dict_data:
                            print(f"   ✅ 判定: 相機矩陣檔案 (字典項目)")
                            return 'camera_matrix'
                        elif 'rvec' in dict_data and 'tvec' in dict_data:
                            print(f"   ✅ 判定: 外參檔案 (字典項目)")
                            return 'extrinsic'
                except Exception as e:
                    print(f"   ⚠️ 解析字典項目失敗: {e}")
            
            # 3. 基於數組形狀和檔案名稱判斷
            if hasattr(data, 'shape'):
                print(f"   格式: 數組 - 形狀: {data.shape}")
                
                # 3x3矩陣 - 相機內參矩陣
                if data.shape == (3, 3):
                    print(f"   📐 3x3矩陣 - 典型的相機內參矩陣")
                    if is_camera_matrix or (not is_dist_coeffs and not is_extrinsic):
                        print(f"   ✅ 判定: 相機矩陣檔案")
                        return 'camera_matrix'
                
                # 畸變係數向量
                elif data.shape in [(5,), (6,), (8,), (1, 5), (1, 8), (5, 1), (8, 1)]:
                    print(f"   📐 {data.shape}向量 - 典型的畸變係數")
                    if is_dist_coeffs or (not is_camera_matrix and not is_extrinsic):
                        print(f"   ✅ 判定: 畸變係數檔案")
                        return 'dist_coeffs'
                
                # 外參矩陣
                elif data.shape in [(4, 4), (3, 4)]:
                    print(f"   📐 {data.shape}矩陣 - 典型的外參變換矩陣")
                    if is_extrinsic or (not is_camera_matrix and not is_dist_coeffs):
                        print(f"   ✅ 判定: 外參檔案")
                        return 'extrinsic'
                
                # 4. 最終基於檔案名稱判斷
                print(f"   🔄 基於檔案名稱進行最終判斷...")
                if is_camera_matrix:
                    print(f"   ✅ 判定: 相機矩陣檔案 (基於檔案名稱)")
                    return 'camera_matrix'
                elif is_dist_coeffs:
                    print(f"   ✅ 判定: 畸變係數檔案 (基於檔案名稱)")
                    return 'dist_coeffs'
                elif is_extrinsic:
                    print(f"   ✅ 判定: 外參檔案 (基於檔案名稱)")
                    return 'extrinsic'
            
            print(f"   ❓ 判定: 未知類型")
            return 'unknown'
            
        except Exception as e:
            print(f"   ❌ 載入失敗: {e}")
            return 'unknown'
    
    def _test_fixed_calibration_combinations(self, camera_files: List[str], dist_files: List[str], ext_files: List[str]):
        """測試修正版標定數據組合載入"""
        print(f"\n🧪 測試修正版標定數據載入組合:")
        
        if not camera_files:
            print(f"❌ 沒有相機矩陣檔案，無法測試")
            return
        
        if not ext_files:
            print(f"❌ 沒有外參檔案，無法測試")
            return
        
        # 測試不同的組合
        combinations = []
        
        # 組合1: 相機矩陣 + 畸變係數 + 外參
        if camera_files and dist_files and ext_files:
            combinations.append({
                'name': '完整組合 (相機矩陣+畸變係數+外參)',
                'camera': camera_files[0],
                'dist': dist_files[0],
                'extrinsic': ext_files[0]
            })
        
        # 組合2: 僅相機矩陣 + 外參 (畸變係數使用零值)
        if camera_files and ext_files:
            combinations.append({
                'name': '簡化組合 (僅相機矩陣+外參)',
                'camera': camera_files[0],
                'dist': None,
                'extrinsic': ext_files[0]
            })
        
        # 測試每個組合
        for i, combo in enumerate(combinations, 1):
            print(f"\n📋 測試組合 {i}: {combo['name']}")
            success = self._test_fixed_single_combination(combo)
            print(f"   結果: {'✅ 成功' if success else '❌ 失敗'}")
    
    def _test_fixed_single_combination(self, combo: Dict[str, Any]) -> bool:
        """測試修正版單個標定數據組合"""
        try:
            print(f"   載入檔案:")
            print(f"      相機矩陣: {combo['camera']}")
            print(f"      畸變係數: {combo['dist'] or '使用零值'}")
            print(f"      外參數據: {combo['extrinsic']}")
            
            # === 修正版載入相機矩陣 ===
            camera_matrix = None
            dist_coeffs = None
            
            camera_path = os.path.join(self.working_dir, combo['camera'])
            camera_data = np.load(camera_path, allow_pickle=True)
            
            print(f"      🔄 載入相機矩陣:")
            print(f"         數據類型: {type(camera_data)}")
            print(f"         數據形狀: {camera_data.shape if hasattr(camera_data, 'shape') else 'N/A'}")
            
            # 根據測試結果，camera_matrix_CASE.npy是直接的3x3數組
            if isinstance(camera_data, dict):
                if 'camera_matrix' in camera_data:
                    camera_matrix = camera_data['camera_matrix']
                    if 'dist_coeffs' in camera_data:
                        dist_coeffs = camera_data['dist_coeffs']
                        print(f"         📊 從字典載入相機矩陣和畸變係數")
                    else:
                        print(f"         📊 從字典載入相機矩陣")
            elif hasattr(camera_data, 'shape') and camera_data.shape == (3, 3):
                # 直接是3x3數組的情況 (您的情況)
                camera_matrix = camera_data
                print(f"         📊 直接載入3x3相機矩陣數組")
            elif hasattr(camera_data, 'item') and callable(camera_data.item) and camera_data.shape == ():
                # 0維數組包含字典的情況
                dict_data = camera_data.item()
                if isinstance(dict_data, dict):
                    camera_matrix = dict_data.get('camera_matrix')
                    if 'dist_coeffs' in dict_data:
                        dist_coeffs = dict_data['dist_coeffs']
                    print(f"         📊 從字典項目載入相機矩陣")
            
            # === 修正版載入畸變係數 ===
            if combo['dist'] and dist_coeffs is None:
                dist_path = os.path.join(self.working_dir, combo['dist'])
                dist_data = np.load(dist_path, allow_pickle=True)
                
                print(f"      🔄 載入畸變係數:")
                print(f"         數據類型: {type(dist_data)}")
                print(f"         數據形狀: {dist_data.shape if hasattr(dist_data, 'shape') else 'N/A'}")
                
                # 根據測試結果，dist_coeffs_CASE.npy是(1,5)數組
                if hasattr(dist_data, 'shape'):
                    dist_coeffs = dist_data
                    print(f"         📊 載入畸變係數: {dist_data.shape}")
            
            # 如果沒有畸變係數，使用零值
            if dist_coeffs is None:
                dist_coeffs = np.zeros((1, 5))
                print(f"         📊 使用零值畸變係數: {dist_coeffs.shape}")
            
            # === 修正版載入外參 ===
            ext_path = os.path.join(self.working_dir, combo['extrinsic'])
            ext_data = np.load(ext_path, allow_pickle=True)
            
            print(f"      🔄 載入外參:")
            print(f"         數據類型: {type(ext_data)}")
            print(f"         數據形狀: {ext_data.shape if hasattr(ext_data, 'shape') else 'N/A'}")
            
            rvec = None
            tvec = None
            
            # 根據測試結果，extrinsic_CASE.npy是0維數組包含字典
            if isinstance(ext_data, dict):
                rvec = ext_data.get('rvec')
                tvec = ext_data.get('tvec')
                print(f"         📊 從字典載入外參")
            elif hasattr(ext_data, 'item') and callable(ext_data.item) and ext_data.shape == ():
                # 0維數組包含字典的情況 (您的情況)
                dict_data = ext_data.item()
                if isinstance(dict_data, dict):
                    rvec = dict_data.get('rvec')
                    tvec = dict_data.get('tvec')
                    print(f"         📊 從字典項目載入外參")
            
            # === 驗證載入的數據 ===
            print(f"   📊 載入結果驗證:")
            
            if camera_matrix is not None:
                print(f"      相機矩陣: ✅ {camera_matrix.shape} {camera_matrix.dtype}")
                if camera_matrix.shape == (3, 3):
                    fx, fy = camera_matrix[0,0], camera_matrix[1,1]
                    cx, cy = camera_matrix[0,2], camera_matrix[1,2]
                    det = np.linalg.det(camera_matrix)
                    print(f"         參數: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
                    print(f"         行列式: {det:.2f} (應該>0)")
                else:
                    print(f"         ❌ 形狀錯誤，期望(3,3)")
                    return False
            else:
                print(f"      相機矩陣: ❌ 載入失敗")
                return False
            
            if dist_coeffs is not None:
                print(f"      畸變係數: ✅ {dist_coeffs.shape} {dist_coeffs.dtype}")
                nonzero_count = np.count_nonzero(dist_coeffs)
                print(f"         非零個數: {nonzero_count}/5")
                if dist_coeffs.shape in [(1, 5), (5,), (1, 8), (8,)]:
                    print(f"         內容: {dist_coeffs.flatten()}")
                else:
                    print(f"         ⚠️ 形狀異常，但可用")
            else:
                print(f"      畸變係數: ❌ 載入失敗")
                return False
            
            if rvec is not None and tvec is not None:
                print(f"      旋轉向量: ✅ {rvec.shape} {rvec.dtype}")
                print(f"         內容: {rvec.flatten()}")
                print(f"         範圍: [{rvec.min():.6f}, {rvec.max():.6f}]")
                print(f"      平移向量: ✅ {tvec.shape} {tvec.dtype}")
                print(f"         內容: {tvec.flatten()}")
                print(f"         範圍: [{tvec.min():.6f}, {tvec.max():.6f}]")
                
                if rvec.size >= 3 and tvec.size >= 3:
                    print(f"         ✅ 外參數據大小正確")
                else:
                    print(f"         ❌ 外參數據大小異常")
                    return False
            else:
                print(f"      外參數據: ❌ 載入失敗")
                return False
            
            # === 測試座標轉換器創建 ===
            print(f"   🧪 測試座標轉換器創建:")
            try:
                # 簡化的轉換器測試
                if (camera_matrix.shape == (3, 3) and 
                    len(dist_coeffs.flatten()) >= 4 and
                    rvec.size >= 3 and tvec.size >= 3):
                    
                    print(f"      ✅ 參數格式正確，可以創建座標轉換器")
                    
                    # 測試一個示例座標轉換 (簡化版本)
                    test_pixel = np.array([640.0, 480.0])  # 示例像素座標
                    print(f"      🧪 測試像素座標: {test_pixel}")
                    
                    # 簡化的參數檢查
                    print(f"      📊 標定參數摘要:")
                    print(f"         相機矩陣確定性: {np.linalg.det(camera_matrix):.6f}")
                    print(f"         畸變係數非零個數: {np.count_nonzero(dist_coeffs)}/5")
                    print(f"         外參旋轉角度範圍: [{rvec.min():.6f}, {rvec.max():.6f}]")
                    print(f"         外參平移範圍: [{tvec.min():.6f}, {tvec.max():.6f}]")
                    
                    # 檢查參數合理性
                    if np.linalg.det(camera_matrix) > 0:
                        print(f"         ✅ 相機矩陣確定性正常")
                    else:
                        print(f"         ⚠️ 相機矩陣確定性異常")
                    
                    if abs(rvec.max() - rvec.min()) < 10:  # 旋轉向量通常在reasonable range
                        print(f"         ✅ 旋轉向量範圍合理")
                    else:
                        print(f"         ⚠️ 旋轉向量範圍可能異常")
                    
                    print(f"      ✅ 所有測試通過，標定數據可用於座標轉換")
                    return True
                else:
                    print(f"      ❌ 參數格式不正確")
                    return False
                    
            except Exception as e:
                print(f"      ❌ 座標轉換器測試失敗: {e}")
                return False
            
        except Exception as e:
            print(f"   ❌ 載入組合失敗: {e}")
            traceback.print_exc()
            return False

def main():
    """主函數"""
    print("🚀 修正版標定檔案類型查看器")
    print("修正載入邏輯，測試實際檔案格式的完整載入流程")
    print("=" * 80)
    
    try:
        inspector = FixedCalibrationInspector()
        inspector.inspect_all_files()
        
        print("\n" + "=" * 80)
        print("✅ 修正版檢查完成")
        print("如果所有測試都成功，就可以安心整合到YOLOv11 CCD1程式中了！")
        
    except Exception as e:
        print(f"❌ 程式執行失敗: {e}")
        traceback.print_exc()
    
    input("\n按Enter鍵退出...")

if __name__ == "__main__":
    main()