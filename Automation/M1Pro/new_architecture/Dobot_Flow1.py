#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_new.py - Flow1 VP視覺抓取流程 (修正版 - 使用外部點位檔案)
基於統一Flow架構的運動控制執行器
使用外部點位檔案，無法讀取時報錯跳過
"""

import time
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


@dataclass
class RobotPoint:
    """機械臂點位數據結構"""
    name: str
    x: float
    y: float
    z: float
    r: float
    j1: float
    j2: float
    j3: float
    j4: float


class PointsManager:
    """點位管理器 - 支援cartesian格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        # 確保使用絕對路徑，相對於當前執行檔案的目錄
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian格式"""
        try:
            print(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                print(f"錯誤: 點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        # 原始格式
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        # 新格式
                        pose_data = point_data["cartesian"]
                    else:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
                    # 檢查關節數據
                    if "joint" not in point_data:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少關節數據")
                        continue
                    
                    joint_data = point_data["joint"]
                    
                    point = RobotPoint(
                        name=point_data["name"],
                        x=float(pose_data["x"]),
                        y=float(pose_data["y"]),
                        z=float(pose_data["z"]),
                        r=float(pose_data["r"]),
                        j1=float(joint_data["j1"]),
                        j2=float(joint_data["j2"]),
                        j3=float(joint_data["j3"]),
                        j4=float(joint_data["j4"])
                    )
                    
                    # 處理點位名稱的拼寫錯誤
                    point_name = point.name
                    if point_name == "stanby":
                        point_name = "standby"
                        print(f"自動修正點位名稱: stanby -> standby")
                    
                    self.points[point_name] = point
                    
                except Exception as e:
                    print(f"處理點位 {point_data.get('name', 'unknown')} 時發生錯誤: {e}")
                    continue
                
            print(f"載入點位數據成功，共{len(self.points)}個點位: {list(self.points.keys())}")
            return True
            
        except Exception as e:
            print(f"錯誤: 載入點位數據失敗: {e}")
            return False
    
    def get_point(self, name: str) -> Optional[RobotPoint]:
        """獲取指定點位"""
        return self.points.get(name)
    
    def list_points(self) -> List[str]:
        """列出所有點位名稱"""
        return list(self.points.keys())
    
    def has_point(self, name: str) -> bool:
        """檢查是否存在指定點位"""
        return name in self.points


class Flow1VisionPickExecutor(FlowExecutor):
    """Flow1: VP視覺抓取流程執行器 - 使用外部點位檔案版本"""
    
    def __init__(self):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程")
        self.motion_steps = []
        
        # 流程高度參數（根據實際機台調整）
        self.VP_DETECT_HEIGHT = 244.65    # VP檢測高度（與vp_topside等高）
        self.PICKUP_HEIGHT = 148.92       # VP夾取高度（你的機台定義值）
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "vp_topside",   # VP震動盤上方點
            "flip_pre",     # 翻轉預備點
            "flip_top",     # 翻轉頂部點
            "flip_down"     # 翻轉底部點
        ]
        
        # CCD2 IO控制腳位
        self.CCD2_TRIGGER_PIN = 8  # DO8: 觸發CCD2檢測
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow1正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow1無法執行")
            self.points_loaded = False
            return
        
        # 檢查所有必要點位是否存在
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            print(f"錯誤: 缺少必要點位: {missing_points}")
            print(f"可用點位: {self.points_manager.list_points()}")
            self.points_loaded = False
            return
        
        print("✓ 所有必要點位載入成功")
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow1步驟"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        self.motion_steps = [
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            
            # 2. VP視覺檢測序列
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            {'type': 'ccd1_detection', 'params': {}},
            
            # 3. 移動到檢測位置 (等高)
            {'type': 'move_to_detected_position_high', 'params': {}},
            
            # 4. 下降夾取
            {'type': 'move_to_detected_position_low', 'params': {}},
            {'type': 'gripper_smart_release', 'params': {'position': 370}},
            
            # 5. 上升離開
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 6. 翻轉檢測序列
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_down', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 7. 觸發CCD2檢測
            {'type': 'trigger_ccd2', 'params': {}}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1流程步驟建構完成，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯"""
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow1",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        detected_position = None
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow1 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'ccd1_detection':
                    detected_position = self._execute_ccd1_detection()
                    success = detected_position is not None
                elif step['type'] == 'move_to_detected_position_high':
                    success = self._execute_move_to_detected_high(detected_position)
                elif step['type'] == 'move_to_detected_position_low':
                    success = self._execute_move_to_detected_low(detected_position)
                elif step['type'] == 'trigger_ccd2':
                    success = self._execute_trigger_ccd2()
                else:
                    print(f"未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={'detected_position': detected_position} if detected_position else None
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow1執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到外部點位檔案的點位 - 修正版使用關節角度"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name}")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
            if move_type == 'J':
                # 使用關節角度運動
                return self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標
                return self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _execute_gripper_close(self) -> bool:
        """執行夾爪關閉"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                return gripper_api.quick_close()
            else:
                print("夾爪API未初始化")
                return False
        except Exception as e:
            print(f"夾爪關閉失敗: {e}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開"""
        try:
            position = params.get('position', 370)
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                return gripper_api.smart_release(position)
            else:
                print("夾爪API未初始化")
                return False
        except Exception as e:
            print(f"夾爪智能撐開失敗: {e}")
            return False
    
    def _execute_ccd1_detection(self) -> Optional[Dict[str, float]]:
        """執行CCD1視覺檢測"""
        try:
            ccd1_api = self.external_modules.get('ccd1')
            if not ccd1_api:
                print("CCD1 API未初始化")
                return None
            
            # 執行拍照和檢測
            if not ccd1_api.capture_and_detect():
                print("CCD1拍照檢測失敗")
                return None
            
            # 獲取檢測結果
            circle_coord = ccd1_api.get_next_circle_world_coord()
            if circle_coord:
                # 獲取vp_topside點位的Z高度和R值
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if not vp_topside_point:
                    print("錯誤: 無法獲取vp_topside點位")
                    return None
                
                detected_pos = {
                    'x': circle_coord.world_x,
                    'y': circle_coord.world_y,
                    'z': vp_topside_point.z,  # 使用vp_topside的Z高度
                    'r': vp_topside_point.r   # 繼承vp_topside的R值
                }
                print(f"CCD1檢測成功: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                print(f"繼承vp_topside - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
                return detected_pos
            else:
                print("CCD1未檢測到有效物件")
                return None
                
        except Exception as e:
            print(f"CCD1檢測異常: {e}")
            return None
    
    def _execute_move_to_detected_high(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(等高) - 修正版，確保sync到位"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            print(f"移動到檢測位置(等高): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.VP_DETECT_HEIGHT:.2f})")
            
            # 🔥 關鍵修正：使用完整的MovL+sync流程
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.VP_DETECT_HEIGHT,
                detected_position['r']
            )
            
            if success:
                # 🔥 關鍵：確保MovL到位後才繼續
                self.robot.sync()
                print(f"MovL已完成並同步: 檢測高度={self.VP_DETECT_HEIGHT:.2f}mm, R={detected_position['r']:.2f}°")
                return True
            else:
                print(f"MovL指令執行失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度) - 使用你的機台定義值"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            print(f"移動到檢測位置(夾取): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.PICKUP_HEIGHT:.2f})")
            
            # 使用你的機台定義的夾取高度
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.PICKUP_HEIGHT,  # 148.92mm
                detected_position['r']
            )
            
            if success:
                print(f"下降到夾取位置完成，夾取高度={self.PICKUP_HEIGHT:.2f}mm, R={detected_position['r']:.2f}°")
            else:
                print(f"下降到夾取位置失敗")
                print(f"目標座標: X={detected_position['x']:.2f}, Y={detected_position['y']:.2f}, Z={self.PICKUP_HEIGHT:.2f}, R={detected_position['r']:.2f}")
            
            return success
            
        except Exception as e:
            print(f"移動到檢測位置(夾取高度)失敗: {e}")
            return False
    
    def _execute_trigger_ccd2(self) -> bool:
        """觸發CCD2檢測"""
        try:
            print("觸發CCD2物件正反面辨識")
            
            # 使用機械臂dashboard_api執行IO操作
            # 觸發CCD2檢測 (DO8 = 1)
            if not self.robot.set_do(self.CCD2_TRIGGER_PIN, 1):
                print("觸發CCD2失敗")
                return False
            
            # 延遲一段時間後復位
            time.sleep(0.1)  # 100ms脈衝
            
            if not self.robot.set_do(self.CCD2_TRIGGER_PIN, 0):
                print("CCD2復位失敗")
                return False
            
            print("✓ CCD2觸發成功")
            return True
            
        except Exception as e:
            print(f"觸發CCD2異常: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow1已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow1已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow1已停止")
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return self.points_loaded and self.total_steps > 0