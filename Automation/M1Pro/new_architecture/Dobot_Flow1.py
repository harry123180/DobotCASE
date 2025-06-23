#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_new.py - Flow1 VP視覺抓取流程 (新架構版 - 移除模擬代碼修正版)
基於統一Flow架構的運動控制執行器
禁止任何模擬代碼，全部使用真實API連接
"""

import time
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow1VisionPickExecutor(FlowExecutor):
    """Flow1: VP視覺抓取流程執行器"""
    
    def __init__(self):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程")
        self.motion_steps = []
        self.build_flow_steps()
        
        # 預定義點位 (真實座標)
        self.PREDEFINED_POINTS = {
            'standby': {'x': 250.0, 'y': 0.0, 'z': 150.0, 'r': 0.0},
            'vp_topside': {'x': 100.0, 'y': 200.0, 'z': 120.0, 'r': 0.0},
            'flip_pre': {'x': 300.0, 'y': -100.0, 'z': 150.0, 'r': 0.0},
            'flip_top': {'x': 300.0, 'y': -100.0, 'z': 100.0, 'r': 0.0},
            'flip_down': {'x': 300.0, 'y': -100.0, 'z': 50.0, 'r': 0.0}
        }
        
        # CCD2 IO控制腳位
        self.CCD2_TRIGGER_PIN = 8  # DO8: 觸發CCD2檢測
        
    def build_flow_steps(self):
        """建構Flow1步驟"""
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
            {'type': 'move_to_point', 'params': {'point_name': 'flip_down', 'move_type': 'L'}},
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_top', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 7. 觸發CCD2檢測
            {'type': 'trigger_ccd2', 'params': {}}
        ]
        
        self.total_steps = len(self.motion_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯"""
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
        """執行移動到預定義點位"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            if point_name not in self.PREDEFINED_POINTS:
                print(f"未定義的點位: {point_name}")
                return False
            
            point = self.PREDEFINED_POINTS[point_name]
            
            if move_type == 'J':
                return self.robot.move_j(point['x'], point['y'], point['z'], point['r'])
            elif move_type == 'L':
                return self.robot.move_l(point['x'], point['y'], point['z'], point['r'])
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
        """執行CCD1視覺檢測 - PyModbus 3.9.2修正版"""
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
                detected_pos = {
                    'x': circle_coord.world_x,
                    'y': circle_coord.world_y,
                    'z': self.PREDEFINED_POINTS['vp_topside']['z'],  # 使用vp_topside的Z高度
                    'r': 0.0
                }
                print(f"CCD1檢測成功: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                return detected_pos
            else:
                print("CCD1未檢測到有效物件")
                return None
                
        except Exception as e:
            print(f"CCD1檢測異常: {e}")
            return None
    
    def _execute_move_to_detected_high(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(等高)"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 移動到檢測位置，保持vp_topside的Z高度
            return self.robot.move_j(
                detected_position['x'],
                detected_position['y'],
                detected_position['z'],
                detected_position['r']
            )
            
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度)"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 下降到夾取高度 (Z軸降低50mm)
            pick_z = detected_position['z'] - 50.0
            
            return self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                pick_z,
                detected_position['r']
            )
            
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