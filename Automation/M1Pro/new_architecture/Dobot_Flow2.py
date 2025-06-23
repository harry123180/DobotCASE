#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2_new.py - Flow2 出料流程 (新架構版 - 移除模擬代碼修正版)
基於統一Flow架構的運動控制執行器
禁止任何模擬代碼，全部使用真實API連接
"""

import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow2UnloadExecutor(FlowExecutor):
    """Flow2: 出料流程執行器"""
    
    def __init__(self):
        super().__init__(flow_id=2, flow_name="出料流程")
        self.motion_steps = []
        self.build_flow_steps()
        
        # 預定義點位 (真實座標)
        self.PREDEFINED_POINTS = {
            'standby': {'x': 250.0, 'y': 0.0, 'z': 150.0, 'r': 0.0},
            'Goal_CV_top': {'x': 400.0, 'y': 150.0, 'z': 120.0, 'r': 0.0},
            'Goal_CV_down': {'x': 400.0, 'y': 150.0, 'z': 60.0, 'r': 0.0},
            'rotate_top': {'x': 200.0, 'y': -200.0, 'z': 120.0, 'r': 0.0},
            'rotate_down': {'x': 200.0, 'y': -200.0, 'z': 80.0, 'r': 0.0},
            'put_asm_pre': {'x': 150.0, 'y': -250.0, 'z': 150.0, 'r': 0.0}
        }
        
    def build_flow_steps(self):
        """建構Flow2步驟"""
        self.motion_steps = [
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            
            # 2. CV出料序列
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_down', 'move_type': 'L'}},
            
            # 3. 智能撐開
            {'type': 'gripper_smart_release', 'params': {'position': 370}},
            
            # 4. 上升離開
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'L'}},
            
            # 5. 移動到旋轉工位
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'L'}},
            
            # 6. 快速關閉夾爪
            {'type': 'gripper_quick_close', 'params': {}},
            
            # 7. 上升離開
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'L'}},
            
            # 8. 移動到預組裝位置 (Flow2結束點)
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_pre', 'move_type': 'J'}}
        ]
        
        self.total_steps = len(self.motion_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow2主邏輯"""
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
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow2 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'gripper_quick_close':
                    success = self._execute_gripper_quick_close()
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
                flow_data={'end_position': 'put_asm_pre'}
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow2執行異常: {str(e)}",
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
    
    def _execute_gripper_quick_close(self) -> bool:
        """執行夾爪快速關閉"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                return gripper_api.quick_close()
            else:
                print("夾爪API未初始化")
                return False
        except Exception as e:
            print(f"夾爪快速關閉失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow2已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow2已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow2已停止")
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)