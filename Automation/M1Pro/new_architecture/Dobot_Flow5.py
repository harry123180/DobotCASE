#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow5.py - Flow5 機械臂運轉流程 (新架構版)
基於統一Flow架構的運動控制執行器
流程: standby → Goal_CV_top → rotate_top → put_asm_pre → put_asm_top → put_asm_down → 夾爪快速關閉 → put_asm_top
使用真實API連接，禁止模擬代碼
"""

import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow5AssemblyExecutor(FlowExecutor):
    """Flow5: 機械臂運轉流程執行器"""
    
    def __init__(self):
        super().__init__(flow_id=5, flow_name="機械臂運轉流程")
        self.motion_steps = []
        self.build_flow_steps()
        
        # 實際點位座標 (基於專案知識中的robot_points.json)
        self.PREDEFINED_POINTS = {
            'standby': {
                'x': 120.59, 'y': -313.56, 'z': 244.65, 'r': 94.56
            },
            'Goal_CV_top': {
                'x': 262.06, 'y': -108.36, 'z': 244.0, 'r': 94.56
            },
            'rotate_top': {
                'x': 263.83, 'y': 16.73, 'z': 244.26, 'r': 112.52
            },
            'put_asm_pre': {
                'x': 153.97, 'y': 212.03, 'z': 230.34, 'r': 149.29
            },
            'put_asm_top': {
                'x': -78.18, 'y': 368.51, 'z': 244.26, 'r': 193.99
            },
            'put_asm_down': {
                'x': -78.18, 'y': 368.51, 'z': 162.76, 'r': 193.99
            }
        }
        
        # 夾爪參數
        self.GRIPPER_POSITIONS = {
            'CLOSE': 0,
            'OPEN': 370
        }
        
    def build_flow_steps(self):
        """建構Flow5步驟序列"""
        self.motion_steps = [
            # 1. 移動到standby
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 2. 移動到Goal_CV_top
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            
            # 3. 移動到rotate_top
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 4. 移動到put_asm_pre
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_pre', 'move_type': 'J'}},
            
            # 5. 移動到put_asm_top
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            
            # 6. 移動到put_asm_down
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_down', 'move_type': 'L'}},
            
            # 7. 夾爪快速關閉
            {'type': 'gripper_quick_close', 'params': {}},
            
            # 8. 移動到put_asm_top
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_top', 'move_type': 'L'}}
        ]
        
        self.total_steps = len(self.motion_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow5主邏輯"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        print("\n" + "="*60)
        print("開始執行Flow5 - 機械臂運轉流程")
        print("="*60)
        
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
                
                print(f"Flow5 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_quick_close':
                    success = self._execute_gripper_quick_close()
                else:
                    print(f"未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {self.current_step + 1} 執行失敗: {step['type']}",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
                
                # 更新進度
                if self.state_machine:
                    progress = int((self.current_step / self.total_steps) * 100)
                    self.state_machine.set_flow_progress(progress)
            
            # 流程完成
            execution_time = time.time() - self.start_time
            self.status = FlowStatus.COMPLETED
            
            print(f"\n✓ Flow5執行完成！總耗時: {execution_time:.2f}秒")
            print("="*60)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            error_msg = f"Flow5執行異常: {str(e)}"
            print(f"✗ {error_msg}")
            
            return FlowResult(
                success=False,
                error_message=error_msg,
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到指定點位"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            if point_name not in self.PREDEFINED_POINTS:
                print(f"✗ 未找到點位: {point_name}")
                return False
            
            point = self.PREDEFINED_POINTS[point_name]
            print(f"  → 移動到 {point_name}: ({point['x']:.2f}, {point['y']:.2f}, {point['z']:.2f}, {point['r']:.2f})")
            
            # 執行移動指令
            if move_type == 'J':
                # 關節運動
                success = self.robot.movj_to_point(
                    point['x'], point['y'], point['z'], point['r']
                )
            else:
                # 直線運動
                success = self.robot.movl_to_point(
                    point['x'], point['y'], point['z'], point['r']
                )
            
            if success:
                print(f"  ✓ 成功移動到 {point_name}")
                return True
            else:
                print(f"  ✗ 移動到 {point_name} 失敗")
                return False
                
        except Exception as e:
            print(f"  ✗ 移動操作異常: {e}")
            return False
    
    def _execute_gripper_quick_close(self) -> bool:
        """執行夾爪快速關閉"""
        try:
            print(f"  → 夾爪快速關閉到位置: {self.GRIPPER_POSITIONS['CLOSE']}")
            
            if self.gripper:
                success = self.gripper.quick_close()
                if success:
                    print("  ✓ 夾爪快速關閉成功")
                    return True
                else:
                    print("  ✗ 夾爪快速關閉失敗")
                    return False
            else:
                print("  ✗ 夾爪控制器未初始化")
                return False
                
        except Exception as e:
            print(f"  ✗ 夾爪操作異常: {e}")
            return False
    
    def pause(self):
        """暫停流程"""
        self.status = FlowStatus.PAUSED
        print("Flow5 已暫停")
    
    def resume(self):
        """恢復流程"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow5 已恢復")
    
    def stop(self):
        """停止流程"""
        self.status = FlowStatus.ERROR
        print("Flow5 已停止")
    
    def get_status_info(self) -> Dict[str, Any]:
        """取得流程狀態資訊"""
        return {
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'status': self.status.value if self.status else 'UNKNOWN',
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0,
            'execution_time': time.time() - self.start_time if self.start_time else 0
        }


# 工廠函數
def create_flow5_executor() -> Flow5AssemblyExecutor:
    """建立Flow5執行器實例"""
    return Flow5AssemblyExecutor()


if __name__ == "__main__":
    # 測試代碼 (僅用於開發除錯)
    print("Flow5 機械臂運轉流程模組載入完成")
    executor = create_flow5_executor()
    print(f"Flow5執行器建立完成: {executor.flow_name}")
    print(f"總步驟數: {executor.total_steps}")
    
    # 列印流程步驟
    print("\nFlow5流程步驟:")
    for i, step in enumerate(executor.motion_steps, 1):
        print(f"  {i}. {step['type']} - {step['params']}")