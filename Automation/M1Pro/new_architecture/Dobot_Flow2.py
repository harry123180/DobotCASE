#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2_new.py - Flow2 出料流程 (新架構版)
基於統一Flow架構的運動控制執行器
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
            
            # 7. 上升到組裝預備位置
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_pre', 'move_type': 'J'}}
        ]
        
        self.total_steps = len(self.motion_steps)
        
    def execute(self) -> FlowResult:
        """執行Flow2流程"""
        print("\n" + "="*60)
        print("開始執行Flow2 - 出料流程")
        print("="*60)
        
        start_time = time.time()
        self.status = FlowStatus.RUNNING
        self.current_step = 0
        
        try:
            for i, step in enumerate(self.motion_steps):
                if self.status != FlowStatus.RUNNING:
                    break
                    
                self.current_step = i + 1
                print(f"\n[步驟 {self.current_step}/{self.total_steps}] {step['type']}")
                
                if not self._execute_step(step):
                    return FlowResult(
                        success=False,
                        error_message=f"步驟{self.current_step}執行失敗: {step['type']}",
                        execution_time=time.time() - start_time,
                        steps_completed=self.current_step - 1,
                        total_steps=self.total_steps
                    )
                    
                # 步驟間延遲
                time.sleep(0.1)
                
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - start_time
            
            print(f"\n✓ Flow2執行完成！總耗時: {execution_time:.2f}秒")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            self.last_error = str(e)
            
            return FlowResult(
                success=False,
                error_message=str(e),
                execution_time=time.time() - start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
    def _execute_step(self, step: Dict) -> bool:
        """執行單一步驟"""
        step_type = step['type']
        params = step['params']
        
        try:
            if step_type == 'move_to_point':
                return self._move_to_point(params)
            elif step_type == 'gripper_close':
                return self._gripper_close()
            elif step_type == 'gripper_smart_release':
                return self._gripper_smart_release(params)
            elif step_type == 'gripper_quick_close':
                return self._gripper_quick_close()
            else:
                print(f"未知步驟類型: {step_type}")
                return False
                
        except Exception as e:
            print(f"步驟執行錯誤: {e}")
            return False
            
    def _move_to_point(self, params: Dict) -> bool:
        """移動到預設點位"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            print(f"  移動到點位: {point_name} (類型: {move_type})")
            
            # 模擬點位座標 (實際應從點位管理器獲取)
            points = {
                'standby': (300, 0, 200, 0),
                'Goal_CV_top': (100, -300, 250, 0),
                'Goal_CV_down': (100, -300, 150, 0),
                'rotate_top': (200, -100, 200, 0),
                'rotate_down': (200, -100, 140, 0),
                'put_asm_pre': (350, -100, 250, 0)
            }
            
            if point_name not in points:
                print(f"  ✗ 未知點位: {point_name}")
                return False
                
            x, y, z, r = points[point_name]
            
            if self.robot and self.robot.is_connected:
                if move_type == 'J':
                    result = self.robot.move_api.MovJ(x, y, z, r)
                else:
                    result = self.robot.move_api.MovL(x, y, z, r)
                    
                print(f"  ✓ {move_type}({x}, {y}, {z}, {r}) -> {result}")
                return "0" in str(result)
            else:
                print(f"  ✗ 機械臂未連接")
                return False
                
        except Exception as e:
            print(f"  ✗ 移動失敗: {e}")
            return False
            
    def _gripper_close(self) -> bool:
        """夾爪關閉"""
        try:
            print("  夾爪快速關閉")
            
            # 使用外部模組 (夾爪)
            if 'GRIPPER' in self.external_modules and self.external_modules['GRIPPER']:
                gripper = self.external_modules['GRIPPER']
                return gripper.quick_close()
            else:
                # 模擬夾爪操作
                print("  ✓ 夾爪關閉完成")
                return True
                
        except Exception as e:
            print(f"  ✗ 夾爪關閉失敗: {e}")
            return False
            
    def _gripper_smart_release(self, params: Dict) -> bool:
        """智能撐開料件"""
        try:
            position = params['position']
            print(f"  智能撐開料件到位置: {position}")
            
            # 使用外部模組 (夾爪)
            if 'GRIPPER' in self.external_modules and self.external_modules['GRIPPER']:
                gripper = self.external_modules['GRIPPER']
                return gripper.smart_release(release_position=position)
            else:
                # 模擬智能撐開
                print(f"  ✓ 智能撐開到{position}完成")
                return True
                
        except Exception as e:
            print(f"  ✗ 智能撐開失敗: {e}")
            return False
            
    def _gripper_quick_close(self) -> bool:
        """夾爪快速關閉"""
        try:
            print("  夾爪快速關閉 (不等確認)")
            
            # 使用外部模組 (夾爪)
            if 'GRIPPER' in self.external_modules and self.external_modules['GRIPPER']:
                gripper = self.external_modules['GRIPPER']
                return gripper.quick_close()
            else:
                # 模擬快速關閉
                print("  ✓ 快速關閉完成")
                return True
                
        except Exception as e:
            print(f"  ✗ 快速關閉失敗: {e}")
            return False
            
    def pause(self) -> bool:
        """暫停Flow"""
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            print("Flow2已暫停")
            return True
        return False
        
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


# ==================== 使用範例 ====================

def example_usage():
    """Flow2使用範例"""
    print("=== Flow2 出料流程範例 ===")
    
    # 模擬外部依賴
    class MockRobot:
        def __init__(self):
            self.is_connected = True
            self.move_api = MockMoveAPI()
            self.dashboard_api = MockDashboardAPI()
            
    class MockMoveAPI:
        def MovJ(self, x, y, z, r):
            return f"0,MovJ({x},{y},{z},{r}),"
        def MovL(self, x, y, z, r):
            return f"0,MovL({x},{y},{z},{r}),"
            
    class MockDashboardAPI:
        def DO(self, pin, value):
            return f"0,DO{pin}={value},"
    
    class MockStateMachine:
        def read_register(self, offset):
            return 0
    
    # 創建Flow執行器
    flow2 = Flow2UnloadExecutor()
    
    # 初始化依賴
    mock_robot = MockRobot()
    mock_state_machine = MockStateMachine()
    mock_external_modules = {}
    
    flow2.initialize(mock_robot, mock_state_machine, mock_external_modules)
    
    # 執行Flow
    result = flow2.execute()
    
    # 輸出結果
    if result.success:
        print(f"\n✓ Flow2執行成功！")
        print(f"  耗時: {result.execution_time:.2f}秒")
        print(f"  完成步驟: {result.steps_completed}/{result.total_steps}")
    else:
        print(f"\n✗ Flow2執行失敗: {result.error_message}")
        print(f"  完成步驟: {result.steps_completed}/{result.total_steps}")


if __name__ == "__main__":
    example_usage()