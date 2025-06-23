#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_new.py - Flow1 VP視覺抓取流程 (新架構版)
基於統一Flow架構的運動控制執行器
"""

import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow1VisionPickExecutor(FlowExecutor):
    """Flow1: VP視覺抓取流程執行器"""
    
    def __init__(self):
        super().__init__(flow_id=1, flow_name="VP視覺抓取")
        self.motion_steps = []
        self.build_flow_steps()
        
    def build_flow_steps(self):
        """建構Flow1步驟"""
        self.motion_steps = [
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            
            # 2. 移動到VP檢測位置
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            
            # 3. CCD1視覺檢測
            {'type': 'ccd1_detect', 'params': {}},
            
            # 4. 移動到檢測物件位置(高度與vp_topside等高)
            {'type': 'move_to_detected_position', 'params': {'z_level': 'detection'}},
            
            # 5. 下降到夾取位置
            {'type': 'move_to_detected_position', 'params': {'z_level': 'pickup'}},
            
            # 6. 夾爪撐開至370
            {'type': 'gripper_position', 'params': {'position': 370}},
            
            # 7. 上升到安全高度
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L'}},
            
            # 8. 回到待機位置
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 9. 翻轉站序列
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_top', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_down', 'move_type': 'L'}},
            
            # 10. 夾爪關閉
            {'type': 'gripper_close', 'params': {}},
            
            # 11. 離開翻轉站
            {'type': 'move_to_point', 'params': {'point_name': 'flip_top', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 12. 觸發CCD2翻轉檢測
            {'type': 'trigger_ccd2', 'params': {}}
        ]
        
        self.total_steps = len(self.motion_steps)
        
    def execute(self) -> FlowResult:
        """執行Flow1流程"""
        print("\n" + "="*60)
        print("開始執行Flow1 - VP視覺抓取流程")
        print("="*60)
        
        start_time = time.time()
        self.status = FlowStatus.RUNNING
        self.current_step = 0
        
        # 儲存檢測結果
        self.detected_position = None
        
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
            
            print(f"\n✓ Flow1執行完成！總耗時: {execution_time:.2f}秒")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps,
                flow_data={'detected_position': self.detected_position}
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
            elif step_type == 'gripper_position':
                return self._gripper_position(params)
            elif step_type == 'ccd1_detect':
                return self._ccd1_detect()
            elif step_type == 'move_to_detected_position':
                return self._move_to_detected_position(params)
            elif step_type == 'trigger_ccd2':
                return self._trigger_ccd2()
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
                'vp_topside': (150, 250, 238.86, 0),
                'flip_pre': (200, -200, 250, 0),
                'flip_top': (200, -200, 200, 0),
                'flip_down': (200, -200, 150, 0)
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
            
    def _gripper_position(self, params: Dict) -> bool:
        """夾爪移動到指定位置"""
        try:
            position = params['position']
            print(f"  夾爪移動到位置: {position}")
            
            # 使用外部模組 (夾爪)
            if 'GRIPPER' in self.external_modules and self.external_modules['GRIPPER']:
                gripper = self.external_modules['GRIPPER']
                return gripper.move_to_and_wait(position)
            else:
                # 模擬夾爪操作
                print(f"  ✓ 夾爪移動到{position}完成")
                return True
                
        except Exception as e:
            print(f"  ✗ 夾爪移動失敗: {e}")
            return False
            
    def _ccd1_detect(self) -> bool:
        """CCD1視覺檢測"""
        try:
            print("  執行CCD1視覺檢測")
            
            # 使用外部模組 (CCD1)
            if 'CCD1' in self.external_modules and self.external_modules['CCD1']:
                ccd1 = self.external_modules['CCD1']
                success = ccd1.send_command(16, {}, timeout=10.0)  # 檢測指令
                
                if success:
                    # 讀取檢測結果
                    self.detected_position = self._read_ccd1_results()
                    return self.detected_position is not None
                else:
                    return False
            else:
                # 模擬檢測結果
                self.detected_position = {'x': 160, 'y': 260, 'z_detection': 238.86, 'z_pickup': 137.52}
                print(f"  ✓ 檢測到物件: {self.detected_position}")
                return True
                
        except Exception as e:
            print(f"  ✗ CCD1檢測失敗: {e}")
            return False
            
    def _read_ccd1_results(self) -> Optional[Dict]:
        """讀取CCD1檢測結果"""
        try:
            # 讀取檢測數量
            count = self.state_machine.read_register(40)  # 240-200=40
            
            if count and count > 0:
                # 讀取第一個物件座標
                x = self.state_machine.read_register(41)  # 241
                y = self.state_machine.read_register(42)  # 242
                
                # 座標轉換 (像素→機械臂座標)
                robot_x = x * 0.1 + 150  # 假設轉換係數
                robot_y = y * 0.1 + 250
                
                return {
                    'x': robot_x,
                    'y': robot_y,
                    'z_detection': 238.86,  # vp_topside高度
                    'z_pickup': 137.52      # 夾取高度
                }
            else:
                print("  ✗ 未檢測到物件")
                return None
                
        except Exception as e:
            print(f"  ✗ 讀取檢測結果失敗: {e}")
            return None
            
    def _move_to_detected_position(self, params: Dict) -> bool:
        """移動到檢測位置"""
        try:
            if not self.detected_position:
                print("  ✗ 無檢測位置資訊")
                return False
                
            z_level = params['z_level']
            x = self.detected_position['x']
            y = self.detected_position['y']
            
            if z_level == 'detection':
                z = self.detected_position['z_detection']
                print(f"  移動到檢測高度: ({x}, {y}, {z})")
            elif z_level == 'pickup':
                z = self.detected_position['z_pickup']
                print(f"  移動到夾取高度: ({x}, {y}, {z})")
            else:
                print(f"  ✗ 未知Z軸層級: {z_level}")
                return False
                
            if self.robot and self.robot.is_connected:
                result = self.robot.move_api.MovL(x, y, z, 0)
                print(f"  ✓ MovL({x}, {y}, {z}, 0) -> {result}")
                return "0" in str(result)
            else:
                print(f"  ✗ 機械臂未連接")
                return False
                
        except Exception as e:
            print(f"  ✗ 移動到檢測位置失敗: {e}")
            return False
            
    def _trigger_ccd2(self) -> bool:
        """觸發CCD2翻轉檢測"""
        try:
            print("  觸發CCD2翻轉檢測模組")
            
            # 使用機械臂IO執行CCD2控制
            if self.robot and self.robot.is_connected:
                # 假設CCD2觸發為DO1脈衝
                self.robot.dashboard_api.DO(1, 1)
                time.sleep(0.1)
                self.robot.dashboard_api.DO(1, 0)
                
                print("  ✓ CCD2觸發完成 (異步執行)")
                return True
            else:
                print("  ✓ CCD2觸發完成 (模擬)")
                return True
                
        except Exception as e:
            print(f"  ✗ CCD2觸發失敗: {e}")
            return False
            
    def pause(self) -> bool:
        """暫停Flow"""
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            print("Flow1已暫停")
            return True
        return False
        
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


# ==================== 使用範例 ====================

def example_usage():
    """Flow1使用範例"""
    print("=== Flow1 VP視覺抓取流程範例 ===")
    
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
            # 模擬CCD1檢測結果
            if offset == 40:  # 檢測數量
                return 1
            elif offset == 41:  # X座標
                return 100
            elif offset == 42:  # Y座標
                return 100
            return 0
    
    # 創建Flow執行器
    flow1 = Flow1VisionPickExecutor()
    
    # 初始化依賴
    mock_robot = MockRobot()
    mock_state_machine = MockStateMachine()
    mock_external_modules = {}
    
    flow1.initialize(mock_robot, mock_state_machine, mock_external_modules)
    
    # 執行Flow
    result = flow1.execute()
    
    # 輸出結果
    if result.success:
        print(f"\n✓ Flow1執行成功！")
        print(f"  耗時: {result.execution_time:.2f}秒")
        print(f"  完成步驟: {result.steps_completed}/{result.total_steps}")
        if result.flow_data and 'detected_position' in result.flow_data:
            pos = result.flow_data['detected_position']
            print(f"  檢測位置: ({pos['x']}, {pos['y']})")
    else:
        print(f"\n✗ Flow1執行失敗: {result.error_message}")
        print(f"  完成步驟: {result.steps_completed}/{result.total_steps}")


if __name__ == "__main__":
    example_usage()