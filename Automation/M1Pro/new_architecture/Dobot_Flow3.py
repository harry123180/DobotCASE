#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow3_flip.py - Flow3 翻轉站控制流程 (DIO控制架構版)
基於統一Flow架構的DIO控制執行器
控制翻轉站的升降缸、夾爪、翻轉缸、輸送帶
"""

import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class FlowFlipStationExecutor(FlowExecutor):
    """Flow3: 翻轉站控制流程執行器 (DIO控制)"""
    
    def __init__(self):
        super().__init__(flow_id=3, flow_name="翻轉站控制")
        self.dio_steps = []
        self.build_flow_steps()
        
        # DIO腳位定義
        self.DIO_PINS = {
            # 輸送帶控制
            'CONVEYOR': 2,          # DO2: 輸送帶啟動/關閉
            
            # 翻轉缸控制
            'FLIP_CYLINDER': 5,     # DO5: 翻轉缸 (HIGH=180度, LOW=0度)
            
            # 升降缸控制
            'LIFT_TRIGGER': 11,     # DO11: 升降缸啟動脈衝
            'LIFT_DIR1': 12,        # DO12: 升降缸方向控制位1
            'LIFT_DIR2': 13,        # DO13: 升降缸方向控制位2
            'LIFT_HOME': 14,        # DO14: 升降缸回原點脈衝
            
            # 感測器輸入
            'SENSOR_DI13': 13       # DI13: 輸送帶感測器
        }
        
        # 時間延遲設定
        self.TIMING_CONFIG = {
            'LIFT_MOTION_TIME': 1.0,    # 升降缸上升/下降時間 (秒)
            'LIFT_HOME_TIME': 5.0,      # 升降缸回原點時間 (秒)
            'FLIP_TIME': 2.0,           # 翻轉缸翻轉時間 (秒)
            'GRIPPER_TIME': 1.0,        # PGE夾爪動作時間 (秒)
            'DIRECTION_SETUP_DELAY': 0.1,  # 方向設定延遲 (秒)
            'PULSE_WIDTH': 100          # 脈衝寬度 (毫秒)
        }
        
        # PGE夾爪位置定義
        self.GRIPPER_POSITIONS = {
            'GRIP': 500,            # 夾住位置
            'RELEASE': 1000         # 放開位置
        }
        
    def build_flow_steps(self):
        """建構Flow3步驟"""
        self.dio_steps = [
            # 1. 升降缸回原點
            {'type': 'lift_home', 'params': {}},
            
            # 2. 下降
            {'type': 'lift_down', 'params': {}},
            
            # 3. 夾持 (智慧夾持)
            {'type': 'pge_smart_grip', 'params': {'position': self.GRIPPER_POSITIONS['GRIP']}},
            
            # 4. 上升
            {'type': 'lift_up', 'params': {}},
            
            # 5. 翻轉180度
            {'type': 'flip_180', 'params': {}},
            
            # 6. 下降
            {'type': 'lift_down', 'params': {}},
            
            # 7. 開爪 (快速開爪)
            {'type': 'pge_quick_release', 'params': {'position': self.GRIPPER_POSITIONS['RELEASE']}},
            
            # 8. 上升
            {'type': 'lift_up', 'params': {}},
            
            # 9. 啟動輸送帶並等待感測器
            {'type': 'conveyor_run_until_sensor', 'params': {'sensor_pin': self.DIO_PINS['SENSOR_DI13']}}
        ]
        
        self.total_steps = len(self.dio_steps)
        
    def execute(self) -> FlowResult:
        """執行Flow3翻轉站流程"""
        print("\n" + "="*60)
        print("開始執行Flow3 - 翻轉站控制流程")
        print("="*60)
        
        start_time = time.time()
        self.status = FlowStatus.RUNNING
        self.current_step = 0
        
        try:
            for i, step in enumerate(self.dio_steps):
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
                time.sleep(0.2)
                
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - start_time
            
            print(f"\n✓ Flow3翻轉站控制完成！總耗時: {execution_time:.2f}秒")
            
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
            if step_type == 'lift_home':
                return self._lift_home()
            elif step_type == 'lift_down':
                return self._lift_down()
            elif step_type == 'lift_up':
                return self._lift_up()
            elif step_type == 'pge_smart_grip':
                return self._pge_smart_grip(params)
            elif step_type == 'pge_quick_release':
                return self._pge_quick_release(params)
            elif step_type == 'flip_180':
                return self._flip_180()
            elif step_type == 'conveyor_run_until_sensor':
                return self._conveyor_run_until_sensor(params)
            else:
                print(f"未知步驟類型: {step_type}")
                return False
                
        except Exception as e:
            print(f"步驟執行錯誤: {e}")
            return False
            
    def _set_do(self, pin: int, value: int) -> bool:
        """設定數位輸出"""
        try:
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.DO(pin, value)
                print(f"    DO({pin}, {value}) -> {result}")
                return "0" in str(result)
            else:
                print(f"    模擬 DO({pin}, {value})")
                return True
        except Exception as e:
            print(f"    ✗ DO({pin}, {value}) 失敗: {e}")
            return False
            
    def _get_di(self, pin: int) -> int:
        """讀取數位輸入"""
        try:
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.DI(pin)
                print(f"    DI({pin}) -> {result}")
                
                # 解析DI回應格式 "0,pin,value,"
                if "," in result:
                    parts = result.split(",")
                    if len(parts) >= 3:
                        return int(parts[2])
                return 0
            else:
                # 模擬感測器讀取
                print(f"    模擬 DI({pin}) -> 1")
                return 1
        except Exception as e:
            print(f"    ✗ DI({pin}) 失敗: {e}")
            return 0
            
    def _pulse_do(self, pin: int, pulse_width_ms: int = 100) -> bool:
        """脈衝輸出"""
        try:
            print(f"  脈衝輸出 DO{pin} ({pulse_width_ms}ms)")
            
            # 設定為高
            if not self._set_do(pin, 1):
                return False
                
            # 等待
            time.sleep(pulse_width_ms / 1000.0)
            
            # 設定為低
            if not self._set_do(pin, 0):
                return False
                
            print(f"  ✓ 脈衝輸出完成")
            return True
            
        except Exception as e:
            print(f"  ✗ 脈衝輸出失敗: {e}")
            return False
            
    def _lift_home(self) -> bool:
        """升降缸回原點"""
        try:
            print("  升降缸回原點")
            
            # 對DO14下一個脈衝 (賦歸訊號)
            success = self._pulse_do(self.DIO_PINS['LIFT_HOME'], 200)
            
            if success:
                print("  ✓ 升降缸回原點指令已發送")
                # 等待回原點完成 (1秒)
                time.sleep(self.TIMING_CONFIG['LIFT_HOME_TIME'])
                print("  ✓ 升降缸回原點完成")
            
            return success
            
        except Exception as e:
            print(f"  ✗ 升降缸回原點失敗: {e}")
            return False
            
    def _lift_down(self) -> bool:
        """升降缸下降"""
        try:
            print("  升降缸下降")
            
            # 設定方向: DO12=LOW, DO13=HIGH (01)
            if not self._set_do(self.DIO_PINS['LIFT_DIR1'], 0):  # DO12=LOW
                return False
            if not self._set_do(self.DIO_PINS['LIFT_DIR2'], 1):  # DO13=HIGH
                return False
                
            time.sleep(self.TIMING_CONFIG['DIRECTION_SETUP_DELAY'])  # 方向設定延遲
            
            # 對DO11下脈衝啟動
            success = self._pulse_do(self.DIO_PINS['LIFT_TRIGGER'], self.TIMING_CONFIG['PULSE_WIDTH'])
            
            if success:
                print("  ✓ 升降缸下降指令已發送")
                # 等待下降完成 (1秒)
                time.sleep(self.TIMING_CONFIG['LIFT_MOTION_TIME'])
                print("  ✓ 升降缸下降完成")
            
            return success
            
        except Exception as e:
            print(f"  ✗ 升降缸下降失敗: {e}")
            return False
            
    def _lift_up(self) -> bool:
        """升降缸上升"""
        try:
            print("  升降缸上升")
            
            # 設定方向: DO12=HIGH, DO13=LOW (10)
            if not self._set_do(self.DIO_PINS['LIFT_DIR1'], 1):  # DO12=HIGH
                return False
            if not self._set_do(self.DIO_PINS['LIFT_DIR2'], 0):  # DO13=LOW
                return False
                
            time.sleep(self.TIMING_CONFIG['DIRECTION_SETUP_DELAY'])  # 方向設定延遲
            
            # 對DO11下脈衝啟動
            success = self._pulse_do(self.DIO_PINS['LIFT_TRIGGER'], self.TIMING_CONFIG['PULSE_WIDTH'])
            
            if success:
                print("  ✓ 升降缸上升指令已發送")
                # 等待上升完成 (1秒)
                time.sleep(self.TIMING_CONFIG['LIFT_MOTION_TIME'])
                print("  ✓ 升降缸上升完成")
            
            return success
            
        except Exception as e:
            print(f"  ✗ 升降缸上升失敗: {e}")
            return False
            
    def _pge_smart_grip(self, params: Dict) -> bool:
        """PGE夾爪智慧夾持"""
        try:
            position = params['position']
            print(f"  PGE夾爪智慧夾持到位置: {position}")
            
            # 使用外部模組 (PGE夾爪)
            if 'PGE_GRIPPER' in self.external_modules and self.external_modules['PGE_GRIPPER']:
                pge_gripper = self.external_modules['PGE_GRIPPER']
                success = pge_gripper.smart_grip(target_position=position)
                
                if success:
                    print(f"  ✓ PGE智慧夾持到{position}完成")
                else:
                    print(f"  ✗ PGE智慧夾持失敗")
                return success
            else:
                # 模擬PGE智慧夾持
                print(f"  ✓ PGE智慧夾持到{position}完成 (模擬)")
                time.sleep(self.TIMING_CONFIG['GRIPPER_TIME'])  # 模擬夾持時間
                return True
                
        except Exception as e:
            print(f"  ✗ PGE智慧夾持失敗: {e}")
            return False
            
    def _pge_quick_release(self, params: Dict) -> bool:
        """PGE夾爪快速開爪"""
        try:
            position = params['position']
            print(f"  PGE夾爪快速開爪到位置: {position}")
            
            # 使用外部模組 (PGE夾爪)
            if 'PGE_GRIPPER' in self.external_modules and self.external_modules['PGE_GRIPPER']:
                pge_gripper = self.external_modules['PGE_GRIPPER']
                success = pge_gripper.quick_open(position)
                
                if success:
                    print(f"  ✓ PGE快速開爪到{position}完成")
                else:
                    print(f"  ✗ PGE快速開爪失敗")
                return success
            else:
                # 模擬PGE快速開爪
                print(f"  ✓ PGE快速開爪到{position}完成 (模擬)")
                time.sleep(self.TIMING_CONFIG['GRIPPER_TIME'] * 0.5)  # 快速開爪時間較短
                return True
                
        except Exception as e:
            print(f"  ✗ PGE快速開爪失敗: {e}")
            return False
            
    def _flip_180(self) -> bool:
        """翻轉缸180度翻轉"""
        try:
            print("  翻轉缸180度翻轉")
            
            # DO5=HIGH -> 180度
            if not self._set_do(self.DIO_PINS['FLIP_CYLINDER'], 1):
                return False
                
            print("  ✓ 翻轉指令已發送")
            
            # 等待翻轉完成
            time.sleep(self.TIMING_CONFIG['FLIP_TIME'])
            print("  ✓ 翻轉180度完成")
            
            return True
            
        except Exception as e:
            print(f"  ✗ 翻轉失敗: {e}")
            return False
            
    def _conveyor_run_until_sensor(self, params: Dict) -> bool:
        """啟動輸送帶直到感測器觸發"""
        try:
            sensor_pin = params['sensor_pin']
            print(f"  啟動輸送帶，等待感測器DI{sensor_pin}觸發")
            
            # 啟動輸送帶 DO2=HIGH
            if not self._set_do(self.DIO_PINS['CONVEYOR'], 1):
                return False
                
            print("  ✓ 輸送帶已啟動")
            
            # 等待感測器觸發 (最多等待30秒)
            timeout = 30.0
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                sensor_value = self._get_di(sensor_pin)
                
                if sensor_value == 1:
                    print(f"  ✓ 感測器DI{sensor_pin}已觸發")
                    break
                    
                time.sleep(0.1)  # 100ms檢查間隔
            else:
                print(f"  ⚠️ 感測器等待超時 ({timeout}秒)")
                
            # 停止輸送帶 DO2=LOW
            if not self._set_do(self.DIO_PINS['CONVEYOR'], 0):
                print("  ⚠️ 停止輸送帶失敗")
            else:
                print("  ✓ 輸送帶已停止")
                
            return True
            
        except Exception as e:
            print(f"  ✗ 輸送帶控制失敗: {e}")
            return False
            
    def pause(self) -> bool:
        """暫停Flow"""
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            print("Flow3翻轉站控制已暫停")
            return True
        return False
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow3翻轉站控制已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow (緊急停止)"""
        self.status = FlowStatus.ERROR
        
        # 緊急停止：關閉所有輸出
        try:
            self._set_do(self.DIO_PINS['CONVEYOR'], 0)      # 停止輸送帶
            self._set_do(self.DIO_PINS['FLIP_CYLINDER'], 0) # 翻轉缸回0度
            print("Flow3翻轉站控制已緊急停止")
        except:
            pass
            
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)


# ==================== 使用範例 ====================

def example_usage():
    """Flow3翻轉站控制使用範例"""
    print("=== Flow3 翻轉站控制流程範例 ===")
    
    # 模擬外部依賴
    class MockRobot:
        def __init__(self):
            self.is_connected = True
            self.dashboard_api = MockDashboardAPI()
            
    class MockDashboardAPI:
        def __init__(self):
            self.di_values = {13: 0}  # 模擬DI13初始為0
            
        def DO(self, pin, value):
            print(f"      [模擬] DO{pin} = {value}")
            return f"0,DO{pin}={value},"
            
        def DI(self, pin):
            # 模擬感測器在5秒後觸發
            if pin == 13:
                import random
                value = random.choice([0, 1])  # 隨機模擬觸發
                print(f"      [模擬] DI{pin} = {value}")
                return f"0,{pin},{value},"
            return f"0,{pin},0,"
    
    class MockStateMachine:
        def read_register(self, offset):
            return 0
    
    # 創建Flow執行器
    flow3 = FlowFlipStationExecutor()
    
    # 初始化依賴
    mock_robot = MockRobot()
    mock_state_machine = MockStateMachine()
    mock_external_modules = {
        'PGE_GRIPPER': None  # 暫時未實現PGE夾爪模組
    }
    
    flow3.initialize(mock_robot, mock_state_machine, mock_external_modules)
    
    # 執行Flow
    result = flow3.execute()
    
    # 輸出結果
    if result.success:
        print(f"\n✓ Flow3翻轉站控制執行成功！")
        print(f"  耗時: {result.execution_time:.2f}秒")
        print(f"  完成步驟: {result.steps_completed}/{result.total_steps}")
    else:
        print(f"\n✗ Flow3翻轉站控制執行失敗: {result.error_message}")
        print(f"  完成步驟: {result.steps_completed}/{result.total_steps}")


if __name__ == "__main__":
    example_usage()