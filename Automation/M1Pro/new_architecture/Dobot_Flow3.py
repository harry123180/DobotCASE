#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow3_flip.py - Flow3 翻轉站控制流程 (DIO控制架構版 - 移除模擬代碼修正版)
基於統一Flow架構的DIO控制執行器
控制翻轉站的升降缸、夾爪、翻轉缸、輸送帶
禁止任何模擬代碼，全部使用真實API連接
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
        
        # DIO腳位定義 (基於真實硬體配置)
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
        
        # PGE夾爪位置定義 - 修正屬性名稱
        self.GRIPPER_POSITIONS = {
            'GRIP': 500,            # 夾住位置
            'RELEASE': 1000         # 放開位置
        }
        
        # 建構流程步驟
        self.build_flow_steps()
        
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
            
            # 9. 翻轉回0度
            {'type': 'flip_0', 'params': {}},
            
            # 10. 啟動輸送帶
            {'type': 'conveyor_start', 'params': {}},
            
            # 11. 等待感測器觸發
            {'type': 'wait_sensor', 'params': {'timeout': 10.0}},
            
            # 12. 停止輸送帶
            {'type': 'conveyor_stop', 'params': {}}
        ]
        
        self.total_steps = len(self.dio_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow3主邏輯"""
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
            for step in self.dio_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow3 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'lift_home':
                    success = self._execute_lift_home()
                elif step['type'] == 'lift_down':
                    success = self._execute_lift_down()
                elif step['type'] == 'lift_up':
                    success = self._execute_lift_up()
                elif step['type'] == 'pge_smart_grip':
                    success = self._execute_pge_smart_grip(step['params'])
                elif step['type'] == 'pge_quick_release':
                    success = self._execute_pge_quick_release(step['params'])
                elif step['type'] == 'flip_180':
                    success = self._execute_flip_180()
                elif step['type'] == 'flip_0':
                    success = self._execute_flip_0()
                elif step['type'] == 'conveyor_start':
                    success = self._execute_conveyor_start()
                elif step['type'] == 'conveyor_stop':
                    success = self._execute_conveyor_stop()
                elif step['type'] == 'wait_sensor':
                    success = self._execute_wait_sensor(step['params'])
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
                flow_data={'flip_station_completed': True}
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow3執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_lift_home(self) -> bool:
        """執行升降缸回原點"""
        try:
            print("升降缸回原點")
            
            # 觸發回原點脈衝
            if not self.robot.set_do(self.DIO_PINS['LIFT_HOME'], 1):
                return False
            
            # 脈衝寬度
            time.sleep(self.TIMING_CONFIG['PULSE_WIDTH'] / 1000.0)
            
            if not self.robot.set_do(self.DIO_PINS['LIFT_HOME'], 0):
                return False
            
            # 等待回原點完成
            time.sleep(self.TIMING_CONFIG['LIFT_HOME_TIME'])
            
            print("✓ 升降缸回原點完成")
            return True
            
        except Exception as e:
            print(f"升降缸回原點失敗: {e}")
            return False
    
    def _execute_lift_down(self) -> bool:
        """執行升降缸下降"""
        try:
            print("升降缸下降")
            
            # 設定方向 (下降)
            if not self.robot.set_do(self.DIO_PINS['LIFT_DIR1'], 0):
                return False
            if not self.robot.set_do(self.DIO_PINS['LIFT_DIR2'], 1):
                return False
            
            time.sleep(self.TIMING_CONFIG['DIRECTION_SETUP_DELAY'])
            
            # 觸發下降脈衝
            if not self.robot.set_do(self.DIO_PINS['LIFT_TRIGGER'], 1):
                return False
            
            time.sleep(self.TIMING_CONFIG['PULSE_WIDTH'] / 1000.0)
            
            if not self.robot.set_do(self.DIO_PINS['LIFT_TRIGGER'], 0):
                return False
            
            # 等待下降完成
            time.sleep(self.TIMING_CONFIG['LIFT_MOTION_TIME'])
            
            print("✓ 升降缸下降完成")
            return True
            
        except Exception as e:
            print(f"升降缸下降失敗: {e}")
            return False
    
    def _execute_lift_up(self) -> bool:
        """執行升降缸上升"""
        try:
            print("升降缸上升")
            
            # 設定方向 (上升)
            if not self.robot.set_do(self.DIO_PINS['LIFT_DIR1'], 1):
                return False
            if not self.robot.set_do(self.DIO_PINS['LIFT_DIR2'], 0):
                return False
            
            time.sleep(self.TIMING_CONFIG['DIRECTION_SETUP_DELAY'])
            
            # 觸發上升脈衝
            if not self.robot.set_do(self.DIO_PINS['LIFT_TRIGGER'], 1):
                return False
            
            time.sleep(self.TIMING_CONFIG['PULSE_WIDTH'] / 1000.0)
            
            if not self.robot.set_do(self.DIO_PINS['LIFT_TRIGGER'], 0):
                return False
            
            # 等待上升完成
            time.sleep(self.TIMING_CONFIG['LIFT_MOTION_TIME'])
            
            print("✓ 升降缸上升完成")
            return True
            
        except Exception as e:
            print(f"升降缸上升失敗: {e}")
            return False
    
    def _execute_pge_smart_grip(self, params: Dict[str, Any]) -> bool:
        """執行PGE夾爪智慧夾持"""
        try:
            position = params.get('position', self.GRIPPER_POSITIONS['GRIP'])
            print(f"PGE夾爪智慧夾持到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                return gripper_api.smart_grip(position)
            else:
                print("夾爪API未初始化")
                return False
                
        except Exception as e:
            print(f"PGE夾爪智慧夾持失敗: {e}")
            return False
    
    def _execute_pge_quick_release(self, params: Dict[str, Any]) -> bool:
        """執行PGE夾爪快速開爪"""
        try:
            position = params.get('position', self.GRIPPER_POSITIONS['RELEASE'])
            print(f"PGE夾爪快速開爪到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                return gripper_api.quick_open(position)
            else:
                print("夾爪API未初始化")
                return False
                
        except Exception as e:
            print(f"PGE夾爪快速開爪失敗: {e}")
            return False
    
    def _execute_flip_180(self) -> bool:
        """執行翻轉缸翻轉到180度"""
        try:
            print("翻轉缸翻轉到180度")
            
            # 設定翻轉缸到180度位置
            if not self.robot.set_do(self.DIO_PINS['FLIP_CYLINDER'], 1):
                return False
            
            # 等待翻轉完成
            time.sleep(self.TIMING_CONFIG['FLIP_TIME'])
            
            print("✓ 翻轉到180度完成")
            return True
            
        except Exception as e:
            print(f"翻轉到180度失敗: {e}")
            return False
    
    def _execute_flip_0(self) -> bool:
        """執行翻轉缸回到0度"""
        try:
            print("翻轉缸回到0度")
            
            # 設定翻轉缸到0度位置
            if not self.robot.set_do(self.DIO_PINS['FLIP_CYLINDER'], 0):
                return False
            
            # 等待翻轉完成
            time.sleep(self.TIMING_CONFIG['FLIP_TIME'])
            
            print("✓ 翻轉回0度完成")
            return True
            
        except Exception as e:
            print(f"翻轉回0度失敗: {e}")
            return False
    
    def _execute_conveyor_start(self) -> bool:
        """執行輸送帶啟動"""
        try:
            print("啟動輸送帶")
            
            if not self.robot.set_do(self.DIO_PINS['CONVEYOR'], 1):
                return False
            
            print("✓ 輸送帶啟動成功")
            return True
            
        except Exception as e:
            print(f"輸送帶啟動失敗: {e}")
            return False
    
    def _execute_conveyor_stop(self) -> bool:
        """執行輸送帶停止"""
        try:
            print("停止輸送帶")
            
            if not self.robot.set_do(self.DIO_PINS['CONVEYOR'], 0):
                return False
            
            print("✓ 輸送帶停止成功")
            return True
            
        except Exception as e:
            print(f"輸送帶停止失敗: {e}")
            return False
    
    def _execute_wait_sensor(self, params: Dict[str, Any]) -> bool:
        """等待感測器觸發"""
        try:
            timeout = params.get('timeout', 10.0)
            print(f"等待感測器觸發 (超時: {timeout}秒)")
            
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # 讀取感測器狀態
                sensor_value = self.robot.get_di(self.DIO_PINS['SENSOR_DI13'])
                
                if sensor_value == 1:  # 感測器觸發
                    print("✓ 感測器觸發")
                    return True
                
                time.sleep(0.1)  # 100ms檢查間隔
            
            print("感測器等待超時")
            return False
            
        except Exception as e:
            print(f"等待感測器失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow3已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow3已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow3已停止")
        
        # 緊急停止所有設備
        try:
            self.robot.set_do(self.DIO_PINS['CONVEYOR'], 0)
            self.robot.set_do(self.DIO_PINS['FLIP_CYLINDER'], 0)
            print("已緊急停止所有DIO設備")
        except Exception as e:
            print(f"緊急停止DIO設備失敗: {e}")
        
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)