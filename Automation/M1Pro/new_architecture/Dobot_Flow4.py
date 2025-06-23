#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow4_vibration.py - Flow4 震動投料流程 (DIO控制架構版)
基於統一Flow架構的DIO控制執行器
控制震動投料：DO4 HIGH 2秒，同時DO1進行HIGH-LOW脈衝操作2次
"""

import time
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow4VibrationFeedExecutor(FlowExecutor):
    """Flow4: 震動投料流程執行器 (DIO控制)"""
    
    def __init__(self):
        super().__init__(flow_id=4, flow_name="震動投料流程")
        self.dio_steps = []
        
        # DIO腳位定義
        self.DIO_PINS = {
            'VIBRATION_CONTROL': 1,    # DO1: 震動控制 (HIGH-LOW脈衝)
            'FEED_ENABLE': 4,          # DO4: 投料使能 (持續HIGH 2秒)
        }
        
        # 時間延遲設定
        self.TIMING_CONFIG = {
            'FEED_DURATION': 2.0,      # DO4持續時間 (秒)
            'PULSE_HIGH_TIME': 0.5,    # DO1 HIGH持續時間 (秒)
            'PULSE_LOW_TIME': 0.5,     # DO1 LOW持續時間 (秒)
            'PULSE_COUNT': 2           # DO1脈衝次數
        }
        
        # 建構流程步驟
        self.build_flow_steps()
        
    def build_flow_steps(self):
        """建構Flow4步驟"""
        self.dio_steps = [
            # 1. 同時啟動投料使能和震動脈衝
            {'type': 'start_vibration_feed', 'params': {}},
            
            # 2. 等待流程完成
            {'type': 'wait_completion', 'params': {'duration': self.TIMING_CONFIG['FEED_DURATION']}},
            
            # 3. 確保所有輸出關閉
            {'type': 'stop_all_outputs', 'params': {}}
        ]
        
        self.total_steps = len(self.dio_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow4主邏輯"""
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
                
                print(f"Flow4 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'start_vibration_feed':
                    success = self._execute_start_vibration_feed()
                elif step['type'] == 'wait_completion':
                    success = self._execute_wait_completion(step['params'])
                elif step['type'] == 'stop_all_outputs':
                    success = self._execute_stop_all_outputs()
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
                flow_data={'vibration_feed_completed': True}
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow4執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_start_vibration_feed(self) -> bool:
        """執行震動投料啟動 - 並行控制DO1和DO4"""
        try:
            print("開始震動投料流程")
            print(f"DO4將持續HIGH {self.TIMING_CONFIG['FEED_DURATION']}秒")
            print(f"DO1將執行{self.TIMING_CONFIG['PULSE_COUNT']}次脈衝操作")
            
            # 啟動DO4 (投料使能)
            if not self.robot.set_do(self.DIO_PINS['FEED_ENABLE'], 1):
                print("DO4啟動失敗")
                return False
            
            print("✓ DO4已啟動")
            
            # 創建執行緒執行DO1脈衝操作
            pulse_thread = threading.Thread(target=self._execute_do1_pulses, daemon=True)
            pulse_thread.start()
            
            print("✓ DO1脈衝執行緒已啟動")
            return True
            
        except Exception as e:
            print(f"震動投料啟動失敗: {e}")
            return False
    
    def _execute_do1_pulses(self):
        """執行DO1脈衝操作 - 在獨立執行緒中運行"""
        try:
            print("DO1脈衝操作開始")
            
            for pulse_num in range(self.TIMING_CONFIG['PULSE_COUNT']):
                print(f"DO1脈衝 {pulse_num + 1}/{self.TIMING_CONFIG['PULSE_COUNT']}")
                
                # DO1 HIGH
                if not self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 1):
                    print(f"DO1脈衝{pulse_num + 1} HIGH失敗")
                    continue
                
                print(f"  DO1 HIGH (持續{self.TIMING_CONFIG['PULSE_HIGH_TIME']}秒)")
                time.sleep(self.TIMING_CONFIG['PULSE_HIGH_TIME'])
                
                # DO1 LOW
                if not self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 0):
                    print(f"DO1脈衝{pulse_num + 1} LOW失敗")
                    continue
                
                print(f"  DO1 LOW (持續{self.TIMING_CONFIG['PULSE_LOW_TIME']}秒)")
                time.sleep(self.TIMING_CONFIG['PULSE_LOW_TIME'])
                
                print(f"✓ DO1脈衝{pulse_num + 1}完成")
            
            print("✓ DO1所有脈衝操作完成")
            
        except Exception as e:
            print(f"DO1脈衝操作失敗: {e}")
    
    def _execute_wait_completion(self, params: Dict[str, Any]) -> bool:
        """等待流程完成"""
        try:
            duration = params.get('duration', 2.0)
            print(f"等待流程完成 ({duration}秒)")
            
            time.sleep(duration)
            
            print("✓ 等待完成")
            return True
            
        except Exception as e:
            print(f"等待流程失敗: {e}")
            return False
    
    def _execute_stop_all_outputs(self) -> bool:
        """停止所有輸出"""
        try:
            print("關閉所有DIO輸出")
            
            # 關閉DO1
            if not self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 0):
                print("DO1關閉失敗")
                return False
            
            # 關閉DO4
            if not self.robot.set_do(self.DIO_PINS['FEED_ENABLE'], 0):
                print("DO4關閉失敗")
                return False
            
            print("✓ 所有DIO輸出已關閉")
            return True
            
        except Exception as e:
            print(f"關閉DIO輸出失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow4已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow4已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow4已停止")
        
        # 緊急停止所有設備
        try:
            self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 0)
            self.robot.set_do(self.DIO_PINS['FEED_ENABLE'], 0)
            print("已緊急停止所有DIO設備")
        except Exception as e:
            print(f"緊急停止DIO設備失敗: {e}")
        
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)