#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow5.py - Flow5 機械臂運轉流程執行器  
基於Flow3組裝作業流程，整合角度檢測與第四軸旋轉控制
參考Flow1/Flow2點位載入方式，禁止使用內建點位
修改版：優化角度控制邏輯 + 添加waitkey功能
"""

import time
import os
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入修改版AngleHighLevel
from AngleHighLevel import AngleHighLevel, AngleOperationResult


class FlowStatus(Enum):
    """Flow執行狀態"""
    IDLE = 0
    RUNNING = 1
    COMPLETED = 2
    ERROR = 3
    PAUSED = 4


@dataclass
class FlowResult:
    """Flow執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 14


class Flow5AssemblyExecutor:
    """Flow5: 機械臂運轉流程執行器 - 整合角度檢測與第四軸控制"""
    
    # 硬編碼第四軸原始角度
    J4_ORIGINAL_DEGREE = 176.96
    
    def __init__(self):
        self.flow_id = 5
        self.flow_name = "機械臂運轉流程"
        self.status = FlowStatus.IDLE
        self.current_step = 0
        self.total_steps = 14  # 更新總步驟數
        self.start_time = 0.0
        self.last_error = ""
        
        # 共用資源 (由Main傳入)
        self.robot = None
        self.gripper = None
        self.state_machine = None
        self.external_modules = {}
        
        # 點位管理
        self.loaded_points = {}
        self.points_file_path = ""
        
        # 角度檢測相關
        self.angle_detector = None
        self.target_angle = None
        self.command_angle = None
        
        # 流程步驟
        self.motion_steps = []
        self.build_flow_steps()
        
        # 必要點位列表 (按新流程順序)
        self.REQUIRED_POINTS = [
            "standby",             # 待機位置 (起點)
            "rotate_top",          # 旋轉頂部點
            "rotate_down",         # 旋轉下方點
            "put_asm_pre",         # 組裝預備位置
            "put_asm_top",         # 組裝頂部位置
            "put_asm_down"         # 組裝放下位置
        ]
        
    def initialize(self, robot, state_machine, external_modules):
        """初始化Flow5 (由Main呼叫)"""
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
        # 初始化夾爪控制器
        self.gripper = external_modules.get('gripper')
        
        # 初始化角度檢測器
        self.angle_detector = AngleHighLevel()
        
        # 載入外部點位檔案
        if not self._load_external_points():
            raise RuntimeError("載入外部點位檔案失敗，Flow5無法初始化")
            
        print("✓ Flow5執行器初始化完成 - 機械臂運轉流程 (含角度檢測)")
        print(f"✓ 第四軸原始角度: {self.J4_ORIGINAL_DEGREE}度")
        
    def _load_external_points(self) -> bool:
        """載入外部點位檔案 - 修正陣列格式JSON"""
        try:
            print("Flow5正在載入外部點位檔案...")
            
            # 取得當前執行檔案的目錄
            current_dir = os.path.dirname(os.path.abspath(__file__))
            points_dir = os.path.join(current_dir, "saved_points")
            self.points_file_path = os.path.join(points_dir, "robot_points.json")
            
            print(f"嘗試載入點位檔案: {self.points_file_path}")
            
            # 檢查檔案是否存在
            if not os.path.exists(self.points_file_path):
                self.last_error = f"點位檔案不存在: {self.points_file_path}"
                print(f"✗ {self.last_error}")
                return False
                
            # 讀取點位檔案
            with open(self.points_file_path, 'r', encoding='utf-8') as f:
                points_data = json.load(f)
                
            if not points_data:
                self.last_error = "點位檔案為空"
                print(f"✗ {self.last_error}")
                return False
            
            # 檢查JSON格式：陣列或物件
            if isinstance(points_data, list):
                # 陣列格式：轉換為name:data字典
                self.loaded_points = {}
                for point_item in points_data:
                    if isinstance(point_item, dict) and 'name' in point_item:
                        point_name = point_item['name']
                        self.loaded_points[point_name] = point_item
                    else:
                        print(f"跳過無效點位項目: {point_item}")
                        
            elif isinstance(points_data, dict):
                # 物件格式：直接使用
                self.loaded_points = points_data
            else:
                self.last_error = f"不支援的JSON格式: {type(points_data)}"
                print(f"✗ {self.last_error}")
                return False
                
            if not self.loaded_points:
                self.last_error = "沒有有效的點位數據"
                print(f"✗ {self.last_error}")
                return False
                
            # 顯示載入的點位
            point_names = list(self.loaded_points.keys())
            print(f"載入點位數據成功，共{len(point_names)}個點位: {point_names}")
            
            # 檢查必要點位是否存在
            missing_points = []
            for required_point in self.REQUIRED_POINTS:
                if required_point not in self.loaded_points:
                    missing_points.append(required_point)
                    
            if missing_points:
                self.last_error = f"缺少必要點位: {missing_points}"
                print(f"✗ {self.last_error}")
                return False
                
            print("✓ 所有必要點位載入成功")
            return True
            
        except Exception as e:
            self.last_error = f"載入點位檔案異常: {e}"
            print(f"✗ {self.last_error}")
            return False
    
    def build_flow_steps(self):
        """建構Flow5步驟 - 完整流程序列 (含waitkey) - 修改版角度控制"""
        self.motion_steps = [
            # 1. 移動到standby (起點)
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 2. 執行角度檢測
            {'type': 'angle_detection', 'params': {}},

            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            
            # 3. 移動到rotate_top (不帶角度，使用原始角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 4. 移動到rotate_down (不帶角度，使用原始角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            
            # 5. 夾爪撐開到470 (智慧撐開)
            {'type': 'gripper_smart_release', 'params': {'position': 480}},
            
            # 6. 移動到rotate_top (不帶角度，使用原始角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 7. 移動到put_asm_pre (不帶角度)
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_pre', 'move_type': 'J'}},
            
            # 8. 移動到put_asm_top (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            
            # 9. 等待終端輸入 - 在put_asm_top之後
            #{'type': 'waitkey', 'params': {'prompt': '請輸入 "go" 繼續到 put_asm_down 位置', 'expected_input': 'go'}},
            
            # 10. 設定機械臂速度
            {'type': 'set_speed', 'params': {'speed_percent': 20}},
            
            # 11. 移動到put_asm_down (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_down', 'move_type': 'J'}},
            #{'type': 'waittime', 'params': {'duration_ms': 1000}},
            #{'type': 'waitkey', 'params': {'prompt': '請輸入 "go" 繼續到 put_asm_down 位置', 'expected_input': 'go'}},
            
            # 12. 夾爪快速關閉
            {'type': 'gripper_quick_close', 'params': {}},
            
            # 13. 移動到put_asm_top (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            
            # 14. 移動到put_asm_pre (不帶角度)
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_pre', 'move_type': 'J'}},
            
            # 15. 移動到rotate_top (不帶角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'set_speed', 'params': {'speed_percent': 80}},
            # 16. 移動到standby (完成)
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow5流程步驟建構完成，共{self.total_steps}步")
        print("角度控制策略：rotate相關點位使用原始角度，put_asm_top/put_asm_down使用commandAngle")
        print("waitkey步驟：在put_asm_top之後等待終端輸入")
    
    def execute(self) -> FlowResult:
        """執行Flow5主邏輯"""
        print("\n" + "="*60)
        print("開始執行Flow5 - 機械臂運轉流程 (修改版角度控制 + waitkey + set_speed)")
        print("流程序列: standby->角度檢測->rotate_top->rotate_down->夾爪撐開->rotate_top->put_asm_pre->put_asm_top(角度)->【waitkey】->【set_speed】->put_asm_down(角度)->夾爪關閉->put_asm_top(角度)->put_asm_pre->rotate_top->standby")
        print(f"第四軸原始角度: {self.J4_ORIGINAL_DEGREE}度")
        print("="*60)
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        self.last_error = ""
        
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
                elif step['type'] == 'move_to_point_with_angle':
                    success = self._execute_move_to_point_with_angle(step['params'])
                elif step['type'] == 'angle_detection':
                    success = self._execute_angle_detection()
                elif step['type'] == 'gripper_quick_close':
                    success = self._execute_gripper_quick_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'waitkey':
                    success = self._execute_waitkey(step['params'])
                elif step['type'] == 'set_speed':
                    success = self._execute_set_speed(step['params'])
                elif step['type'] == 'waittime':
                    success = self._execute_waittime(step['params'])
                else:
                    print(f"未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=self.last_error,
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
                
                # 更新進度
                if self.state_machine:
                    try:
                        progress = int((self.current_step / self.total_steps) * 100)
                        self.state_machine.write_register(503, progress)  # Flow5進度寄存器
                    except Exception:
                        pass
            
            # 流程完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print(f"\n✓ Flow5執行完成！總耗時: {execution_time:.2f}秒")
            if self.target_angle is not None and self.command_angle is not None:
                print(f"✓ 檢測角度: {self.target_angle:.2f}度, 第四軸補償角度: {self.command_angle:.2f}度")
            print("="*60)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.last_error = f"Flow5執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=self.last_error,
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    def _execute_waittime(self, params: Dict[str, Any]) -> bool:
        """執行等待時間功能"""
        try:
            duration_ms = params.get('duration_ms', 500)
            
            # 檢查時間範圍 (1ms - 60000ms)
            if not 1 <= duration_ms <= 60000:
                self.last_error = f"等待時間超出範圍 (1-60000ms): {duration_ms}"
                print(f"  ✗ 等待時間失敗: {self.last_error}")
                return False
            
            duration_seconds = duration_ms / 1000.0
            
            print(f"等待時間: {duration_ms}ms ({duration_seconds:.3f}秒)")
            
            start_time = time.time()
            time.sleep(duration_seconds)
            actual_duration = time.time() - start_time
            
            print(f"  ✓ 等待完成，實際耗時: {actual_duration*1000:.1f}ms")
            return True
            
        except Exception as e:
            self.last_error = f"等待時間異常: {e}"
            print(f"  ✗ 等待時間異常: {self.last_error}")
            return False
    def _execute_set_speed(self, params: Dict[str, Any]) -> bool:
        """執行設定機械臂速度功能"""
        try:
            speed_percent = params.get('speed_percent', 100)
            
            # 檢查速度範圍
            if not 1 <= speed_percent <= 100:
                self.last_error = f"速度超出範圍 (1-100): {speed_percent}"
                print(f"  ✗ 設定速度失敗: {self.last_error}")
                return False
            
            print(f"設定機械臂全局速度: {speed_percent}%")
            
            # 檢查機械臂是否已初始化
            if not self.robot:
                self.last_error = "機械臂控制器未初始化"
                print(f"  ✗ 設定速度失敗: {self.last_error}")
                return False
            
            # 調用機械臂的設定速度方法
            success = self.robot.set_global_speed(speed_percent)
            
            if success:
                print(f"  ✓ 機械臂速度設定成功: {speed_percent}%")
                return True
            else:
                self.last_error = f"機械臂速度設定失敗: {speed_percent}%"
                print(f"  ✗ 設定速度失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"設定速度異常: {e}"
            print(f"  ✗ 設定速度異常: {self.last_error}")
            return False
    
    def _execute_waitkey(self, params: Dict[str, Any]) -> bool:
        """執行等待終端輸入功能"""
        try:
            prompt = params.get('prompt', '請輸入 "go" 繼續')
            expected_input = params.get('expected_input', 'go')
            timeout_seconds = params.get('timeout', None)  # None表示無限等待
            case_sensitive = params.get('case_sensitive', False)  # 預設不區分大小寫
            
            print(f"\n{'='*50}")
            print(f"🔶 Flow5 等待輸入")
            print(f"🔶 {prompt}")
            print(f"🔶 預期輸入: '{expected_input}'")
            if timeout_seconds:
                print(f"🔶 等待時間限制: {timeout_seconds}秒")
            else:
                print(f"🔶 等待時間: 無限制")
            print(f"{'='*50}")
            
            start_wait_time = time.time()
            
            while True:
                try:
                    # 顯示輸入提示
                    user_input = input(">>> ").strip()
                    
                    # 處理大小寫
                    if not case_sensitive:
                        user_input = user_input.lower()
                        expected_input = expected_input.lower()
                    
                    # 檢查輸入是否符合預期
                    if user_input == expected_input:
                        print(f"✓ 輸入正確，繼續執行Flow5...")
                        print(f"{'='*50}\n")
                        return True
                    else:
                        print(f"✗ 輸入不正確，預期: '{expected_input}', 實際: '{user_input}'")
                        print(f"請重新輸入...")
                        
                        # 檢查超時
                        if timeout_seconds:
                            elapsed = time.time() - start_wait_time
                            if elapsed >= timeout_seconds:
                                self.last_error = f"等待輸入超時 ({timeout_seconds}秒)"
                                print(f"✗ {self.last_error}")
                                return False
                        continue
                        
                except KeyboardInterrupt:
                    print(f"\n✗ 用戶中斷輸入")
                    self.last_error = "用戶中斷waitkey輸入"
                    return False
                except EOFError:
                    print(f"\n✗ 輸入結束")
                    self.last_error = "waitkey輸入結束"
                    return False
                    
        except Exception as e:
            self.last_error = f"waitkey執行異常: {e}"
            print(f"✗ waitkey執行異常: {self.last_error}")
            return False
    
    def _execute_angle_detection(self) -> bool:
        """執行角度檢測並計算commandAngle"""
        try:
            print("開始角度檢測...")
            
            # 檢查角度檢測器是否初始化
            if not self.angle_detector:
                self.last_error = "角度檢測器未初始化"
                print(f"  ✗ 角度檢測失敗: {self.last_error}")
                return False
            
            # 連接到角度檢測模組
            if not self.angle_detector.connect():
                self.last_error = "無法連接到角度檢測模組"
                print(f"  ✗ 角度檢測失敗: {self.last_error}")
                return False
            
            # 執行角度檢測 (使用CASE模式)
            detection_result = self.angle_detector.detect_angle(detection_mode=0)
            
            # 斷開連接
            self.angle_detector.disconnect()
            
            # 檢查檢測結果
            if detection_result.result != AngleOperationResult.SUCCESS:
                self.last_error = f"角度檢測失敗: {detection_result.message}"
                print(f"  ✗ 角度檢測失敗: {self.last_error}")
                return False
            
            # 獲取target_angle
            self.target_angle = detection_result.target_angle
            print(f"  ✓ 檢測到目標角度: {self.target_angle:.2f}度")
            self.command_angle = self.target_angle - 3.14
            
            return True
            
        except Exception as e:
            self.last_error = f"角度檢測異常: {e}"
            print(f"  ✗ 角度檢測異常: {self.last_error}")
            return False
    
    def _execute_move_to_point_with_angle(self, params: Dict[str, Any]) -> bool:
        """執行移動到指定點位並使用commandAngle作為第四軸角度 (僅用於put_asm_top/put_asm_down)"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            # 檢查commandAngle是否已計算
            if self.command_angle is None:
                self.last_error = "第四軸補償角度未計算，請先執行角度檢測"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 檢查點位是否存在
            if point_name not in self.loaded_points:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 取得點位數據
            point_item = self.loaded_points[point_name]
            
            # 根據JSON格式提取座標數據
            if 'cartesian' in point_item:
                cartesian_data = point_item['cartesian']
            else:
                self.last_error = f"點位{point_name}缺少cartesian數據"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 根據JSON格式提取關節數據
            if 'joint' in point_item:
                joint_data = point_item['joint']
            else:
                self.last_error = f"點位{point_name}缺少joint數據"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            print(f"移動到點位 {point_name} (使用第四軸補償角度)")
            print(f"  原始關節角度: (j1:{joint_data['j1']:.1f}, j2:{joint_data['j2']:.1f}, j3:{joint_data['j3']:.1f}, j4:{joint_data['j4']:.1f})")
            print(f"  補償關節角度: (j1:{joint_data['j1']:.1f}, j2:{joint_data['j2']:.1f}, j3:{joint_data['j3']:.1f}, j4:{self.command_angle:.1f})")
            print(f"  笛卡爾座標: ({cartesian_data['x']:.2f}, {cartesian_data['y']:.2f}, {cartesian_data['z']:.2f}, {cartesian_data['r']:.2f})")
            
            # 執行移動 - 使用補償後的第四軸角度
            if move_type == 'J':
                # 使用關節角度運動，第四軸使用commandAngle
                success = self.robot.joint_move_j(
                    joint_data['j1'], 
                    joint_data['j2'], 
                    joint_data['j3'], 
                    self.command_angle  # 使用計算出的補償角度
                )
            elif move_type == 'L':
                # 直線運動，第四軸使用commandAngle (需要更新笛卡爾座標的r值)
                success = self.robot.move_l(
                    cartesian_data['x'], 
                    cartesian_data['y'], 
                    cartesian_data['z'], 
                    self.command_angle  # 使用計算出的補償角度
                )
            else:
                self.last_error = f"未知移動類型: {move_type}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            if success:
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) - 第四軸: {self.command_angle:.1f}度")
                return True
            else:
                self.last_error = f"移動到 {point_name} 失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"移動操作異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到指定點位 - 使用原始記錄的角度"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            # 檢查點位是否存在
            if point_name not in self.loaded_points:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 取得點位數據
            point_item = self.loaded_points[point_name]
            
            # 根據JSON格式提取座標數據
            if 'cartesian' in point_item:
                cartesian_data = point_item['cartesian']
            else:
                self.last_error = f"點位{point_name}缺少cartesian數據"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 根據JSON格式提取關節數據
            if 'joint' in point_item:
                joint_data = point_item['joint']
            else:
                self.last_error = f"點位{point_name}缺少joint數據"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            print(f"移動到點位 {point_name} (使用原始記錄角度)")
            print(f"  關節角度: (j1:{joint_data['j1']:.1f}, j2:{joint_data['j2']:.1f}, j3:{joint_data['j3']:.1f}, j4:{joint_data['j4']:.1f})")
            print(f"  笛卡爾座標: ({cartesian_data['x']:.2f}, {cartesian_data['y']:.2f}, {cartesian_data['z']:.2f}, {cartesian_data['r']:.2f})")
            
            # 執行移動 - 使用原始記錄的角度
            if move_type == 'J':
                # 使用關節角度運動 - 使用原始記錄的第四軸角度
                success = self.robot.joint_move_j(
                    joint_data['j1'], 
                    joint_data['j2'], 
                    joint_data['j3'], 
                    joint_data['j4']  # 使用原始記錄的角度
                )
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標 - 使用原始記錄的r值
                success = self.robot.move_l(
                    cartesian_data['x'], 
                    cartesian_data['y'], 
                    cartesian_data['z'], 
                    cartesian_data['r']  # 使用原始記錄的角度
                )
            else:
                self.last_error = f"未知移動類型: {move_type}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            if success:
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) - 使用原始角度")
                return True
            else:
                self.last_error = f"移動到 {point_name} 失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"移動操作異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False
    
    def _execute_gripper_quick_close(self) -> bool:
        """執行夾爪快速關閉"""
        try:
            if not self.gripper:
                self.last_error = "夾爪控制器未初始化"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
            
            print("夾爪快速關閉")
            result = self.gripper.quick_close()
            
            if result:
                print("  ✓ 夾爪快速關閉成功")
                
                # 等待夾爪關閉完成
                time.sleep(1.0)  # 等待1秒確保夾爪完全關閉
                
                # 檢查夾爪狀態
                if hasattr(self.gripper, 'get_current_position'):
                    try:
                        current_pos = self.gripper.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                return True
            else:
                self.last_error = "夾爪快速關閉失敗"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"夾爪操作異常: {e}"
            print(f"  ✗ 夾爪操作異常: {self.last_error}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智慧撐開"""
        try:
            if not self.gripper:
                self.last_error = "夾爪控制器未初始化"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
            
            position = params.get('position', 470)
            print(f"夾爪智能撐開到位置: {position}")
            
            # 執行智能撐開操作
            result = self.gripper.smart_release(position)
            
            if result:
                print(f"  ✓ 夾爪智能撐開指令發送成功")
                
                # 等待夾爪撐開操作完全完成
                print("  等待夾爪撐開動作完成...")
                time.sleep(1.5)  # 等待1.5秒確保夾爪完全撐開
                
                # 檢查夾爪位置確認撐開完成
                if hasattr(self.gripper, 'get_current_position'):
                    try:
                        current_pos = self.gripper.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                            if abs(current_pos - position) <= 20:  # 容差20
                                print(f"  ✓ 夾爪已撐開到目標位置 (誤差: {abs(current_pos - position)})")
                            else:
                                print(f"  ⚠️ 夾爪位置偏差較大 (目標: {position}, 實際: {current_pos})")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                print(f"  ✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                self.last_error = f"夾爪智能撐開至{position}失敗"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"夾爪操作異常: {e}"
            print(f"  ✗ 夾爪操作異常: {self.last_error}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        try:
            self.status = FlowStatus.PAUSED
            print("Flow5已暫停")
            return True
        except Exception as e:
            print(f"暫停Flow5失敗: {e}")
            return False
    
    def resume(self) -> bool:
        """恢復Flow"""
        try:
            if self.status == FlowStatus.PAUSED:
                self.status = FlowStatus.RUNNING
                print("Flow5已恢復")
                return True
            else:
                print("Flow5未處於暫停狀態")
                return False
        except Exception as e:
            print(f"恢復Flow5失敗: {e}")
            return False
    
    def stop(self) -> bool:
        """停止Flow"""
        try:
            self.status = FlowStatus.ERROR
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "Flow5已停止"
            print("Flow5已停止")
            return True
            
        except Exception as e:
            print(f"停止Flow5失敗: {e}")
            return False
    
    def get_progress(self) -> int:
        """取得執行進度 (0-100)"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def get_status_info(self) -> Dict[str, Any]:
        """取得狀態資訊"""
        return {
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'status': self.status.value,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': self.get_progress(),
            'last_error': self.last_error,
            'required_points': self.REQUIRED_POINTS,
            'points_loaded': len(self.loaded_points),
            'points_file_path': self.points_file_path,
            'j4_original_degree': self.J4_ORIGINAL_DEGREE,
            'target_angle': self.target_angle,
            'command_angle': self.command_angle,
            'angle_detection_enabled': True,
            'waitkey_enabled': True
        }