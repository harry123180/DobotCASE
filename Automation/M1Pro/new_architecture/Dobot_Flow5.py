#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow5.py - Flow5 機械臂運轉流程執行器 - 新流程版本
從put_asm_down開始執行，引用Flow1的commandAngle
不再執行角度檢測，直接使用Flow1計算的角度
"""

import time
import os
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum


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
    total_steps: int = 5  # 新流程步驟數


class Flow5AssemblyExecutor:
    """Flow5: 機械臂運轉流程執行器 - 新流程版本 (從put_asm_down開始)"""
    
    def __init__(self, enable_sync: bool = False):
        self.flow_id = 5
        self.flow_name = "機械臂運轉流程(新流程版)"
        self.status = FlowStatus.IDLE
        self.current_step = 0
        self.total_steps = 5  # 新流程步驟數
        self.start_time = 0.0
        self.last_error = ""
        self.enable_sync = enable_sync  # 是否啟用sync
        
        # 共用資源 (由Main傳入)
        self.robot = None
        self.gripper = None
        self.state_machine = None
        self.external_modules = {}
        self.flow1_executor = None  # Flow1執行器引用，用於獲取commandAngle
        
        # 點位管理
        self.loaded_points = {}
        self.points_file_path = ""
        
        # 流程步驟
        self.motion_steps = []
        self.build_flow_steps()
        
        # 必要點位列表 - 新流程版本
        self.REQUIRED_POINTS = [
            "put_asm_down",         # 組裝放下位置
            "put_asm_top",          # 組裝頂部位置  
            "put_asm_pre",          # 組裝預備位置
            "rotate_top",           # 旋轉頂部點
            "flip_pre",             # 翻轉預備位置
            "standby"               # 待機位置 (終點)
        ]
        
        print(f"✓ Flow5新流程版初始化完成 (sync={'啟用' if enable_sync else '停用'})")
        print("✓ 起點: put_asm_down，終點: standby")
        print("✓ 引用Flow1的commandAngle，不再執行角度檢測")
        
    def initialize(self, robot, state_machine, external_modules, flow1_executor=None):
        """初始化Flow5 (由Main呼叫)"""
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        self.flow1_executor = flow1_executor  # 新增：Flow1執行器引用
        
        # 初始化夾爪控制器
        self.gripper = external_modules.get('gripper')
        
        # 載入外部點位檔案
        if not self._load_external_points():
            raise RuntimeError("載入外部點位檔案失敗，Flow5無法初始化")
            
        print("✓ Flow5執行器初始化完成 - 新流程版本")
        print("✓ Flow1執行器引用已設置，可獲取commandAngle")
        
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
        """建構Flow5步驟 - 新流程版本 (從put_asm_down開始)"""
        self.motion_steps = [
            # 1. 移動到put_asm_down (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_down', 'move_type': 'J'}},
            
            # 2. 夾爪快速關閉
            {'type': 'gripper_quick_close', 'params': {}},
            
            # 3. 移動到put_asm_top (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            
            # 4. 返回序列
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            #{'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            #{'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            
            # 5. 移動到standby (完成)
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow5新流程步驟建構完成，共{self.total_steps}步")
        print("角度控制策略：put_asm_down/put_asm_top使用Flow1的commandAngle")
        print("起點: put_asm_down，終點: standby")
    
    def execute(self) -> FlowResult:
        """執行Flow5主邏輯 - 新流程版本"""
        print("\n" + "="*60)
        print("開始執行Flow5 - 機械臂運轉流程 (新流程版)")
        print("流程序列: put_asm_down(角度)->夾爪關閉->put_asm_top(角度)->返回序列->standby")
        print("角度來源: 引用Flow1的commandAngle")
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
        
        # 檢查Flow1執行器引用和commandAngle
        if not self.flow1_executor:
            return FlowResult(
                success=False,
                error_message="Flow1執行器引用未設置",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        if not self.flow1_executor.has_valid_command_angle():
            return FlowResult(
                success=False,
                error_message="Flow1的commandAngle無效，請先執行Flow1",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        command_angle = self.flow1_executor.get_command_angle()
        print(f"✓ 從Flow1獲取commandAngle: {command_angle:.2f}度")
        
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
                    success = self._execute_move_to_point_with_angle(step['params'], command_angle)
                elif step['type'] == 'gripper_quick_close':
                    success = self._execute_gripper_quick_close()
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
                
                # 統一更新進度到寄存器1202
                self._update_progress_to_1202()
            
            # 流程完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 最終進度設為100%
            self._update_progress_to_1202(100)
            
            print(f"\n✓ Flow5執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ 使用角度: {command_angle:.2f}度 (來自Flow1)")
            print("✓ 已返回standby位置")
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
    
    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """統一更新進度到寄存器1202"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            # 方法1：通過state_machine的set_progress方法 (推薦)
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                print(f"[Flow5] 進度已更新到1202: {progress}% (透過MotionStateMachine)")
                return
            
            # 方法2：直接寫入到1202寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    result = self.state_machine.modbus_client.write_register(1202, progress)
                    if hasattr(result, 'isError') and not result.isError():
                        print(f"[Flow5] 進度已更新到1202: {progress}% (直接寫入)")
                    else:
                        print(f"[Flow5] 進度更新失敗: {result}")
                except Exception as e:
                    print(f"[Flow5] 進度更新異常: {e}")
            else:
                print(f"[Flow5] 無法更新進度：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"[Flow5] 進度更新到1202失敗: {e}")
    
    def _execute_move_to_point_with_angle(self, params: Dict[str, Any], command_angle: float) -> bool:
        """執行移動到指定點位並使用Flow1的commandAngle作為第四軸角度"""
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
            
            print(f"移動到點位 {point_name} (使用Flow1的commandAngle)")
            print(f"  原始關節角度: (j1:{joint_data['j1']:.1f}, j2:{joint_data['j2']:.1f}, j3:{joint_data['j3']:.1f}, j4:{joint_data['j4']:.1f})")
            print(f"  補償關節角度: (j1:{joint_data['j1']:.1f}, j2:{joint_data['j2']:.1f}, j3:{joint_data['j3']:.1f}, j4:{command_angle:.1f})")
            print(f"  笛卡爾座標: ({cartesian_data['x']:.2f}, {cartesian_data['y']:.2f}, {cartesian_data['z']:.2f}, {cartesian_data['r']:.2f})")
            
            # 執行移動 - 使用Flow1的commandAngle
            if move_type == 'J':
                # 使用關節角度運動，第四軸使用Flow1的commandAngle
                success = self.robot.joint_move_j(
                    joint_data['j1'], 
                    joint_data['j2'], 
                    joint_data['j3'], 
                    command_angle  # 使用Flow1計算的角度
                )
            elif move_type == 'L':
                # 直線運動，第四軸使用Flow1的commandAngle
                success = self.robot.move_l(
                    cartesian_data['x'], 
                    cartesian_data['y'], 
                    cartesian_data['z'], 
                    command_angle  # 使用Flow1計算的角度
                )
            else:
                self.last_error = f"未知移動類型: {move_type}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 可選的sync控制
            if success and self.enable_sync:
                self.robot.sync()
            
            if success:
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) - 第四軸: {command_angle:.1f}度")
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
            
            # 可選的sync控制
            if success and self.enable_sync:
                self.robot.sync()
            
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
        flow1_command_angle = None
        flow1_has_valid_angle = False
        
        if self.flow1_executor:
            flow1_command_angle = self.flow1_executor.get_command_angle()
            flow1_has_valid_angle = self.flow1_executor.has_valid_command_angle()
        
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
            'enable_sync': self.enable_sync,
            'flow1_command_angle': flow1_command_angle,
            'flow1_has_valid_angle': flow1_has_valid_angle,
            'flow1_executor_available': self.flow1_executor is not None,
            'progress_register': 1202,
            'flow_start_position': 'put_asm_down',
            'flow_end_position': 'standby',
            'angle_source': 'Flow1.commandAngle'
        }