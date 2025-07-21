#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow5.py - Flow5 機械臂運轉流程執行器 - 完全參照Flow1格式重構版
從put_asm_down開始執行，引用Flow1的commandAngle
不再執行角度檢測，直接使用Flow1計算的角度
"""

import time
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類 (假設與Flow1相同)
from flow_base import FlowExecutor, FlowResult, FlowStatus


@dataclass
class RobotPoint:
    """機械臂點位數據結構"""
    name: str
    x: float
    y: float
    z: float
    r: float
    j1: float
    j2: float
    j3: float
    j4: float


class PointsManager:
    """點位管理器 - 支援cartesian格式 (參照Flow1)"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian格式"""
        try:
            if not os.path.exists(self.points_file):
                print(f"✗ 點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        pose_data = point_data["cartesian"]
                    else:
                        continue
                    
                    if "joint" not in point_data:
                        continue
                    
                    joint_data = point_data["joint"]
                    
                    point = RobotPoint(
                        name=point_data["name"],
                        x=float(pose_data["x"]),
                        y=float(pose_data["y"]),
                        z=float(pose_data["z"]),
                        r=float(pose_data["r"]),
                        j1=float(joint_data["j1"]),
                        j2=float(joint_data["j2"]),
                        j3=float(joint_data["j3"]),
                        j4=float(joint_data["j4"])
                    )
                    
                    # 處理點位名稱的拼寫錯誤
                    point_name = point.name
                    if point_name == "stanby":
                        point_name = "standby"
                    
                    self.points[point_name] = point
                    
                except Exception:
                    continue
                
            print(f"✓ 載入點位數據成功，共{len(self.points)}個點位")
            return True
            
        except Exception as e:
            print(f"✗ 載入點位數據失敗: {e}")
            return False
    
    def get_point(self, name: str) -> Optional[RobotPoint]:
        """獲取指定點位"""
        return self.points.get(name)
    
    def list_points(self) -> List[str]:
        """列出所有點位名稱"""
        return list(self.points.keys())
    
    def has_point(self, name: str) -> bool:
        """檢查是否存在指定點位"""
        return name in self.points


class Flow5AssemblyExecutor(FlowExecutor):
    """Flow5: 機械臂運轉流程執行器 - 完全參照Flow1格式重構版"""
    
    def __init__(self, enable_sync: bool = False):
        super().__init__(flow_id=5, flow_name="機械臂運轉流程(參照Flow1格式)")
        
        # 性能優化參數 (參照Flow1)
        self.enable_sync = enable_sync  # 是否啟用機械臂sync
        self.motion_steps = []
        
        # 流程參數
        self.command_angle = None  # 來自Flow1的角度
        
        # 優化的等待時間參數 (參照Flow1)
        self.GRIPPER_CLOSE_WAIT = 0.3  # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 1.0  # 從1.5秒減少到1.0秒
        
        # 初始化點位管理器 (參照Flow1)
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # Flow1執行器引用，用於獲取commandAngle
        self.flow1_executor = None
        
        # Flow5需要的點位名稱
        self.REQUIRED_POINTS = [
            "put_asm_down",         # 組裝放下位置
            "put_asm_top",          # 組裝頂部位置  
            "put_asm_pre",          # 組裝預備位置
            "rotate_top",           # 旋轉頂部點
            "flip_pre",             # 翻轉預備位置
            "standby"               # 待機位置 (終點)
        ]
        
        # 初始化
        self._load_and_validate_points()
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ Flow5參照Flow1格式初始化完成 (sync={'啟用' if enable_sync else '停用'})")
        print("✓ 起點: put_asm_down，終點: standby")
        print("✓ 引用Flow1的commandAngle，不再執行角度檢測")
        
    def set_flow1_executor(self, flow1_executor):
        """設置Flow1執行器引用"""
        self.flow1_executor = flow1_executor
        print("✓ Flow1執行器引用已設置，可獲取commandAngle")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案 (參照Flow1)"""
        if not self.points_manager.load_points():
            self.points_loaded = False
            return
        
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            print(f"✗ 缺少必要點位: {missing_points}")
            self.points_loaded = False
            return
        
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow5步驟 - 參照Flow1格式"""
        if not self.points_loaded:
            self.motion_steps = []
            self.total_steps = 0
            return
            
        self.motion_steps = [
            # 1. 移動到put_asm_down (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {
                'point_name': 'put_asm_down', 
                'move_type': 'JointMovJ',
                'speed_j': 100,       # 精確控制
                'acc_j': 100,
                'sync': True         # 等待完成
            }},
            
            # 2. 夾爪快速關閉
            {'type': 'gripper_close_fast', 'params': {
                'wait_time': 0.1,    # 確保夾緊
                'check_position': False
            }},
            
            # 3. 移動到put_asm_top (帶commandAngle)
            {'type': 'move_to_point_with_angle', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'JointMovJ',
                'speed_j': 100,       # 提高速度
                'acc_j': 100,
                'sync': False
            }},
            
            # 4. 返回序列
            {'type': 'move_to_point', 'params': {
                'point_name': 'put_asm_pre', 
                'move_type': 'JointMovJ',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            
            # 5. 移動到standby (完成)
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'JointMovJ',
                'speed_j': 100,      # 最高速度返回
                'acc_j': 100,
                'sync': False         # 確保到位
            }}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"✓ Flow5步驟建構完成，共{self.total_steps}步")
        print("角度控制策略：put_asm_down/put_asm_top使用Flow1的commandAngle")
        print("起點: put_asm_down，終點: standby")
    
    def execute(self) -> FlowResult:
        """執行Flow5主邏輯 - 參照Flow1格式"""
        print("\n" + "="*60)
        print("開始執行Flow5 - 機械臂運轉流程 (參照Flow1格式)")
        print("流程序列: put_asm_down(角度)->夾爪關閉->put_asm_top(角度)->返回序列->standby")
        print("角度來源: 引用Flow1的commandAngle")
        print("="*60)
        
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接",
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
        
        # 獲取Flow1的commandAngle
        self.command_angle = self.flow1_executor.get_command_angle()
        print(f"✓ 從Flow1獲取commandAngle: {self.command_angle:.2f}度")
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow5 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟 (參照Flow1格式)
                success = self._execute_step(step, None)
                
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
                
                # 更新進度
                self._update_progress_to_1202()
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            self._update_progress_to_1202(100)
            
            print(f"\n✓ Flow5執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ 使用角度: {self.command_angle:.2f}度 (來自Flow1)")
            print("✓ 已返回standby位置")
            print("="*60)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={
                    'command_angle': self.command_angle,
                    'flow1_source': True
                } if self.command_angle else None
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow5執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_step(self, step: Dict, detected_position: Optional[Dict]) -> Any:
        """執行單個步驟 - 參照Flow1增強版支援可選參數"""
        step_type = step['type']
        params = step.get('params', {})
        
        if step_type == 'move_to_point':
            return self._execute_move_to_point_with_parameters(params)
        elif step_type == 'move_to_point_with_angle':
            return self._execute_move_to_point_with_angle_with_parameters(params)
        elif step_type == 'set_arm_orientation':
            return self._execute_set_arm_orientation(params)
        elif step_type == 'gripper_close_fast':
            return self._execute_gripper_close_fast_with_parameters(params)
        elif step_type == 'gripper_smart_release_fast':
            return self._execute_gripper_smart_release_fast(params)
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 支援可選參數版本 (參照Flow1)"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 構建動態參數列表
            dyn_params = []
            
            # 根據運動類型添加對應參數
            if move_type in ['MovJ']:
                if 'user' in params:
                    dyn_params.append(params['user'])
                if 'tool' in params:
                    dyn_params.append(params['tool'])
                if 'speed_j' in params:
                    dyn_params.append(params['speed_j'])
                if 'acc_j' in params:
                    dyn_params.append(params['acc_j'])
                if 'cp' in params:
                    dyn_params.append(params['cp'])
                    
            elif move_type in ['MovL']:
                if 'user' in params:
                    dyn_params.append(params['user'])
                if 'tool' in params:
                    dyn_params.append(params['tool'])
                if 'speed_l' in params:
                    dyn_params.append(params['speed_l'])
                if 'acc_l' in params:
                    dyn_params.append(params['acc_l'])
                if 'cp' in params:
                    dyn_params.append(params['cp'])
                    
            elif move_type in ['JointMovJ']:
                if 'speed_j' in params:
                    dyn_params.append(params['speed_j'])
                if 'acc_j' in params:
                    dyn_params.append(params['acc_j'])
                if 'cp' in params:
                    dyn_params.append(params['cp'])
            
            param_info = f"參數: {dyn_params}" if dyn_params else "無參數"
            print(f"移動到點位 {point_name} ({move_type}) - {param_info}")
            
            # 執行移動 - 根據move_type選擇對應的API方法
            success = False
            try:
                if move_type == 'MovJ':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovJ(point.x, point.y, point.z, point.r, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.move_j(point.x, point.y, point.z, point.r) if hasattr(self.robot, 'move_j') else False
                        
                elif move_type == 'MovL':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovL(point.x, point.y, point.z, point.r, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.move_l(point.x, point.y, point.z, point.r) if hasattr(self.robot, 'move_l') else False
                        
                elif move_type == 'JointMovJ':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.JointMovJ(point.j1, point.j2, point.j3, point.j4, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4) if hasattr(self.robot, 'joint_move_j') else False
                        
                # 向下兼容的簡化類型
                elif move_type == 'J':
                    success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4) if hasattr(self.robot, 'joint_move_j') else False
                elif move_type == 'L':
                    success = self.robot.move_l(point.x, point.y, point.z, point.r) if hasattr(self.robot, 'move_l') else False
                else:
                    self.last_error = f"未知移動類型: {move_type}"
                    print(f"  ✗ 移動操作失敗: {self.last_error}")
                    return False
            
            except Exception as e:
                print(f"  ⚠ API調用異常，嘗試回退: {e}")
                # 回退到基本運動
                if move_type in ['MovJ', 'J']:
                    success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4) if hasattr(self.robot, 'joint_move_j') else False
                elif move_type in ['MovL', 'L']:
                    success = self.robot.move_l(point.x, point.y, point.z, point.r) if hasattr(self.robot, 'move_l') else False
                elif move_type == 'JointMovJ':
                    success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4) if hasattr(self.robot, 'joint_move_j') else False
            
            # Sync控制
            sync_enabled = params.get('sync', self.enable_sync)
            if success and sync_enabled:
                try:
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        self.robot.move_api.Sync()
                    elif hasattr(self.robot, 'sync'):
                        self.robot.sync()
                except Exception as e:
                    print(f"  ⚠ Sync調用異常: {e}")
            
            if success:
                sync_info = " (含Sync)" if sync_enabled else ""
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type}){sync_info}")
                return True
            else:
                self.last_error = f"移動到 {point_name} 失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"移動操作異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False
    
    def _execute_move_to_point_with_angle_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到指定點位並使用commandAngle作為第四軸角度 - 支援參數版本 (參照Flow1)"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'JointMovJ')
            
            # 檢查commandAngle是否已計算
            if self.command_angle is None:
                self.last_error = "第四軸補償角度未計算，請先執行Flow1"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 檢查點位是否存在
            point = self.points_manager.get_point(point_name)
            if not point:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 構建動態參數列表
            dyn_params = []
            if move_type in ['JointMovJ']:
                if 'speed_j' in params:
                    dyn_params.append(params['speed_j'])
                if 'acc_j' in params:
                    dyn_params.append(params['acc_j'])
                if 'cp' in params:
                    dyn_params.append(params['cp'])
            elif move_type in ['MovJ']:
                if 'user' in params:
                    dyn_params.append(params['user'])
                if 'tool' in params:
                    dyn_params.append(params['tool'])
                if 'speed_j' in params:
                    dyn_params.append(params['speed_j'])
                if 'acc_j' in params:
                    dyn_params.append(params['acc_j'])
            elif move_type in ['MovL']:
                if 'user' in params:
                    dyn_params.append(params['user'])
                if 'tool' in params:
                    dyn_params.append(params['tool'])
                if 'speed_l' in params:
                    dyn_params.append(params['speed_l'])
                if 'acc_l' in params:
                    dyn_params.append(params['acc_l'])
            
            param_info = f"參數: {dyn_params}" if dyn_params else "無參數"
            print(f"移動到點位 {point_name} (使用commandAngle={self.command_angle:.1f}°) - {param_info}")
            print(f"  原始關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  補償關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{self.command_angle:.1f})")
            
            # 執行移動 - 使用補償後的第四軸角度
            success = False
            try:
                if move_type == 'JointMovJ':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.JointMovJ(point.j1, point.j2, point.j3, self.command_angle, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.joint_move_j(point.j1, point.j2, point.j3, self.command_angle) if hasattr(self.robot, 'joint_move_j') else False
                        
                elif move_type == 'MovJ':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovJ(point.x, point.y, point.z, self.command_angle, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.move_j(point.x, point.y, point.z, self.command_angle) if hasattr(self.robot, 'move_j') else False
                        
                elif move_type == 'MovL':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovL(point.x, point.y, point.z, self.command_angle, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.move_l(point.x, point.y, point.z, self.command_angle) if hasattr(self.robot, 'move_l') else False
                else:
                    self.last_error = f"未知移動類型: {move_type}"
                    print(f"  ✗ 移動操作失敗: {self.last_error}")
                    return False
                    
            except Exception as e:
                print(f"  ⚠ API調用異常，嘗試回退: {e}")
                # 回退到基本運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, self.command_angle) if hasattr(self.robot, 'joint_move_j') else False
            
            # Sync控制
            sync_enabled = params.get('sync', self.enable_sync)
            if success and sync_enabled:
                try:
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        self.robot.move_api.Sync()
                    elif hasattr(self.robot, 'sync'):
                        self.robot.sync()
                except Exception as e:
                    print(f"  ⚠ Sync調用異常: {e}")
            
            if success:
                sync_info = " (含Sync)" if sync_enabled else ""
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) - 第四軸: {self.command_angle:.1f}度{sync_info}")
                return True
            else:
                self.last_error = f"移動到 {point_name} 失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"移動操作異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False
    
    def _execute_set_arm_orientation(self, params: Dict[str, Any]) -> bool:
        """執行設定機械臂手系 (參照Flow1)"""
        try:
            orientation = params.get('orientation', 0)  # 0=左手系, 1=右手系
            
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                result = self.robot.dashboard_api.SetArmOrientation(orientation)
                success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                
                if success:
                    orientation_name = "左手系" if orientation == 0 else "右手系"
                    print(f"  ✓ 設定機械臂手系: {orientation} ({orientation_name})")
                    return True
                else:
                    print(f"  ✗ 設定機械臂手系失敗: {result}")
                    return False
            else:
                print("  ✗ dashboard_api未連接，無法設定手系")
                return False
                
        except Exception as e:
            print(f"設定機械臂手系異常: {e}")
            return False
    
    def _execute_gripper_close_fast_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行夾爪快速關閉 - 支援參數版本 (參照Flow1)"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                return False
            
            # 可選參數
            wait_time = params.get('wait_time', self.GRIPPER_CLOSE_WAIT)
            check_position = params.get('check_position', True)
            
            print("夾爪快速關閉")
            success = gripper_api.quick_close()
            
            if success:
                print("  ✓ 夾爪快速關閉成功")
                time.sleep(wait_time)
                
                # 可選的位置檢查
                if check_position and hasattr(gripper_api, 'get_current_position'):
                    try:
                        current_pos = gripper_api.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                return True
            return False
                
        except Exception as e:
            print(f"夾爪關閉異常: {e}")
            return False
    
    def _execute_gripper_smart_release_fast(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開 - 優化版 (參照Flow1)"""
        try:
            position = params.get('position', 470)
            wait_time = params.get('wait_time', self.GRIPPER_RELEASE_WAIT)
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                return False
            
            print(f"夾爪智能撐開到位置: {position}")
            success = gripper_api.smart_release(position)
            
            if success:
                print("  ✓ 夾爪智能撐開成功")
                time.sleep(0.1)
                return True
            return False
                
        except Exception as e:
            print(f"夾爪撐開異常: {e}")
            return False
    
    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """更新進度到寄存器1202 (參照Flow1)"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                
        except Exception:
            pass
    
    def _parse_api_response(self, result) -> bool:
        """解析API響應 (參照Flow1)"""
        if result is None:
            return False
        if isinstance(result, bool):
            return result
        if isinstance(result, str):
            error_keywords = ['error', 'failed', 'fail', 'err']
            return not any(keyword in result.lower() for keyword in error_keywords)
        return True
    
    def cleanup(self):
        """清理資源 (參照Flow1)"""
        pass
    
    def pause(self) -> bool:
        """暫停Flow (參照Flow1)"""
        self.status = FlowStatus.PAUSED
        return True
        
    def resume(self) -> bool:
        """恢復Flow (參照Flow1)"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            return True
        return False
    
    def stop(self) -> bool:
        """停止Flow (參照Flow1)"""
        self.status = FlowStatus.ERROR
        self.cleanup()
        return True
    
    def get_command_angle(self) -> Optional[float]:
        """供其他Flow調用：獲取commandAngle (參照Flow1)"""
        return self.command_angle
    
    def has_valid_command_angle(self) -> bool:
        """供其他Flow調用：檢查commandAngle是否有效 (參照Flow1)"""
        return self.command_angle is not None
    
    def get_progress(self) -> int:
        """取得進度百分比 (參照Flow1)"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow5是否準備好執行 (參照Flow1)"""
        return (self.points_loaded and 
                self.total_steps > 0 and 
                self.flow1_executor is not None and
                self.flow1_executor.has_valid_command_angle())


# 使用範例
if __name__ == "__main__":
    # 建立Flow5執行器 - 參照Flow1格式
    flow5 = Flow5AssemblyExecutor(enable_sync=False)  # 高速模式
    
    print("Flow5執行器已建立 - 參照Flow1格式")
    print(f"準備狀態: {flow5.is_ready()}")