#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_AutoProgram_Interface.py - Flow1 VP視覺抓取流程 (AutoProgram接口版)
修改交握流程：
1. Flow1 -> 檢查AutoProgram狀態 -> 向AutoProgram拿座標 -> Flow1執行
2. 移除與AutoFeeding的直接連接
3. 統一與AutoProgram模組交握
"""

import time
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus
from AngleHighLevel import AngleHighLevel, AngleOperationResult
# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False


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


class AutoProgramInterface:
    """AutoProgram座標接口 - 統一交握版本"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self._connection_retries = 0
        self._max_retries = 3
        
        # AutoProgram交握寄存器映射
        self.REGISTERS = {
            # AutoProgram系統狀態寄存器 (1300-1319)
            'SYSTEM_STATUS': 1300,           # 系統狀態 (0=停止, 1=運行, 2=Flow1觸發, 3=Flow5完成, 4=錯誤)
            'PREPARE_DONE': 1301,            # prepare_done狀態 (0=需要取料, 1=已完成取料)
            'AUTO_PROGRAM_ENABLED': 1302,    # 自動程序啟用狀態
            'AF_CASE_F_STATUS': 1303,        # AutoFeeding CASE_F狀態
            'FLOW5_STATUS': 1304,            # Flow5完成狀態
            
            # AutoProgram控制寄存器 (1320-1339)  
            'SYSTEM_CONTROL': 1320,          # 系統控制指令
            'AUTO_PROGRAM_CONTROL': 1321,    # 自動程序啟用控制
            
            # AutoProgram座標寄存器 (1340-1349)
            'COORDS_AVAILABLE': 1340,    # 座標可用標誌
            'TARGET_X_HIGH': 1341,       # X座標高位
            'TARGET_X_LOW': 1342,        # X座標低位  
            'TARGET_Y_HIGH': 1343,       # Y座標高位
            'TARGET_Y_LOW': 1344,        # Y座標低位
            
            # 原始AutoFeeding寄存器 (向下兼容)
            'AF_CASE_F_AVAILABLE': 940,      # CASE_F可用標誌
            'AF_COORDS_TAKEN': 945,          # 座標已讀取標誌確認
        }
        
        # 預先建立連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立，包含重連邏輯"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['SYSTEM_STATUS'], 1, slave=1
                )
                if not test_result.isError():
                    return True
            except:
                pass
        
        # 需要重新連接
        return self._establish_connection()
    
    def _establish_connection(self) -> bool:
        """建立連接的內部方法"""
        try:
            if self.modbus_client:
                try:
                    self.modbus_client.close()
                except:
                    pass
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=2.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self._connection_retries = 0
                print("✓ AutoProgram連接已建立")
                return True
            else:
                self.connected = False
                self._connection_retries += 1
                print(f"✗ AutoProgram連接失敗 (嘗試 {self._connection_retries}/{self._max_retries})")
                return False
                
        except Exception as e:
            self.connected = False
            self._connection_retries += 1
            print(f"AutoProgram連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
            except:
                pass
        self.connected = False
        self.modbus_client = None
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 - 包含自動重連"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception:
            # 連接可能中斷，標記為未連接
            self.connected = False
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 - 包含自動重連"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            self.connected = False
            return False
    
    def check_autoprogram_ready_and_coordinates(self) -> bool:
        """檢查AutoProgram狀態並確認座標可用 - 統一地址版"""
        try:
            # 1. 檢查AutoProgram系統狀態(1300) 
            # 接受 1(運行中), 2(Flow1觸發狀態), 3(Flow1等待狀態)
            system_status = self.read_register('SYSTEM_STATUS')
            if system_status not in [1, 2, 3]:
                return False
            
            # 2. 檢查prepare_done狀態(1301) = 0(需要取料)
            prepare_done = self.read_register('PREPARE_DONE') 
            if prepare_done != 0:
                return False
                
            # 3. 檢查座標可用標誌(1340) = 1
            coords_available = self.read_register('COORDS_AVAILABLE') or 0
            if coords_available != 1:
                return False
                
            return True
            
        except Exception as e:
            print(f"檢查AutoProgram狀態異常: {e}")
            return False

    def read_target_coordinates_from_autoprogram(self) -> Optional[Dict[str, float]]:
        """從AutoProgram讀取目標座標 - 統一地址版"""
        try:
            # 首先檢查座標可用標誌
            coords_available = self.read_register('COORDS_AVAILABLE') or 0
            if coords_available != 1:
                print("AutoProgram座標不可用 (1340=0)")
                return None
            
            # 批量讀取座標寄存器 (1341-1344)
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['TARGET_X_HIGH'], 4, slave=1
                )
                if result.isError():
                    return None
                
                registers = result.registers
                x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                
            except Exception:
                # 批量讀取失敗，回退到單個讀取
                x_high = self.read_register('TARGET_X_HIGH') or 0
                x_low = self.read_register('TARGET_X_LOW') or 0
                y_high = self.read_register('TARGET_Y_HIGH') or 0
                y_low = self.read_register('TARGET_Y_LOW') or 0
            
            # 32位合併並轉換精度
            world_x_int = (x_high << 16) | x_low
            world_y_int = (y_high << 16) | y_low
            
            # 處理負數 (補碼轉換)
            if world_x_int >= 2**31:
                world_x_int -= 2**32
            if world_y_int >= 2**31:
                world_y_int -= 2**32
            
            # 恢復精度 (÷100)
            world_x = world_x_int / 100.0
            world_y = world_y_int / 100.0
            
            print(f"從AutoProgram讀取座標: ({world_x:.2f}, {world_y:.2f})")
            
            return {
                'x': world_x,
                'y': world_y,
                'source': 'autoprogram_interface'
            }
            
        except Exception as e:
            print(f"讀取AutoProgram目標座標異常: {e}")
            return None

    def confirm_coordinate_read(self) -> bool:
        """確認座標已讀取 - 通知AutoFeeding座標已被取用"""
        return self.write_register('AF_COORDS_TAKEN', 1)

    def get_autoprogram_status_info(self) -> Dict[str, Any]:
        """獲取AutoProgram狀態資訊"""
        try:
            return {
                'system_status': self.read_register('SYSTEM_STATUS'),
                'prepare_done': self.read_register('PREPARE_DONE'),
                'auto_program_enabled': self.read_register('AUTO_PROGRAM_ENABLED'),
                'af_case_f_status': self.read_register('AF_CASE_F_STATUS'),
                'flow5_status': self.read_register('FLOW5_STATUS'),
                'af_case_f_available': self.read_register('AF_CASE_F_AVAILABLE'),
                'coords_available': self.read_register('COORDS_AVAILABLE'),  # 新增：座標可用狀態
                'connected': self.connected
            }
        except Exception as e:
            print(f"獲取AutoProgram狀態資訊異常: {e}")
            return {'connected': False, 'error': str(e)}


class PointsManager:
    """點位管理器 - 支援cartesian格式"""
    
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


class Flow1VisionPickExecutor(FlowExecutor):
    """Flow1: VP視覺抓取流程執行器 - AutoProgram接口版"""
    
    def __init__(self, enable_sync: bool = False):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程(AutoProgram接口版)")
        
        # 性能優化參數
        self.enable_sync = enable_sync  # 是否啟用機械臂sync
        self.motion_steps = []
        
        # 流程高度參數
        self.VP_DETECT_HEIGHT = 244.65
        self.PICKUP_HEIGHT = 158
        
        # 優化的等待時間參數
        self.GRIPPER_CLOSE_WAIT = 0.3  # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 1.0  # 從1.5秒減少到1.0秒
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 預先建立AutoProgram連接 (替代AutoFeeding)
        self.autoprogram_interface = AutoProgramInterface()
        # 角度檢測相關 - 成員變數供Flow5引用
        self.angle_detector = None
        self.target_angle = None
        self.command_angle = None  # 關鍵：供Flow5引用的角度變數

        # 初始化角度檢測器
        self.angle_detector = AngleHighLevel()
        print(f"✓ Flow1 新流程版初始化完成 (sync={'啟用' if enable_sync else '停用'})")
        print("✓ commandAngle作為成員變數，供Flow5引用")
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby", "vp_topside", "flip_pre", 
            "Goal_CV_top", "rotate_top", "rotate_down"
        ]
        
        # 初始化
        self._load_and_validate_points()
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ Flow1 AutoProgram接口版初始化完成 (sync={'啟用' if enable_sync else '停用'})")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
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
        """建構Flow1步驟 - AutoProgram接口版"""
        if not self.points_loaded:
            self.motion_steps = []
            self.total_steps = 0
            return
            
        self.motion_steps = [
            # 1. 從AutoProgram讀取座標
            {'type': 'read_autoprogram_coordinates', 'params': {}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 250}},
            # 2. 初始準備
            #{'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            #{'type': 'gripper_close_fast', 'params': {}},
            
            # 3. VP視覺序列
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'JointMovJ','sync': False}},
            {'type': 'move_to_detected_position_high', 'params': {'sync': False}},
            {'type': 'move_to_detected_position_low', 'params': {'sync': True}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 460}},
            
            # 4. 返回序列
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L','sync': False}},
            {'type': 'move_to_detected_position_high', 'params': {'sync': False}},
            
            # 5. 翻轉序列
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'JointMovJ'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'JointMovJ','sync': True }},
            
            # 6. 翻轉操作
            #{'type': 'gripper_close_fast', 'params': {}},
            #{'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            #{'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            #{'type': 'gripper_smart_release_fast', 'params': {'position': 460}},
            {'type': 'gripper_close_fast', 'params': {}},
            
            # 7. 前往角度檢測位置
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'JointMovJ'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'JointMovJ','sync': True }},
            
            # 8. 執行CCD3角度檢測
            {'type': 'angle_detection', 'params': {}},
            
            # 9. 前往組裝位置
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'JointMovJ'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'JointMovJ','sync': True}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 460}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'JointMovJ'}},
        #     {'type': 'move_to_point', 'params': {
        #     'point_name': 'put_asm_pre', 
        #     'move_type': 'J',
        #     'sync': False,  # 明確指定不同步
        #     'is_waypoint': True  # 標記為經過點
        # }},
            
            # 10. 移動到put_asm_top (帶commandAngle) - Flow1終點
            {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_top', 'move_type': 'JointMovJ'}},
             {'type': 'move_to_point_with_angle', 'params': {'point_name': 'put_asm_top_2', 'move_type': 'JointMovJ'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"✓ Flow1新流程步驟建構完成，共{self.total_steps}步")
        print("✓ 流程終點: put_asm_top (帶commandAngle)")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - 新流程版本"""
        print("\n" + "="*60)
        print("開始執行Flow1 - VP視覺抓取流程 (新流程版)")
        print("流程序列: AutoProgram座標 -> VP視覺 -> 翻轉 -> CCD3角度檢測 -> put_asm_top")
        print("終點: put_asm_top，commandAngle供Flow5使用")
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
        
        # 重置角度檢測參數
        self.target_angle = None
        self.command_angle = None
        
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查AutoProgram連接
        if not self.autoprogram_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoProgram連接失效",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        detected_position = None
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow1 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = self._execute_step(step, detected_position)
                
                if step['type'] == 'read_autoprogram_coordinates':
                    detected_position = success  # 特殊處理座標讀取
                    success = detected_position is not None
                
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
            
            print(f"\n✓ Flow1執行完成！總耗時: {execution_time:.2f}秒")
            if self.target_angle is not None and self.command_angle is not None:
                print(f"✓ 角度檢測成功: target_angle={self.target_angle:.2f}°, command_angle={self.command_angle:.2f}°")
                print(f"✓ commandAngle已保存為成員變數，供Flow5引用")
            print("✓ 流程已到達put_asm_top位置，等待Flow5接續")
            print("="*60)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={
                    'detected_position': detected_position,
                    'target_angle': self.target_angle,
                    'command_angle': self.command_angle
                } if detected_position else None
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow1執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_step(self, step: Dict, detected_position: Optional[Dict]) -> Any:
        """執行單個步驟 - 增強版支援可選參數"""
        step_type = step['type']
        params = step.get('params', {})
        
        if step_type == 'move_to_point':
            return self._execute_move_to_point_with_parameters(params)
        elif step_type == 'move_to_point_with_angle':
            return self._execute_move_to_point_with_angle_with_parameters(params)
        elif step_type == 'set_arm_orientation':
            return self._execute_set_arm_orientation(params)
        elif step_type == 'gripper_close_fast':
            return self._execute_gripper_close_fast()
        elif step_type == 'gripper_smart_release_fast':
            
            return self._execute_gripper_smart_release_fast(params)
        elif step_type == 'read_autoprogram_coordinates':
            return self._execute_read_autoprogram_coordinates()
        elif step_type == 'move_to_detected_position_high':
            params['target_height'] = self.VP_DETECT_HEIGHT
            return self._execute_move_to_detected_position_with_parameters(detected_position, params)
        elif step_type == 'move_to_detected_position_low':
            params['target_height'] = self.PICKUP_HEIGHT
            return self._execute_move_to_detected_position_with_parameters(detected_position, params)
        elif step_type == 'angle_detection':
            return self._execute_angle_detection()
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 支援可選參數版本"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 構建動態參數列表 - 按照dobot_api.py的 *dynParams 格式
            dyn_params = []
            
            # 根據運動類型添加對應參數
            if move_type in ['MovJ']:
                # MovJ: dynParams順序 (User, Tool, SpeedJ, AccJ, CP等)
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
                # MovL: dynParams順序 (User, Tool, SpeedL, AccL, CP等)
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
                # JointMovJ: dynParams順序 (SpeedJ, AccJ, CP等)
                if 'speed_j' in params:
                    dyn_params.append(params['speed_j'])
                if 'acc_j' in params:
                    dyn_params.append(params['acc_j'])
                if 'cp' in params:
                    dyn_params.append(params['cp'])
            
            # 顯示執行資訊
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
                        # 回退到簡化API
                        success = self.robot.move_j(point.x, point.y, point.z, point.r) if hasattr(self.robot, 'move_j') else False
                        
                elif move_type == 'MovL':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovL(point.x, point.y, point.z, point.r, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        # 回退到簡化API
                        success = self.robot.move_l(point.x, point.y, point.z, point.r) if hasattr(self.robot, 'move_l') else False
                        
                elif move_type == 'JointMovJ':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.JointMovJ(point.j1, point.j2, point.j3, point.j4, *dyn_params)
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        # 回退到簡化API
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
            
            # Sync控制 - 支援三種方式
            sync_enabled = False
            if 'sync' in params:
                sync_enabled = params['sync']  # 步驟級別的sync設定
            elif hasattr(self, 'enable_sync'):
                sync_enabled = self.enable_sync  # 全局sync設定
            
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
        """執行移動到指定點位並使用commandAngle作為第四軸角度 - 支援參數版本"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'JointMovJ')
            
            # 檢查commandAngle是否已計算
            if self.command_angle is None:
                self.last_error = "第四軸補償角度未計算，請先執行角度檢測"
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
            sync_enabled = params.get('sync', self.enable_sync if hasattr(self, 'enable_sync') else False)
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
    def _execute_move_to_detected_position_with_parameters(self, detected_position: Optional[Dict[str, float]], params: Dict[str, Any]) -> bool:
        """移動到檢測位置 - 支援參數版本"""
        try:
            if not detected_position:
                self.last_error = "檢測位置資料無效"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            move_type = params.get('move_type', 'MovL')
            target_height = params.get('target_height', self.VP_DETECT_HEIGHT)
            
            # 構建動態參數列表
            dyn_params = []
            if move_type == 'MovL':
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
            elif move_type == 'MovJ':
                if 'user' in params:
                    dyn_params.append(params['user'])
                if 'tool' in params:
                    dyn_params.append(params['tool'])
                if 'speed_j' in params:
                    dyn_params.append(params['speed_j'])
                if 'acc_j' in params:
                    dyn_params.append(params['acc_j'])
            
            param_info = f"參數: {dyn_params}" if dyn_params else "無參數"
            print(f"移動到檢測位置 ({move_type}) - Z:{target_height} - {param_info}")

            
            # 快速切換座標系 (如果需要)
            if 'arm_orientation' in params:
                try:
                    if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                        self.robot.dashboard_api.SetArmOrientation(params['arm_orientation'])
                except Exception as e:
                    print(f"  ⚠ 設定手系失敗: {e}")
            elif hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    self.robot.dashboard_api.SetArmOrientation(0)  # 預設左手系
                except:
                    pass
            
            # 執行移動
            success = False
            try:
                if move_type == 'MovL':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovL(
                            detected_position['x'],
                            detected_position['y'],
                            target_height,
                            detected_position['r'],
                            *dyn_params
                        )
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.move_l(
                            detected_position['x'],
                            detected_position['y'],
                            target_height,
                            detected_position['r']
                        ) if hasattr(self.robot, 'move_l') else False
                        
                elif move_type == 'MovJ':
                    if hasattr(self.robot, 'move_api') and self.robot.move_api:
                        result = self.robot.move_api.MovJ(
                            detected_position['x'],
                            detected_position['y'],
                            target_height,
                            detected_position['r'],
                            *dyn_params
                        )
                        success = self._parse_api_response(result) if hasattr(self, '_parse_api_response') else True
                    else:
                        success = self.robot.move_j(
                            detected_position['x'],
                            detected_position['y'],
                            target_height,
                            detected_position['r']
                        ) if hasattr(self.robot, 'move_j') else False
                        
            except Exception as e:
                print(f"  ⚠ API調用異常，嘗試回退: {e}")
                # 回退到基本運動
                success = self.robot.move_l(
                    detected_position['x'],
                    detected_position['y'],
                    target_height,
                    detected_position['r']
                ) if hasattr(self.robot, 'move_l') else False
            
            # Sync控制
            sync_enabled = params.get('sync', self.enable_sync if hasattr(self, 'enable_sync') else False)
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
                print(f"  ✓ 移動到檢測位置成功 ({move_type}) - Z:{target_height}{sync_info}")
                return True
            else:
                self.last_error = f"移動到檢測位置失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"移動到檢測位置異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False


    def _execute_set_arm_orientation(self, params: Dict[str, Any]) -> bool:
        """執行設定機械臂手系"""
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
    def _execute_angle_detection(self) -> bool:
        """執行CCD3角度檢測並計算commandAngle - 參考Flow5實現"""
        try:
            print("開始CCD3角度檢測...")
            time.sleep(0.05)
            # 檢查角度檢測器是否初始化
            if not self.angle_detector:
                self.last_error = "角度檢測器未初始化"
                #print(f"  ✗ 角度檢測失敗: {self.last_error}")
                return False
            
            # 連接到角度檢測模組
            if not self.angle_detector.connect():
                self.last_error = "無法連接到角度檢測模組"
                #print(f"  ✗ 角度檢測失敗: {self.last_error}")
                return False
            
            # 執行角度檢測 (使用CASE模式)
            detection_result = self.angle_detector.detect_angle(detection_mode=0)
            
            # 斷開連接
            self.angle_detector.disconnect()
            
            # 檢查檢測結果
            if detection_result.result != AngleOperationResult.SUCCESS:
                self.last_error = f"角度檢測失敗: {detection_result.message}"
                #print(f"  ✗ 角度檢測失敗: {self.last_error}")
                return False
            
            # 獲取target_angle並計算command_angle
            self.target_angle = detection_result.target_angle
            self.command_angle = self.target_angle + 21.5  # 參考Flow5的計算方式 原本是20
            
            # print(f"  ✓ CCD3角度檢測成功: target_angle={self.target_angle:.2f}°")
            # print(f"  ✓ 計算commandAngle: {self.command_angle:.2f}° (target_angle + 20)")
            # print(f"  ✓ commandAngle已保存為Flow1成員變數，供Flow5引用")
            
            return True
            
        except Exception as e:
            #self.last_error = f"角度檢測異常: {e}"
            #print(f"  ✗ 角度檢測異常: {self.last_error}")
            return False
    
    def _execute_move_to_point_with_angle(self, params: Dict[str, Any]) -> bool:
        """執行移動到指定點位並使用commandAngle作為第四軸角度"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            # 檢查commandAngle是否已計算
            if self.command_angle is None:
                self.last_error = "第四軸補償角度未計算，請先執行角度檢測"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 檢查點位是否存在
            point = self.points_manager.get_point(point_name)
            if not point:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            print(f"移動到點位 {point_name} (使用commandAngle)")
            print(f"  原始關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  補償關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{self.command_angle:.1f})")
            
            # 執行移動 - 使用補償後的第四軸角度
            success = False
            if move_type == 'J':
                success = self.robot.joint_move_j(
                    point.j1, 
                    point.j2, 
                    point.j3, 
                    self.command_angle  # 使用計算出的補償角度
                )
            elif move_type == 'L':
                success = self.robot.move_l(
                    point.x, 
                    point.y, 
                    point.z, 
                    self.command_angle  # 使用計算出的補償角度
                )
            else:
                self.last_error = f"未知移動類型: {move_type}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 可選的sync控制
            if success and self.enable_sync:
                self.robot.sync()
            
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
    def _execute_read_autoprogram_coordinates(self) -> Optional[Dict[str, float]]:
        """從AutoProgram讀取座標 - 增強調試版本"""
        max_retries = 20
        retry_count = 0
        
        print("\n" + "="*50)
        print("開始從AutoProgram讀取座標")
        print("="*50)
        
        while retry_count < max_retries:
            try:
                retry_count += 1
                
                print(f"\n[重試 {retry_count}/{max_retries}] 檢查AutoProgram狀態...")
                
                # Step 1: 檢查連接狀態
                if not self.autoprogram_interface.ensure_connection():
                    print(f"  ✗ AutoProgram連接失敗")
                    if retry_count <= 3:  # 前3次重試時詳細輸出
                        print(f"    - Modbus服務器: {self.autoprogram_interface.modbus_host}:{self.autoprogram_interface.modbus_port}")
                        print(f"    - 連接狀態: {self.autoprogram_interface.connected}")
                    time.sleep(0.2)
                    continue
                else:
                    print(f"  ✓ AutoProgram連接正常")
                
                # Step 2: 讀取AutoProgram系統狀態
                system_status = self.autoprogram_interface.read_register('SYSTEM_STATUS')
                prepare_done = self.autoprogram_interface.read_register('PREPARE_DONE')
                auto_enabled = self.autoprogram_interface.read_register('AUTO_PROGRAM_ENABLED')
                af_case_f = self.autoprogram_interface.read_register('AF_CASE_F_STATUS')
                
                print(f"  AutoProgram狀態檢查:")
                print(f"    - SYSTEM_STATUS(1300): {system_status} (需要1或2)")
                print(f"    - PREPARE_DONE(1301): {prepare_done} (需要0)")
                print(f"    - AUTO_PROGRAM_ENABLED(1302): {auto_enabled}")
                print(f"    - AF_CASE_F_STATUS(1303): {af_case_f}")
                
                # Step 3: 檢查系統狀態是否符合要求
                # 接受狀態: 1=運行中, 2=Flow1觸發, 3=Flow1等待
                if system_status not in [1, 2, 3]:
                    print(f"  ✗ 系統狀態不符合要求: {system_status} (需要1=運行中, 2=Flow1觸發, 或 3=Flow1等待)")
                    time.sleep(0.2)
                    continue
                else:
                    status_name = {1: "運行中", 2: "Flow1觸發", 3: "Flow1等待"}.get(system_status, "未知")
                    print(f"  ✓ 系統狀態符合要求: {system_status} ({status_name})")
                
                # Step 4: 檢查prepare_done狀態
                if prepare_done != 0:
                    print(f"  ✗ prepare_done狀態錯誤: {prepare_done} (需要0=需要取料)")
                    time.sleep(0.2)
                    continue
                else:
                    print(f"  ✓ prepare_done狀態正確: {prepare_done}")
                
                # Step 5: 詳細檢查座標寄存器
                print(f"  座標寄存器檢查:")
                
                # 使用批量讀取
                try:
                    result = self.autoprogram_interface.modbus_client.read_holding_registers(
                        self.autoprogram_interface.REGISTERS['TARGET_X_HIGH'], 4, slave=1
                    )
                    if result.isError():
                        print(f"    ✗ 批量讀取座標失敗: {result}")
                        x_high = self.autoprogram_interface.read_register('TARGET_X_HIGH') or 0
                        x_low = self.autoprogram_interface.read_register('TARGET_X_LOW') or 0
                        y_high = self.autoprogram_interface.read_register('TARGET_Y_HIGH') or 0
                        y_low = self.autoprogram_interface.read_register('TARGET_Y_LOW') or 0
                        print(f"    - 改用單個讀取")
                    else:
                        registers = result.registers
                        x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                        print(f"    ✓ 批量讀取座標成功")
                except Exception as e:
                    print(f"    ✗ 批量讀取異常: {e}")
                    x_high = self.autoprogram_interface.read_register('TARGET_X_HIGH') or 0
                    x_low = self.autoprogram_interface.read_register('TARGET_X_LOW') or 0
                    y_high = self.autoprogram_interface.read_register('TARGET_Y_HIGH') or 0
                    y_low = self.autoprogram_interface.read_register('TARGET_Y_LOW') or 0
                    print(f"    - 改用單個讀取")
                
                print(f"    - TARGET_X_HIGH(1341): {x_high}")
                print(f"    - TARGET_X_LOW(1342): {x_low}")
                print(f"    - TARGET_Y_HIGH(1343): {y_high}")
                print(f"    - TARGET_Y_LOW(1344): {y_low}")
                
                # Step 6: 檢查座標是否有效（非全零）
                if x_high == 0 and x_low == 0 and y_high == 0 and y_low == 0:
                    print(f"  ✗ 座標寄存器全為0，座標未準備好")
                    time.sleep(0.2)
                    continue
                else:
                    print(f"  ✓ 座標寄存器有有效數據")
                
                # Step 7: 座標轉換和驗證
                print(f"  座標轉換:")
                
                # 32位合併
                world_x_int = (x_high << 16) | x_low
                world_y_int = (y_high << 16) | y_low
                print(f"    - 合併後整數值: X={world_x_int}, Y={world_y_int}")
                
                # 處理負數 (補碼轉換)
                if world_x_int >= 2**31:
                    world_x_int -= 2**32
                    print(f"    - X負數轉換: {world_x_int}")
                if world_y_int >= 2**31:
                    world_y_int -= 2**32
                    print(f"    - Y負數轉換: {world_y_int}")
                
                # 恢復精度 (÷100)
                world_x = world_x_int / 100.0
                world_y = world_y_int / 100.0
                print(f"    - 最終座標: X={world_x:.2f}, Y={world_y:.2f}")
                
                # Step 8: 座標合理性檢查
                if abs(world_x) > 1000 or abs(world_y) > 1000:
                    print(f"  ⚠ 座標值異常，可能存在轉換錯誤")
                    print(f"    - 檢查原始寄存器值是否正確")
                
                # Step 9: 確認座標已讀取
                print(f"  確認座標讀取:")
                coords_taken_success = self.autoprogram_interface.write_register('AF_COORDS_TAKEN', 1)
                if not coords_taken_success:
                    print(f"    ✗ 寫入AF_COORDS_TAKEN(945)失敗")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"    ✓ AF_COORDS_TAKEN(945)寫入成功")
                
                # Step 10: 驗證vp_topside點位
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if not vp_topside_point:
                    print(f"  ✗ vp_topside點位不存在")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"  ✓ vp_topside點位存在: Z={vp_topside_point.z}, R={vp_topside_point.r}")
                
                # Step 11: 構建最終結果
                detected_pos = {
                    'x': world_x,
                    'y': world_y,
                    'z': vp_topside_point.z,
                    'r': vp_topside_point.r,
                    'source': 'autoprogram_interface',
                    'retry_count': retry_count,
                    'raw_registers': {
                        'x_high': x_high,
                        'x_low': x_low,
                        'y_high': y_high,
                        'y_low': y_low
                    }
                }
                
                print(f"\n" + "="*50)
                print(f"✓ AutoProgram座標讀取成功！")
                print(f"  重試次數: {retry_count}")
                print(f"  最終座標: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f}, {detected_pos['z']:.2f}, {detected_pos['r']:.2f})")
                print(f"  原始寄存器: X({x_high},{x_low}) Y({y_high},{y_low})")
                print(f"  AutoProgram狀態: system_status={system_status}, prepare_done={prepare_done}")
                print("="*50)
                
                return detected_pos
                
            except Exception as e:
                print(f"  ✗ 重試{retry_count} 發生異常: {e}")
                print(f"    異常類型: {type(e).__name__}")
                if hasattr(e, '__traceback__'):
                    import traceback
                    print(f"    異常詳情: {traceback.format_exc()}")
                time.sleep(0.1)
                continue
        
        # 所有重試都失敗
        print(f"\n" + "="*50)
        print(f"✗ AutoProgram座標讀取失敗！")
        print(f"  已重試: {max_retries}次")
        print(f"  最後狀態檢查:")
        
        try:
            # 最後一次狀態檢查
            final_status = self.autoprogram_interface.get_autoprogram_status_info()
            for key, value in final_status.items():
                print(f"    - {key}: {value}")
        except Exception as e:
            print(f"    - 無法獲取最終狀態: {e}")
        
        print("="*50)
        return None
    
    def _execute_move_to_point_optimized(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 優化版sync控制"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            point = self.points_manager.get_point(point_name)
            if not point:
                return False
            
            success = False
            if move_type == 'J':
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                success = self.robot.move_l(point.x, point.y, point.z, point.r)
            
            # 可選的sync控制
            if success and self.enable_sync:
                self.robot.sync()
                
            return success
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _execute_gripper_close_fast(self) -> bool:
        """執行夾爪快速關閉 - 優化版"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                return False
            
            success = gripper_api.quick_close()
            
            if success:
                #time.sleep(self.GRIPPER_CLOSE_WAIT)  # 0.3秒等待
                return True
            return False
                
        except Exception as e:
            print(f"夾爪關閉異常: {e}")
            return False
    
    def _execute_gripper_smart_release_fast(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開 - 優化版"""
        try:
            position = params.get('position', 470)
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                return False
            
            success = gripper_api.smart_release(position)
            
            if success:
                time.sleep(0.3)  # 1.0秒等待
                return True
            return False
                
        except Exception as e:
            print(f"夾爪撐開異常: {e}")
            return False
    
    def _execute_move_to_detected_high_optimized(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(等高) - 優化版"""
        try:
            if not detected_position:
                return False
            
            # 快速切換座標系 (減少print)
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    self.robot.dashboard_api.SetArmOrientation(0)
                except:
                    pass
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.VP_DETECT_HEIGHT,
                detected_position['r']
            )
            
            if success and self.enable_sync:
                self.robot.sync()
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low_optimized(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度) - 優化版"""
        try:
            if not detected_position:
                return False
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.PICKUP_HEIGHT,
                detected_position['r']
            )
            
            if success and self.enable_sync:
                self.robot.sync()
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置(夾取)失敗: {e}")
            return False
    
    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """更新進度到寄存器1202"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                
        except Exception:
            pass
    
    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'autoprogram_interface'):
            self.autoprogram_interface.disconnect()
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            return True
        return False
    def get_command_angle(self) -> Optional[float]:
        """供Flow5調用：獲取commandAngle"""
        return self.command_angle    
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        self.cleanup()
        return True
    def has_valid_command_angle(self) -> bool:
        """供Flow5調用：檢查commandAngle是否有效"""
        return self.command_angle is not None    
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return (self.points_loaded and 
                self.total_steps > 0 and 
                self.autoprogram_interface.connected)


