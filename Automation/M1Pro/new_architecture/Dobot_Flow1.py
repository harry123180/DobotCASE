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
            'TARGET_X_HIGH': 1340,           # 目標座標X高位
            'TARGET_X_LOW': 1341,            # 目標座標X低位
            'TARGET_Y_HIGH': 1342,           # 目標座標Y高位  
            'TARGET_Y_LOW': 1343,            # 目標座標Y低位
            
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
        """檢查AutoProgram狀態並確認座標可用"""
        try:
            # 1. 檢查AutoProgram系統狀態(1300) 
            # 接受 1(運行中) 或 2(Flow1觸發狀態)
            system_status = self.read_register('SYSTEM_STATUS')
            if system_status not in [1, 2]:
                return False
            
            # 2. 檢查prepare_done狀態(1301) = 0(需要取料)
            prepare_done = self.read_register('PREPARE_DONE') 
            if prepare_done != 0:
                return False
                
            # 3. 檢查座標是否已準備在AutoProgram中
            # 讀取座標寄存器檢查是否有有效座標
            x_high = self.read_register('TARGET_X_HIGH') or 0
            x_low = self.read_register('TARGET_X_LOW') or 0
            y_high = self.read_register('TARGET_Y_HIGH') or 0
            y_low = self.read_register('TARGET_Y_LOW') or 0
            
            # 檢查座標是否非零(有效)
            if x_high == 0 and x_low == 0 and y_high == 0 and y_low == 0:
                return False
                
            return True
            
        except Exception as e:
            print(f"檢查AutoProgram狀態異常: {e}")
            return False
    
    def read_target_coordinates_from_autoprogram(self) -> Optional[Dict[str, float]]:
        """從AutoProgram讀取目標座標"""
        try:
            # 批量讀取座標寄存器 (1340-1343)
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
            
            return {
                'x': world_x,
                'y': world_y,
                'source': 'autoprogram_interface'
            }
            
        except Exception as e:
            print(f"讀取AutoProgram目標座標異常: {e}")
            return None
    
    def confirm_coordinate_read(self) -> bool:
        """確認座標已讀取"""
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
        self.PICKUP_HEIGHT = 160
        
        # 優化的等待時間參數
        self.GRIPPER_CLOSE_WAIT = 0.3  # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 1.0  # 從1.5秒減少到1.0秒
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 預先建立AutoProgram連接 (替代AutoFeeding)
        self.autoprogram_interface = AutoProgramInterface()
        
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
            
            # 2. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close_fast', 'params': {}},
            {'type': 'gripper_close_fast', 'params': {}},
            
            # 3. VP視覺序列
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            {'type': 'move_to_detected_position_high', 'params': {}},
            {'type': 'move_to_detected_position_low', 'params': {}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 470}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 470}},
            
            # 4. 返回序列
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 5. 翻轉序列
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            
            # 6. 翻轉操作
            {'type': 'gripper_close_fast', 'params': {}},
            {'type': 'gripper_close_fast', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 460}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 460}},
            {'type': 'gripper_close_fast', 'params': {}},
            {'type': 'gripper_close_fast', 'params': {}},
            
            # 7. 返回待機
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"✓ Flow1步驟建構完成，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - AutoProgram接口版"""
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
                
                # 減少print輸出，只在關鍵步驟輸出
                if step['type'] in ['read_autoprogram_coordinates', 'move_to_detected_position_high', 'move_to_detected_position_low']:
                    print(f"Flow1 關鍵步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
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
                
                # 減少進度更新頻率 (只在重要節點更新)
                if self.current_step % 3 == 0 or self.current_step == self.total_steps:
                    self._update_progress_to_1202()
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            self._update_progress_to_1202(100)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={'detected_position': detected_position} if detected_position else None
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
        """執行單個步驟 - 統一入口"""
        step_type = step['type']
        params = step.get('params', {})
        
        if step_type == 'move_to_point':
            return self._execute_move_to_point_optimized(params)
        elif step_type == 'gripper_close_fast':
            return self._execute_gripper_close_fast()
        elif step_type == 'gripper_smart_release_fast':
            return self._execute_gripper_smart_release_fast(params)
        elif step_type == 'read_autoprogram_coordinates':
            return self._execute_read_autoprogram_coordinates()
        elif step_type == 'move_to_detected_position_high':
            return self._execute_move_to_detected_high_optimized(detected_position)
        elif step_type == 'move_to_detected_position_low':
            return self._execute_move_to_detected_low_optimized(detected_position)
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    
    def _execute_read_autoprogram_coordinates(self) -> Optional[Dict[str, float]]:
        """從AutoProgram讀取座標 - 增加重試邏輯版本"""
        max_retries = 20
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                retry_count += 1
                
                # 如果不是第一次嘗試，輸出重試資訊
                if retry_count > 1:
                    print(f"[AutoProgram] 座標讀取重試 {retry_count}/{max_retries}")
                
                # 1. 檢查AutoProgram狀態並確認座標已準備
                if not self.autoprogram_interface.check_autoprogram_ready_and_coordinates():
                    # 輸出詳細狀態資訊
                    status_info = self.autoprogram_interface.get_autoprogram_status_info()
                    print(f"[AutoProgram] 重試{retry_count}: AutoProgram狀態未就緒或座標未準備")
                    print(f"  狀態詳情: {status_info}")
                    
                    # 檢查座標是否已存在
                    x_high = self.autoprogram_interface.read_register('TARGET_X_HIGH') or 0
                    x_low = self.autoprogram_interface.read_register('TARGET_X_LOW') or 0 
                    y_high = self.autoprogram_interface.read_register('TARGET_Y_HIGH') or 0
                    y_low = self.autoprogram_interface.read_register('TARGET_Y_LOW') or 0
                    print(f"  座標寄存器: X_HIGH={x_high}, X_LOW={x_low}, Y_HIGH={y_high}, Y_LOW={y_low}")
                    
                    time.sleep(0.2)  # 重試間隔200ms
                    continue
                
                # 2. 讀取座標
                coord_data = self.autoprogram_interface.read_target_coordinates_from_autoprogram()
                if not coord_data:
                    print(f"[AutoProgram] 重試{retry_count}: 座標資料讀取失敗")
                    time.sleep(0.1)
                    continue
                
                # 3. 確認讀取
                if not self.autoprogram_interface.confirm_coordinate_read():
                    print(f"[AutoProgram] 重試{retry_count}: 確認讀取失敗")
                    time.sleep(0.1)
                    continue
                
                # 4. 構建結果座標
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if not vp_topside_point:
                    print(f"[AutoProgram] 重試{retry_count}: vp_topside點位不存在")
                    time.sleep(0.1)
                    continue
                
                detected_pos = {
                    'x': coord_data['x'],
                    'y': coord_data['y'],
                    'z': vp_topside_point.z,
                    'r': vp_topside_point.r,
                    'source': 'autoprogram_interface',
                    'retry_count': retry_count
                }
                
                print(f"✓ AutoProgram座標讀取成功 (重試{retry_count}次): ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                print(f"  AutoProgram狀態: system_status={self.autoprogram_interface.read_register('SYSTEM_STATUS')}, prepare_done={self.autoprogram_interface.read_register('PREPARE_DONE')}")
                return detected_pos
                
            except Exception as e:
                print(f"[AutoProgram] 重試{retry_count} 異常: {e}")
                time.sleep(0.1)
                continue
        
        # 所有重試都失敗
        print(f"✗ AutoProgram座標讀取失敗，已重試{max_retries}次")
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
                time.sleep(self.GRIPPER_CLOSE_WAIT)  # 0.3秒等待
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
                time.sleep(self.GRIPPER_RELEASE_WAIT)  # 1.0秒等待
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
        """更新進度到寄存器1202 - 優化版減少輸出"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                
        except Exception:
            pass  # 靜默處理錯誤，避免影響流程
    
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
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        self.cleanup()
        return True
        
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


