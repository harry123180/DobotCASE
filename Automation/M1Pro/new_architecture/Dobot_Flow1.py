#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_AutoFeeding_Optimized.py - Flow1 VP視覺抓取流程 (優化版)
實作所有性能改善方案：
1. 預先建立AutoFeeding連接
2. 可選關閉機械臂sync動作 (enable_sync參數)
3. 減少夾爪等待時間
4. 優化AutoFeeding讀取邏輯
5. 減少不必要的print輸出
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


class OptimizedAutoFeedingInterface:
    """優化版AutoFeeding座標接口 - 預先建立連接版本"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self._connection_retries = 0
        self._max_retries = 3
        
        # AutoFeeding交握寄存器映射 (基地址900)
        self.REGISTERS = {
            'AF_MODULE_STATUS': 900,      # AutoFeeding模組狀態
            'FEEDING_COMPLETE': 940,      # 入料完成標誌
            'TARGET_X_HIGH': 941,         # 料件座標X高位
            'TARGET_X_LOW': 942,          # 料件座標X低位
            'TARGET_Y_HIGH': 943,         # 料件座標Y高位
            'TARGET_Y_LOW': 944,          # 料件座標Y低位
            'AUTOPROGRAM_CONFIRM': 945,   # AutoProgram確認讀取
            'TOTAL_DETECTIONS': 946,      # 檢測結果總數
            'CASE_F_COUNT': 947,          # CASE_F總數
        }
        
        # 預先建立連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立，包含重連邏輯"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(self.REGISTERS['AF_MODULE_STATUS'], 1, slave=1)
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
                timeout=1.0  # 從3.0秒縮短到1.0秒
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self._connection_retries = 0
                print("✓ AutoFeeding連接已建立")
                return True
            else:
                self.connected = False
                self._connection_retries += 1
                print(f"✗ AutoFeeding連接失敗 (嘗試 {self._connection_retries}/{self._max_retries})")
                return False
                
        except Exception as e:
            self.connected = False
            self._connection_retries += 1
            print(f"AutoFeeding連接異常: {e}")
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
    
    def check_feeding_complete(self) -> bool:
        """檢查入料是否完成"""
        feeding_complete = self.read_register('FEEDING_COMPLETE')
        return feeding_complete == 1
    
    def read_target_coordinates_fast(self) -> Optional[Dict[str, float]]:
        """快速讀取AutoFeeding座標 - 優化版"""
        try:
            # 快速狀態檢查
            if not self.check_feeding_complete():
                return None
            
            # 批量讀取座標寄存器 (941-947)
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['TARGET_X_HIGH'], 7, slave=1
                )
                if result.isError():
                    return None
                
                registers = result.registers
                x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                total_detections, case_f_count = registers[5], registers[6]
                
            except Exception:
                # 批量讀取失敗，回退到單個讀取
                x_high = self.read_register('TARGET_X_HIGH') or 0
                x_low = self.read_register('TARGET_X_LOW') or 0
                y_high = self.read_register('TARGET_Y_HIGH') or 0
                y_low = self.read_register('TARGET_Y_LOW') or 0
                total_detections = self.read_register('TOTAL_DETECTIONS') or 0
                case_f_count = self.read_register('CASE_F_COUNT') or 0
            
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
                'total_detections': total_detections,
                'case_f_count': case_f_count
            }
            
        except Exception as e:
            print(f"讀取AutoFeeding目標座標異常: {e}")
            return None
    
    def confirm_coordinate_read_fast(self) -> bool:
        """快速確認讀取座標"""
        return self.write_register('AUTOPROGRAM_CONFIRM', 1)


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
    """Flow1: VP視覺抓取流程執行器 - 優化版"""
    
    def __init__(self, enable_sync: bool = False):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程(優化版)")
        
        # 性能優化參數
        self.enable_sync = enable_sync  # 是否啟用機械臂sync
        self.motion_steps = []
        
        # 流程高度參數
        self.VP_DETECT_HEIGHT = 244.65
        self.PICKUP_HEIGHT = 150
        
        # 優化的等待時間參數
        self.GRIPPER_CLOSE_WAIT = 0.3  # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 1.0  # 從1.5秒減少到1.0秒
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 預先建立AutoFeeding連接
        self.autofeeding_interface = OptimizedAutoFeedingInterface()
        
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby", "vp_topside", "flip_pre", 
            "Goal_CV_top", "rotate_top", "rotate_down"
        ]
        
        # 初始化
        self._load_and_validate_points()
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ Flow1優化版初始化完成 (sync={'啟用' if enable_sync else '停用'})")
        
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
        """建構Flow1步驟 - 優化版"""
        if not self.points_loaded:
            self.motion_steps = []
            self.total_steps = 0
            return
            
        self.motion_steps = [
            # 1. 快速讀取AutoFeeding座標
            {'type': 'read_autofeeding_coordinates_fast', 'params': {}},
            
            # 2. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close_fast', 'params': {}},
            
            # 3. VP視覺序列
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            {'type': 'move_to_detected_position_high', 'params': {}},
            {'type': 'move_to_detected_position_low', 'params': {}},
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
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 470}},
            {'type': 'gripper_close_fast', 'params': {}},
            
            # 7. 返回待機
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"✓ Flow1步驟建構完成，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - 優化版"""
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
        
        # 檢查AutoFeeding連接 (快速檢查，不重建)
        if not self.autofeeding_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoFeeding連接失效",
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
                if step['type'] in ['read_autofeeding_coordinates_fast', 'move_to_detected_position_high', 'move_to_detected_position_low']:
                    print(f"Flow1 關鍵步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = self._execute_step(step, detected_position)
                
                if step['type'] == 'read_autofeeding_coordinates_fast':
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
        elif step_type == 'read_autofeeding_coordinates_fast':
            return self._execute_read_autofeeding_coordinates_fast()
        elif step_type == 'move_to_detected_position_high':
            return self._execute_move_to_detected_high_optimized(detected_position)
        elif step_type == 'move_to_detected_position_low':
            return self._execute_move_to_detected_low_optimized(detected_position)
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    
    def _execute_read_autofeeding_coordinates_fast(self) -> Optional[Dict[str, float]]:
        """快速讀取AutoFeeding座標 - 極度優化版"""
        try:
            # 快速狀態檢查 (無備用檢查)
            af_status = self.autofeeding_interface.read_register('AF_MODULE_STATUS')
            if af_status not in [1, 2]:
                return None
            
            # 快速等待入料完成 (縮短超時)
            timeout = 5.0  # 從10秒進一步縮短到5秒
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                if self.autofeeding_interface.check_feeding_complete():
                    break
                time.sleep(0.05)  # 50ms檢查間隔
            else:
                return None
            
            # 快速讀取座標
            coord_data = self.autofeeding_interface.read_target_coordinates_fast()
            if not coord_data:
                return None
            
            # 快速確認讀取
            self.autofeeding_interface.confirm_coordinate_read_fast()
            
            # 快速清除標誌等待 (極度縮短)
            clear_start = time.time()
            while time.time() - clear_start < 0.5:  # 從1.0秒縮短到0.5秒
                if not self.autofeeding_interface.check_feeding_complete():
                    break
                time.sleep(0.02)  # 20ms檢查間隔
            
            # 構建結果座標
            vp_topside_point = self.points_manager.get_point('vp_topside')
            if not vp_topside_point:
                return None
            
            detected_pos = {
                'x': coord_data['x'],
                'y': coord_data['y'],
                'z': vp_topside_point.z,
                'r': vp_topside_point.r,
                'source': 'autofeeding_optimized'
            }
            
            print(f"✓ AutoFeeding座標讀取成功: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
            return detected_pos
            
        except Exception as e:
            print(f"AutoFeeding座標讀取異常: {e}")
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
        if hasattr(self, 'autofeeding_interface'):
            self.autofeeding_interface.disconnect()
    
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
                self.autofeeding_interface.connected)


# 使用範例
if __name__ == "__main__":
    # 建立Flow1執行器 - 保持原類名
    # enable_sync=False 表示關閉機械臂sync，提升執行速度
    # enable_sync=True 表示啟用機械臂sync，確保運動精度
    
    flow1_fast = Flow1VisionPickExecutor(enable_sync=False)  # 高速模式
    #flow1_precise = Flow1VisionPickExecutor(enable_sync=True)  # 精確模式
    
    print("Flow1執行器已建立")
    print(f"高速模式ready: {flow1_fast.is_ready()}")
    #print(f"精確模式ready: {flow1_precise.is_ready()}")