#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_AutoFeeding.py - Flow1 VP視覺抓取流程 (AutoFeeding座標版本)
基於統一Flow架構的運動控制執行器
從AutoFeeding模組讀取經過篩選的目標座標(941-944地址)
修正：統一將進度更新到寄存器1202
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


class AutoFeedingCoordinateInterface:
    """AutoFeeding座標接口 - 讀取經過篩選的目標座標"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # AutoFeeding交握寄存器映射 (基地址900)
        self.REGISTERS = {
            # 入料交握寄存器 (940-959)
            'FEEDING_COMPLETE': 940,      # 入料完成標誌
            'TARGET_X_HIGH': 941,         # 料件座標X高位
            'TARGET_X_LOW': 942,          # 料件座標X低位
            'TARGET_Y_HIGH': 943,         # 料件座標Y高位
            'TARGET_Y_LOW': 944,          # 料件座標Y低位
            'AUTOPROGRAM_CONFIRM': 945,   # AutoProgram確認讀取
            'TOTAL_DETECTIONS': 946,      # 檢測結果總數
            'CASE_F_COUNT': 947,          # CASE_F總數
        }
    
    def connect(self) -> bool:
        """連接到AutoFeeding Modbus服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            print(f"連接AutoFeeding模組: {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                print("✓ AutoFeeding模組連接成功")
                return True
            else:
                print("✗ AutoFeeding模組連接失敗")
                return False
                
        except Exception as e:
            print(f"AutoFeeding連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
                print("AutoFeeding連接已斷開")
            except:
                pass
        self.connected = False
        self.modbus_client = None
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def check_feeding_complete(self) -> bool:
        """檢查入料是否完成"""
        feeding_complete = self.read_register('FEEDING_COMPLETE')
        return feeding_complete == 1
    
    def read_target_coordinates(self) -> Optional[Dict[str, float]]:
        """讀取AutoFeeding提供的目標座標"""
        try:
            # 檢查入料完成標誌
            if not self.check_feeding_complete():
                print("AutoFeeding入料未完成，無法讀取座標")
                return None
            
            # 讀取32位座標
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
            
            # 讀取統計資訊
            total_detections = self.read_register('TOTAL_DETECTIONS') or 0
            case_f_count = self.read_register('CASE_F_COUNT') or 0
            
            print(f"AutoFeeding座標讀取成功:")
            print(f"  目標座標: ({world_x:.2f}, {world_y:.2f}) mm")
            print(f"  檢測統計: 總數={total_detections}, CASE_F={case_f_count}")
            
            return {
                'x': world_x,
                'y': world_y,
                'total_detections': total_detections,
                'case_f_count': case_f_count
            }
            
        except Exception as e:
            print(f"讀取AutoFeeding目標座標異常: {e}")
            return None
    
    def confirm_coordinate_read(self) -> bool:
        """確認已讀取座標 - AutoProgram協議"""
        try:
            success = self.write_register('AUTOPROGRAM_CONFIRM', 1)
            if success:
                print("已確認讀取AutoFeeding座標")
                return True
            else:
                print("確認讀取AutoFeeding座標失敗")
                return False
        except Exception as e:
            print(f"確認讀取座標異常: {e}")
            return False


class PointsManager:
    """點位管理器 - 支援cartesian格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        # 確保使用絕對路徑，相對於當前執行檔案的目錄
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian格式"""
        try:
            print(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                print(f"錯誤: 點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        # 原始格式
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        # 新格式
                        pose_data = point_data["cartesian"]
                    else:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
                    # 檢查關節數據
                    if "joint" not in point_data:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少關節數據")
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
                        print(f"自動修正點位名稱: stanby -> standby")
                    
                    self.points[point_name] = point
                    
                except Exception as e:
                    print(f"處理點位 {point_data.get('name', 'unknown')} 時發生錯誤: {e}")
                    continue
                
            print(f"載入點位數據成功，共{len(self.points)}個點位: {list(self.points.keys())}")
            return True
            
        except Exception as e:
            print(f"錯誤: 載入點位數據失敗: {e}")
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
    """Flow1: VP視覺抓取流程執行器 - AutoFeeding座標版本"""
    
    def __init__(self):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程(AutoFeeding)")
        self.motion_steps = []
        
        # 流程高度參數
        self.VP_DETECT_HEIGHT = 244.65    # VP檢測高度（與vp_topside等高）
        self.PICKUP_HEIGHT = 150       # VP夾取高度
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 初始化AutoFeeding座標接口
        self.autofeeding_interface = AutoFeedingCoordinateInterface()
        
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "vp_topside",   # VP震動盤上方點
            "flip_pre",     # 翻轉預備點
            "Goal_CV_top",  # 翻轉目標頂部點
            "rotate_top",   # 旋轉頂部點
            "rotate_down"   # 旋轉底部點
        ]
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow1正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow1無法執行")
            self.points_loaded = False
            return
        
        # 檢查所有必要點位是否存在
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            print(f"錯誤: 缺少必要點位: {missing_points}")
            print(f"可用點位: {self.points_manager.list_points()}")
            self.points_loaded = False
            return
        
        print("✓ 所有必要點位載入成功")
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow1步驟 - AutoFeeding座標版本"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # Flow1流程步驟 - AutoFeeding座標版本
        self.motion_steps = [
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            
            # 2. VP視覺檢測序列 - 讀取AutoFeeding座標版本
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            {'type': 'read_autofeeding_coordinates', 'params': {}},  # 從AutoFeeding讀取座標
            
            # 3. 移動到檢測位置 (等高)
            {'type': 'move_to_detected_position_high', 'params': {}},
            
            # 4. 下降夾取
            {'type': 'move_to_detected_position_low', 'params': {}},
            {'type': 'gripper_smart_release', 'params': {'position': 470}},
            
            # 5. 上升回到待機點
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 6. 翻轉檢測序列
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            
            # 7. 翻轉操作序列
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            {'type': 'gripper_smart_release', 'params': {'position': 470}},
            {'type': 'gripper_close', 'params': {}},
            
            # 8. 返回待機
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'flip_pre', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1流程步驟建構完成(AutoFeeding座標版本)，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - AutoFeeding座標版本"""
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow1",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
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
        
        # 連接AutoFeeding模組
        if not self.autofeeding_interface.connect():
            return FlowResult(
                success=False,
                error_message="AutoFeeding模組連接失敗",
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
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'read_autofeeding_coordinates':  # 讀取AutoFeeding座標
                    detected_position = self._execute_read_autofeeding_coordinates()
                    success = detected_position is not None
                elif step['type'] == 'move_to_detected_position_high':
                    success = self._execute_move_to_detected_high(detected_position)
                elif step['type'] == 'move_to_detected_position_low':
                    success = self._execute_move_to_detected_low(detected_position)
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
                
                # 統一更新進度到寄存器1202
                self._update_progress_to_1202()
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 最終進度設為100%
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
        finally:
            # 斷開AutoFeeding連接
            self.autofeeding_interface.disconnect()
    
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
                print(f"[Flow1] 進度已更新到1202: {progress}% (透過MotionStateMachine)")
                return
            
            # 方法2：直接寫入到1202寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    result = self.state_machine.modbus_client.write_register(1202, progress)
                    if hasattr(result, 'isError') and not result.isError():
                        print(f"[Flow1] 進度已更新到1202: {progress}% (直接寫入)")
                    else:
                        print(f"[Flow1] 進度更新失敗: {result}")
                except Exception as e:
                    print(f"[Flow1] 進度更新異常: {e}")
            else:
                print(f"[Flow1] 無法更新進度：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"[Flow1] 進度更新到1202失敗: {e}")
    
    def _execute_read_autofeeding_coordinates(self) -> Optional[Dict[str, float]]:
        """從AutoFeeding模組讀取經過篩選的目標座標"""
        try:
            print("  從AutoFeeding模組讀取目標座標...")
            
            # 等待AutoFeeding入料完成 (最多等待10秒)
            timeout = 10.0
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                if self.autofeeding_interface.check_feeding_complete():
                    print("✓ AutoFeeding入料完成，讀取座標")
                    break
                print("  等待AutoFeeding入料完成...")
                time.sleep(0.5)
            else:
                print("✗ 等待AutoFeeding入料完成超時")
                return None
            
            # 讀取目標座標
            coord_data = self.autofeeding_interface.read_target_coordinates()
            if not coord_data:
                print("✗ 讀取AutoFeeding目標座標失敗")
                return None
            
            # 確認已讀取座標
            if not self.autofeeding_interface.confirm_coordinate_read():
                print("⚠️ 確認讀取座標失敗，但繼續執行")
            
            # 獲取vp_topside點位的Z高度和R值
            vp_topside_point = self.points_manager.get_point('vp_topside')
            if not vp_topside_point:
                print("錯誤: 無法獲取vp_topside點位")
                return None
            
            detected_pos = {
                'x': coord_data['x'],
                'y': coord_data['y'],
                'z': vp_topside_point.z,  # 使用vp_topside的Z高度
                'r': vp_topside_point.r,  # 繼承vp_topside的R值
                'source': 'autofeeding'
            }
            
            print(f"✓ AutoFeeding座標讀取成功:")
            print(f"  目標位置: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
            print(f"  繼承vp_topside - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
            print(f"  檢測統計: 總數={coord_data['total_detections']}, CASE_F={coord_data['case_f_count']}")
            
            return detected_pos
            
        except Exception as e:
            print(f"讀取AutoFeeding座標異常: {e}")
            return None
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到外部點位檔案的點位"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name}")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
            if move_type == 'J':
                # 使用關節角度運動
                return self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標
                return self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _execute_gripper_close(self) -> bool:
        """執行夾爪快速關閉"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("錯誤: 夾爪API未初始化")
                return False
            
            print("夾爪快速關閉")
            success = gripper_api.quick_close()
            
            if success:
                print("✓ 夾爪快速關閉成功")
                time.sleep(1.0)  # 等待1秒確保夾爪完全關閉
                return True
            else:
                print("✗ 夾爪快速關閉失敗")
                return False
                
        except Exception as e:
            print(f"夾爪快速關閉異常: {e}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開"""
        try:
            position = params.get('position', 470)
            print(f"夾爪智能撐開到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("夾爪API未初始化")
                return False
            
            success = gripper_api.smart_release(position)
            
            if success:
                print(f"✓ 夾爪智能撐開指令發送成功")
                time.sleep(1.5)  # 等待夾爪撐開動作完成
                print(f"✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                print(f"✗ 夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            print(f"夾爪智能撐開異常: {e}")
            return False
    
    def _execute_move_to_detected_high(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(等高)"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 切換到左手系
            print("  切換到左手系（LorR=0）...")
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    result = self.robot.dashboard_api.SetArmOrientation(0)  # 0 = 左手系
                    if "0," in str(result):
                        print("  ✓ 已切換到左手系")
                    else:
                        print(f"  ⚠️ 切換到左手系可能失敗: {result}")
                except Exception as e:
                    print(f"  ⚠️ 切換座標系異常: {e}")
            
            print(f"移動到檢測位置(等高): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.VP_DETECT_HEIGHT:.2f})")
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.VP_DETECT_HEIGHT,
                detected_position['r']
            )
            
            if success:
                self.robot.sync()
                print(f"MovL已完成並同步: 檢測高度={self.VP_DETECT_HEIGHT:.2f}mm")
                return True
            else:
                print(f"MovL指令執行失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度)"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            print(f"移動到檢測位置(夾取): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.PICKUP_HEIGHT:.2f})")
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.PICKUP_HEIGHT,
                detected_position['r']
            )
            
            if success:
                self.robot.sync()
                print(f"✓ 下降到夾取位置完成並已同步，夾取高度={self.PICKUP_HEIGHT:.2f}mm")
                return True
            else:
                print(f"✗ 下降到夾取位置失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置(夾取高度)失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow1已暫停")
        return True
        
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
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return self.points_loaded and self.total_steps > 0