#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - VP震動盤視覺抓取流程 (依據CASE流程敘述.md修正版)
實現完整的VP視覺抓取 + 翻轉檢測流程
流程序列: standby → vp_topside → CCD1檢測 → 抓取 → flip系列 → CCD2觸發
"""

import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 16  # 依據CASE流程敘述.md的步驟數
    ccd2_triggered: bool = False
    ccd2_result: Optional[str] = None


class DobotFlow1:
    """
    VP震動盤視覺抓取流程執行器 (依據CASE流程敘述.md)
    實現完整的視覺抓取 + 翻轉檢測流程
    """
    
    def __init__(self, robot, gripper, ccd1, ccd3, state_machine):
        """初始化流程執行器"""
        # 共享資源
        self.robot = robot
        self.gripper = gripper
        self.ccd1 = ccd1
        self.ccd3 = ccd3
        self.state_machine = state_machine
        
        # 流程配置
        self.flow_id = 1
        self.total_steps = 16  # 依據CASE流程敘述.md
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # 流程參數 - 依據CASE流程敘述.md
        self.SPEED_RATIO = 100
        self.POINT_DELAY = 0.1
        self.CCD1_DETECT_HEIGHT = 238.86  # vp_topside的Z高度
        self.PICKUP_HEIGHT = 137.52       # 實際夾取Z高度
        self.GRIP_OPEN_POSITION = 370     # 夾爪撐開位置
        
        # 必要點位列表 - 依據CASE流程敘述.md
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "vp_topside",   # VP震動盤上方點
            "flip_pre",     # 翻轉預備點
            "flip_top",     # 翻轉頂部點
            "flip_down"     # 翻轉底部點
        ]
    
    def execute(self) -> FlowResult:
        """執行VP震動盤視覺抓取流程 - 依據CASE流程敘述.md"""
        print("\n" + "="*60)
        print("開始執行流程1 - VP視覺抓取 + 翻轉檢測流程")
        print("依據CASE流程敘述.md規範實現")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        ccd2_triggered = False
        ccd2_result = None
        
        detected_coord = None
        
        try:
            # 步驟1: 系統檢查
            if not self._execute_step(1, "系統檢查", self._step_system_check):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟2: 夾爪關閉 (依據CASE流程敘述.md)
            if not self._execute_step(2, "夾爪關閉", self._step_gripper_close):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟3: 移動到vp_topside
            if not self._execute_step(3, "移動到vp_topside", self._step_move_to_vp_topside):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟4: CCD1檢測API (依據CASE流程敘述.md)
            coord_result = self._execute_step_with_return(4, "CCD1檢測API", self._step_ccd1_detection)
            if coord_result is False:
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            detected_coord = coord_result
            
            # 步驟5-7: 視覺抓取流程
            if detected_coord:
                print(f"  檢測到物體 (FIFO佇列ID: {detected_coord.id})")
                print(f"  世界座標: ({detected_coord.world_x:.2f}, {detected_coord.world_y:.2f})mm")
                
                # 步驟5: 移動到視覺檢測到的物件座標(Z軸高度與vp_topside等高)
                if not self._execute_step(5, "移動到物體上方(與vp_topside等高)", 
                                        lambda: self._step_move_to_object_same_height(detected_coord)):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
                
                # 步驟6: 移動到檢測到的物件座標(Z軸到夾取位置)
                if not self._execute_step(6, "下降到夾取位置", 
                                        lambda: self._step_move_to_pickup_height(detected_coord)):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
                
                # 步驟7: 夾爪撐開至370
                if not self._execute_step(7, "夾爪撐開至370", self._step_gripper_open_370):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
                
                # 步驟8: 移動到vp_topside
                if not self._execute_step(8, "移動到vp_topside", self._step_move_to_vp_topside):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            else:
                print("  未檢測到物體，跳過抓取流程")
                for step in range(5, 9):
                    self._execute_step(step, f"跳過步驟{step}", lambda: True)
            
            # 步驟9: 移動到standby
            if not self._execute_step(9, "移動到standby", self._step_move_to_standby):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟10-14: 翻轉檢測序列 (依據CASE流程敘述.md)
            flip_sequence = [
                (10, "移動到flip_pre", "flip_pre"),
                (11, "移動到flip_top", "flip_top"),
                (12, "移動到flip_down", "flip_down"),
                (13, "移動到flip_top", "flip_top"),
                (14, "移動到flip_pre", "flip_pre")
            ]
            
            for step_num, step_name, point_name in flip_sequence:
                if not self._execute_step(step_num, step_name, 
                                        lambda p=point_name: self._step_move_to_point(p)):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟15: 移動到standby
            if not self._execute_step(15, "移動到standby", self._step_move_to_standby):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟16: 觸發CCD2(物件正反面辨識與輸送帶翻轉機構的IO控制)
            ccd2_result = self._execute_step_with_return(16, "觸發CCD2翻轉檢測", self._step_trigger_ccd2)
            if ccd2_result is not False:
                ccd2_triggered = True
                print(f"  CCD2觸發結果: {ccd2_result}")
            else:
                ccd2_triggered = False
                ccd2_result = "觸發失敗"
                print("  CCD2觸發失敗，但流程繼續")
            
            # 設置Flow1完成狀態 (只有CCD2觸發成功才設置)
            if ccd2_triggered:
                if not self._set_flow1_completion_status():
                    self.last_error = "設置Flow1完成狀態失敗"
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 流程完成
            execution_time = time.time() - start_time
            print(f"\n✓ 流程1執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ CCD2觸發狀態: {ccd2_triggered}")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps,
                ccd2_triggered=ccd2_triggered,
                ccd2_result=ccd2_result
            )
            
        except Exception as e:
            self.last_error = f"流程執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
        
        finally:
            self.is_running = False
    
    def _execute_step(self, step_num: int, step_name: str, step_func) -> bool:
        """執行單個步驟並更新進度"""
        self.current_step = step_num
        self._update_progress()
        
        print(f"[{step_num}/{self.total_steps}] {step_name}...")
        
        step_start = time.time()
        success = step_func()
        step_time = time.time() - step_start
        
        if success:
            print(f"  ✓ {step_name}完成 (耗時: {step_time*1000:.1f}ms)")
            return True
        else:
            print(f"  ✗ {step_name}失敗")
            return False
    
    def _execute_step_with_return(self, step_num: int, step_name: str, step_func):
        """執行單個步驟並返回結果"""
        self.current_step = step_num
        self._update_progress()
        
        print(f"[{step_num}/{self.total_steps}] {step_name}...")
        
        step_start = time.time()
        result = step_func()
        step_time = time.time() - step_start
        
        if result is not False:
            print(f"  ✓ {step_name}完成 (耗時: {step_time*1000:.1f}ms)")
            return result
        else:
            print(f"  ✗ {step_name}失敗")
            return False
    
    def _update_progress(self):
        """更新進度到狀態機"""
        if (self.state_machine and 
            hasattr(self.state_machine, 'modbus_client') and 
            self.state_machine.modbus_client is not None):
            try:
                progress = int((self.current_step / self.total_steps) * 100)
                self.state_machine.modbus_client.write_register(403, progress)
            except Exception:
                pass
    
    def _create_result(self, success: bool, start_time: float, 
                      ccd2_triggered: bool, ccd2_result: Optional[str]) -> FlowResult:
        """創建流程結果"""
        return FlowResult(
            success=success,
            error_message=self.last_error,
            execution_time=time.time() - start_time,
            steps_completed=self.current_step,
            total_steps=self.total_steps,
            ccd2_triggered=ccd2_triggered,
            ccd2_result=ccd2_result
        )
    
    # =================================================================
    # 流程步驟實現 - 依據CASE流程敘述.md
    # =================================================================
    
    def _step_system_check(self) -> bool:
        """步驟1: 系統檢查"""
        if not self.robot.is_ready():
            self.last_error = "機械臂未準備好"
            return False
        
        for point_name in self.REQUIRED_POINTS:
            if not self.robot.points_manager.get_point(point_name):
                self.last_error = f"缺少必要點位: {point_name}"
                return False
        
        if self.gripper:
            status = self.gripper.get_status()
            if not status['connected']:
                self.last_error = "PGC夾爪未連接"
                return False
            print("  PGC夾爪狀態正常")
        
        if self.ccd1:
            status = self.ccd1.get_system_status()
            if not status['connected']:
                print("  CCD1視覺系統未連接，但繼續執行")
            else:
                print("  CCD1視覺系統準備就緒")
        
        return True
    
    def _step_gripper_close(self) -> bool:
        """步驟2: 夾爪關閉 (依據CASE流程敘述.md)"""
        if not self.gripper:
            print("  跳過夾爪關閉 (夾爪未啟用)")
            return True
        
        success = self.gripper.quick_close()
        
        if success:
            print("  PGC夾爪關閉完成")
        else:
            self.last_error = "PGC夾爪關閉失敗"
        
        return success
    
    def _step_move_to_vp_topside(self) -> bool:
        """步驟3&8: 移動到vp_topside"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        if not self.robot.MovJ("vp_topside"):
            self.last_error = "移動到vp_topside失敗"
            return False
        
        self.robot.sync()
        print("  移動到vp_topside完成")
        return True
    
    def _step_ccd1_detection(self):
        """步驟4: CCD1檢測API (依據CASE流程敘述.md)"""
        if not self.ccd1:
            print("  跳過CCD1檢測 (CCD1未啟用)")
            return None
        
        print("  執行CCD1視覺檢測...")
        
        # 使用CCD1HighLevel API的FIFO佇列功能
        coord = self.ccd1.get_next_circle_world_coord()
        
        if coord:
            # 繼承vp_topside的R值
            vp_topside_point = self.robot.points_manager.get_point("vp_topside")
            if vp_topside_point and hasattr(vp_topside_point, 'r'):
                coord.r = vp_topside_point.r
                print(f"    繼承vp_topside的R值: {coord.r}°")
            else:
                coord.r = 0.0
                print(f"    使用預設R值: {coord.r}°")
            
            print(f"    檢測成功: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm, R={coord.r}°")
            print(f"    來源: FIFO佇列第{coord.id}個物體")
            return coord
        else:
            print("    未檢測到物體或佇列已空")
            return None
    
    def _step_move_to_object_same_height(self, coord) -> bool:
        """步驟5: 移動到視覺檢測到的物件座標(Z軸高度與vp_topside等高)"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        # 使用vp_topside的Z高度
        vp_topside_point = self.robot.points_manager.get_point("vp_topside")
        if not vp_topside_point:
            self.last_error = "找不到vp_topside點位"
            return False
        
        z_height = vp_topside_point.z
        r_value = getattr(coord, 'r', 0.0)
        
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, z_height, r_value):
            self.last_error = "移動到物體上方失敗"
            return False
        
        self.robot.sync()
        print(f"    移動到物體上方完成: Z={z_height}mm (與vp_topside等高)")
        return True
    
    def _step_move_to_pickup_height(self, coord) -> bool:
        """步驟6: 移動到檢測到的物件座標(Z軸到夾取位置)"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        r_value = getattr(coord, 'r', 0.0)
        
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.PICKUP_HEIGHT, r_value):
            self.last_error = "下降到夾取高度失敗"
            return False
        
        self.robot.sync()
        print(f"    下降到夾取高度完成: Z={self.PICKUP_HEIGHT}mm")
        return True
    
    def _step_gripper_open_370(self) -> bool:
        """步驟7: 夾爪撐開至370 (依據CASE流程敘述.md)"""
        if not self.gripper:
            print("  跳過夾爪撐開 (夾爪未啟用)")
            return True
        
        # 使用智能夾取，目標位置370
        if not self.gripper.smart_grip(target_position=self.GRIP_OPEN_POSITION):
            self.last_error = "夾爪撐開至370失敗"
            return False
        
        print(f"  夾爪撐開至{self.GRIP_OPEN_POSITION}完成")
        return True
    
    def _step_move_to_standby(self) -> bool:
        """步驟9&15: 移動到standby"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        if not self.robot.MovJ("standby"):
            self.last_error = "移動到standby失敗"
            return False
        
        self.robot.sync()
        print("  移動到standby完成")
        return True
    
    def _step_move_to_point(self, point_name: str) -> bool:
        """通用點位移動方法 - 用於flip系列點位"""
        if not self.robot.MovJ(point_name):
            self.last_error = f"移動到{point_name}失敗"
            return False
        
        self.robot.sync()
        time.sleep(self.POINT_DELAY)
        print(f"  移動到{point_name}完成")
        return True
    
    def _step_trigger_ccd2(self) -> str:
        """步驟16: 觸發CCD2(物件正反面辨識與輸送帶翻轉機構的IO控制)"""
        try:
            print("  正在觸發CCD2翻轉檢測系統...")
            
            # 使用機械臂dashboard_api的IO操作觸發CCD2
            # 依據CASE流程敘述.md的IO操作設計
            success = self.robot.trigger_ccd2_flip_detection()
            
            if success:
                print("  ✓ CCD2翻轉檢測已成功觸發")
                print("  ✓ 採用異步IO操作設計，手臂可立即執行其他流程")
                return "CCD2觸發成功(異步IO)"
            else:
                print("  ✗ CCD2翻轉檢測觸發失敗")
                return False
                
        except Exception as e:
            print(f"  ✗ CCD2觸發過程異常: {e}")
            return False
    
    def _set_flow1_completion_status(self) -> bool:
        """設置Flow1完成狀態到寄存器 - 只有CCD2觸發成功才設置"""
        try:
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                
                # 設置Flow1完成狀態 - 使用寄存器420
                # 1 = Flow1完成且CCD2觸發成功
                self.state_machine.modbus_client.write_register(420, 1)
                print("  ✓ Flow1完成狀態已設置 (寄存器420=1)")
                
                # 同時更新流程進度為100%
                self.state_machine.modbus_client.write_register(403, 100)
                print("  ✓ 流程進度已設置為100%")
                
                return True
            else:
                print("  ✗ 狀態機Modbus連接不可用，無法設置完成狀態")
                return False
                
        except Exception as e:
            print(f"  ✗ 設置Flow1完成狀態失敗: {e}")
            return False
    
    # =================================================================
    # 狀態查詢和控制方法
    # =================================================================
    
    def get_progress(self) -> int:
        """獲取當前進度百分比"""
        return int((self.current_step / self.total_steps) * 100)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取流程狀態"""
        return {
            "flow_id": self.flow_id,
            "is_running": self.is_running,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "progress_percent": self.get_progress(),
            "last_error": self.last_error,
            "required_points": self.REQUIRED_POINTS,
            "gripper_enabled": self.gripper is not None,
            "ccd1_enabled": self.ccd1 is not None,
            "flow_description": "VP視覺抓取 + 翻轉檢測 (依據CASE流程敘述.md)",
            "flow_sequence": [
                "standby → 夾爪關閉 → vp_topside → CCD1檢測API",
                "→ 移動到物件座標(Z軸高度與vp_topside等高)",
                "→ 移動到物件座標(Z軸到夾取位置) → 夾爪撐開至370",
                "→ 移動到vp_topside → 移動到standby",
                "→ flip_pre → flip_top → flip_down → flip_top → flip_pre",
                "→ standby → 觸發CCD2(物件正反面辨識與輸送帶翻轉機構的IO控制)"
            ],
            "key_features": [
                "CCD1HighLevel API的FIFO佇列管理",
                "GripperHighLevel API自動判斷夾取成功", 
                "CCD2異步IO操作觸發翻轉檢測",
                "智能夾取至370位置",
                "完整的flip系列翻轉檢測動作"
            ]
        }
    
    def stop(self) -> bool:
        """停止流程執行"""
        try:
            self.is_running = False
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "流程已停止"
            return True
            
        except Exception as e:
            print(f"停止流程失敗: {e}")
            return False


class Flow1Executor(DobotFlow1):
    """Flow1執行器 - 兼容性包裝器"""
    pass