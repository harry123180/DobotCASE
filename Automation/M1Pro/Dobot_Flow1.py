#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - VP震動盤視覺抓取流程 (正確修正版)
基於原始成功的sync策略，僅修正關鍵的手勢切換問題
"""

import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

# =================================================================
# 配置變數 - CCD2功能控制
# =================================================================
ENABLE_CCD2 = False  # 是否啟用CCD2翻轉檢測功能 (True=啟用, False=停用)

# =================================================================
# 配置變數 - VP工作高度
# =================================================================
VP_WORK_Z_LEVEL = 145.17  # VP工作台Z軸高度 (夾取位置)


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 17  # 預設值，實際會根據ENABLE_CCD2動態調整 (新流程17/16步)
    ccd2_triggered: bool = False
    ccd2_result: Optional[str] = None


class DobotFlow1:
    """
    VP震動盤視覺抓取流程執行器 (正確修正版)
    保持原始成功的sync策略，僅修正關鍵的手勢切換問題
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
        # 動態調整總步驟數：CCD2啟用=17步驟，停用=16步驟 (新流程)
        self.total_steps = 17 if ENABLE_CCD2 else 16
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # CCD2功能狀態
        self.ccd2_enabled = ENABLE_CCD2
        print(f"Flow1初始化: CCD2功能{'啟用' if self.ccd2_enabled else '停用'}")
        
        # 流程參數 - 依據新流程需求
        self.SPEED_RATIO = 20
        self.POINT_DELAY = 0.1
        self.CCD1_DETECT_HEIGHT = 238.86  # vp_topside的Z高度
        self.VP_WORK_Z_LEVEL = VP_WORK_Z_LEVEL  # VP工作台Z軸高度 (夾取位置)
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
        """執行VP震動盤視覺抓取流程 - 正確修正版"""
        print("\n" + "="*60)
        print("開始執行流程1 - VP視覺抓取 + 翻轉檢測流程 (正確修正版)")
        print(f"CCD2功能: {'啟用' if self.ccd2_enabled else '停用'}")
        print(f"總步驟數: {self.total_steps}")
        print(f"VP工作Z高度: {self.VP_WORK_Z_LEVEL}mm")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        ccd2_triggered = False
        ccd2_result = None
        
        detected_coord = None
        
        try:
            # 步驟1: 移動到standby (JointMovJ+sync)
            if not self._execute_step(1, "移動到standby", self._step_move_to_standby_joint):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟2: 夾爪快速關閉，並觸發CCD1檢測物件位置
            if not self._execute_step(2, "夾爪快速關閉並觸發CCD1檢測", self._step_gripper_close_and_ccd1):
                return self._create_result(False, start_time, False, None)
            
            # 步驟3: CCD1檢測結果處理
            coord_result = self._execute_step_with_return(3, "處理CCD1檢測結果", self._step_process_ccd1_result)
            if coord_result is False:
                # 沒有檢測到物件，設置警報
                self.last_error = "CCD1未檢測到物件，流程進入Alarm狀態"
                self._set_alarm_state()
                return self._create_result(False, start_time, False, None)
            detected_coord = coord_result
            
            print(f"  檢測到物體 (FIFO佇列ID: {detected_coord.id})")
            print(f"  世界座標: ({detected_coord.world_x:.2f}, {detected_coord.world_y:.2f})mm")
            
            # 步驟4: 移動到vp_topside (JointMovJ+sync) - 必須sync確保到位
            if not self._execute_step(4, "移動到vp_topside", self._step_move_to_vp_topside_joint):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟5: 移動到視覺料件座標(Z軸與vp_topside同高) (MovL+sync) - 手勢切換關鍵點
            if not self._execute_step(5, "移動到視覺料件座標(與vp_topside同高)", 
                                    lambda: self._step_move_to_object_vp_height(detected_coord)):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟6: 下降到VP工作Z高度 (MovL+sync)
            if not self._execute_step(6, f"下降到VP工作Z高度({self.VP_WORK_Z_LEVEL}mm)", 
                                    lambda: self._step_move_to_work_height(detected_coord)):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟7: 夾爪張開到370位置(智慧夾取)
            if not self._execute_step(7, "夾爪張開到370位置(智慧夾取)", self._step_smart_grip_open):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟8: 回到vp_topside (MovL+JointMovJ+sync)
            if not self._execute_step(8, "回到vp_topside", 
                                    lambda: self._step_return_to_vp_topside(detected_coord)):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟9: 到standby (JointMovJ+sync)
            if not self._execute_step(9, "到standby", self._step_move_to_standby_joint):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟10-12: 翻轉檢測序列 (全部使用JointMovJ+sync)
            flip_sequence = [
                (10, "到flip_pre", "flip_pre"),
                (11, "到flip_top", "flip_top"),
                (12, "到flip_down", "flip_down")
            ]
            
            for step_num, step_name, point_name in flip_sequence:
                if not self._execute_step(step_num, step_name, 
                                        lambda p=point_name: self._step_move_to_point_joint(p)):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟13: 夾爪快速關閉 (在flip_down位置)
            if not self._execute_step(13, "夾爪快速關閉", self._step_gripper_quick_close):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟14-16: 繼續翻轉序列
            final_flip_sequence = [
                (14, "到flip_top", "flip_top"),
                (15, "到flip_pre", "flip_pre")
            ]
            
            for step_num, step_name, point_name in final_flip_sequence:
                if not self._execute_step(step_num, step_name, 
                                        lambda p=point_name: self._step_move_to_point_joint(p)):
                    return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟16: 到standby (JointMovJ+sync)
            if not self._execute_step(16, "到standby", self._step_move_to_standby_joint):
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 步驟17: 觸發CCD2 (條件執行)
            if self.ccd2_enabled:
                ccd2_result = self._execute_step_with_return(17, "觸發CCD2翻轉檢測", self._step_trigger_ccd2)
                if ccd2_result is not False:
                    ccd2_triggered = True
                    print(f"  CCD2觸發結果: {ccd2_result}")
                else:
                    ccd2_triggered = False
                    ccd2_result = "觸發失敗"
                    print("  CCD2觸發失敗，但流程繼續")
            else:
                # CCD2停用時的處理
                print("  [步驟17] CCD2功能已停用，跳過觸發")
                ccd2_triggered = True  # 設為True表示"完成"狀態
                ccd2_result = "CCD2功能已停用"
            
            # 設置Flow1完成狀態
            if not self._set_flow1_completion_status(ccd2_triggered):
                self.last_error = "設置Flow1完成狀態失敗"
                return self._create_result(False, start_time, ccd2_triggered, ccd2_result)
            
            # 流程完成
            execution_time = time.time() - start_time
            print(f"\n✓ 流程1執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ CCD2狀態: {'觸發成功' if ccd2_triggered and self.ccd2_enabled else '已停用' if not self.ccd2_enabled else '觸發失敗'}")
            
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
    # 流程步驟實現 - 正確修正版
    # =================================================================
    
    def _step_move_to_standby_joint(self) -> bool:
        """移動到standby (JointMovJ+sync)"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        if not self.robot.MovJ("standby"):
            self.last_error = "移動到standby失敗"
            return False
        
        self.robot.sync()
        print("  移動到standby完成 (JointMovJ+sync)")
        return True
    
    def _step_gripper_close_and_ccd1(self) -> bool:
        """夾爪快速關閉，並觸發CCD1檢測物件位置"""
        # 1. 夾爪快速關閉
        if self.gripper:
            success = self.gripper.quick_close()
            if not success:
                self.last_error = "PGC夾爪快速關閉失敗"
                return False
            print("  PGC夾爪快速關閉完成")
        else:
            print("  跳過夾爪關閉 (夾爪未啟用)")
        
        # 2. 觸發CCD1檢測
        if self.ccd1:
            print("  觸發CCD1檢測...")
            # 使用CCD1HighLevel API觸發檢測
            success = self.ccd1.capture_and_detect()
            if not success:
                self.last_error = "CCD1檢測觸發失敗"
                return False
            print("  CCD1檢測已觸發")
        else:
            print("  跳過CCD1檢測 (CCD1未啟用)")
        
        return True
    
    def _step_process_ccd1_result(self):
        """處理CCD1檢測結果"""
        if not self.ccd1:
            print("  CCD1未啟用，模擬檢測結果")
            return None
        
        print("  處理CCD1檢測結果...")
        
        # 從FIFO佇列獲取檢測結果
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
            return False  # 明確返回False表示失敗
    
    def _step_move_to_vp_topside_joint(self) -> bool:
        """移動到vp_topside (JointMovJ+sync) - 關鍵：必須sync確保到位"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        if not self.robot.MovJ("vp_topside"):
            self.last_error = "移動到vp_topside失敗"
            return False
        
        # 🔥 關鍵修正：必須sync確保JointMovJ完全到位，為MovL準備正確的起始姿態
        self.robot.sync()
        print("  移動到vp_topside完成 (JointMovJ+sync)")
        
        # 🔥 額外修正：添加小延遲確保手臂穩定
        time.sleep(0.2)
        print("  手臂姿態已穩定")
        return True
    
    def _step_move_to_object_vp_height(self, coord) -> bool:
        """移動到視覺料件座標(Z軸與vp_topside同高) (MovL+sync) - 手勢切換關鍵點"""
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
        
        # 🔥 關鍵修正：確保R值與vp_topside完全一致
        if hasattr(vp_topside_point, 'r'):
            if abs(r_value - vp_topside_point.r) > 0.1:  # 容差0.1度
                print(f"    ⚠️ R值校正：{r_value}° → {vp_topside_point.r}°")
                r_value = vp_topside_point.r
        
        print(f"    準備MovL到物體上方: X={coord.world_x:.2f}, Y={coord.world_y:.2f}, Z={z_height:.2f}, R={r_value:.2f}")
        
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, z_height, r_value):
            self.last_error = "移動到物體上方失敗"
            return False
        
        # 🔥 關鍵修正：MovL完成後sync確保到位
        self.robot.sync()
        print(f"    移動到物體上方完成: Z={z_height}mm (與vp_topside同高) (MovL+sync)")
        return True
    
    def _step_move_to_work_height(self, coord) -> bool:
        """下降到VP工作Z高度 (MovL+sync)"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        r_value = getattr(coord, 'r', 0.0)
        
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.VP_WORK_Z_LEVEL, r_value):
            self.last_error = f"下降到VP工作Z高度失敗"
            return False
        
        self.robot.sync()
        print(f"    下降到VP工作Z高度完成: Z={self.VP_WORK_Z_LEVEL}mm, R={r_value}° (MovL+sync)")
        return True
    
    def _step_smart_grip_open(self) -> bool:
        """夾爪張開到370位置(智慧夾取)"""
        if not self.gripper:
            print("  跳過夾爪張開 (夾爪未啟用)")
            return True
        
        # 使用智能夾取，目標位置370
        if not self.gripper.smart_grip(target_position=self.GRIP_OPEN_POSITION):
            self.last_error = f"夾爪張開到{self.GRIP_OPEN_POSITION}失敗"
            return False
        
        print(f"  夾爪張開到{self.GRIP_OPEN_POSITION}完成 (智慧夾取)")
        return True
    
    def _step_return_to_vp_topside(self, coord) -> bool:
        """回到vp_topside (MovL+JointMovJ+sync)"""
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
        
        # 先上升到安全高度
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, z_height, r_value):
            self.last_error = "上升到安全高度失敗"
            return False
        
        # 然後移動到vp_topside
        if not self.robot.MovJ("vp_topside"):
            self.last_error = "移動到vp_topside失敗"
            return False
        
        self.robot.sync()
        print(f"    回到vp_topside完成 (MovL+JointMovJ+sync)")
        return True
    
    def _step_move_to_point_joint(self, point_name: str) -> bool:
        """通用點位移動方法 - 使用JointMovJ+sync"""
        if not self.robot.MovJ(point_name):
            self.last_error = f"移動到{point_name}失敗"
            return False
        
        self.robot.sync()
        time.sleep(self.POINT_DELAY)
        print(f"  移動到{point_name}完成 (JointMovJ+sync)")
        return True
    
    def _step_gripper_quick_close(self) -> bool:
        """夾爪快速關閉"""
        if not self.gripper:
            print("  跳過夾爪關閉 (夾爪未啟用)")
            return True
        
        success = self.gripper.quick_close()
        
        if success:
            print("  PGC夾爪快速關閉完成")
        else:
            self.last_error = "PGC夾爪快速關閉失敗"
        
        return success
    
    def _set_alarm_state(self):
        """設置系統警報狀態"""
        try:
            if (self.state_machine and 
                hasattr(self.state_machine, 'set_alarm')):
                self.state_machine.set_alarm(True)
                print("  系統已設置為Alarm狀態")
        except Exception as e:
            print(f"  設置Alarm狀態失敗: {e}")
    
    def _step_trigger_ccd2(self) -> str:
        """步驟17: 觸發CCD2(物件正反面辨識與輸送帶翻轉機構的IO控制) - 條件執行"""
        if not self.ccd2_enabled:
            print("  CCD2功能已停用，跳過觸發")
            return "CCD2功能已停用"
        
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
    
    def _set_flow1_completion_status(self, ccd2_success: bool = True) -> bool:
        """設置Flow1完成狀態到寄存器 - 修正版，支援CCD2停用情況"""
        try:
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                
                # 設置Flow1完成狀態 - 使用寄存器420
                # CCD2啟用時：1 = Flow1完成且CCD2觸發成功
                # CCD2停用時：1 = Flow1完成(跳過CCD2)
                completion_value = 1 if ccd2_success else 0
                
                self.state_machine.modbus_client.write_register(420, completion_value)
                
                if self.ccd2_enabled:
                    print(f"  ✓ Flow1完成狀態已設置 (寄存器420={completion_value}, CCD2觸發{'成功' if ccd2_success else '失敗'})")
                else:
                    print(f"  ✓ Flow1完成狀態已設置 (寄存器420={completion_value}, CCD2功能已停用)")
                
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
            "ccd2_enabled": self.ccd2_enabled,
            "hand_gesture_fix_applied": True,  # 新增：標識已修正手勢切換問題
            "flow_description": f"VP視覺抓取 + 翻轉檢測 (正確修正版, CCD2={'啟用' if self.ccd2_enabled else '停用'})",
            "flow_sequence": [
                "1. standby(sync) → 2. 夾爪快速關閉並觸發CCD1檢測",
                "3. 處理CCD1檢測結果(無物體則Alarm) → 4. vp_topside(sync+穩定)",
                "5. 視覺料件座標(MovL+sync) → 6. 下降到VP工作Z高度(MovL+sync)",
                "7. 夾爪張開到370位置(智慧夾取) → 8. 回到vp_topside(MovL+MovJ+sync)",
                "9. standby(sync) → 10-12. flip序列(JointMovJ+sync)",
                "13. 夾爪快速關閉 → 14-15. flip序列(JointMovJ+sync)",
                f"16. standby(sync) → {'17. 觸發CCD2' if self.ccd2_enabled else '跳過CCD2'}"
            ],
            "key_features": [
                "修正手勢切換錯誤：JointMovJ到MovL確保sync+穩定",
                "R值嚴格一致性檢查，容差0.1度",
                "vp_topside到位後添加0.2秒穩定延遲",
                f"VP工作Z高度: {self.VP_WORK_Z_LEVEL}mm",
                f"CCD2異步IO操作觸發翻轉檢測 ({'啟用' if self.ccd2_enabled else '停用'})",
                "智能夾取至370位置",
                "所有關鍵點都有sync確保到位",
                f"動態步驟數調整: {self.total_steps}步"
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