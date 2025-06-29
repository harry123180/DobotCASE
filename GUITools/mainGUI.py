#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mainGUI.py - 統一機台調適工具主程式
整合ui.py (UI層) 和 ux.py (UX業務邏輯層)
實現完整的MVC架構分離
"""

import sys
import os
import traceback
from typing import Dict, Any

# 添加當前目錄到路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    # 導入UI層和UX層
    from ui import (
        MainUI, 
        VPControlPanel, 
        LightControlPanel, 
        GUIClassControlPanel
    )
    from ux import UnifiedUXManager
    
    print("✓ 成功導入UI和UX模組")
except ImportError as e:
    print(f"✗ 導入失敗: {e}")
    traceback.print_exc()
    sys.exit(1)

class UnifiedMachineGUI:
    """統一機台調適工具 - 主程式控制器"""
    
    def __init__(self):
        print("統一機台調適工具啟動中...")
        
        # 初始化UX管理器 (業務邏輯層)
        self.ux_manager = UnifiedUXManager()
        
        # 初始化UI (介面層)
        self.ui = MainUI(callbacks=self._get_ui_callbacks())
        
        # 設置UX->UI回調
        self._setup_ux_callbacks()
        
        # 初始化面板
        self._init_panels()
        
        # 設置關閉回調
        self.ui.set_close_callback(self.on_closing)
        
        # 更新系統資訊
        self.ui.update_system_info(self.ux_manager.get_system_info())
        
        print("✓ 統一機台調適工具初始化完成")
    
    def _get_ui_callbacks(self) -> Dict[str, callable]:
        """獲取UI層回調函數"""
        return {
            'toggle_connection': self._on_toggle_connection,
            'show_module': self._on_show_module
        }
    
    def _setup_ux_callbacks(self):
        """設置UX層回調"""
        self.ux_manager.set_ui_callback('add_log', self.ui.add_log)
        self.ux_manager.set_ui_callback('update_connection_status', self.ui.update_connection_status)
        self.ux_manager.set_ui_callback('update_connect_button', self.ui.update_connect_button)
        self.ux_manager.set_ui_callback('add_advanced_module_button', self.ui.add_advanced_module_button)
        self.ux_manager.set_ui_callback('update_guiclass_status', self._update_guiclass_status)
    
    def _init_panels(self):
        """初始化所有控制面板"""
        # 初始化VP面板
        vp_callbacks = self.ux_manager.get_vp_callbacks()
        # 添加特殊回調來獲取震動參數
        vp_callbacks['execute_action'] = self._vp_execute_action_with_params
        
        vp_panel = VPControlPanel(self.ui.main_content, vp_callbacks)
        self.ui.add_panel('VP', vp_panel)
        
        # 註冊VP狀態更新回調
        self.ux_manager.vp_controller.add_status_callback(vp_panel.update_status)
        
        # 初始化Light面板
        light_callbacks = self.ux_manager.get_light_callbacks()
        light_panel = LightControlPanel(self.ui.main_content, light_callbacks)
        self.ui.add_panel('Light', light_panel)
        
        # 註冊Light狀態更新回調
        self.ux_manager.light_controller.add_status_callback(light_panel.update_status)
        
        # 初始化GUIClass面板
        module_list = self.ux_manager.get_module_list()
        for module_name in module_list['advanced']:
            self._create_guiclass_panel(module_name)
            self.ui.add_advanced_module_button(module_name)
        
        # 默認顯示VP面板
        self.ui.show_panel('VP')
        self.ui.add_log("介面初始化完成，預設顯示VP控制面板")
    
    def _create_guiclass_panel(self, module_name: str):
        """創建GUIClass控制面板"""
        callbacks = self.ux_manager.get_guiclass_callbacks(module_name)
        panel = GUIClassControlPanel(self.ui.main_content, module_name, callbacks)
        self.ui.add_panel(module_name, panel)
        
        # 初始化狀態顯示
        initial_status = self.ux_manager.get_controller_status_text(module_name)
        panel.update_status_display(initial_status)
    
    def _vp_execute_action_with_params(self, action: str):
        """VP執行動作 - 帶參數獲取"""
        # 從VP面板獲取當前參數
        vp_panel = self.ui.panels.get('VP')
        if vp_panel:
            vibration_params = vp_panel.get_vibration_params()
            self.ux_manager._vp_execute_action(action, vibration_params)
        else:
            self.ux_manager._vp_execute_action(action)
    
    def _on_toggle_connection(self):
        """處理連接切換"""
        connection_info = self.ui.get_connection_info()
        host = connection_info['host']
        port = connection_info['port']
        
        self.ui.add_log(f"嘗試連接到 {host}:{port}")
        
        # 執行連接
        results = self.ux_manager.toggle_connection(host, port)
        
        # 更新UI狀態
        self.ui.update_connection_status(results['vp'], results['light'])
        
        total_connected = sum([results['vp'], results['light']]) + sum(1 for item in results['advanced'] if item['connected'])
        self.ui.update_connect_button(total_connected > 0)
    
    def _on_show_module(self, module_name: str):
        """處理模組切換"""
        self.ui.show_panel(module_name)
        self.ui.add_log(f"切換到{module_name}控制面板")
        
        # 如果是GUIClass模組，更新狀態顯示
        if module_name in self.ux_manager.guiclass_controllers:
            self._update_guiclass_status_delayed(module_name)
    
    def _update_guiclass_status_delayed(self, module_name: str):
        """延遲更新GUIClass狀態 - 避免切換時的狀態衝突"""
        def delayed_update():
            import time
            time.sleep(0.1)  # 短暫延遲
            status_text = self.ux_manager.get_controller_status_text(module_name)
            panel = self.ui.panels.get(module_name)
            if panel:
                panel.update_status_display(status_text)
        
        import threading
        threading.Thread(target=delayed_update, daemon=True).start()
    
    def _update_guiclass_status(self, module_name: str, status_text: str):
        """更新GUIClass狀態顯示"""
        panel = self.ui.panels.get(module_name)
        if panel and hasattr(panel, 'update_status_display'):
            panel.update_status_display(status_text)
    
    def run(self):
        """運行主程式"""
        try:
            self.ui.add_log("統一機台調適工具啟動完成")
            self.ui.add_log("請設定Modbus連接參數並點擊連接")
            
            # 啟動UI主循環
            self.ui.mainloop()
            
        except KeyboardInterrupt:
            self.ui.add_log("收到中斷信號，正在關閉...")
            self.on_closing()
        except Exception as e:
            print(f"運行異常: {e}")
            traceback.print_exc()
            self.on_closing()
    
    def on_closing(self):
        """程式關閉處理"""
        try:
            self.ui.add_log("系統正在關閉...")
            
            # 斷開所有連接
            self.ux_manager.disconnect_all()
            
            # 銷毀UI
            self.ui.destroy()
            
            print("統一機台調適工具已安全關閉")
            
        except Exception as e:
            print(f"關閉過程異常: {e}")
        finally:
            # 強制退出
            import os
            os._exit(0)

def main():
    """主函數"""
    try:
        # 創建並運行主程式
        app = UnifiedMachineGUI()
        app.run()
        
    except Exception as e:
        print(f"程式啟動失敗: {e}")
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()