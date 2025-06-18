# -*- coding: utf-8 -*-
"""
ModbusTestTool_Fixed.py - CCD1[U+8996][U+89BA][U+7CFB][U+7D71] Modbus TCP [U+6E2C][U+8A66][U+5DE5][U+5177] ([U+4FEE][U+6B63][U+7248])
[U+57FA][U+65BC]CustomTkinter[U+7684]GUI[U+5DE5][U+5177][U+FF0C][U+7528][U+65BC][U+6A21][U+64EC]PLC[U+64CD][U+4F5C][U+548C][U+76E3][U+63A7][U+7CFB][U+7D71][U+72C0][U+614B]
"""

import customtkinter as ctk
import threading
import time
import json
from typing import Optional, Dict, Any
import tkinter as tk
from tkinter import messagebox, scrolledtext

# [U+5C0E][U+5165]Modbus TCP Client
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    print("[OK] Modbus Client[U+6A21][U+7D44][U+53EF][U+7528]")
except ImportError as e:
    print(f"[WARN][U+FE0F] Modbus Client[U+6A21][U+7D44][U+4E0D][U+53EF][U+7528]: {e}")
    MODBUS_AVAILABLE = False

# [U+8A2D][U+7F6E][U+5916][U+89C0][U+6A21][U+5F0F]
ctk.set_appearance_mode("light")  # "light" [U+6216] "dark"
ctk.set_default_color_theme("blue")  # "blue", "green", "dark-blue"


class ModbusTestTool:
    """Modbus TCP [U+6E2C][U+8A66][U+5DE5][U+5177][U+4E3B][U+985E]"""
    
    def __init__(self):
        # [U+5275][U+5EFA][U+4E3B][U+7A97][U+53E3]
        self.root = ctk.CTk()
        self.root.title("[U+1F91D] CCD1[U+8996][U+89BA][U+7CFB][U+7D71] - Modbus TCP [U+6E2C][U+8A66][U+5DE5][U+5177]")
        self.root.geometry("1200x800")
        self.root.resizable(True, True)
        
        # Modbus[U+9023][U+63A5][U+53C3][U+6578]
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # [U+76E3][U+63A7][U+7DDA][U+7A0B][U+63A7][U+5236]
        self.monitoring = False
        self.monitor_thread = None
        self.update_interval = 1.0  # 1[U+79D2][U+66F4][U+65B0][U+9593][U+9694]
        
        # [U+5BC4][U+5B58][U+5668][U+6620][U+5C04] ([U+8207]CCD1VisionCode.py[U+4FDD][U+6301][U+4E00][U+81F4])
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,        # [U+63A7][U+5236][U+6307][U+4EE4][U+5BC4][U+5B58][U+5668]
            'STATUS_REGISTER': 201,        # [U+72C0][U+614B][U+5BC4][U+5B58][U+5668]
            'MIN_AREA_HIGH': 210,          # [U+6700][U+5C0F][U+9762][U+7A4D] ([U+9AD8]16[U+4F4D])
            'MIN_AREA_LOW': 211,           # [U+6700][U+5C0F][U+9762][U+7A4D] ([U+4F4E]16[U+4F4D])
            'MIN_ROUNDNESS': 212,          # [U+6700][U+5C0F][U+5713][U+5EA6] ([U+00D7]1000)
            'GAUSSIAN_KERNEL': 213,        # [U+9AD8][U+65AF][U+6838][U+5927][U+5C0F]
            'CANNY_LOW': 214,              # Canny[U+4F4E][U+95BE][U+503C]
            'CANNY_HIGH': 215,             # Canny[U+9AD8][U+95BE][U+503C]
            'CIRCLE_COUNT': 240,           # [U+6AA2][U+6E2C][U+5713][U+5F62][U+6578][U+91CF]
            'CIRCLE_1_X': 241,             # [U+5713][U+5F62]1 X[U+5EA7][U+6A19]
            'CIRCLE_1_Y': 242,             # [U+5713][U+5F62]1 Y[U+5EA7][U+6A19]
            'CIRCLE_1_RADIUS': 243,        # [U+5713][U+5F62]1 [U+534A][U+5F91]
            'CIRCLE_2_X': 244,             # [U+5713][U+5F62]2 X[U+5EA7][U+6A19]
            'CIRCLE_2_Y': 245,             # [U+5713][U+5F62]2 Y[U+5EA7][U+6A19]
            'CIRCLE_2_RADIUS': 246,        # [U+5713][U+5F62]2 [U+534A][U+5F91]
            'CIRCLE_3_X': 247,             # [U+5713][U+5F62]3 X[U+5EA7][U+6A19]
            'CIRCLE_3_Y': 248,             # [U+5713][U+5F62]3 Y[U+5EA7][U+6A19]
            'CIRCLE_3_RADIUS': 249,        # [U+5713][U+5F62]3 [U+534A][U+5F91]
            'OPERATION_COUNT': 283,        # [U+64CD][U+4F5C][U+8A08][U+6578][U+5668]
            'ERROR_COUNT': 284,            # [U+932F][U+8AA4][U+8A08][U+6578][U+5668]
            'LAST_CAPTURE_TIME': 280,      # [U+6700][U+5F8C][U+62CD][U+7167][U+8017][U+6642]
            'LAST_PROCESS_TIME': 281,      # [U+6700][U+5F8C][U+8655][U+7406][U+8017][U+6642]
            'LAST_TOTAL_TIME': 282,        # [U+6700][U+5F8C][U+7E3D][U+8017][U+6642]
        }
        
        # [U+72C0][U+614B][U+8B8A][U+91CF]
        self.status_bits = {'ready': 0, 'running': 0, 'alarm': 0, 'initialized': 0}
        self.detection_results = {'circle_count': 0, 'circles': []}
        self.last_values = {}  # [U+7528][U+65BC][U+6AA2][U+6E2C][U+6578][U+503C][U+8B8A][U+5316]
        
        # [U+56FA][U+5B9A][U+72C0][U+614B][U+503C][U+5B9A][U+7FA9] ([U+5C0D][U+61C9]CCD1VisionCode_Enhanced.py)
        self.STATUS_VALUES = {
            0: "[U+5168][U+90E8][U+505C][U+6B62] (0000)",      # [U+6240][U+6709][U+4F4D][U+90FD][U+662F]0
            1: "[U+6E96][U+5099][U+5C31][U+7DD2] (0001)",      # Ready=1 ([U+521D][U+59CB][U+72C0][U+614B])
            2: "[U+57F7][U+884C][U+4E2D]_A (0010)",      # Running=1
            3: "[U+57F7][U+884C][U+4E2D]_B (0011)",      # Ready=1, Running=1
            4: "[U+7CFB][U+7D71][U+7570][U+5E38] (0100)",      # Alarm=1
            5: "[U+7570][U+5E38][U+6E96][U+5099] (0101)",      # Ready=1, Alarm=1
            8: "[U+5DF2][U+521D][U+59CB][U+5316] (1000)",      # Initialized=1
            9: "[U+5B8C][U+5168][U+5C31][U+7DD2] (1001)",      # Ready=1, Initialized=1 ([U+6B63][U+5E38][U+5DE5][U+4F5C][U+72C0][U+614B])
            10: "[U+521D][U+59CB][U+57F7][U+884C] (1010)",     # Running=1, Initialized=1
            11: "[U+5168][U+90E8][U+555F][U+52D5] (1011)",     # Ready=1, Running=1, Initialized=1
            12: "[U+521D][U+59CB][U+7570][U+5E38] (1100)",     # Alarm=1, Initialized=1
        }
        
        # [U+9023][U+63A5][U+72C0][U+614B][U+6A19][U+8A18]
        self.connection_error_message = ""
        self.auto_connect_attempted = False
        
        # [U+5275][U+5EFA]GUI
        self.create_widgets()
        
        # [U+7D81][U+5B9A][U+95DC][U+9589][U+4E8B][U+4EF6]
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # [U+5EF6][U+9072][U+81EA][U+52D5][U+9023][U+63A5] ([U+7B49][U+5F85]GUI[U+5B8C][U+5168][U+52A0][U+8F09])
        self.root.after(1000, self.auto_connect_on_startup)

    def auto_connect_on_startup(self):
        """[U+7A0B][U+5E8F][U+555F][U+52D5][U+6642][U+81EA][U+52D5][U+9023][U+63A5]"""
        if not MODBUS_AVAILABLE:
            self.connection_error_message = "Modbus[U+6A21][U+7D44][U+4E0D][U+53EF][U+7528][U+FF0C][U+8ACB][U+5B89][U+88DD]pymodbus"
            self.conn_status.configure(text="[FAIL] [U+6A21][U+7D44][U+932F][U+8AA4]", text_color="red")
            self.conn_detail.configure(text=self.connection_error_message)
            self.log_message("[FAIL] " + self.connection_error_message)
            return
        
        self.log_message("[U+1F504] [U+7A0B][U+5E8F][U+555F][U+52D5][U+FF0C][U+5617][U+8A66][U+81EA][U+52D5][U+9023][U+63A5][U+5230] 127.0.0.1:502...")
        self.auto_connect_attempted = True
        
        try:
            # [U+66F4][U+65B0][U+72C0][U+614B][U+986F][U+793A]
            self.conn_status.configure(text="[U+1F504] [U+81EA][U+52D5][U+9023][U+63A5][U+4E2D]...", text_color="orange")
            self.conn_detail.configure(text="[U+6B63][U+5728][U+5617][U+8A66][U+9023][U+63A5][U+5230]Modbus TCP[U+670D][U+52D9][U+5668]...")
            
            # [U+78BA][U+4FDD][U+4F7F][U+7528][U+6B63][U+78BA][U+7684]IP[U+548C][U+7AEF][U+53E3]
            self.server_ip = "127.0.0.1"
            self.server_port = 502
            
            # [U+5617][U+8A66][U+9023][U+63A5]
            success = self._attempt_connection()
            
            if success:
                self.log_message("[OK] [U+81EA][U+52D5][U+9023][U+63A5][U+6210][U+529F][U+FF0C][U+958B][U+59CB][U+76E3][U+63A7]")
                # [U+7ACB][U+5373][U+8B80][U+53D6][U+4E00][U+6B21][U+72C0][U+614B]
                self.root.after(500, self.initial_status_read)
            else:
                self._show_connection_error()
                
        except Exception as e:
            self.connection_error_message = f"[U+81EA][U+52D5][U+9023][U+63A5][U+7570][U+5E38]: {str(e)}"
            self._show_connection_error()
    
    def _attempt_connection(self) -> bool:
        """[U+5617][U+8A66][U+5EFA][U+7ACB][U+9023][U+63A5]"""
        try:
            # [U+5148][U+95DC][U+9589][U+73FE][U+6709][U+9023][U+63A5][U+FF08][U+5982][U+679C][U+6709][U+FF09]
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None
            
            self.log_message(f"[U+1F517] [U+5617][U+8A66][U+9023][U+63A5][U+5230] {self.server_ip}:{self.server_port}")
            
            # pymodbus 3.9.2 [U+7684][U+57FA][U+672C][U+53C3][U+6578][U+FF08][U+79FB][U+9664][U+4E0D][U+652F][U+63F4][U+7684][U+53C3][U+6578][U+FF09]
            self.client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=5,
                retries=3,
                reconnect_delay=1,
                reconnect_delay_max=10
            )
            
            self.log_message("[OK] [U+4F7F][U+7528] pymodbus 3.9.2 [U+53C3][U+6578][U+5275][U+5EFA][U+5BA2][U+6236][U+7AEF]")
            
            # [U+5617][U+8A66][U+9023][U+63A5]
            connect_result = self.client.connect()
            self.log_message(f"[U+1F517] [U+9023][U+63A5][U+7D50][U+679C]: {connect_result}")
            
            if connect_result:
                self.connected = True
                self.log_message("[OK] TCP[U+9023][U+63A5][U+5EFA][U+7ACB][U+6210][U+529F]")
                
                # [U+7B49][U+5F85][U+4E00][U+4E0B][U+8B93][U+9023][U+63A5][U+7A69][U+5B9A]
                time.sleep(0.1)
                
                # [U+6E2C][U+8A66][U+8B80][U+53D6][U+72C0][U+614B][U+5BC4][U+5B58][U+5668] - [U+4F7F][U+7528] pymodbus 3.x [U+7684][U+65B9][U+5F0F]
                self.log_message(f"[U+1F527] [U+6E2C][U+8A66][U+8B80][U+53D6][U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+5730][U+5740]: {self.REGISTERS['STATUS_REGISTER']}")
                
                test_result = self.client.read_holding_registers(
                    address=self.REGISTERS['STATUS_REGISTER'], 
                    count=1, 
                    slave=1  # pymodbus 3.x [U+4ECD][U+7136][U+652F][U+6301] slave [U+53C3][U+6578]
                )
                
                if test_result.isError():
                    self.log_message(f"[FAIL] [U+8B80][U+53D6][U+6E2C][U+8A66][U+5931][U+6557]: {test_result}")
                    self.connection_error_message = f"[U+9023][U+63A5][U+6210][U+529F][U+4F46][U+8B80][U+53D6][U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+5931][U+6557]: {test_result}"
                    self.client.close()
                    self.connected = False
                    return False
                else:
                    status_value = test_result.registers[0]
                    self.log_message(f"[OK] [U+8B80][U+53D6][U+6E2C][U+8A66][U+6210][U+529F][U+FF0C][U+72C0][U+614B][U+503C]: {status_value}")
                    self.conn_status.configure(text="[OK] [U+5DF2][U+9023][U+63A5]", text_color="green")
                    self.conn_detail.configure(text=f"[U+6210][U+529F][U+9023][U+63A5][U+5230] {self.server_ip}:{self.server_port}[U+FF0C][U+72C0][U+614B][U+5BC4][U+5B58][U+5668]: {status_value}")
                    
                    # [U+555F][U+7528][U+63A7][U+5236][U+6309][U+9215]
                    self.enable_control_buttons(True)
                    
                    # [U+958B][U+59CB][U+76E3][U+63A7]
                    if not self.monitoring:
                        self.start_monitoring()
                    
                    return True
                    
            else:
                self.connection_error_message = f"[U+7121][U+6CD5][U+5EFA][U+7ACB]TCP[U+9023][U+63A5][U+5230] {self.server_ip}:{self.server_port}"
                self.log_message(f"[FAIL] {self.connection_error_message}")
                return False
                
        except Exception as e:
            self.connection_error_message = f"[U+9023][U+63A5][U+932F][U+8AA4]: {str(e)}"
            self.log_message(f"[FAIL] [U+9023][U+63A5][U+6642][U+767C][U+751F][U+932F][U+8AA4]: {str(e)}")
            self.log_message(f"[FAIL] [U+932F][U+8AA4][U+985E][U+578B]: {type(e).__name__}")
            import traceback
            self.log_message(f"[FAIL] [U+8A73][U+7D30][U+932F][U+8AA4]: {traceback.format_exc()}")
            return False
    
    def _show_connection_error(self):
        """[U+986F][U+793A][U+9023][U+63A5][U+932F][U+8AA4][U+4FE1][U+606F]"""
        self.conn_status.configure(text="[FAIL] [U+9023][U+63A5][U+5931][U+6557]", text_color="red")
        self.conn_detail.configure(text=self.connection_error_message)
        self.log_message(f"[FAIL] {self.connection_error_message}")
        self.log_message("[U+1F4A1] [U+8ACB][U+78BA][U+8A8D]:")
        self.log_message("   1. CCD1VisionCode_Enhanced.py [U+662F][U+5426][U+6B63][U+5728][U+904B][U+884C]")
        self.log_message("   2. Modbus TCP[U+670D][U+52D9][U+5668][U+662F][U+5426][U+5728] 127.0.0.1:502 [U+76E3][U+807D]")
        self.log_message("   3. [U+9632][U+706B][U+7246][U+662F][U+5426][U+963B][U+64CB][U+9023][U+63A5]")
    
    def initial_status_read(self):
        """[U+521D][U+59CB][U+72C0][U+614B][U+8B80][U+53D6]"""
        if self.connected:
            threading.Thread(target=self._initial_read_worker, daemon=True).start()
    
    def _initial_read_worker(self):
        """[U+521D][U+59CB][U+8B80][U+53D6][U+5DE5][U+4F5C][U+7DDA][U+7A0B]"""
        try:
            self.read_status_register()
            self.read_detection_results()
            self.read_statistics()
        except Exception as e:
            self.log_message(f"[FAIL] [U+521D][U+59CB][U+72C0][U+614B][U+8B80][U+53D6][U+5931][U+6557]: {str(e)}")
    
    def create_widgets(self):
        """[U+5275][U+5EFA]GUI[U+7D44][U+4EF6]"""
        # [U+5275][U+5EFA][U+4E3B][U+6846][U+67B6]
        main_frame = ctk.CTkFrame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # [U+9802][U+90E8][U+6A19][U+984C]
        title_label = ctk.CTkLabel(main_frame, text="[U+1F91D] [U+904B][U+52D5][U+63A7][U+5236][U+63E1][U+624B][U+6E2C][U+8A66][U+5DE5][U+5177]", 
                                 font=ctk.CTkFont(size=24, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # [U+5275][U+5EFA][U+5DE6][U+53F3][U+5206][U+6B04]
        content_frame = ctk.CTkFrame(main_frame)
        content_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # [U+5DE6][U+5074][U+63A7][U+5236][U+9762][U+677F]
        left_frame = ctk.CTkFrame(content_frame)
        left_frame.pack(side="left", fill="y", padx=(0, 10), pady=10)
        left_frame.configure(width=400)
        
        # [U+53F3][U+5074][U+76E3][U+63A7][U+9762][U+677F]
        right_frame = ctk.CTkFrame(content_frame)
        right_frame.pack(side="right", fill="both", expand=True, padx=(10, 0), pady=10)
        
        # [U+5275][U+5EFA][U+5DE6][U+5074][U+9762][U+677F][U+5167][U+5BB9]
        self.create_control_panel(left_frame)
        
        # [U+5275][U+5EFA][U+53F3][U+5074][U+9762][U+677F][U+5167][U+5BB9]
        self.create_monitor_panel(right_frame)
    
    def create_control_panel(self, parent):
        """[U+5275][U+5EFA][U+63A7][U+5236][U+9762][U+677F]"""
        # === [U+9023][U+63A5][U+63A7][U+5236][U+5340][U+57DF] ===
        conn_frame = ctk.CTkFrame(parent)
        conn_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(conn_frame, text="[U+1F517] Modbus TCP [U+9023][U+63A5]", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # IP[U+5730][U+5740][U+8F38][U+5165]
        ip_frame = ctk.CTkFrame(conn_frame)
        ip_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(ip_frame, text="[U+670D][U+52D9][U+5668]IP:").pack(side="left", padx=(10, 5))
        self.ip_entry = ctk.CTkEntry(ip_frame, placeholder_text="127.0.0.1")
        self.ip_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.ip_entry.insert(0, self.server_ip)
        
        # [U+7AEF][U+53E3][U+8F38][U+5165]
        port_frame = ctk.CTkFrame(conn_frame)
        port_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(port_frame, text="[U+7AEF][U+53E3]:").pack(side="left", padx=(10, 5))
        self.port_entry = ctk.CTkEntry(port_frame, placeholder_text="502", width=100)
        self.port_entry.pack(side="left", padx=5)
        self.port_entry.insert(0, str(self.server_port))
        
        # [U+9023][U+63A5][U+6309][U+9215]
        conn_btn_frame = ctk.CTkFrame(conn_frame)
        conn_btn_frame.pack(fill="x", padx=10, pady=10)
        
        self.connect_btn = ctk.CTkButton(conn_btn_frame, text="[U+1F517] [U+9023][U+63A5]", 
                                       command=self.toggle_connection, width=100)
        self.connect_btn.pack(side="left", padx=5)
        
        self.test_btn = ctk.CTkButton(conn_btn_frame, text="[U+1F527] [U+6E2C][U+8A66]", 
                                    command=self.test_connection, width=100)
        self.test_btn.pack(side="left", padx=5)
        
        # [U+9023][U+63A5][U+72C0][U+614B][U+6307][U+793A] - [U+589E][U+5F37][U+7248]
        status_container = ctk.CTkFrame(conn_frame)
        status_container.pack(fill="x", padx=10, pady=5)
        
        self.conn_status = ctk.CTkLabel(status_container, text="[FAIL] [U+672A][U+9023][U+63A5]", 
                                      text_color="red", font=ctk.CTkFont(size=12, weight="bold"))
        self.conn_status.pack(pady=5)
        
        # [U+9023][U+63A5][U+8A73][U+7D30][U+4FE1][U+606F][U+986F][U+793A]
        self.conn_detail = ctk.CTkLabel(status_container, text="", 
                                      text_color="gray", font=ctk.CTkFont(size=10),
                                      wraplength=350)
        self.conn_detail.pack(pady=2)
        
        # === [U+63A7][U+5236][U+6307][U+4EE4][U+5340][U+57DF] ===
        cmd_frame = ctk.CTkFrame(parent)
        cmd_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(cmd_frame, text="[U+1F3AE] [U+63A7][U+5236][U+6307][U+4EE4]", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # [U+63A7][U+5236][U+6309][U+9215]
        cmd_btn_frame = ctk.CTkFrame(cmd_frame)
        cmd_btn_frame.pack(fill="x", padx=10, pady=10)
        
        self.clear_btn = ctk.CTkButton(cmd_btn_frame, text="[U+1F5D1][U+FE0F] [U+6E05][U+7A7A] (0)", 
                                     command=lambda: self.send_command(0), width=160)
        self.clear_btn.pack(pady=2)
        
        self.capture_btn = ctk.CTkButton(cmd_btn_frame, text="[U+1F4F8] [U+62CD][U+7167] (8)", 
                                       command=lambda: self.send_command(8), width=160)
        self.capture_btn.pack(pady=2)
        
        self.detect_btn = ctk.CTkButton(cmd_btn_frame, text="[U+1F50D] [U+62CD][U+7167]+[U+6AA2][U+6E2C] (16)", 
                                      command=lambda: self.send_command(16), width=160)
        self.detect_btn.pack(pady=2)
        
        self.init_btn = ctk.CTkButton(cmd_btn_frame, text="[U+1F504] [U+91CD][U+65B0][U+521D][U+59CB][U+5316] (32)", 
                                    command=lambda: self.send_command(32), width=160)
        self.init_btn.pack(pady=2)
        
        # === [U+8996][U+89BA][U+53C3][U+6578][U+8A2D][U+5B9A][U+5340][U+57DF] ===
        param_frame = ctk.CTkFrame(parent)
        param_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(param_frame, text="[U+2699][U+FE0F] [U+8996][U+89BA][U+6AA2][U+6E2C][U+53C3][U+6578]", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # [U+53C3][U+6578][U+8F38][U+5165]
        self.create_param_inputs(param_frame)
        
        # [U+53C3][U+6578][U+6309][U+9215]
        param_btn_frame = ctk.CTkFrame(param_frame)
        param_btn_frame.pack(fill="x", padx=10, pady=10)
        
        self.read_params_btn = ctk.CTkButton(param_btn_frame, text="[U+1F4D6] [U+8B80][U+53D6][U+53C3][U+6578]", 
                                           command=self.read_parameters, width=100)
        self.read_params_btn.pack(side="left", padx=5)
        
        self.write_params_btn = ctk.CTkButton(param_btn_frame, text="[U+1F4BE] [U+5BEB][U+5165][U+53C3][U+6578]", 
                                            command=self.write_parameters, width=100)
        self.write_params_btn.pack(side="left", padx=5)
    
    def create_param_inputs(self, parent):
        """[U+5275][U+5EFA][U+53C3][U+6578][U+8F38][U+5165][U+7D44][U+4EF6]"""
        params_container = ctk.CTkFrame(parent)
        params_container.pack(fill="x", padx=10, pady=5)
        
        # [U+6700][U+5C0F][U+9762][U+7A4D]
        area_frame = ctk.CTkFrame(params_container)
        area_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(area_frame, text="[U+6700][U+5C0F][U+9762][U+7A4D]:", width=80).pack(side="left", padx=5)
        self.area_entry = ctk.CTkEntry(area_frame, placeholder_text="50000", width=100)
        self.area_entry.pack(side="left", padx=5)
        
        # [U+6700][U+5C0F][U+5713][U+5EA6]
        roundness_frame = ctk.CTkFrame(params_container)
        roundness_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(roundness_frame, text="[U+6700][U+5C0F][U+5713][U+5EA6]:", width=80).pack(side="left", padx=5)
        self.roundness_entry = ctk.CTkEntry(roundness_frame, placeholder_text="0.8", width=100)
        self.roundness_entry.pack(side="left", padx=5)
        
        # [U+9AD8][U+65AF][U+6838][U+5927][U+5C0F]
        gaussian_frame = ctk.CTkFrame(params_container)
        gaussian_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(gaussian_frame, text="[U+9AD8][U+65AF][U+6838]:", width=80).pack(side="left", padx=5)
        self.gaussian_entry = ctk.CTkEntry(gaussian_frame, placeholder_text="9", width=100)
        self.gaussian_entry.pack(side="left", padx=5)
        
        # Canny[U+95BE][U+503C]
        canny_frame = ctk.CTkFrame(params_container)
        canny_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(canny_frame, text="Canny[U+4F4E]:", width=80).pack(side="left", padx=5)
        self.canny_low_entry = ctk.CTkEntry(canny_frame, placeholder_text="20", width=60)
        self.canny_low_entry.pack(side="left", padx=2)
        ctk.CTkLabel(canny_frame, text="[U+9AD8]:", width=30).pack(side="left", padx=2)
        self.canny_high_entry = ctk.CTkEntry(canny_frame, placeholder_text="60", width=60)
        self.canny_high_entry.pack(side="left", padx=2)
    
    def create_monitor_panel(self, parent):
        """[U+5275][U+5EFA][U+76E3][U+63A7][U+9762][U+677F]"""
        # [U+5275][U+5EFA][U+53EF][U+6EFE][U+52D5][U+7684][U+4E3B][U+6846][U+67B6]
        main_scroll_frame = ctk.CTkScrollableFrame(parent)
        main_scroll_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # === [U+7CFB][U+7D71][U+72C0][U+614B][U+5340][U+57DF] ===
        status_frame = ctk.CTkFrame(main_scroll_frame)
        status_frame.pack(fill="x", padx=5, pady=10)
        
        ctk.CTkLabel(status_frame, text="[U+1F4CA] [U+7CFB][U+7D71][U+72C0][U+614B][U+6A5F]", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # [U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+503C][U+986F][U+793A] - [U+66F4][U+9192][U+76EE][U+7684][U+8A2D][U+8A08]
        status_header_frame = ctk.CTkFrame(status_frame)
        status_header_frame.pack(fill="x", padx=10, pady=5)
        
        # [U+5927][U+5B57][U+9AD4][U+986F][U+793A][U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+503C]
        status_value_frame = ctk.CTkFrame(status_header_frame)
        status_value_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(status_value_frame, text="[U+72C0][U+614B][U+5BC4][U+5B58][U+5668] ([U+5730][U+5740]201):", 
                    font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=5)
        
        self.status_reg_label = ctk.CTkLabel(status_value_frame, text="0", 
                                           font=ctk.CTkFont(size=28, weight="bold"),
                                           text_color="blue")
        self.status_reg_label.pack(side="left", padx=10)
        
        self.status_binary_label = ctk.CTkLabel(status_value_frame, text="(0000)", 
                                              font=ctk.CTkFont(size=16),
                                              text_color="gray")
        self.status_binary_label.pack(side="left", padx=5)
        
        # [U+986F][U+793A][U+8B80][U+53D6][U+5730][U+5740][U+4FE1][U+606F]
        address_info_frame = ctk.CTkFrame(status_header_frame)
        address_info_frame.pack(fill="x", padx=5, pady=2)
        
        self.address_info_label = ctk.CTkLabel(address_info_frame, 
                                             text=f"[U+8B80][U+53D6][U+5730][U+5740]: {self.REGISTERS['STATUS_REGISTER']} (STATUS_REGISTER)", 
                                             font=ctk.CTkFont(size=11),
                                             text_color="darkblue")
        self.address_info_label.pack(anchor="w", padx=5)
        
        # [U+72C0][U+614B][U+63CF][U+8FF0]
        self.status_desc_label = ctk.CTkLabel(status_header_frame, text="[U+7CFB][U+7D71][U+672A][U+9023][U+63A5]", 
                                            font=ctk.CTkFont(size=14, weight="bold"),
                                            text_color="gray")
        self.status_desc_label.pack(pady=5)
        
        # [U+72C0][U+614B][U+503C][U+6B77][U+53F2][U+8A18][U+9304]
        history_frame = ctk.CTkFrame(status_frame)
        history_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(history_frame, text="[U+1F4CB] [U+5E38][U+898B][U+72C0][U+614B][U+503C][U+542B][U+7FA9]:", 
                    font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=5, pady=2)
        
        # [U+5E38][U+898B][U+72C0][U+614B][U+503C][U+8AAA][U+660E] - [U+8868][U+683C][U+5F62][U+5F0F]
        states_info_frame = ctk.CTkFrame(history_frame)
        states_info_frame.pack(fill="x", padx=5, pady=5)
        
        common_states = [
            ("1", "[U+6E96][U+5099][U+5C31][U+7DD2]", "Modbus[U+9023][U+63A5][U+5B8C][U+6210]", "orange"),
            ("9", "[U+5B8C][U+5168][U+5C31][U+7DD2]", "[U+76F8][U+6A5F]+Modbus[U+6B63][U+5E38]", "green"),
            ("10", "[U+57F7][U+884C][U+4E2D]", "[U+6B63][U+5728][U+8655][U+7406][U+6307][U+4EE4]", "blue"),
            ("4", "[U+7CFB][U+7D71][U+7570][U+5E38]", "[U+6709][U+932F][U+8AA4][U+767C][U+751F]", "red")
        ]
        
        for i, (value, name, desc, color) in enumerate(common_states):
            state_row = ctk.CTkFrame(states_info_frame)
            state_row.pack(fill="x", padx=2, pady=1)
            
            ctk.CTkLabel(state_row, text=value, 
                        font=ctk.CTkFont(size=12, weight="bold"),
                        text_color=color, width=30).pack(side="left", padx=5)
            ctk.CTkLabel(state_row, text=name, 
                        font=ctk.CTkFont(size=11, weight="bold"),
                        width=80).pack(side="left", padx=5)
            ctk.CTkLabel(state_row, text=desc, 
                        font=ctk.CTkFont(size=10),
                        text_color="gray").pack(side="left", padx=5)
        
        # [U+5BC4][U+5B58][U+5668][U+6620][U+5C04][U+4FE1][U+606F][U+986F][U+793A]
        registers_info_frame = ctk.CTkFrame(history_frame)
        registers_info_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(registers_info_frame, text="[U+1F4CB] [U+4E3B][U+8981][U+5BC4][U+5B58][U+5668][U+5730][U+5740]:", 
                    font=ctk.CTkFont(size=11, weight="bold")).pack(anchor="w", padx=5)
        
        # [U+986F][U+793A][U+95DC][U+9375][U+5BC4][U+5B58][U+5668][U+5730][U+5740]
        key_registers = [
            ("[U+63A7][U+5236][U+6307][U+4EE4]", "CONTROL_COMMAND", 200),
            ("[U+72C0][U+614B][U+5BC4][U+5B58][U+5668]", "STATUS_REGISTER", 201),
            ("[U+5713][U+5F62][U+6578][U+91CF]", "CIRCLE_COUNT", 240),
            ("[U+64CD][U+4F5C][U+8A08][U+6578]", "OPERATION_COUNT", 283)
        ]
        
        for name, var_name, addr in key_registers:
            reg_row = ctk.CTkFrame(registers_info_frame)
            reg_row.pack(fill="x", padx=2, pady=1)
            
            color = "red" if var_name == "STATUS_REGISTER" else "gray"
            weight = "bold" if var_name == "STATUS_REGISTER" else "normal"
            
            ctk.CTkLabel(reg_row, text=f"[U+5730][U+5740]{addr}:", 
                        font=ctk.CTkFont(size=10, weight=weight),
                        text_color=color, width=60).pack(side="left", padx=2)
            ctk.CTkLabel(reg_row, text=name, 
                        font=ctk.CTkFont(size=10, weight=weight),
                        text_color=color, width=60).pack(side="left", padx=2)
            ctk.CTkLabel(reg_row, text=f"({var_name})", 
                        font=ctk.CTkFont(size=9),
                        text_color="darkgray").pack(side="left", padx=2)
        
        # [U+56DB][U+500B][U+72C0][U+614B][U+4F4D][U+986F][U+793A]
        status_bits_frame = ctk.CTkFrame(status_frame)
        status_bits_frame.pack(fill="x", padx=10, pady=10)
        
        # [U+5275][U+5EFA][U+56DB][U+500B][U+72C0][U+614B][U+4F4D][U+986F][U+793A]
        self.create_status_bits(status_bits_frame)
        
        # === [U+6AA2][U+6E2C][U+7D50][U+679C][U+5340][U+57DF] ===
        results_frame = ctk.CTkFrame(main_scroll_frame)
        results_frame.pack(fill="x", padx=5, pady=10)
        
        ctk.CTkLabel(results_frame, text="[U+1F3AF] [U+6AA2][U+6E2C][U+7D50][U+679C]", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # [U+7D50][U+679C][U+7D71][U+8A08]
        self.create_results_summary(results_frame)
        
        # [U+8A73][U+7D30][U+7D50][U+679C][U+986F][U+793A]
        self.create_results_detail(results_frame)
        
        # === [U+5BE6][U+6642][U+76E3][U+63A7][U+65E5][U+8A8C] ===
        log_frame = ctk.CTkFrame(main_scroll_frame)
        log_frame.pack(fill="x", padx=5, pady=10)
        
        ctk.CTkLabel(log_frame, text="[U+1F4DD] [U+5BE6][U+6642][U+76E3][U+63A7][U+65E5][U+8A8C]", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # [U+65E5][U+8A8C][U+6587][U+672C][U+6846]
        self.log_text = ctk.CTkTextbox(log_frame, height=200)
        self.log_text.pack(fill="x", expand=False, padx=10, pady=10)
        
        # [U+65E5][U+8A8C][U+63A7][U+5236][U+6309][U+9215]
        log_btn_frame = ctk.CTkFrame(log_frame)
        log_btn_frame.pack(fill="x", padx=10, pady=(0, 10))
        
        self.monitor_btn = ctk.CTkButton(log_btn_frame, text="[U+25B6][U+FE0F] [U+958B][U+59CB][U+76E3][U+63A7]", 
                                       command=self.toggle_monitoring, width=100)
        self.monitor_btn.pack(side="left", padx=5)
        
        self.clear_log_btn = ctk.CTkButton(log_btn_frame, text="[U+1F5D1][U+FE0F] [U+6E05][U+7A7A][U+65E5][U+8A8C]", 
                                         command=self.clear_log, width=100)
        self.clear_log_btn.pack(side="left", padx=5)
        
        # [U+589E][U+52A0][U+624B][U+52D5][U+5237][U+65B0][U+6309][U+9215]
        self.refresh_btn = ctk.CTkButton(log_btn_frame, text="[U+1F504] [U+7ACB][U+5373][U+5237][U+65B0]", 
                                       command=self.force_refresh_all, width=100)
        self.refresh_btn.pack(side="left", padx=5)
    
    def create_status_bits(self, parent):
        """[U+5275][U+5EFA][U+72C0][U+614B][U+4F4D][U+986F][U+793A]"""
        # [U+4F7F][U+7528][U+7DB2][U+683C][U+5E03][U+5C40]
        status_grid = ctk.CTkFrame(parent)
        status_grid.pack(fill="x", padx=5, pady=5)
        
        # Ready[U+72C0][U+614B]
        self.ready_frame = ctk.CTkFrame(status_grid)
        self.ready_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.ready_frame, text="Ready", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.ready_frame, text="bit0", font=ctk.CTkFont(size=10)).pack()
        self.ready_status = ctk.CTkLabel(self.ready_frame, text="0", 
                                       font=ctk.CTkFont(size=20, weight="bold"),
                                       text_color="gray")
        self.ready_status.pack(pady=2)
        
        # Running[U+72C0][U+614B]
        self.running_frame = ctk.CTkFrame(status_grid)
        self.running_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.running_frame, text="Running", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.running_frame, text="bit1", font=ctk.CTkFont(size=10)).pack()
        self.running_status = ctk.CTkLabel(self.running_frame, text="0", 
                                         font=ctk.CTkFont(size=20, weight="bold"),
                                         text_color="gray")
        self.running_status.pack(pady=2)
        
        # Alarm[U+72C0][U+614B]
        self.alarm_frame = ctk.CTkFrame(status_grid)
        self.alarm_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.alarm_frame, text="Alarm", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.alarm_frame, text="bit2", font=ctk.CTkFont(size=10)).pack()
        self.alarm_status = ctk.CTkLabel(self.alarm_frame, text="0", 
                                       font=ctk.CTkFont(size=20, weight="bold"),
                                       text_color="gray")
        self.alarm_status.pack(pady=2)
        
        # Initialized[U+72C0][U+614B]
        self.initialized_frame = ctk.CTkFrame(status_grid)
        self.initialized_frame.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.initialized_frame, text="Initialized", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.initialized_frame, text="bit3", font=ctk.CTkFont(size=10)).pack()
        self.initialized_status = ctk.CTkLabel(self.initialized_frame, text="0", 
                                             font=ctk.CTkFont(size=20, weight="bold"),
                                             text_color="gray")
        self.initialized_status.pack(pady=2)
        
        # [U+8A2D][U+7F6E][U+7DB2][U+683C][U+6B0A][U+91CD]
        status_grid.grid_columnconfigure(0, weight=1)
        status_grid.grid_columnconfigure(1, weight=1)
    
    def create_results_summary(self, parent):
        """[U+5275][U+5EFA][U+7D50][U+679C][U+7D71][U+8A08][U+986F][U+793A]"""
        summary_frame = ctk.CTkFrame(parent)
        summary_frame.pack(fill="x", padx=10, pady=5)
        
        # [U+7D71][U+8A08][U+4FE1][U+606F]
        stats_grid = ctk.CTkFrame(summary_frame)
        stats_grid.pack(fill="x", padx=5, pady=5)
        
        # [U+5713][U+5F62][U+6578][U+91CF]
        circle_frame = ctk.CTkFrame(stats_grid)
        circle_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(circle_frame, text="[U+6AA2][U+6E2C][U+5713][U+5F62]", font=ctk.CTkFont(size=12)).pack()
        self.circle_count_label = ctk.CTkLabel(circle_frame, text="0", 
                                             font=ctk.CTkFont(size=18, weight="bold"),
                                             text_color="blue")
        self.circle_count_label.pack()
        
        # [U+64CD][U+4F5C][U+8A08][U+6578]
        op_frame = ctk.CTkFrame(stats_grid)
        op_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(op_frame, text="[U+64CD][U+4F5C][U+8A08][U+6578]", font=ctk.CTkFont(size=12)).pack()
        self.op_count_label = ctk.CTkLabel(op_frame, text="0", 
                                         font=ctk.CTkFont(size=18, weight="bold"))
        self.op_count_label.pack()
        
        # [U+932F][U+8AA4][U+8A08][U+6578]
        err_frame = ctk.CTkFrame(stats_grid)
        err_frame.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(err_frame, text="[U+932F][U+8AA4][U+8A08][U+6578]", font=ctk.CTkFont(size=12)).pack()
        self.err_count_label = ctk.CTkLabel(err_frame, text="0", 
                                          font=ctk.CTkFont(size=18, weight="bold"),
                                          text_color="red")
        self.err_count_label.pack()
        
        # [U+6642][U+9593][U+7D71][U+8A08]
        time_frame = ctk.CTkFrame(stats_grid)
        time_frame.grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(time_frame, text="[U+7E3D][U+8017][U+6642](ms)", font=ctk.CTkFont(size=12)).pack()
        self.time_label = ctk.CTkLabel(time_frame, text="0", 
                                     font=ctk.CTkFont(size=18, weight="bold"),
                                     text_color="green")
        self.time_label.pack()
        
        # [U+8A2D][U+7F6E][U+7DB2][U+683C][U+6B0A][U+91CD]
        stats_grid.grid_columnconfigure(0, weight=1)
        stats_grid.grid_columnconfigure(1, weight=1)
        stats_grid.grid_columnconfigure(2, weight=1)
        stats_grid.grid_columnconfigure(3, weight=1)
    
    def create_results_detail(self, parent):
        """[U+5275][U+5EFA][U+8A73][U+7D30][U+7D50][U+679C][U+986F][U+793A]"""
        detail_frame = ctk.CTkFrame(parent)
        detail_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(detail_frame, text="[U+5713][U+5F62][U+8A73][U+7D30][U+4FE1][U+606F]", 
                    font=ctk.CTkFont(size=14, weight="bold")).pack(pady=5)
        
        # [U+5713][U+5F62][U+4FE1][U+606F][U+6587][U+672C][U+6846] - [U+56FA][U+5B9A][U+9AD8][U+5EA6]
        self.results_text = ctk.CTkTextbox(detail_frame, height=100)
        self.results_text.pack(fill="x", expand=False, padx=5, pady=5)
    
    def toggle_connection(self):
        """[U+5207][U+63DB][U+9023][U+63A5][U+72C0][U+614B]"""
        if not MODBUS_AVAILABLE:
            messagebox.showerror("[U+932F][U+8AA4]", "Modbus[U+6A21][U+7D44][U+4E0D][U+53EF][U+7528][U+FF0C][U+8ACB][U+5B89][U+88DD]pymodbus")
            return
        
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """[U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]"""
        try:
            # [U+5F9E][U+8F38][U+5165][U+6846][U+7372][U+53D6]IP[U+548C][U+7AEF][U+53E3]
            input_ip = self.ip_entry.get().strip()
            input_port = self.port_entry.get().strip()
            
            if input_ip:
                self.server_ip = input_ip
            if input_port:
                self.server_port = int(input_port)
            
            if not self.server_ip:
                messagebox.showerror("[U+932F][U+8AA4]", "[U+8ACB][U+8F38][U+5165][U+6709][U+6548][U+7684]IP[U+5730][U+5740]")
                return
            
            self.log_message(f"[U+1F517] [U+624B][U+52D5][U+9023][U+63A5][U+5230] {self.server_ip}:{self.server_port}...")
            
            # [U+66F4][U+65B0][U+9023][U+63A5][U+72C0][U+614B][U+986F][U+793A]
            self.conn_status.configure(text="[U+1F504] [U+9023][U+63A5][U+4E2D]...", text_color="orange")
            self.conn_detail.configure(text=f"[U+6B63][U+5728][U+9023][U+63A5][U+5230] {self.server_ip}:{self.server_port}...")
            
            # [U+5728][U+65B0][U+7DDA][U+7A0B][U+4E2D][U+5617][U+8A66][U+9023][U+63A5][U+FF0C][U+907F][U+514D][U+963B][U+585E]UI
            threading.Thread(target=self._connect_worker, daemon=True).start()
                
        except ValueError:
            error_msg = "[U+7AEF][U+53E3][U+865F][U+5FC5][U+9808][U+662F][U+6578][U+5B57]"
            self.log_message(f"[FAIL] {error_msg}")
            messagebox.showerror("[U+53C3][U+6578][U+932F][U+8AA4]", error_msg)
        except Exception as e:
            error_msg = f"[U+9023][U+63A5][U+5931][U+6557]: {str(e)}"
            self.log_message(f"[FAIL] {error_msg}")
            messagebox.showerror("[U+9023][U+63A5][U+5931][U+6557]", error_msg)
    
    def _connect_worker(self):
        """[U+9023][U+63A5][U+5DE5][U+4F5C][U+7DDA][U+7A0B]"""
        success = self._attempt_connection()
        
        # [U+5728][U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+66F4][U+65B0]UI
        self.root.after(0, self._update_connection_result, success)
    
    def _update_connection_result(self, success):
        """[U+66F4][U+65B0][U+9023][U+63A5][U+7D50][U+679C][U+FF08][U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+8ABF][U+7528][U+FF09]"""
        if success:
            self.connect_btn.configure(text="[U+1F50C] [U+65B7][U+958B]")
            self.log_message("[OK] [U+624B][U+52D5][U+9023][U+63A5][U+6210][U+529F]")
        else:
            self._show_connection_error()
            messagebox.showerror("[U+9023][U+63A5][U+5931][U+6557]", self.connection_error_message)
    
    def disconnect(self):
        """[U+65B7][U+958B]Modbus[U+9023][U+63A5]"""
        try:
            if self.monitoring:
                self.stop_monitoring()
            
            if self.client:
                self.client.close()
            
            self.connected = False
            self.connect_btn.configure(text="[U+1F517] [U+9023][U+63A5]")
            self.conn_status.configure(text="[FAIL] [U+5DF2][U+65B7][U+958B]", text_color="red")
            self.conn_detail.configure(text="[U+624B][U+52D5][U+65B7][U+958B][U+9023][U+63A5]")
            self.log_message("[U+1F50C] [U+5DF2][U+65B7][U+958B][U+9023][U+63A5]")
            
            # [U+7981][U+7528][U+63A7][U+5236][U+6309][U+9215]
            self.enable_control_buttons(False)
            
            # [U+91CD][U+7F6E][U+72C0][U+614B][U+986F][U+793A]
            self.update_status_display(0)
            
        except Exception as e:
            self.log_message(f"[FAIL] [U+65B7][U+958B][U+9023][U+63A5][U+5931][U+6557]: {str(e)}")
    
    def enable_control_buttons(self, enabled):
        """[U+555F][U+7528]/[U+7981][U+7528][U+63A7][U+5236][U+6309][U+9215]"""
        state = "normal" if enabled else "disabled"
        self.clear_btn.configure(state=state)
        self.capture_btn.configure(state=state)
        self.detect_btn.configure(state=state)
        self.init_btn.configure(state=state)
        self.read_params_btn.configure(state=state)
        self.write_params_btn.configure(state=state)
    
    def test_connection(self):
        """[U+6E2C][U+8A66][U+9023][U+63A5]"""
        if not self.connected:
            messagebox.showwarning("[U+8B66][U+544A]", "[U+8ACB][U+5148][U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]")
            return
        
        try:
            # [U+6E2C][U+8A66][U+8B80][U+53D6][U+72C0][U+614B][U+5BC4][U+5B58][U+5668]
            status_addr = self.REGISTERS['STATUS_REGISTER']
            
            # pymodbus 3.9.2 [U+7684][U+6A19][U+6E96][U+8B80][U+53D6][U+65B9][U+5F0F]
            result = self.client.read_holding_registers(
                address=status_addr, 
                count=1, 
                slave=1
            )
            
            if not result.isError():
                status_value = result.registers[0]
                status_desc = self.STATUS_VALUES.get(status_value, f"[U+672A][U+77E5][U+72C0][U+614B]")
                
                test_msg = (f"[U+1F527] [U+9023][U+63A5][U+6E2C][U+8A66][U+6210][U+529F]\n"
                          f"[U+8B80][U+53D6][U+5730][U+5740]: {status_addr} (STATUS_REGISTER)\n"
                          f"[U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+503C]: {status_value}\n"
                          f"[U+72C0][U+614B][U+63CF][U+8FF0]: {status_desc}")
                
                self.log_message(test_msg.replace('\n', ', '))
                messagebox.showinfo("[U+6E2C][U+8A66][U+6210][U+529F]", test_msg)
            else:
                raise Exception(f"[U+8B80][U+53D6][U+5931][U+6557]: {result}")
                
        except Exception as e:
            self.log_message(f"[FAIL] [U+9023][U+63A5][U+6E2C][U+8A66][U+5931][U+6557]: {str(e)}")
            messagebox.showerror("[U+6E2C][U+8A66][U+5931][U+6557]", str(e))
    
    def send_command(self, command):
        """[U+767C][U+9001][U+63A7][U+5236][U+6307][U+4EE4]"""
        if not self.connected:
            messagebox.showwarning("[U+8B66][U+544A]", "[U+8ACB][U+5148][U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]")
            return
        
        try:
            command_names = {0: "[U+6E05][U+7A7A]", 8: "[U+62CD][U+7167]", 16: "[U+62CD][U+7167]+[U+6AA2][U+6E2C]", 32: "[U+91CD][U+65B0][U+521D][U+59CB][U+5316]"}
            command_name = command_names.get(command, f"[U+672A][U+77E5]({command})")
            
            # [U+9810][U+671F][U+7684][U+72C0][U+614B][U+8B8A][U+5316][U+8AAA][U+660E]
            expected_changes = {
                0: "[U+72C0][U+614B][U+61C9][U+56DE][U+5230]: 1([U+6E96][U+5099]) [U+6216] 9([U+5B8C][U+5168][U+5C31][U+7DD2])",
                8: "[U+72C0][U+614B][U+8B8A][U+5316]: 9[U+2192]10([U+57F7][U+884C])[U+2192]9([U+5B8C][U+6210])",
                16: "[U+72C0][U+614B][U+8B8A][U+5316]: 9[U+2192]10([U+57F7][U+884C])[U+2192]9([U+5B8C][U+6210])",
                32: "[U+72C0][U+614B][U+8B8A][U+5316]: [U+91CD][U+65B0][U+521D][U+59CB][U+5316][U+904E][U+7A0B]"
            }
            
            self.log_message(f"[U+1F3AE] [U+767C][U+9001][U+63A7][U+5236][U+6307][U+4EE4]: {command} ({command_name})")
            self.log_message(f"[U+1F4A1] {expected_changes.get(command, '[U+72C0][U+614B][U+8B8A][U+5316][U+672A][U+77E5]')}")
            
            # pymodbus 3.9.2 [U+7684][U+5BEB][U+5165][U+65B9][U+5F0F]
            result = self.client.write_register(
                address=self.REGISTERS['CONTROL_COMMAND'], 
                value=command, 
                slave=1
            )
            
            if not result.isError():
                self.log_message(f"[OK] [U+63A7][U+5236][U+6307][U+4EE4][U+767C][U+9001][U+6210][U+529F]: {command_name}")
                # [U+7ACB][U+5373][U+8B80][U+53D6][U+4E00][U+6B21][U+72C0][U+614B][U+67E5][U+770B][U+8B8A][U+5316]
                self.root.after(100, self.force_status_read)
            else:
                raise Exception(f"[U+5BEB][U+5165][U+5931][U+6557]: {result}")
                
        except Exception as e:
            self.log_message(f"[FAIL] [U+767C][U+9001][U+63A7][U+5236][U+6307][U+4EE4][U+5931][U+6557]: {str(e)}")
            messagebox.showerror("[U+767C][U+9001][U+5931][U+6557]", str(e))
    
    def force_status_read(self):
        """[U+5F37][U+5236][U+8B80][U+53D6][U+4E00][U+6B21][U+72C0][U+614B] ([U+7528][U+65BC][U+89C0][U+5BDF][U+6307][U+4EE4][U+5F8C][U+7684][U+72C0][U+614B][U+8B8A][U+5316])"""
        if self.connected:
            threading.Thread(target=self.read_status_register, daemon=True).start()
    
    def read_parameters(self):
        """[U+8B80][U+53D6][U+8996][U+89BA][U+6AA2][U+6E2C][U+53C3][U+6578]"""
        if not self.connected:
            messagebox.showwarning("[U+8B66][U+544A]", "[U+8ACB][U+5148][U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]")
            return
        
        try:
            self.log_message("[U+1F4D6] [U+6B63][U+5728][U+8B80][U+53D6][U+8996][U+89BA][U+6AA2][U+6E2C][U+53C3][U+6578]...")
            
            # [U+8B80][U+53D6][U+53C3][U+6578][U+5BC4][U+5B58][U+5668]
            params = {}
            param_registers = ['MIN_AREA_HIGH', 'MIN_AREA_LOW', 'MIN_ROUNDNESS', 
                             'GAUSSIAN_KERNEL', 'CANNY_LOW', 'CANNY_HIGH']
            
            for reg_name in param_registers:
                result = self.client.read_holding_registers(
                    self.REGISTERS[reg_name], 1, slave=1
                )
                if not result.isError():
                    params[reg_name] = result.registers[0]
                else:
                    raise Exception(f"[U+8B80][U+53D6]{reg_name}[U+5931][U+6557]")
            
            # [U+7D44][U+5408][U+9762][U+7A4D][U+503C]
            area_value = (params['MIN_AREA_HIGH'] << 16) + params['MIN_AREA_LOW']
            roundness_value = params['MIN_ROUNDNESS'] / 1000.0
            
            # [U+66F4][U+65B0][U+754C][U+9762]
            self.area_entry.delete(0, tk.END)
            self.area_entry.insert(0, str(area_value))
            
            self.roundness_entry.delete(0, tk.END)
            self.roundness_entry.insert(0, str(roundness_value))
            
            self.gaussian_entry.delete(0, tk.END)
            self.gaussian_entry.insert(0, str(params['GAUSSIAN_KERNEL']))
            
            self.canny_low_entry.delete(0, tk.END)
            self.canny_low_entry.insert(0, str(params['CANNY_LOW']))
            
            self.canny_high_entry.delete(0, tk.END)
            self.canny_high_entry.insert(0, str(params['CANNY_HIGH']))
            
            self.log_message(f"[OK] [U+53C3][U+6578][U+8B80][U+53D6][U+6210][U+529F]: [U+9762][U+7A4D]={area_value}, [U+5713][U+5EA6]={roundness_value}")
            
        except Exception as e:
            self.log_message(f"[FAIL] [U+8B80][U+53D6][U+53C3][U+6578][U+5931][U+6557]: {str(e)}")
            messagebox.showerror("[U+8B80][U+53D6][U+5931][U+6557]", str(e))
    
    def write_parameters(self):
        """[U+5BEB][U+5165][U+8996][U+89BA][U+6AA2][U+6E2C][U+53C3][U+6578]"""
        if not self.connected:
            messagebox.showwarning("[U+8B66][U+544A]", "[U+8ACB][U+5148][U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]")
            return
        
        try:
            self.log_message("[U+1F4BE] [U+6B63][U+5728][U+5BEB][U+5165][U+8996][U+89BA][U+6AA2][U+6E2C][U+53C3][U+6578]...")
            
            # [U+7372][U+53D6][U+53C3][U+6578][U+503C]
            area_value = int(float(self.area_entry.get() or "50000"))
            roundness_value = float(self.roundness_entry.get() or "0.8")
            gaussian_value = int(self.gaussian_entry.get() or "9")
            canny_low_value = int(self.canny_low_entry.get() or "20")
            canny_high_value = int(self.canny_high_entry.get() or "60")
            
            # [U+5206][U+89E3][U+9762][U+7A4D][U+503C][U+70BA][U+9AD8][U+4F4E]16[U+4F4D]
            area_high = (area_value >> 16) & 0xFFFF
            area_low = area_value & 0xFFFF
            roundness_int = int(roundness_value * 1000)
            
            # [U+5BEB][U+5165][U+53C3][U+6578]
            params_to_write = [
                ('MIN_AREA_HIGH', area_high),
                ('MIN_AREA_LOW', area_low),
                ('MIN_ROUNDNESS', roundness_int),
                ('GAUSSIAN_KERNEL', gaussian_value),
                ('CANNY_LOW', canny_low_value),
                ('CANNY_HIGH', canny_high_value)
            ]
            
            for reg_name, value in params_to_write:
                result = self.client.write_register(
                    self.REGISTERS[reg_name], value, slave=1
                )
                if result.isError():
                    raise Exception(f"[U+5BEB][U+5165]{reg_name}[U+5931][U+6557]")
            
            self.log_message(f"[OK] [U+53C3][U+6578][U+5BEB][U+5165][U+6210][U+529F]: [U+9762][U+7A4D]={area_value}, [U+5713][U+5EA6]={roundness_value}")
            
        except ValueError as e:
            messagebox.showerror("[U+53C3][U+6578][U+932F][U+8AA4]", "[U+8ACB][U+8F38][U+5165][U+6709][U+6548][U+7684][U+6578][U+503C]")
        except Exception as e:
            self.log_message(f"[FAIL] [U+5BEB][U+5165][U+53C3][U+6578][U+5931][U+6557]: {str(e)}")
            messagebox.showerror("[U+5BEB][U+5165][U+5931][U+6557]", str(e))
    
    def toggle_monitoring(self):
        """[U+5207][U+63DB][U+76E3][U+63A7][U+72C0][U+614B]"""
        if self.monitoring:
            self.stop_monitoring()
        else:
            if self.connected:
                self.start_monitoring()
            else:
                messagebox.showwarning("[U+8B66][U+544A]", "[U+8ACB][U+5148][U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]")
    
    def start_monitoring(self):
        """[U+958B][U+59CB][U+76E3][U+63A7]"""
        if not self.connected:
            return
        
        self.monitoring = True
        self.monitor_btn.configure(text="[U+23F8][U+FE0F] [U+505C][U+6B62][U+76E3][U+63A7]")
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        self.log_message("[U+25B6][U+FE0F] [U+958B][U+59CB][U+5BE6][U+6642][U+76E3][U+63A7]")
    
    def stop_monitoring(self):
        """[U+505C][U+6B62][U+76E3][U+63A7]"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        self.monitor_btn.configure(text="[U+25B6][U+FE0F] [U+958B][U+59CB][U+76E3][U+63A7]")
        self.log_message("[U+23F8][U+FE0F] [U+505C][U+6B62][U+5BE6][U+6642][U+76E3][U+63A7]")
    
    def monitor_loop(self):
        """[U+76E3][U+63A7][U+5FAA][U+74B0]"""
        while self.monitoring and self.connected:
            try:
                # [U+8B80][U+53D6][U+72C0][U+614B][U+5BC4][U+5B58][U+5668]
                self.read_status_register()
                
                # [U+8B80][U+53D6][U+6AA2][U+6E2C][U+7D50][U+679C]
                self.read_detection_results()
                
                # [U+8B80][U+53D6][U+7D71][U+8A08][U+4FE1][U+606F]
                self.read_statistics()
                
                time.sleep(self.update_interval)
                
            except Exception as e:
                self.log_message(f"[FAIL] [U+76E3][U+63A7][U+932F][U+8AA4]: {str(e)}")
                time.sleep(2.0)  # [U+932F][U+8AA4][U+6642][U+5EF6][U+9577][U+9593][U+9694]
    
    def read_status_register(self):
        """[U+8B80][U+53D6][U+72C0][U+614B][U+5BC4][U+5B58][U+5668]"""
        try:
            status_addr = self.REGISTERS['STATUS_REGISTER']
            
            # pymodbus 3.9.2 [U+7684][U+6A19][U+6E96][U+8B80][U+53D6][U+65B9][U+5F0F]
            result = self.client.read_holding_registers(
                address=status_addr, 
                count=1, 
                slave=1
            )
            
            if not result.isError():
                status_value = result.registers[0]
                
                # [U+89E3][U+6790][U+72C0][U+614B][U+4F4D]
                ready = (status_value >> 0) & 1
                running = (status_value >> 1) & 1
                alarm = (status_value >> 2) & 1
                initialized = (status_value >> 3) & 1
                
                # [U+6AA2][U+67E5][U+72C0][U+614B][U+8B8A][U+5316]
                old_status = self.status_bits.copy()
                old_status_value = self.last_values.get('status_register', -1)
                
                self.status_bits = {
                    'ready': ready,
                    'running': running,
                    'alarm': alarm,
                    'initialized': initialized
                }
                
                # [U+8A18][U+9304][U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+503C][U+8B8A][U+5316]
                if old_status_value != status_value:
                    self.last_values['status_register'] = status_value
                    status_desc = self.STATUS_VALUES.get(status_value, f"[U+672A][U+77E5][U+72C0][U+614B] ({status_value:04b})")
                    self.log_message(f"[U+1F4CA] [U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+8B8A][U+5316] [[U+5730][U+5740]{status_addr}]: {old_status_value} [U+2192] {status_value} ({status_desc})")
                
                # [U+66F4][U+65B0][U+754C][U+9762] ([U+5728][U+4E3B][U+7DDA][U+7A0B][U+4E2D])
                self.root.after(0, self.update_status_display, status_value)
                
                # [U+8A18][U+9304][U+500B][U+5225][U+72C0][U+614B][U+4F4D][U+8B8A][U+5316]
                for bit_name, new_value in self.status_bits.items():
                    old_value = old_status.get(bit_name, -1)
                    if old_value != new_value and old_value != -1:
                        change_type = "[U+555F][U+52D5]" if new_value else "[U+505C][U+6B62]"
                        self.log_message(f"[U+1F539] {bit_name} {change_type}: {old_value} [U+2192] {new_value}")
                
        except Exception as e:
            pass  # [U+975C][U+9ED8][U+8655][U+7406][U+8B80][U+53D6][U+932F][U+8AA4]
    
    def read_detection_results(self):
        """[U+8B80][U+53D6][U+6AA2][U+6E2C][U+7D50][U+679C]"""
        try:
            # [U+8B80][U+53D6][U+5713][U+5F62][U+6578][U+91CF]
            result = self.client.read_holding_registers(
                address=self.REGISTERS['CIRCLE_COUNT'], 
                count=1, 
                slave=1
            )
            
            if not result.isError():
                circle_count = result.registers[0]
                
                # [U+6AA2][U+67E5][U+6578][U+91CF][U+8B8A][U+5316]
                if self.last_values.get('circle_count', -1) != circle_count:
                    self.last_values['circle_count'] = circle_count
                    
                    # [U+8B80][U+53D6][U+5713][U+5F62][U+8A73][U+7D30][U+4FE1][U+606F]
                    circles = []
                    for i in range(min(circle_count, 3)):  # [U+6700][U+591A][U+986F][U+793A]3[U+500B][U+5713][U+5F62]
                        x_result = self.client.read_holding_registers(
                            address=self.REGISTERS[f'CIRCLE_{i+1}_X'], 
                            count=1, 
                            slave=1
                        )
                        y_result = self.client.read_holding_registers(
                            address=self.REGISTERS[f'CIRCLE_{i+1}_Y'], 
                            count=1, 
                            slave=1
                        )
                        r_result = self.client.read_holding_registers(
                            address=self.REGISTERS[f'CIRCLE_{i+1}_RADIUS'], 
                            count=1, 
                            slave=1
                        )
                        
                        if (not x_result.isError() and not y_result.isError() and 
                            not r_result.isError()):
                            circles.append({
                                'id': i + 1,
                                'x': x_result.registers[0],
                                'y': y_result.registers[0],
                                'radius': r_result.registers[0]
                            })
                    
                    # [U+66F4][U+65B0][U+6AA2][U+6E2C][U+7D50][U+679C][U+986F][U+793A]
                    self.root.after(0, self.update_results_display, circle_count, circles)
                    
                    if circle_count > 0:
                        self.log_message(f"[U+1F3AF] [U+6AA2][U+6E2C][U+7D50][U+679C][U+66F4][U+65B0]: [U+627E][U+5230] {circle_count} [U+500B][U+5713][U+5F62]")
                
        except Exception as e:
            pass  # [U+975C][U+9ED8][U+8655][U+7406][U+8B80][U+53D6][U+932F][U+8AA4]
    
    def read_statistics(self):
        """[U+8B80][U+53D6][U+7D71][U+8A08][U+4FE1][U+606F]"""
        try:
            # [U+8B80][U+53D6][U+64CD][U+4F5C][U+8A08][U+6578][U+548C][U+932F][U+8AA4][U+8A08][U+6578]
            op_result = self.client.read_holding_registers(
                address=self.REGISTERS['OPERATION_COUNT'], 
                count=1, 
                slave=1
            )
            err_result = self.client.read_holding_registers(
                address=self.REGISTERS['ERROR_COUNT'], 
                count=1, 
                slave=1
            )
            time_result = self.client.read_holding_registers(
                address=self.REGISTERS['LAST_TOTAL_TIME'], 
                count=1, 
                slave=1
            )
            
            if (not op_result.isError() and not err_result.isError() and 
                not time_result.isError()):
                
                op_count = op_result.registers[0]
                err_count = err_result.registers[0]
                total_time = time_result.registers[0]
                
                # [U+6AA2][U+67E5][U+8B8A][U+5316]
                old_op = self.last_values.get('op_count', -1)
                old_err = self.last_values.get('err_count', -1)
                
                if old_op != op_count:
                    self.last_values['op_count'] = op_count
                    self.log_message(f"[U+1F4C8] [U+64CD][U+4F5C][U+8A08][U+6578]: {op_count}")
                
                if old_err != err_count:
                    self.last_values['err_count'] = err_count
                    if err_count > old_err and old_err >= 0:
                        self.log_message(f"[WARN][U+FE0F] [U+932F][U+8AA4][U+8A08][U+6578][U+589E][U+52A0]: {err_count}")
                
                # [U+66F4][U+65B0][U+7D71][U+8A08][U+986F][U+793A]
                self.root.after(0, self.update_statistics_display, op_count, err_count, total_time)
                
        except Exception as e:
            pass  # [U+975C][U+9ED8][U+8655][U+7406][U+8B80][U+53D6][U+932F][U+8AA4]
    
    def update_status_display(self, status_value):
        """[U+66F4][U+65B0][U+72C0][U+614B][U+986F][U+793A] ([U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+8ABF][U+7528])"""
        # [U+66F4][U+65B0][U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+986F][U+793A] - [U+5927][U+5B57][U+9AD4][U+9192][U+76EE][U+986F][U+793A]
        binary_str = f"{status_value:04b}"
        self.status_reg_label.configure(text=str(status_value))
        self.status_binary_label.configure(text=f"({binary_str})")
        
        # [U+66F4][U+65B0][U+72C0][U+614B][U+63CF][U+8FF0][U+548C][U+984F][U+8272]
        status_desc = self.STATUS_VALUES.get(status_value, f"[U+672A][U+77E5][U+72C0][U+614B]")
        if status_value == 1:
            self.status_desc_label.configure(text=status_desc, text_color="orange")
            self.status_reg_label.configure(text_color="orange")
        elif status_value == 9:
            self.status_desc_label.configure(text=status_desc, text_color="green")
            self.status_reg_label.configure(text_color="green")
        elif status_value in [10, 11]:
            self.status_desc_label.configure(text=status_desc, text_color="blue")
            self.status_reg_label.configure(text_color="blue")
        elif status_value in [4, 5, 12]:
            self.status_desc_label.configure(text=status_desc, text_color="red")
            self.status_reg_label.configure(text_color="red")
        else:
            self.status_desc_label.configure(text=status_desc, text_color="gray")
            self.status_reg_label.configure(text_color="gray")
        
        # [U+66F4][U+65B0][U+5404][U+72C0][U+614B][U+4F4D][U+986F][U+793A]
        self.update_status_bit(self.ready_status, self.ready_frame, self.status_bits['ready'])
        self.update_status_bit(self.running_status, self.running_frame, self.status_bits['running'])
        self.update_status_bit(self.alarm_status, self.alarm_frame, self.status_bits['alarm'])
        self.update_status_bit(self.initialized_status, self.initialized_frame, self.status_bits['initialized'])
    
    def update_status_bit(self, label, frame, value):
        """[U+66F4][U+65B0][U+55AE][U+500B][U+72C0][U+614B][U+4F4D][U+986F][U+793A]"""
        label.configure(text=str(value))
        
        if value == 1:
            label.configure(text_color="white")
            frame.configure(fg_color=("green", "darkgreen"))  # [U+555F][U+52D5][U+72C0][U+614B]-[U+7DA0][U+8272]
        else:
            label.configure(text_color="gray")
            frame.configure(fg_color=("gray85", "gray20"))  # [U+505C][U+6B62][U+72C0][U+614B]-[U+7070][U+8272]
    
    def update_results_display(self, circle_count, circles):
        """[U+66F4][U+65B0][U+6AA2][U+6E2C][U+7D50][U+679C][U+986F][U+793A] ([U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+8ABF][U+7528])"""
        self.circle_count_label.configure(text=str(circle_count))
        
        # [U+6E05][U+7A7A][U+4E26][U+66F4][U+65B0][U+8A73][U+7D30][U+7D50][U+679C]
        self.results_text.delete("1.0", tk.END)
        
        if circle_count > 0:
            result_text = f"[U+6AA2][U+6E2C][U+5230] {circle_count} [U+500B][U+5713][U+5F62]:\n\n"
            for circle in circles:
                result_text += (f"[U+5713][U+5F62] {circle['id']}: "
                              f"[U+4E2D][U+5FC3]({circle['x']}, {circle['y']}) "
                              f"[U+534A][U+5F91]={circle['radius']}\n")
            
            self.circle_count_label.configure(text_color="blue")
        else:
            result_text = "[U+672A][U+6AA2][U+6E2C][U+5230][U+5713][U+5F62]"
            self.circle_count_label.configure(text_color="gray")
        
        self.results_text.insert("1.0", result_text)
    
    def update_statistics_display(self, op_count, err_count, total_time):
        """[U+66F4][U+65B0][U+7D71][U+8A08][U+4FE1][U+606F][U+986F][U+793A] ([U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+8ABF][U+7528])"""
        self.op_count_label.configure(text=str(op_count))
        self.err_count_label.configure(text=str(err_count))
        self.time_label.configure(text=str(total_time))
        
        # [U+6839][U+64DA][U+932F][U+8AA4][U+6578][U+91CF][U+8A2D][U+7F6E][U+984F][U+8272]
        if err_count > 0:
            self.err_count_label.configure(text_color="red")
        else:
            self.err_count_label.configure(text_color="green")
    
    def log_message(self, message):
        """[U+6DFB][U+52A0][U+65E5][U+8A8C][U+8A0A][U+606F] ([U+7DDA][U+7A0B][U+5B89][U+5168])"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        # [U+5728][U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+66F4][U+65B0]UI
        self.root.after(0, self._append_log, log_entry)
    
    def _append_log(self, log_entry):
        """[U+5728][U+4E3B][U+7DDA][U+7A0B][U+4E2D][U+6DFB][U+52A0][U+65E5][U+8A8C] ([U+5167][U+90E8][U+65B9][U+6CD5])"""
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)  # [U+81EA][U+52D5][U+6EFE][U+52D5][U+5230][U+5E95][U+90E8]
        
        # [U+9650][U+5236][U+65E5][U+8A8C][U+9577][U+5EA6][U+FF0C][U+4FDD][U+7559][U+6700][U+5F8C]1000[U+884C]
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 1000:
            self.log_text.delete("1.0", f"{len(lines)-1000}.0")
    
    def force_refresh_all(self):
        """[U+5F37][U+5236][U+5237][U+65B0][U+6240][U+6709][U+6578][U+64DA]"""
        if self.connected:
            self.log_message("[U+1F504] [U+624B][U+52D5][U+5237][U+65B0][U+6240][U+6709][U+6578][U+64DA]...")
            # [U+7ACB][U+5373][U+57F7][U+884C][U+4E00][U+6B21][U+5B8C][U+6574][U+7684][U+76E3][U+63A7][U+5FAA][U+74B0]
            threading.Thread(target=self._manual_refresh, daemon=True).start()
        else:
            messagebox.showwarning("[U+8B66][U+544A]", "[U+8ACB][U+5148][U+9023][U+63A5][U+5230]Modbus[U+670D][U+52D9][U+5668]")
    
    def _manual_refresh(self):
        """[U+624B][U+52D5][U+5237][U+65B0][U+7684][U+5167][U+90E8][U+65B9][U+6CD5]"""
        try:
            self.read_status_register()
            self.read_detection_results()
            self.read_statistics()
            self.log_message("[OK] [U+624B][U+52D5][U+5237][U+65B0][U+5B8C][U+6210]")
        except Exception as e:
            self.log_message(f"[FAIL] [U+624B][U+52D5][U+5237][U+65B0][U+5931][U+6557]: {str(e)}")
    
    def clear_log(self):
        """[U+6E05][U+7A7A][U+65E5][U+8A8C]"""
        self.log_text.delete("1.0", tk.END)
        self.log_message("[U+1F4DD] [U+65E5][U+8A8C][U+5DF2][U+6E05][U+7A7A]")
    
    def on_closing(self):
        """[U+95DC][U+9589][U+7A0B][U+5E8F][U+6642][U+7684][U+6E05][U+7406][U+5DE5][U+4F5C]"""
        if self.monitoring:
            self.stop_monitoring()
        if self.connected:
            self.disconnect()
        self.root.destroy()
    
    def run(self):
        """[U+904B][U+884C]GUI[U+7A0B][U+5E8F]"""
        self.log_message("[U+1F680] CCD1[U+8996][U+89BA][U+7CFB][U+7D71] Modbus TCP [U+6E2C][U+8A66][U+5DE5][U+5177][U+5DF2][U+555F][U+52D5]")
        self.log_message("[U+1F504] [U+6B63][U+5728][U+81EA][U+52D5][U+9023][U+63A5][U+5230] 127.0.0.1:502...")
        self.log_message("[U+1F4CA] [U+72C0][U+614B][U+5BC4][U+5B58][U+5668][U+56FA][U+5B9A][U+503C][U+8AAA][U+660E]:")
        self.log_message("   [U+2022] 1: [U+6E96][U+5099][U+5C31][U+7DD2] (Modbus[U+9023][U+63A5][U+5B8C][U+6210])")
        self.log_message("   [U+2022] 9: [U+5B8C][U+5168][U+5C31][U+7DD2] (Modbus+[U+76F8][U+6A5F][U+90FD][U+9023][U+63A5])")
        self.log_message("   [U+2022] 10: [U+57F7][U+884C][U+4E2D] ([U+8655][U+7406][U+6307][U+4EE4][U+6642])")
        self.log_message("   [U+2022] 4: [U+7CFB][U+7D71][U+7570][U+5E38] ([U+6709][U+932F][U+8AA4][U+767C][U+751F])")
        self.log_message("[U+1F3AF] [U+4F7F][U+7528][U+6EFE][U+52D5][U+689D][U+67E5][U+770B][U+5B8C][U+6574][U+754C][U+9762]")
        self.root.mainloop()


def main():
    """[U+4E3B][U+51FD][U+6578]"""
    print("[U+1F680] [U+6B63][U+5728][U+555F][U+52D5] CCD1[U+8996][U+89BA][U+7CFB][U+7D71] Modbus TCP [U+6E2C][U+8A66][U+5DE5][U+5177]...")
    
    if not MODBUS_AVAILABLE:
        print("[WARN][U+FE0F] [U+8B66][U+544A]: Modbus[U+6A21][U+7D44][U+4E0D][U+53EF][U+7528][U+FF0C][U+90E8][U+5206][U+529F][U+80FD][U+5C07][U+7121][U+6CD5][U+4F7F][U+7528]")
        print("[U+1F4A1] [U+8ACB][U+5B89][U+88DD]: pip install pymodbus>=3.0.0")
    
    try:
        app = ModbusTestTool()
        app.run()
    except Exception as e:
        print(f"[FAIL] [U+7A0B][U+5E8F][U+904B][U+884C][U+932F][U+8AA4]: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()