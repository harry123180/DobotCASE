#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
from tkinter import messagebox

class LEDTestTool:
    def __init__(self):
        # [U+8A2D][U+7F6E]CustomTkinter[U+4E3B][U+984C]
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        # [U+4E3B][U+8996][U+7A97]
        self.root = ctk.CTk()
        self.root.title("LED[U+63A7][U+5236][U+5668][U+6E2C][U+8A66][U+5DE5][U+5177] (COM6)")
        self.root.geometry("800x600")
        
        # [U+4E32][U+53E3][U+9023][U+63A5]
        self.serial_connection = None
        self.connected = False
        
        # LED[U+72C0][U+614B]
        self.led_states = [False, False, False, False]  # L1-L4
        self.led_brightness = [0, 0, 0, 0]  # L1-L4[U+4EAE][U+5EA6] (0-511)
        
        self.setup_ui()
        
    def setup_ui(self):
        """[U+8A2D][U+7F6E][U+7528][U+6236][U+754C][U+9762]"""
        # [U+4E3B][U+6846][U+67B6]
        main_frame = ctk.CTkFrame(self.root)
        main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        # [U+6A19][U+984C]
        title_label = ctk.CTkLabel(
            main_frame, 
            text="LED[U+63A7][U+5236][U+5668][U+6E2C][U+8A66][U+5DE5][U+5177]", 
            font=ctk.CTkFont(size=24, weight="bold")
        )
        title_label.pack(pady=(20, 30))
        
        # [U+9023][U+63A5][U+5340][U+57DF]
        self.setup_connection_frame(main_frame)
        
        # LED[U+63A7][U+5236][U+5340][U+57DF]
        self.setup_led_control_frame(main_frame)
        
        # [U+65E5][U+8A8C][U+5340][U+57DF]
        self.setup_log_frame(main_frame)
        
    def setup_connection_frame(self, parent):
        """[U+8A2D][U+7F6E][U+9023][U+63A5][U+63A7][U+5236][U+5340][U+57DF]"""
        conn_frame = ctk.CTkFrame(parent)
        conn_frame.pack(fill="x", padx=20, pady=(0, 20))
        
        # [U+6A19][U+984C]
        conn_title = ctk.CTkLabel(conn_frame, text="[U+4E32][U+53E3][U+9023][U+63A5]", font=ctk.CTkFont(size=16, weight="bold"))
        conn_title.pack(pady=(15, 10))
        
        # [U+9023][U+63A5][U+63A7][U+5236]
        controls_frame = ctk.CTkFrame(conn_frame)
        controls_frame.pack(fill="x", padx=20, pady=(0, 15))
        
        # COM[U+7AEF][U+53E3][U+9078][U+64C7]
        port_frame = ctk.CTkFrame(controls_frame)
        port_frame.pack(side="left", padx=(10, 20), pady=10)
        
        ctk.CTkLabel(port_frame, text="COM[U+7AEF][U+53E3]:").pack(side="left", padx=(10, 5))
        self.port_var = ctk.StringVar(value="COM6")
        self.port_combo = ctk.CTkComboBox(
            port_frame, 
            variable=self.port_var,
            values=self.get_com_ports(),
            width=100
        )
        self.port_combo.pack(side="left", padx=(0, 10))
        
        # [U+5237][U+65B0][U+7AEF][U+53E3][U+6309][U+9215]
        refresh_btn = ctk.CTkButton(
            port_frame, 
            text="[U+5237][U+65B0]", 
            command=self.refresh_ports,
            width=60
        )
        refresh_btn.pack(side="left", padx=(0, 10))
        
        # [U+9023][U+63A5][U+6309][U+9215]
        self.connect_btn = ctk.CTkButton(
            controls_frame, 
            text="[U+9023][U+63A5]", 
            command=self.toggle_connection,
            width=100
        )
        self.connect_btn.pack(side="right", padx=(20, 10), pady=10)
        
        # [U+72C0][U+614B][U+6307][U+793A]
        self.status_label = ctk.CTkLabel(
            controls_frame, 
            text="[U+672A][U+9023][U+63A5]", 
            text_color="red"
        )
        self.status_label.pack(side="right", padx=(20, 10), pady=10)
        
    def setup_led_control_frame(self, parent):
        """[U+8A2D][U+7F6E]LED[U+63A7][U+5236][U+5340][U+57DF]"""
        led_frame = ctk.CTkFrame(parent)
        led_frame.pack(fill="both", expand=True, padx=20, pady=(0, 20))
        
        # [U+6A19][U+984C]
        led_title = ctk.CTkLabel(led_frame, text="LED[U+63A7][U+5236]", font=ctk.CTkFont(size=16, weight="bold"))
        led_title.pack(pady=(15, 20))
        
        # LED[U+63A7][U+5236][U+7DB2][U+683C]
        grid_frame = ctk.CTkFrame(led_frame)
        grid_frame.pack(fill="both", expand=True, padx=20, pady=(0, 15))
        
        # [U+914D][U+7F6E][U+7DB2][U+683C][U+6B0A][U+91CD]
        for i in range(4):
            grid_frame.grid_columnconfigure(i, weight=1)
        for i in range(4):
            grid_frame.grid_rowconfigure(i, weight=1)
        
        # [U+5275][U+5EFA]LED[U+63A7][U+5236][U+5143][U+4EF6]
        self.led_controls = []
        for i in range(4):
            led_control = self.create_led_control(grid_frame, i)
            led_control.grid(row=0, column=i, padx=10, pady=10, sticky="nsew")
            self.led_controls.append(led_control)
        
        # [U+5168][U+57DF][U+63A7][U+5236][U+6309][U+9215]
        global_frame = ctk.CTkFrame(led_frame)
        global_frame.pack(fill="x", padx=20, pady=(10, 15))
        
        all_on_btn = ctk.CTkButton(
            global_frame, 
            text="[U+5168][U+90E8][U+958B][U+555F]", 
            command=self.all_on,
            height=40
        )
        all_on_btn.pack(side="left", padx=(20, 10), pady=15)
        
        all_off_btn = ctk.CTkButton(
            global_frame, 
            text="[U+5168][U+90E8][U+95DC][U+9589]", 
            command=self.all_off,
            height=40
        )
        all_off_btn.pack(side="left", padx=(10, 10), pady=15)
        
        reset_btn = ctk.CTkButton(
            global_frame, 
            text="[U+91CD][U+7F6E][U+8A2D][U+5099]", 
            command=self.reset_device,
            height=40
        )
        reset_btn.pack(side="right", padx=(10, 20), pady=15)
        
    def create_led_control(self, parent, channel):
        """[U+5275][U+5EFA][U+55AE][U+500B]LED[U+63A7][U+5236][U+7D44][U+4EF6]"""
        led_frame = ctk.CTkFrame(parent)
        
        # [U+901A][U+9053][U+6A19][U+984C]
        title = ctk.CTkLabel(
            led_frame, 
            text=f"L{channel+1} [U+901A][U+9053]", 
            font=ctk.CTkFont(size=14, weight="bold")
        )
        title.pack(pady=(15, 10))
        
        # [U+958B][U+95DC][U+6309][U+9215]
        toggle_btn = ctk.CTkButton(
            led_frame,
            text="[U+95DC][U+9589]",
            command=lambda: self.toggle_led(channel),
            height=40,
            fg_color="gray",
            text_color="white"
        )
        toggle_btn.pack(pady=(0, 15))
        
        # [U+4EAE][U+5EA6][U+6A19][U+7C64]
        brightness_label = ctk.CTkLabel(led_frame, text="[U+4EAE][U+5EA6]: 0")
        brightness_label.pack(pady=(0, 5))
        
        # [U+4EAE][U+5EA6][U+6ED1][U+687F]
        brightness_slider = ctk.CTkSlider(
            led_frame,
            from_=0,
            to=511,
            number_of_steps=511,
            command=lambda value, ch=channel: self.set_brightness(ch, int(value)),
            height=20
        )
        brightness_slider.set(0)
        brightness_slider.pack(pady=(0, 15), padx=15, fill="x")
        
        # [U+5B58][U+5132][U+63A7][U+5236][U+5143][U+4EF6][U+5F15][U+7528]
        led_frame.toggle_btn = toggle_btn
        led_frame.brightness_label = brightness_label
        led_frame.brightness_slider = brightness_slider
        
        return led_frame
        
    def setup_log_frame(self, parent):
        """[U+8A2D][U+7F6E][U+65E5][U+8A8C][U+5340][U+57DF]"""
        log_frame = ctk.CTkFrame(parent)
        log_frame.pack(fill="x", padx=20, pady=(0, 20))
        
        log_title = ctk.CTkLabel(log_frame, text="[U+901A][U+8A0A][U+65E5][U+8A8C]", font=ctk.CTkFont(size=14, weight="bold"))
        log_title.pack(pady=(10, 5))
        
        self.log_text = ctk.CTkTextbox(log_frame, height=120)
        self.log_text.pack(fill="x", padx=15, pady=(0, 15))
        
    def get_com_ports(self):
        """[U+7372][U+53D6][U+53EF][U+7528]COM[U+7AEF][U+53E3]"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports] if ports else ["COM6"]
        
    def refresh_ports(self):
        """[U+5237][U+65B0]COM[U+7AEF][U+53E3][U+5217][U+8868]"""
        ports = self.get_com_ports()
        self.port_combo.configure(values=ports)
        self.log_message("[U+5DF2][U+5237][U+65B0]COM[U+7AEF][U+53E3][U+5217][U+8868]")
        
    def toggle_connection(self):
        """[U+5207][U+63DB][U+9023][U+63A5][U+72C0][U+614B]"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """[U+9023][U+63A5][U+4E32][U+53E3]"""
        try:
            port = self.port_var.get()
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=9600,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=1.0
            )
            
            self.connected = True
            self.connect_btn.configure(text="[U+65B7][U+958B]")
            self.status_label.configure(text="[U+5DF2][U+9023][U+63A5]", text_color="green")
            self.log_message(f"[U+6210][U+529F][U+9023][U+63A5][U+5230] {port}")
            
        except Exception as e:
            messagebox.showerror("[U+9023][U+63A5][U+932F][U+8AA4]", f"[U+7121][U+6CD5][U+9023][U+63A5][U+5230] {self.port_var.get()}:\n{str(e)}")
            self.log_message(f"[U+9023][U+63A5][U+5931][U+6557]: {str(e)}")
            
    def disconnect(self):
        """[U+65B7][U+958B][U+4E32][U+53E3]"""
        try:
            if self.serial_connection:
                self.serial_connection.close()
                self.serial_connection = None
                
            self.connected = False
            self.connect_btn.configure(text="[U+9023][U+63A5]")
            self.status_label.configure(text="[U+672A][U+9023][U+63A5]", text_color="red")
            self.log_message("[U+5DF2][U+65B7][U+958B][U+9023][U+63A5]")
            
        except Exception as e:
            self.log_message(f"[U+65B7][U+958B][U+9023][U+63A5][U+932F][U+8AA4]: {str(e)}")
            
    def send_rs232_command(self, command):
        """[U+767C][U+9001]RS232[U+6307][U+4EE4]"""
        if not self.connected or not self.serial_connection:
            self.log_message("[U+932F][U+8AA4]: [U+4E32][U+53E3][U+672A][U+9023][U+63A5]")
            return False
            
        try:
            # [U+6DFB][U+52A0][U+63DB][U+884C][U+7B26] ([U+6839][U+64DA][U+624B][U+518A][U+8981][U+6C42])
            full_command = command + "\r\n"
            self.serial_connection.write(full_command.encode('ascii'))
            
            # [U+8B80][U+53D6][U+56DE][U+61C9]
            time.sleep(0.1)
            if self.serial_connection.in_waiting > 0:
                response = self.serial_connection.read(self.serial_connection.in_waiting).decode('ascii', errors='ignore')
                self.log_message(f"[U+767C][U+9001]: {command} | [U+56DE][U+61C9]: {response.strip()}")
            else:
                self.log_message(f"[U+767C][U+9001]: {command}")
                
            return True
            
        except Exception as e:
            self.log_message(f"[U+767C][U+9001][U+6307][U+4EE4][U+5931][U+6557]: {str(e)}")
            return False
            
    def toggle_led(self, channel):
        """[U+5207][U+63DB]LED[U+958B][U+95DC]"""
        current_state = self.led_states[channel]
        new_state = not current_state
        
        if new_state:
            # [U+958B][U+555F] - [U+8A2D][U+7F6E][U+70BA][U+7576][U+524D][U+6ED1][U+687F][U+4EAE][U+5EA6][U+FF0C][U+5982][U+679C][U+70BA]0[U+5247][U+8A2D][U+70BA]255
            brightness = self.led_brightness[channel] if self.led_brightness[channel] > 0 else 255
            command = f"CH{channel+1}:{brightness}"
        else:
            # [U+95DC][U+9589] - [U+8A2D][U+7F6E][U+4EAE][U+5EA6][U+70BA]0
            brightness = 0
            command = f"CH{channel+1}:0"
            
        if self.send_rs232_command(command):
            self.led_states[channel] = new_state
            self.led_brightness[channel] = brightness
            self.update_led_ui(channel)
            
    def set_brightness(self, channel, brightness):
        """[U+8A2D][U+7F6E]LED[U+4EAE][U+5EA6]"""
        command = f"CH{channel+1}:{brightness}"
        
        if self.send_rs232_command(command):
            self.led_brightness[channel] = brightness
            self.led_states[channel] = brightness > 0
            self.update_led_ui(channel)
            
    def update_led_ui(self, channel):
        """[U+66F4][U+65B0]LED UI[U+72C0][U+614B]"""
        control = self.led_controls[channel]
        state = self.led_states[channel]
        brightness = self.led_brightness[channel]
        
        # [U+66F4][U+65B0][U+6309][U+9215]
        if state:
            control.toggle_btn.configure(
                text="[U+958B][U+555F]", 
                fg_color=["#3B8ED0", "#1F6AA5"],
                text_color="white"
            )
        else:
            control.toggle_btn.configure(
                text="[U+95DC][U+9589]", 
                fg_color="gray",
                text_color="white"
            )
            
        # [U+66F4][U+65B0][U+4EAE][U+5EA6][U+6A19][U+7C64][U+548C][U+6ED1][U+687F]
        control.brightness_label.configure(text=f"[U+4EAE][U+5EA6]: {brightness}")
        control.brightness_slider.set(brightness)
        
    def all_on(self):
        """[U+5168][U+90E8][U+958B][U+555F]"""
        for i in range(4):
            brightness = 255  # [U+8A2D][U+70BA][U+6700][U+5927][U+4EAE][U+5EA6]
            command = f"CH{i+1}:{brightness}"
            if self.send_rs232_command(command):
                self.led_states[i] = True
                self.led_brightness[i] = brightness
                self.update_led_ui(i)
            time.sleep(0.05)  # [U+5C0F][U+5EF6][U+9072][U+907F][U+514D][U+6307][U+4EE4][U+885D][U+7A81]
            
    def all_off(self):
        """[U+5168][U+90E8][U+95DC][U+9589]"""
        for i in range(4):
            command = f"CH{i+1}:0"
            if self.send_rs232_command(command):
                self.led_states[i] = False
                self.led_brightness[i] = 0
                self.update_led_ui(i)
            time.sleep(0.05)  # [U+5C0F][U+5EF6][U+9072][U+907F][U+514D][U+6307][U+4EE4][U+885D][U+7A81]
            
    def reset_device(self):
        """[U+91CD][U+7F6E][U+8A2D][U+5099]"""
        if self.send_rs232_command("RESET"):
            self.log_message("[U+5DF2][U+767C][U+9001][U+91CD][U+7F6E][U+6307][U+4EE4]")
            # [U+91CD][U+7F6E][U+672C][U+5730][U+72C0][U+614B]
            for i in range(4):
                self.led_states[i] = False
                self.led_brightness[i] = 0
                self.update_led_ui(i)
                
    def log_message(self, message):
        """[U+6DFB][U+52A0][U+65E5][U+8A8C][U+6D88][U+606F]"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert("end", log_entry)
        self.log_text.see("end")
        
        # [U+9650][U+5236][U+65E5][U+8A8C][U+884C][U+6578]
        lines = self.log_text.get("1.0", "end").split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", "2.0")
            
    def run(self):
        """[U+904B][U+884C][U+61C9][U+7528][U+7A0B][U+5E8F]"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
        
    def on_closing(self):
        """[U+95DC][U+9589][U+61C9][U+7528][U+7A0B][U+5E8F]"""
        if self.connected:
            self.disconnect()
        self.root.destroy()

def main():
    """[U+4E3B][U+51FD][U+6578]"""
    print("LED[U+63A7][U+5236][U+5668][U+6E2C][U+8A66][U+5DE5][U+5177][U+555F][U+52D5][U+4E2D]...")
    print("[U+652F][U+63F4][U+6307][U+4EE4][U+683C][U+5F0F]:")
    print("  CH1:255  - [U+8A2D][U+5B9A]L1[U+4EAE][U+5EA6][U+70BA]255")
    print("  CH2:0    - [U+95DC][U+9589]L2")
    print("  RESET    - [U+91CD][U+7F6E][U+8A2D][U+5099]")
    print("[U+652F][U+63F4][U+4EAE][U+5EA6][U+7BC4][U+570D]: 0-511")
    
    app = LEDTestTool()
    app.run()

if __name__ == "__main__":
    main()