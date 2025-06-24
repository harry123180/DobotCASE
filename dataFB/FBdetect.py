import cv2
import numpy as np
import os
import customtkinter as ctk
import threading
from PIL import Image, ImageTk
import queue
from skimage.feature import local_binary_pattern
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import json
from datetime import datetime
from tkinter import filedialog

# 全域變數用於ROI選擇
roi_selecting = False
roi_start_point = None
roi_end_point = None
roi_rect = None
roi_image_copy = None
current_image = None
roi_control_window = None
update_queue = queue.Queue()  # 用於線程間通訊
bmp_files = []  # 全域變數存儲圖片列表
current_index = 0  # 全域變數存儲當前圖片索引
data_folder = ""  # 全域變數存儲資料夾路徑

def mouse_callback(event, x, y, flags, param):
    """
    滑鼠回調函數用於ROI選擇
    """
    global roi_selecting, roi_start_point, roi_end_point, roi_rect, roi_image_copy, roi_control_window
    
    if event == cv2.EVENT_LBUTTONDOWN:
        roi_selecting = True
        roi_start_point = (x, y)
        roi_end_point = None
        roi_rect = None
        
    elif event == cv2.EVENT_MOUSEMOVE:
        if roi_selecting:
            roi_end_point = (x, y)
            
    elif event == cv2.EVENT_LBUTTONUP:
        if roi_selecting:
            roi_selecting = False
            roi_end_point = (x, y)
            
            # 計算ROI矩形
            if roi_start_point and roi_end_point:
                x1, y1 = roi_start_point
                x2, y2 = roi_end_point
                
                # 確保座標順序正確
                roi_rect = (min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1))
                print(f"ROI選擇完成: x={roi_rect[0]}, y={roi_rect[1]}, w={roi_rect[2]}, h={roi_rect[3]}")
                
                # 更新CTK介面
                if roi_control_window:
                    roi_control_window.update_roi_values(roi_rect)

def draw_roi_selection(image):
    """
    在圖像上繪製ROI選擇框
    """
    global roi_selecting, roi_start_point, roi_end_point, roi_rect
    
    img_copy = image.copy()
    
    # 繪製選擇中的矩形
    if roi_selecting and roi_start_point and roi_end_point:
        cv2.rectangle(img_copy, roi_start_point, roi_end_point, (0, 255, 0), 2)
        
    # 繪製確定的ROI矩形
    if roi_rect:
        x, y, w, h = roi_rect
        cv2.rectangle(img_copy, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(img_copy, "ROI", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return img_copy

def apply_roi_to_image(image):
    """
    將ROI套用到圖像上，返回裁切後的圖像
    """
    global roi_rect
    
    if roi_rect:
        x, y, w, h = roi_rect
        # 確保ROI範圍在圖像內
        height, width = image.shape[:2]
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = min(w, width - x)
        h = min(h, height - y)
        
        if w > 0 and h > 0:
            return image[y:y+h, x:x+w]
    
    return image

def read_bmp_with_chinese_path(file_path):
    """
    支援中文路徑的BMP檔案讀取
    """
    try:
        # 使用numpy讀取檔案來支援中文路徑
        with open(file_path, 'rb') as f:
            file_bytes = np.frombuffer(f.read(), dtype=np.uint8)
        
        # 使用cv2解碼圖像
        image = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
        
        if image is None:
            print(f"無法讀取圖像: {file_path}")
            return None
        
        return image
        
    except Exception as e:
        print(f"讀取檔案錯誤 {file_path}: {str(e)}")
        return None

def load_and_display_image(index):
    """載入並顯示指定索引的圖像"""
    global current_image, bmp_files, data_folder, roi_control_window
    
    if not bmp_files or index < 0 or index >= len(bmp_files):
        return False
    
    file_path = os.path.join(data_folder, bmp_files[index])
    print(f"正在顯示: {bmp_files[index]}")
    
    # 讀取圖像
    image = read_bmp_with_chinese_path(file_path)
    
    if image is not None:
        current_image = image.copy()
        
        # 更新CTK視窗的圖像
        if roi_control_window:
            roi_control_window.update_image(image)
            roi_control_window.update_file_info(bmp_files[index], index + 1, len(bmp_files))
        
        return True
    else:
        print(f"跳過無法讀取的檔案: {bmp_files[index]}")
        return False

def next_image():
    """切換到下一張圖片"""
    global current_index, bmp_files
    if bmp_files:
        current_index = (current_index + 1) % len(bmp_files)
        load_and_display_image(current_index)

def previous_image():
    """切換到上一張圖片"""
    global current_index, bmp_files
    if bmp_files:
        current_index = (current_index - 1) % len(bmp_files)
        load_and_display_image(current_index)

def calculate_contour_roughness(contour):
    """
    計算輪廓粗糙度
    """
    if len(contour) < 5:
        return 0
    
    # 計算輪廓周長
    perimeter = cv2.arcLength(contour, True)
    
    # 計算凸包
    hull = cv2.convexHull(contour)
    hull_perimeter = cv2.arcLength(hull, True)
    
    # 粗糙度 = 實際周長 / 凸包周長
    if hull_perimeter > 0:
        roughness = perimeter / hull_perimeter
    else:
        roughness = 1.0
    
    return roughness

def analyze_brightness_distribution(image):
    """
    分析亮度分布
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # 計算直方圖
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    
    # 計算統計特徵
    mean_brightness = np.mean(gray)
    std_brightness = np.std(gray)
    
    # 計算亮度分布的偏度和峰度
    flat_pixels = gray.flatten()
    
    # 中心矩
    normalized_pixels = (flat_pixels - mean_brightness) / std_brightness if std_brightness > 0 else flat_pixels
    
    # 偏度 (skewness)
    skewness = np.mean(normalized_pixels ** 3) if std_brightness > 0 else 0
    
    # 峰度 (kurtosis)
    kurtosis = np.mean(normalized_pixels ** 4) - 3 if std_brightness > 0 else 0
    
    return {
        'histogram': hist,
        'mean': mean_brightness,
        'std': std_brightness,
        'skewness': skewness,
        'kurtosis': kurtosis
    }

def create_brightness_heatmap(image):
    """
    創建亮度分布熱圖
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # 使用高斯模糊平滑化
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)
    
    # 轉換為彩色熱圖
    heatmap = cv2.applyColorMap(blurred, cv2.COLORMAP_JET)
    
    return heatmap

def process_image_pipeline(image, roi_control_window):
    """
    圖像處理管線，生成所有處理結果
    """
    if image is None:
        return {}
    
    results = {}
    
    try:
        # 原圖
        results['original'] = image.copy()
        
        # 轉換為灰階
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results['gray'] = gray
        
        # 套用ROI（如果有選擇的話）
        roi_gray = apply_roi_to_image(gray)
        results['roi'] = roi_gray
        
        # 取得圖像處理參數
        if roi_control_window:
            gaussian_kernel = roi_control_window.gaussian_kernel.get()
            use_otsu = roi_control_window.use_otsu.get()
            manual_threshold_value = roi_control_window.manual_threshold.get()
            canny_low = roi_control_window.canny_low.get()
            canny_high = roi_control_window.canny_high.get()
        else:
            gaussian_kernel = 5
            use_otsu = True
            manual_threshold_value = 127
            canny_low = 50
            canny_high = 150
        
        # 高斯模糊處理
        if gaussian_kernel > 1:
            # 確保核大小為奇數
            if gaussian_kernel % 2 == 0:
                gaussian_kernel += 1
            roi_gray_blurred = cv2.GaussianBlur(roi_gray, (gaussian_kernel, gaussian_kernel), 0)
        else:
            roi_gray_blurred = roi_gray
        
        results['blurred'] = roi_gray_blurred
        
        # 二值化處理
        if use_otsu:
            # OTSU自動閾值
            threshold_used, binary = cv2.threshold(roi_gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            # 手動閾值
            threshold_used, binary = cv2.threshold(roi_gray_blurred, manual_threshold_value, 255, cv2.THRESH_BINARY)
        
        results['binary'] = binary
        
        # Canny邊緣檢測
        canny = cv2.Canny(roi_gray_blurred, canny_low, canny_high)
        results['canny'] = canny
        
        # LBP紋理分析
        if roi_control_window:
            lbp_radius = roi_control_window.lbp_radius.get()
            lbp_points = roi_control_window.lbp_points.get()
        else:
            lbp_radius = 3
            lbp_points = 24
        
        lbp = local_binary_pattern(roi_gray, lbp_points, lbp_radius, method='uniform')
        # 將LBP結果歸一化到0-255範圍
        lbp_normalized = ((lbp - lbp.min()) / (lbp.max() - lbp.min()) * 255).astype(np.uint8)
        results['lbp'] = lbp_normalized
        
        # 輪廓粗糙度檢測
        # 先進行二值化處理
        if use_otsu:
            _, contour_binary = cv2.threshold(roi_gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            _, contour_binary = cv2.threshold(roi_gray_blurred, manual_threshold_value, 255, cv2.THRESH_BINARY)
        
        # 查找輪廓
        contours, _ = cv2.findContours(contour_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 創建輪廓顯示圖像
        contour_image = cv2.cvtColor(roi_gray_blurred, cv2.COLOR_GRAY2BGR)
        
        if contours:
            # 找到最大輪廓
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 計算粗糙度
            roughness = calculate_contour_roughness(largest_contour)
            
            # 繪製輪廓
            cv2.drawContours(contour_image, [largest_contour], -1, (0, 255, 0), 2)
            
            # 繪製凸包
            hull = cv2.convexHull(largest_contour)
            cv2.drawContours(contour_image, [hull], -1, (255, 0, 0), 2)
            
            # 在圖像上顯示粗糙度
            cv2.putText(contour_image, f'Roughness: {roughness:.3f}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        results['contour_analysis'] = contour_image
        
        # 亮度分布分析
        brightness_stats = analyze_brightness_distribution(roi_gray)
        results['brightness_stats'] = brightness_stats
        
        # 亮度分布熱圖
        heatmap = create_brightness_heatmap(roi_gray)
        results['brightness_heatmap'] = heatmap
        
        # 帶ROI框的原圖
        display_image = image.copy()
        if roi_rect:
            x, y, w, h = roi_rect
            # 確保座標在圖像範圍內
            height, width = display_image.shape[:2]
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            w = min(w, width - x)
            h = min(h, height - y)
            
            if w > 0 and h > 0:
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(display_image, "ROI", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        results['original_with_roi'] = display_image
        
        return results
        
    except Exception as e:
        print(f"圖像處理錯誤: {e}")
        return {}

def process_update_queue():
    """處理更新佇列中的任務"""
    try:
        while not update_queue.empty():
            task = update_queue.get_nowait()
            if task == "refresh_images":
                if roi_control_window:
                    roi_control_window.refresh_display()
    except queue.Empty:
        pass

class ROIControlWindow:
    def __init__(self):
        self.root = ctk.CTk()
        self.root.title("圖像處理控制面板")
        self.root.geometry("1200x800")
        
        # ROI參數變數
        self.x_var = ctk.IntVar(value=0)
        self.y_var = ctk.IntVar(value=0)
        self.w_var = ctk.IntVar(value=100)
        self.h_var = ctk.IntVar(value=100)
        
        # 圖像處理參數
        self.use_otsu = ctk.BooleanVar(value=True)  # 是否使用OTSU
        self.manual_threshold = ctk.IntVar(value=127)  # 手動閾值
        self.gaussian_kernel = ctk.IntVar(value=5)  # 高斯模糊核大小
        self.canny_low = ctk.IntVar(value=50)  # Canny低閾值
        self.canny_high = ctk.IntVar(value=150)  # Canny高閾值
        self.lbp_radius = ctk.IntVar(value=3)  # LBP半徑
        self.lbp_points = ctk.IntVar(value=24)  # LBP點數
        
        # 條件分類參數 - 修改為支援ID的版本
        self.classification_enabled = ctk.BooleanVar(value=False)
        self.show_classification_result = ctk.BooleanVar(value=True)  # 是否在圖像上顯示分類結果
        
        # 類別ID管理
        self.next_category_id = 1
        self.classification_categories = []
        self.classification_conditions = {}
        
        # 初始化預設類別
        self.init_default_categories()
        
        # 圖像顯示選項
        self.image_options = [
            "原圖",
            "原圖(含ROI框)",
            "灰階圖",
            "ROI區域",
            "高斯模糊",
            "二值化",
            "Canny邊緣",
            "LBP紋理分析",
            "輪廓粗糙度檢測",
            "亮度分布熱圖"
        ]
        
        self.left_image_var = ctk.StringVar(value="原圖(含ROI框)")
        self.right_image_var = ctk.StringVar(value="二值化")
        
        # ROI選擇變數
        self.roi_selecting = False
        self.roi_start_point = None
        self.roi_end_point = None
        self.left_canvas_image = None
        self.right_canvas_image = None
        self.left_tk_image = None
        self.right_tk_image = None
        self.scale_factor = 1.0
        
        # 圖像瀏覽控制變數
        self.left_zoom_factor = 1.0
        self.right_zoom_factor = 1.0
        self.left_pan_x = 0
        self.left_pan_y = 0
        self.right_pan_x = 0
        self.right_pan_y = 0
        self.left_pan_start = None
        self.right_pan_start = None
        self.left_panning = False
        self.right_panning = False
        
        # 當前處理結果
        self.processed_images = {}
        
        # 分類結果
        self.classification_result = None
        
        self.setup_ui()
    
    def get_next_category_id(self) -> int:
        """獲取下一個類別ID"""
        current_id = self.next_category_id
        self.next_category_id += 1
        return current_id
    
    def init_default_categories(self):
        """初始化預設類別 - 確保每個類別都有唯一ID"""
        # 創建預設類別
        category_a = {
            "id": self.get_next_category_id(),
            "name": "類型A",
            "enabled": True,
            "logic": "AND",
            "description": "高亮度物件分類"
        }
        
        category_b = {
            "id": self.get_next_category_id(),
            "name": "類型B",
            "enabled": True,
            "logic": "AND", 
            "description": "低亮度物件分類"
        }
        
        self.classification_categories = [category_a, category_b]
        
        # 初始化條件
        self.classification_conditions = {
            category_a["name"]: {
                "id": category_a["id"],
                "enabled": ctk.BooleanVar(value=True),
                "logic": ctk.StringVar(value="AND"),
                "conditions": [
                    {
                        "feature": ctk.StringVar(value="平均值"),
                        "operator": ctk.StringVar(value=">"),
                        "value": ctk.DoubleVar(value=100.0),
                        "enabled": ctk.BooleanVar(value=True),
                        "description": "亮度平均值大於100"
                    }
                ]
            },
            category_b["name"]: {
                "id": category_b["id"],
                "enabled": ctk.BooleanVar(value=True),
                "logic": ctk.StringVar(value="AND"),
                "conditions": [
                    {
                        "feature": ctk.StringVar(value="平均值"),
                        "operator": ctk.StringVar(value="<="),
                        "value": ctk.DoubleVar(value=100.0),
                        "enabled": ctk.BooleanVar(value=True),
                        "description": "亮度平均值小於等於100"
                    }
                ]
            }
        }
        
    def setup_ui(self):
        # 主容器分為左右兩部分
        main_container = ctk.CTkFrame(self.root)
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # 左側圖像顯示區域
        left_frame = ctk.CTkFrame(main_container)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        # 右側圖像顯示區域
        right_frame = ctk.CTkFrame(main_container)
        right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        # 左側圖像控制
        left_control_frame = ctk.CTkFrame(left_frame)
        left_control_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(left_control_frame, text="左側顯示:", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=5)
        self.left_dropdown = ctk.CTkComboBox(
            left_control_frame,
            values=self.image_options,
            variable=self.left_image_var,
            command=self.on_image_selection_changed,
            width=150
        )
        self.left_dropdown.pack(side="left", padx=10)
        
        # 左側Canvas
        self.left_canvas = ctk.CTkCanvas(left_frame, width=400, height=300, bg="gray")
        self.left_canvas.pack(fill="both", expand=True, padx=10, pady=5)
        
        # 綁定滑鼠事件到左側Canvas（用於ROI選擇和圖像瀏覽）
        self.left_canvas.bind("<Button-1>", self.on_left_canvas_click)
        self.left_canvas.bind("<B1-Motion>", self.on_left_canvas_drag)
        self.left_canvas.bind("<ButtonRelease-1>", self.on_left_canvas_release)
        self.left_canvas.bind("<Button-3>", self.on_left_canvas_right_click)  # 右鍵平移
        self.left_canvas.bind("<B3-Motion>", self.on_left_canvas_right_drag)
        self.left_canvas.bind("<ButtonRelease-3>", self.on_left_canvas_right_release)
        self.left_canvas.bind("<MouseWheel>", self.on_left_canvas_zoom)
        self.left_canvas.bind("<Double-Button-1>", self.on_left_canvas_reset)  # 雙擊重置
        
        # 右側圖像控制
        right_control_frame = ctk.CTkFrame(right_frame)
        right_control_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(right_control_frame, text="右側顯示:", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=5)
        self.right_dropdown = ctk.CTkComboBox(
            right_control_frame,
            values=self.image_options,
            variable=self.right_image_var,
            command=self.on_image_selection_changed,
            width=150
        )
        self.right_dropdown.pack(side="left", padx=10)
        
        # 右側Canvas
        self.right_canvas = ctk.CTkCanvas(right_frame, width=400, height=300, bg="gray")
        self.right_canvas.pack(fill="both", expand=True, padx=10, pady=5)
        
        # 綁定滑鼠事件到右側Canvas（用於圖像瀏覽）
        self.right_canvas.bind("<Button-3>", self.on_right_canvas_right_click)  # 右鍵平移
        self.right_canvas.bind("<B3-Motion>", self.on_right_canvas_right_drag)
        self.right_canvas.bind("<ButtonRelease-3>", self.on_right_canvas_right_release)
        self.right_canvas.bind("<MouseWheel>", self.on_right_canvas_zoom)
        self.right_canvas.bind("<Double-Button-1>", self.on_right_canvas_reset)  # 雙擊重置
        
        # 底部控制面板
        self.setup_control_panel()
        
    def setup_control_panel(self):
        # 底部控制面板
        control_panel = ctk.CTkFrame(self.root)
        control_panel.pack(fill="x", padx=10, pady=5)
        
        # 檔案導航區域
        nav_frame = ctk.CTkFrame(control_panel)
        nav_frame.pack(fill="x", padx=10, pady=5)
        
        nav_label = ctk.CTkLabel(nav_frame, text="圖片導航", font=ctk.CTkFont(size=16, weight="bold"))
        nav_label.pack(pady=5)
        
        nav_buttons = ctk.CTkFrame(nav_frame)
        nav_buttons.pack()
        
        prev_btn = ctk.CTkButton(nav_buttons, text="上一張", command=self.previous_image, width=100)
        prev_btn.pack(side="left", padx=5)
        
        next_btn = ctk.CTkButton(nav_buttons, text="下一張", command=self.next_image, width=100)
        next_btn.pack(side="left", padx=5)
        
        self.file_info_label = ctk.CTkLabel(nav_buttons, text="等待載入圖像")
        self.file_info_label.pack(side="left", padx=20)
        
        # ROI控制區域
        roi_frame = ctk.CTkFrame(control_panel)
        roi_frame.pack(fill="x", padx=10, pady=5)
        
        roi_label = ctk.CTkLabel(roi_frame, text="ROI控制", font=ctk.CTkFont(size=16, weight="bold"))
        roi_label.pack(pady=5)
        
        roi_coords = ctk.CTkFrame(roi_frame)
        roi_coords.pack()
        
        # ROI座標輸入
        ctk.CTkLabel(roi_coords, text="X:").grid(row=0, column=0, padx=5)
        self.x_entry = ctk.CTkEntry(roi_coords, textvariable=self.x_var, width=80)
        self.x_entry.grid(row=0, column=1, padx=5)
        self.x_entry.bind('<Return>', self.on_coord_changed)
        
        ctk.CTkLabel(roi_coords, text="Y:").grid(row=0, column=2, padx=5)
        self.y_entry = ctk.CTkEntry(roi_coords, textvariable=self.y_var, width=80)
        self.y_entry.grid(row=0, column=3, padx=5)
        self.y_entry.bind('<Return>', self.on_coord_changed)
        
        ctk.CTkLabel(roi_coords, text="寬:").grid(row=0, column=4, padx=5)
        self.w_entry = ctk.CTkEntry(roi_coords, textvariable=self.w_var, width=80)
        self.w_entry.grid(row=0, column=5, padx=5)
        self.w_entry.bind('<Return>', self.on_coord_changed)
        
        ctk.CTkLabel(roi_coords, text="高:").grid(row=0, column=6, padx=5)
        self.h_entry = ctk.CTkEntry(roi_coords, textvariable=self.h_var, width=80)
        self.h_entry.grid(row=0, column=7, padx=5)
        self.h_entry.bind('<Return>', self.on_coord_changed)
        
        roi_buttons = ctk.CTkFrame(roi_frame)
        roi_buttons.pack(pady=5)
        
        apply_btn = ctk.CTkButton(roi_buttons, text="套用ROI", command=self.apply_roi, width=100)
        apply_btn.pack(side="left", padx=5)
        
        clear_btn = ctk.CTkButton(roi_buttons, text="清除ROI", command=self.clear_roi, width=100)
        clear_btn.pack(side="left", padx=5)
        
        # 圖像處理參數區域改為左右布局
        params_container = ctk.CTkFrame(control_panel)
        params_container.pack(fill="x", padx=10, pady=5)
        
        # 左側: 圖像處理參數
        left_params_frame = ctk.CTkFrame(params_container)
        left_params_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        params_label = ctk.CTkLabel(left_params_frame, text="圖像處理參數", font=ctk.CTkFont(size=16, weight="bold"))
        params_label.pack(pady=5)
        
        # 參數控制區域分為多列
        params_grid = ctk.CTkFrame(left_params_frame)
        params_grid.pack(fill="x", padx=5)
        
        # 高斯模糊
        gaussian_frame = ctk.CTkFrame(params_grid)
        gaussian_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        ctk.CTkLabel(gaussian_frame, text="高斯核大小:").pack()
        self.gaussian_value_label = ctk.CTkLabel(gaussian_frame, text="5")
        self.gaussian_value_label.pack()
        self.gaussian_slider = ctk.CTkSlider(gaussian_frame, from_=1, to=15, number_of_steps=7, command=self.on_gaussian_changed)
        self.gaussian_slider.set(5)
        self.gaussian_slider.pack(fill="x", padx=5)
        
        # 閾值控制
        threshold_frame = ctk.CTkFrame(params_grid)
        threshold_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.otsu_checkbox = ctk.CTkCheckBox(threshold_frame, text="OTSU自動", variable=self.use_otsu, command=self.on_threshold_mode_changed)
        self.otsu_checkbox.pack()
        
        ctk.CTkLabel(threshold_frame, text="手動閾值:").pack()
        self.threshold_value_label = ctk.CTkLabel(threshold_frame, text="127")
        self.threshold_value_label.pack()
        self.threshold_slider = ctk.CTkSlider(threshold_frame, from_=0, to=255, number_of_steps=255, command=self.on_threshold_changed)
        self.threshold_slider.set(127)
        self.threshold_slider.pack(fill="x", padx=5)
        
        # Canny參數
        canny_frame = ctk.CTkFrame(params_grid)
        canny_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        ctk.CTkLabel(canny_frame, text="Canny低閾值:").pack()
        self.canny_low_label = ctk.CTkLabel(canny_frame, text="50")
        self.canny_low_label.pack()
        self.canny_low_slider = ctk.CTkSlider(canny_frame, from_=0, to=255, number_of_steps=255, command=self.on_canny_low_changed)
        self.canny_low_slider.set(50)
        self.canny_low_slider.pack(fill="x", padx=5)
        
        ctk.CTkLabel(canny_frame, text="Canny高閾值:").pack()
        self.canny_high_label = ctk.CTkLabel(canny_frame, text="150")
        self.canny_high_label.pack()
        self.canny_high_slider = ctk.CTkSlider(canny_frame, from_=0, to=255, number_of_steps=255, command=self.on_canny_high_changed)
        self.canny_high_slider.set(150)
        self.canny_high_slider.pack(fill="x", padx=5)
        
        # LBP參數
        lbp_frame = ctk.CTkFrame(params_grid)
        lbp_frame.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        ctk.CTkLabel(lbp_frame, text="LBP半徑:").pack()
        self.lbp_radius_label = ctk.CTkLabel(lbp_frame, text="3")
        self.lbp_radius_label.pack()
        self.lbp_radius_slider = ctk.CTkSlider(lbp_frame, from_=1, to=8, number_of_steps=7, command=self.on_lbp_radius_changed)
        self.lbp_radius_slider.set(3)
        self.lbp_radius_slider.pack(fill="x", padx=5)
        
        ctk.CTkLabel(lbp_frame, text="LBP點數:").pack()
        self.lbp_points_label = ctk.CTkLabel(lbp_frame, text="24")
        self.lbp_points_label.pack()
        self.lbp_points_slider = ctk.CTkSlider(lbp_frame, from_=8, to=32, number_of_steps=6, command=self.on_lbp_points_changed)
        self.lbp_points_slider.set(24)
        self.lbp_points_slider.pack(fill="x", padx=5)
        
        # 特徵統計顯示區域
        stats_frame = ctk.CTkFrame(left_params_frame)
        stats_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(stats_frame, text="特徵統計", font=ctk.CTkFont(size=14, weight="bold")).pack()
        self.stats_text = ctk.CTkTextbox(stats_frame, height=100, width=300)
        self.stats_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 右側: 條件分類
        right_params_frame = ctk.CTkFrame(params_container)
        right_params_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        # 條件分類區域
        self.setup_classification_panel(right_params_frame)
        
        self.update_threshold_controls()
        
        # 狀態顯示
        self.status_label = ctk.CTkLabel(control_panel, text="狀態: 等待圖像載入")
        self.status_label.pack(pady=5)
        
        # 操作說明
        help_frame = ctk.CTkFrame(control_panel)
        help_frame.pack(fill="x", padx=10, pady=5)
        
        help_text = "操作說明: 左鍵拖拽=ROI選擇(僅左側原圖) | 右鍵拖拽=平移圖像 | 滾輪=縮放 | 雙擊=重置視圖"
        help_label = ctk.CTkLabel(help_frame, text=help_text, font=ctk.CTkFont(size=12))
        help_label.pack(pady=5)
    
    def setup_classification_panel(self, parent):
        """設置條件分類面板 - 添加JSON匯出功能"""
        # 條件分類主框架
        classification_main_frame = ctk.CTkFrame(parent)
        classification_main_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 標題和控制區域
        header_frame = ctk.CTkFrame(classification_main_frame)
        header_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(header_frame, text="條件分類", font=ctk.CTkFont(size=16, weight="bold")).pack(side="left")
        
        # 右側控制按鈕
        controls_frame = ctk.CTkFrame(header_frame)
        controls_frame.pack(side="right")
        
        self.classification_checkbox = ctk.CTkCheckBox(
            controls_frame, 
            text="啟用",
            variable=self.classification_enabled,
            command=self.on_classification_toggle
        )
        self.classification_checkbox.pack(side="left", padx=5)
        
        show_result_checkbox = ctk.CTkCheckBox(
            controls_frame,
            text="顯示結果",
            variable=self.show_classification_result,
            command=self.refresh_display
        )
        show_result_checkbox.pack(side="left", padx=5)
        
        # 類別管理區域
        category_management_frame = ctk.CTkFrame(classification_main_frame)
        category_management_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(category_management_frame, text="類別管理:", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left")
        
        add_category_btn = ctk.CTkButton(
            category_management_frame,
            text="新增類別",
            command=self.add_category,
            width=80
        )
        add_category_btn.pack(side="right", padx=2)
        
        # JSON匯出區域
        export_frame = ctk.CTkFrame(classification_main_frame)
        export_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(export_frame, text="配置匯出:", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left")
        
        export_btn = ctk.CTkButton(
            export_frame,
            text="匯出JSON配置",
            command=self.export_classification_config,
            width=120
        )
        export_btn.pack(side="right", padx=2)
        
        preview_btn = ctk.CTkButton(
            export_frame,
            text="預覽配置",
            command=self.preview_classification_config,
            width=100
        )
        preview_btn.pack(side="right", padx=2)
        
        # 分類設置框架 - 使用滾動區域
        self.classification_scroll_frame = ctk.CTkScrollableFrame(classification_main_frame)
        self.classification_scroll_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 創建分類設置
        self.create_classification_categories()
        
        # 分類結果顯示
        result_frame = ctk.CTkFrame(classification_main_frame)
        result_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(result_frame, text="分類結果:", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left")
        self.classification_result_label = ctk.CTkLabel(result_frame, text="等待分析", font=ctk.CTkFont(size=14))
        self.classification_result_label.pack(side="left", padx=10)
        
        # 初始狀態設置
        self.update_classification_panel_state()
    
    def add_category(self):
        """新增分類類別 - 修改版本，添加ID管理"""
        # 創建自定義新增對話框
        dialog_window = ctk.CTkToplevel(self.root)
        dialog_window.title("新增類別")
        dialog_window.geometry("300x200")
        dialog_window.transient(self.root)
        dialog_window.grab_set()
        
        # 置中顯示
        dialog_window.update_idletasks()
        x = (dialog_window.winfo_screenwidth() // 2) - (300 // 2)
        y = (dialog_window.winfo_screenheight() // 2) - (200 // 2)
        dialog_window.geometry(f"300x200+{x}+{y}")
        
        # 對話框內容
        ctk.CTkLabel(dialog_window, text="請輸入新類別資訊:", font=ctk.CTkFont(size=14)).pack(pady=10)
        
        # 類別名稱
        ctk.CTkLabel(dialog_window, text="類別名稱:").pack()
        name_entry = ctk.CTkEntry(dialog_window, width=200)
        name_entry.pack(pady=5)
        name_entry.focus()
        
        # 類別描述
        ctk.CTkLabel(dialog_window, text="類別描述:").pack()
        desc_entry = ctk.CTkEntry(dialog_window, width=200)
        desc_entry.pack(pady=5)
        
        result = None
        
        def on_ok():
            nonlocal result
            name = name_entry.get().strip()
            description = desc_entry.get().strip()
            if name:
                result = {
                    "name": name,
                    "description": description if description else f"{name}分類"
                }
            dialog_window.destroy()
        
        def on_cancel():
            nonlocal result
            result = None
            dialog_window.destroy()
        
        # 按鈕區域
        button_frame = ctk.CTkFrame(dialog_window)
        button_frame.pack(pady=10)
        
        ok_btn = ctk.CTkButton(button_frame, text="確定", command=on_ok)
        ok_btn.pack(side="left", padx=5)
        
        cancel_btn = ctk.CTkButton(button_frame, text="取消", command=on_cancel)
        cancel_btn.pack(side="left", padx=5)
        
        # 綁定Enter鍵
        name_entry.bind('<Return>', lambda e: on_ok())
        dialog_window.bind('<Escape>', lambda e: on_cancel())
        
        # 等待對話框關閉
        dialog_window.wait_window()
        
        # 處理結果
        if result and result["name"] not in [cat["name"] for cat in self.classification_categories]:
            new_category = {
                "id": self.get_next_category_id(),
                "name": result["name"],
                "enabled": True,
                "logic": "AND",
                "description": result["description"]
            }
            
            self.classification_categories.append(new_category)
            
            # 初始化新類別的條件
            self.classification_conditions[result["name"]] = {
                "id": new_category["id"],
                "enabled": ctk.BooleanVar(value=True),
                "logic": ctk.StringVar(value="AND"),
                "conditions": [
                    {
                        "feature": ctk.StringVar(value="平均值"),
                        "operator": ctk.StringVar(value=">"),
                        "value": ctk.DoubleVar(value=0.0),
                        "enabled": ctk.BooleanVar(value=True),
                        "description": "預設條件"
                    }
                ]
            }
            
            # 重新創建界面
            self.create_classification_categories()
            self.evaluate_classification()
            print(f"新增類別: ID={new_category['id']}, 名稱='{result['name']}'")
    
    def edit_category_name(self, old_name):
        """編輯類別名稱"""
        # 創建自定義編輯對話框
        dialog_window = ctk.CTkToplevel(self.root)
        dialog_window.title("編輯類別")
        dialog_window.geometry("300x150")
        dialog_window.transient(self.root)
        dialog_window.grab_set()
        
        # 置中顯示
        dialog_window.update_idletasks()
        x = (dialog_window.winfo_screenwidth() // 2) - (300 // 2)
        y = (dialog_window.winfo_screenheight() // 2) - (150 // 2)
        dialog_window.geometry(f"300x150+{x}+{y}")
        
        # 對話框內容
        ctk.CTkLabel(dialog_window, text="編輯類別名稱:", font=ctk.CTkFont(size=14)).pack(pady=10)
        
        entry = ctk.CTkEntry(dialog_window, width=200)
        entry.pack(pady=10)
        entry.insert(0, old_name)  # 設置初始值
        entry.focus()
        
        result = None
        
        def on_ok():
            nonlocal result
            result = entry.get().strip()
            dialog_window.destroy()
        
        def on_cancel():
            nonlocal result
            result = None
            dialog_window.destroy()
        
        # 按鈕區域
        button_frame = ctk.CTkFrame(dialog_window)
        button_frame.pack(pady=10)
        
        ok_btn = ctk.CTkButton(button_frame, text="確定", command=on_ok)
        ok_btn.pack(side="left", padx=5)
        
        cancel_btn = ctk.CTkButton(button_frame, text="取消", command=on_cancel)
        cancel_btn.pack(side="left", padx=5)
        
        # 綁定Enter鍵
        entry.bind('<Return>', lambda e: on_ok())
        dialog_window.bind('<Escape>', lambda e: on_cancel())
        
        # 等待對話框關閉
        dialog_window.wait_window()
        
        # 處理結果
        if result and result != old_name and result not in [cat["name"] for cat in self.classification_categories]:
            # 找到對應的類別並更新名稱
            for category in self.classification_categories:
                if category["name"] == old_name:
                    category["name"] = result
                    break
            
            # 更新條件字典
            self.classification_conditions[result] = self.classification_conditions.pop(old_name)
            
            # 重新創建界面
            self.create_classification_categories()
            self.evaluate_classification()
            print(f"類別名稱已從 '{old_name}' 更改為 '{result}'")
    
    def remove_category(self, category_name):
        """移除類別"""
        if len(self.classification_categories) > 1:  # 至少保留一個類別
            # 移除類別
            self.classification_categories = [cat for cat in self.classification_categories if cat["name"] != category_name]
            
            # 移除條件
            if category_name in self.classification_conditions:
                del self.classification_conditions[category_name]
            
            # 重新創建界面
            self.create_classification_categories()
            self.evaluate_classification()
    
    def create_classification_categories(self):
        """創建分類類別設置"""
        # 清空現有內容
        for widget in self.classification_scroll_frame.winfo_children():
            widget.destroy()
        
        # 為每個類別創建設置區域
        for i, category in enumerate(self.classification_categories):
            self.create_category_panel(category, i)
    
    def create_category_panel(self, category, index):
        """創建單個類別的設置面板"""
        category_name = category["name"]
        
        # 類別主框架
        category_frame = ctk.CTkFrame(self.classification_scroll_frame)
        category_frame.pack(fill="x", padx=5, pady=5)
        
        # 類別標題和控制
        title_frame = ctk.CTkFrame(category_frame)
        title_frame.pack(fill="x", padx=5, pady=5)
        
        # 左側：類別名稱、ID和啟用
        left_title_frame = ctk.CTkFrame(title_frame)
        left_title_frame.pack(side="left", fill="x", expand=True)
        
        category_label = ctk.CTkLabel(left_title_frame, text=f"{category_name} (ID: {category['id']}):", font=ctk.CTkFont(size=14, weight="bold"))
        category_label.pack(side="left")
        
        category_enabled = ctk.CTkCheckBox(
            left_title_frame,
            text="啟用",
            variable=self.classification_conditions[category_name]["enabled"],
            command=lambda: self.evaluate_classification()
        )
        category_enabled.pack(side="left", padx=10)
        
        # 右側：管理按鈕
        right_title_frame = ctk.CTkFrame(title_frame)
        right_title_frame.pack(side="right")
        
        edit_btn = ctk.CTkButton(
            right_title_frame,
            text="編輯",
            command=lambda: self.edit_category_name(category_name),
            width=50
        )
        edit_btn.pack(side="left", padx=2)
        
        if len(self.classification_categories) > 1:
            remove_btn = ctk.CTkButton(
                right_title_frame,
                text="刪除",
                command=lambda: self.remove_category(category_name),
                width=50
            )
            remove_btn.pack(side="left", padx=2)
        
        # 邏輯運算選擇
        logic_frame = ctk.CTkFrame(category_frame)
        logic_frame.pack(fill="x", padx=5, pady=2)
        
        ctk.CTkLabel(logic_frame, text="條件關係:").pack(side="left")
        logic_combo = ctk.CTkComboBox(
            logic_frame,
            values=["AND", "OR"],
            variable=self.classification_conditions[category_name]["logic"],
            command=lambda x: self.evaluate_classification(),
            width=80
        )
        logic_combo.pack(side="left", padx=5)
        
        # 條件列表框架
        conditions_frame = ctk.CTkFrame(category_frame)
        conditions_frame.pack(fill="x", padx=5, pady=5)
        
        # 顯示現有條件
        self.update_category_conditions(category_name, conditions_frame)
        
        # 新增條件按鈕
        add_btn = ctk.CTkButton(
            category_frame,
            text=f"新增條件",
            command=lambda: self.add_condition(category_name),
            width=100
        )
        add_btn.pack(pady=5)
    
    def update_category_conditions(self, category_name, parent_frame):
        """更新類別的條件顯示"""
        # 清空現有條件顯示
        for widget in parent_frame.winfo_children():
            widget.destroy()
        
        conditions = self.classification_conditions[category_name]["conditions"]
        
        for i, condition in enumerate(conditions):
            self.create_condition_widget(parent_frame, category_name, condition, i)
    
    def create_condition_widget(self, parent, category_name, condition, index):
        """創建單個條件的控制元件"""
        condition_frame = ctk.CTkFrame(parent)
        condition_frame.pack(fill="x", padx=5, pady=2)
        
        # 啟用條件的勾選框
        enabled_check = ctk.CTkCheckBox(
            condition_frame,
            text="",
            variable=condition["enabled"],
            command=lambda: self.evaluate_classification(),
            width=20
        )
        enabled_check.pack(side="left", padx=2)
        
        # 特徵選擇
        feature_combo = ctk.CTkComboBox(
            condition_frame,
            values=["平均值", "標準差", "偏度", "峰度"],
            variable=condition["feature"],
            command=lambda x: self.evaluate_classification(),
            width=80
        )
        feature_combo.pack(side="left", padx=2)
        
        # 運算符選擇
        operator_combo = ctk.CTkComboBox(
            condition_frame,
            values=[">", ">=", "<", "<=", "=="],
            variable=condition["operator"],
            command=lambda x: self.evaluate_classification(),
            width=60
        )
        operator_combo.pack(side="left", padx=2)
        
        # 數值輸入 - 支援小數點
        value_entry = ctk.CTkEntry(
            condition_frame,
            width=100,
            placeholder_text="0.0"
        )
        value_entry.pack(side="left", padx=2)
        
        # 設置初始值
        initial_value = condition["value"].get()
        value_entry.insert(0, str(initial_value))
        
        # 數值驗證和更新函數
        def validate_and_update():
            try:
                text = value_entry.get().strip()
                if text == "" or text == "-" or text == ".":
                    return True
                
                value = float(text)
                condition["value"].set(value)
                
                if hasattr(self, '_classification_timer'):
                    self.root.after_cancel(self._classification_timer)
                self._classification_timer = self.root.after(300, self.evaluate_classification)
                
                return True
            except ValueError:
                return True
        
        def on_focus_out(event):
            try:
                text = value_entry.get().strip()
                if text == "" or text == "-" or text == ".":
                    value_entry.delete(0, "end")
                    value_entry.insert(0, "0.0")
                    condition["value"].set(0.0)
                else:
                    value = float(text)
                    condition["value"].set(value)
                    value_entry.delete(0, "end")
                    value_entry.insert(0, str(value))
                
                self.evaluate_classification()
            except ValueError:
                value_entry.delete(0, "end")
                value_entry.insert(0, str(condition["value"].get()))
        
        def on_key_release(event):
            validate_and_update()
        
        # 綁定事件
        value_entry.bind('<KeyRelease>', on_key_release)
        value_entry.bind('<FocusOut>', on_focus_out)
        
        # 刪除按鈕
        delete_btn = ctk.CTkButton(
            condition_frame,
            text="刪除",
            command=lambda: self.remove_condition(category_name, index),
            width=60
        )
        delete_btn.pack(side="right", padx=2)
    
    def add_condition(self, category_name):
        """新增條件"""
        new_condition = {
            "feature": ctk.StringVar(value="平均值"),
            "operator": ctk.StringVar(value=">"),
            "value": ctk.DoubleVar(value=0.0),
            "enabled": ctk.BooleanVar(value=True),
            "description": "新條件"
        }
        
        self.classification_conditions[category_name]["conditions"].append(new_condition)
        self.create_classification_categories()
        self.evaluate_classification()
    
    def remove_condition(self, category_name, index):
        """移除條件"""
        if len(self.classification_conditions[category_name]["conditions"]) > 1:
            del self.classification_conditions[category_name]["conditions"][index]
            self.create_classification_categories()
            self.evaluate_classification()
    
    def export_classification_config(self):
        """匯出分類配置為JSON"""
        try:
            config = self.generate_classification_json()
            
            # 獲取當前時間作為檔案名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"classification_config_{timestamp}.json"
            
            # 使用檔案對話框選擇儲存位置 - 修正參數名稱
            filepath = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                initialfile=filename,  # 修正: 使用 initialfile 而不是 initialvalue
                title="儲存分類配置"
            )
            
            if filepath:
                with open(filepath, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                
                print(f"分類配置已匯出: {filepath}")
                
                # 顯示成功訊息
                success_window = ctk.CTkToplevel(self.root)
                success_window.title("匯出成功")
                success_window.geometry("500x150")
                success_window.transient(self.root)
                success_window.grab_set()
                
                # 置中顯示
                success_window.update_idletasks()
                x = (success_window.winfo_screenwidth() // 2) - (500 // 2)
                y = (success_window.winfo_screenheight() // 2) - (150 // 2)
                success_window.geometry(f"500x150+{x}+{y}")
                
                ctk.CTkLabel(success_window, text="配置匯出成功！", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=20)
                ctk.CTkLabel(success_window, text=f"檔案位置: {filepath}", font=ctk.CTkFont(size=12)).pack(pady=10)
                
                ctk.CTkButton(success_window, text="確定", command=success_window.destroy).pack(pady=10)
            
        except Exception as e:
            print(f"匯出配置失敗: {e}")
            # 顯示錯誤訊息
            error_window = ctk.CTkToplevel(self.root)
            error_window.title("匯出失敗")
            error_window.geometry("400x150")
            error_window.transient(self.root)
            error_window.grab_set()
            
            # 置中顯示
            error_window.update_idletasks()
            x = (error_window.winfo_screenwidth() // 2) - (400 // 2)
            y = (error_window.winfo_screenheight() // 2) - (150 // 2)
            error_window.geometry(f"400x150+{x}+{y}")
            
            ctk.CTkLabel(error_window, text="配置匯出失敗！", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=20)
            ctk.CTkLabel(error_window, text=f"錯誤: {str(e)}", font=ctk.CTkFont(size=12)).pack(pady=10)
            
            ctk.CTkButton(error_window, text="確定", command=error_window.destroy).pack(pady=10)
    
    def preview_classification_config(self):
        """預覽分類配置"""
        try:
            config = self.generate_classification_json()
            
            # 創建預覽視窗
            preview_window = ctk.CTkToplevel(self.root)
            preview_window.title("配置預覽")
            preview_window.geometry("800x600")
            preview_window.transient(self.root)
            
            # 置中顯示
            preview_window.update_idletasks()
            x = (preview_window.winfo_screenwidth() // 2) - (800 // 2)
            y = (preview_window.winfo_screenheight() // 2) - (600 // 2)
            preview_window.geometry(f"800x600+{x}+{y}")
            
            # 標題
            ctk.CTkLabel(preview_window, text="分類配置預覽", font=ctk.CTkFont(size=18, weight="bold")).pack(pady=10)
            
            # JSON內容顯示
            text_frame = ctk.CTkFrame(preview_window)
            text_frame.pack(fill="both", expand=True, padx=20, pady=10)
            
            text_widget = ctk.CTkTextbox(text_frame, width=750, height=500, font=ctk.CTkFont(family="Courier", size=11))
            text_widget.pack(fill="both", expand=True, padx=10, pady=10)
            
            # 插入JSON內容
            json_text = json.dumps(config, indent=2, ensure_ascii=False)
            text_widget.insert("1.0", json_text)
            
            # 底部按鈕
            button_frame = ctk.CTkFrame(preview_window)
            button_frame.pack(fill="x", padx=20, pady=10)
            
            close_btn = ctk.CTkButton(button_frame, text="關閉", command=preview_window.destroy)
            close_btn.pack(side="right", padx=5)
            
            copy_btn = ctk.CTkButton(button_frame, text="複製到剪貼板", command=lambda: self.copy_to_clipboard(json_text))
            copy_btn.pack(side="right", padx=5)
            
        except Exception as e:
            print(f"預覽配置失敗: {e}")
    
    def copy_to_clipboard(self, text):
        """複製文字到剪貼板"""
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            print("配置已複製到剪貼板")
        except Exception as e:
            print(f"複製失敗: {e}")
    
    def generate_classification_json(self) -> dict:
        """生成分類配置JSON"""
        config = {
            "module_info": {
                "module_name": "CCD2分類模組",
                "version": "1.0",
                "created_date": datetime.now().strftime("%Y-%m-%d"),
                "description": "基於圖像特徵的分類條件配置",
                "image_processing_pipeline": [
                    "灰階轉換",
                    "ROI區域提取",
                    "高斯模糊",
                    "二值化處理", 
                    "Canny邊緣檢測",
                    "LBP紋理分析",
                    "輪廓粗糙度檢測",
                    "亮度分布分析"
                ]
            },
            "categories": [],
            "processing_parameters": {
                "gaussian_kernel": self.gaussian_kernel.get(),
                "use_otsu": self.use_otsu.get(),
                "manual_threshold": self.manual_threshold.get(),
                "canny_low": self.canny_low.get(),
                "canny_high": self.canny_high.get(),
                "lbp_radius": self.lbp_radius.get(),
                "lbp_points": self.lbp_points.get(),
                "roi": {
                    "enabled": bool(roi_rect is not None),
                    "x": roi_rect[0] if roi_rect else 0,
                    "y": roi_rect[1] if roi_rect else 0,
                    "width": roi_rect[2] if roi_rect else 100,
                    "height": roi_rect[3] if roi_rect else 100
                }
            },
            "feature_definitions": {
                "平均值": {
                    "type": "brightness_stats",
                    "field": "mean",
                    "unit": "灰階值",
                    "range": [0, 255],
                    "description": "圖像亮度平均值"
                },
                "標準差": {
                    "type": "brightness_stats",
                    "field": "std", 
                    "unit": "灰階值",
                    "range": [0, 255],
                    "description": "圖像亮度標準差"
                },
                "偏度": {
                    "type": "brightness_stats",
                    "field": "skewness",
                    "unit": "統計值",
                    "range": [-3, 3],
                    "description": "亮度分布偏度"
                },
                "峰度": {
                    "type": "brightness_stats",
                    "field": "kurtosis",
                    "unit": "統計值",
                    "range": [-3, 10],
                    "description": "亮度分布峰度"
                }
            },
            "operator_definitions": {
                ">": "大於",
                ">=": "大於等於",
                "<": "小於", 
                "<=": "小於等於",
                "==": "等於"
            },
            "logic_definitions": {
                "AND": "所有條件都必須滿足",
                "OR": "任一條件滿足即可"
            }
        }
        
        # 轉換類別資訊
        for category in self.classification_categories:
            category_name = category["name"]
            category_config = self.classification_conditions.get(category_name)
            
            if category_config:
                category_data = {
                    "id": category["id"],
                    "name": category["name"],
                    "enabled": category_config["enabled"].get(),
                    "logic": category_config["logic"].get(),
                    "description": category.get("description", f"{category_name}分類"),
                    "conditions": []
                }
                
                # 轉換條件
                for condition in category_config["conditions"]:
                    condition_data = {
                        "feature": condition["feature"].get(),
                        "operator": condition["operator"].get(),
                        "threshold": condition["value"].get(),
                        "enabled": condition["enabled"].get(),
                        "description": condition.get("description", "")
                    }
                    category_data["conditions"].append(condition_data)
                
                config["categories"].append(category_data)
        
        return config
    
    def on_classification_toggle(self):
        """條件分類啟用/停用切換"""
        self.update_classification_panel_state()
        if self.classification_enabled.get():
            self.evaluate_classification()
        else:
            self.classification_result_label.configure(text="已停用")
    
    def update_classification_panel_state(self):
        """更新條件分類面板的啟用狀態"""
        enabled = self.classification_enabled.get()
        
        # 設置內容框架的狀態
        for widget in self.classification_scroll_frame.winfo_children():
            self.set_widget_state(widget, "normal" if enabled else "disabled")
    
    def set_widget_state(self, widget, state):
        """遞迴設置控件狀態"""
        try:
            if hasattr(widget, 'configure'):
                if isinstance(widget, (ctk.CTkButton, ctk.CTkComboBox, ctk.CTkEntry, ctk.CTkCheckBox)):
                    widget.configure(state=state)
        except:
            pass
        
        # 遞迴處理子控件
        for child in widget.winfo_children():
            self.set_widget_state(child, state)
    
    def evaluate_classification(self):
        """評估分類條件 - 修改版本，返回類別ID而非名稱"""
        if not self.classification_enabled.get() or 'brightness_stats' not in self.processed_images:
            return
        
        stats = self.processed_images['brightness_stats']
        
        # 特徵值映射
        feature_values = {
            "平均值": stats['mean'],
            "標準差": stats['std'],
            "偏度": stats['skewness'],
            "峰度": stats['kurtosis']
        }
        
        results = []
        
        # 評估每個類別
        for category in self.classification_categories:
            category_name = category["name"]
            category_conditions = self.classification_conditions.get(category_name)
            
            if not category_conditions or not category_conditions["enabled"].get():
                continue
            
            conditions = category_conditions["conditions"]
            logic = category_conditions["logic"].get()
            
            if not conditions:
                continue
            
            # 評估所有條件
            condition_results = []
            for condition in conditions:
                if not condition["enabled"].get():
                    continue
                
                try:
                    feature = condition["feature"].get()
                    operator = condition["operator"].get()
                    threshold = condition["value"].get()
                    
                    if feature in feature_values:
                        value = feature_values[feature]
                        
                        if operator == ">":
                            result = value > threshold
                        elif operator == ">=":
                            result = value >= threshold
                        elif operator == "<":
                            result = value < threshold
                        elif operator == "<=":
                            result = value <= threshold
                        elif operator == "==":
                            result = abs(value - threshold) < 0.001
                        else:
                            result = False
                        
                        condition_results.append(result)
                except:
                    continue
            
            if condition_results:
                # 根據邏輯運算計算最終結果
                if logic == "AND":
                    category_result = all(condition_results)
                else:  # OR
                    category_result = any(condition_results)
                
                if category_result:
                    results.append({
                        "id": category["id"],
                        "name": category_name
                    })
        
        # 更新分類結果顯示
        if results:
            # 優先顯示第一個匹配的類別
            first_result = results[0]
            result_text = f"ID: {first_result['id']} ({first_result['name']})"
            self.classification_result = first_result
        else:
            result_text = "無匹配"
            self.classification_result = None
        
        self.classification_result_label.configure(text=result_text)
        
        # 更新統計顯示
        self.update_feature_stats()
    
    def on_gaussian_changed(self, value):
        """高斯模糊滑桿變更事件"""
        kernel_size = int(value)
        if kernel_size % 2 == 0:
            kernel_size += 1
        
        self.gaussian_kernel.set(kernel_size)
        self.gaussian_value_label.configure(text=str(kernel_size))
        self.refresh_display()
    
    def on_threshold_changed(self, value):
        """手動閾值滑桿變更事件"""
        threshold_value = int(value)
        self.manual_threshold.set(threshold_value)
        self.threshold_value_label.configure(text=str(threshold_value))
        self.refresh_display()
    
    def on_threshold_mode_changed(self):
        """閾值模式切換事件"""
        self.update_threshold_controls()
        self.refresh_display()
    
    def on_canny_low_changed(self, value):
        """Canny低閾值變更事件"""
        canny_value = int(value)
        self.canny_low.set(canny_value)
        self.canny_low_label.configure(text=str(canny_value))
        self.refresh_display()
    
    def on_canny_high_changed(self, value):
        """Canny高閾值變更事件"""
        canny_value = int(value)
        self.canny_high.set(canny_value)
        self.canny_high_label.configure(text=str(canny_value))
        self.refresh_display()
    
    def on_lbp_radius_changed(self, value):
        """LBP半徑變更事件"""
        radius_value = int(value)
        self.lbp_radius.set(radius_value)
        self.lbp_radius_label.configure(text=str(radius_value))
        self.refresh_display()
    
    def on_lbp_points_changed(self, value):
        """LBP點數變更事件"""
        points_value = int(value)
        # 確保是8的倍數
        points_value = max(8, (points_value // 8) * 8)
        self.lbp_points.set(points_value)
        self.lbp_points_label.configure(text=str(points_value))
        self.refresh_display()
    
    def update_threshold_controls(self):
        """更新閾值控制項的啟用狀態"""
        is_manual = not self.use_otsu.get()
        
        if is_manual:
            self.threshold_slider.configure(state="normal")
            self.threshold_value_label.configure(text_color=("black", "white"))
        else:
            self.threshold_slider.configure(state="disabled")
            self.threshold_value_label.configure(text_color=("gray", "gray"))
    
    def on_image_selection_changed(self, value=None):
        """圖像選擇下拉選單變更事件"""
        self.refresh_display()
    
    def on_left_canvas_click(self, event):
        """左側Canvas滑鼠點擊事件 - ROI選擇"""
        # 只有在顯示原圖相關內容時才允許ROI選擇
        if self.left_image_var.get() not in ["原圖", "原圖(含ROI框)"]:
            return
            
        self.roi_selecting = True
        canvas_x, canvas_y = event.x, event.y
        img_x, img_y = self.canvas_to_image_coords(canvas_x, canvas_y, "left")
        self.roi_start_point = (img_x, img_y)
        self.roi_end_point = None
        
        self.left_canvas.delete("roi_rect")
        self.left_canvas.delete("roi_temp")
    
    def on_left_canvas_drag(self, event):
        """左側Canvas滑鼠拖拽事件 - ROI選擇"""
        if self.roi_selecting and self.roi_start_point:
            canvas_x, canvas_y = event.x, event.y
            img_x, img_y = self.canvas_to_image_coords(canvas_x, canvas_y, "left")
            self.roi_end_point = (img_x, img_y)
            
            self.draw_temp_roi_on_canvas()
    
    def on_left_canvas_release(self, event):
        """左側Canvas滑鼠釋放事件 - ROI選擇"""
        if self.roi_selecting and self.roi_start_point:
            canvas_x, canvas_y = event.x, event.y
            img_x, img_y = self.canvas_to_image_coords(canvas_x, canvas_y, "left")
            self.roi_end_point = (img_x, img_y)
            
            if self.roi_start_point and self.roi_end_point:
                x1, y1 = self.roi_start_point
                x2, y2 = self.roi_end_point
                
                x = min(x1, x2)
                y = min(y1, y2)
                w = abs(x2 - x1)
                h = abs(y2 - y1)
                
                if w > 0 and h > 0:
                    roi_rect = (x, y, w, h)
                    self.update_roi_values(roi_rect)
                    self.apply_roi()
        
        self.roi_selecting = False
        self.left_canvas.delete("roi_temp")
    
    # 左側Canvas平移和縮放事件
    def on_left_canvas_right_click(self, event):
        """左側Canvas右鍵點擊 - 開始平移"""
        self.left_panning = True
        self.left_pan_start = (event.x, event.y)
    
    def on_left_canvas_right_drag(self, event):
        """左側Canvas右鍵拖拽 - 平移圖像"""
        if self.left_panning and self.left_pan_start:
            dx = event.x - self.left_pan_start[0]
            dy = event.y - self.left_pan_start[1]
            
            self.left_pan_x += dx
            self.left_pan_y += dy
            
            self.left_pan_start = (event.x, event.y)
            self.update_canvas_image(self.left_canvas, self.get_image_by_selection(self.left_image_var.get()), "left")
    
    def on_left_canvas_right_release(self, event):
        """左側Canvas右鍵釋放 - 停止平移"""
        self.left_panning = False
        self.left_pan_start = None
    
    def on_left_canvas_zoom(self, event):
        """左側Canvas滾輪縮放"""
        if event.delta > 0:
            zoom_factor = 1.1
        else:
            zoom_factor = 0.9
        
        # 限制縮放範圍
        new_zoom = self.left_zoom_factor * zoom_factor
        if 0.1 <= new_zoom <= 5.0:
            self.left_zoom_factor = new_zoom
            self.update_canvas_image(self.left_canvas, self.get_image_by_selection(self.left_image_var.get()), "left")
    
    def on_left_canvas_reset(self, event):
        """左側Canvas雙擊重置視圖"""
        self.left_zoom_factor = 1.0
        self.left_pan_x = 0
        self.left_pan_y = 0
        self.update_canvas_image(self.left_canvas, self.get_image_by_selection(self.left_image_var.get()), "left")
    
    # 右側Canvas平移和縮放事件
    def on_right_canvas_right_click(self, event):
        """右側Canvas右鍵點擊 - 開始平移"""
        self.right_panning = True
        self.right_pan_start = (event.x, event.y)
    
    def on_right_canvas_right_drag(self, event):
        """右側Canvas右鍵拖拽 - 平移圖像"""
        if self.right_panning and self.right_pan_start:
            dx = event.x - self.right_pan_start[0]
            dy = event.y - self.right_pan_start[1]
            
            self.right_pan_x += dx
            self.right_pan_y += dy
            
            self.right_pan_start = (event.x, event.y)
            self.update_canvas_image(self.right_canvas, self.get_image_by_selection(self.right_image_var.get()), "right")
    
    def on_right_canvas_right_release(self, event):
        """右側Canvas右鍵釋放 - 停止平移"""
        self.right_panning = False
        self.right_pan_start = None
    
    def on_right_canvas_zoom(self, event):
        """右側Canvas滾輪縮放"""
        if event.delta > 0:
            zoom_factor = 1.1
        else:
            zoom_factor = 0.9
        
        # 限制縮放範圍
        new_zoom = self.right_zoom_factor * zoom_factor
        if 0.1 <= new_zoom <= 5.0:
            self.right_zoom_factor = new_zoom
            self.update_canvas_image(self.right_canvas, self.get_image_by_selection(self.right_image_var.get()), "right")
    
    def on_right_canvas_reset(self, event):
        """右側Canvas雙擊重置視圖"""
        self.right_zoom_factor = 1.0
        self.right_pan_x = 0
        self.right_pan_y = 0
        self.update_canvas_image(self.right_canvas, self.get_image_by_selection(self.right_image_var.get()), "right")
    
    def canvas_to_image_coords(self, canvas_x, canvas_y, side):
        """轉換Canvas座標為圖像座標"""
        if side == "left":
            zoom = self.left_zoom_factor
            pan_x = self.left_pan_x
            pan_y = self.left_pan_y
        else:
            zoom = self.right_zoom_factor
            pan_x = self.right_pan_x
            pan_y = self.right_pan_y
        
        if self.scale_factor > 0 and zoom > 0:
            # 考慮平移和縮放
            adjusted_x = (canvas_x - pan_x) / zoom
            adjusted_y = (canvas_y - pan_y) / zoom
            
            img_x = int(adjusted_x / self.scale_factor)
            img_y = int(adjusted_y / self.scale_factor)
            return img_x, img_y
        return canvas_x, canvas_y
    
    def image_to_canvas_coords(self, img_x, img_y, side):
        """轉換圖像座標為Canvas座標"""
        if side == "left":
            zoom = self.left_zoom_factor
            pan_x = self.left_pan_x
            pan_y = self.left_pan_y
        else:
            zoom = self.right_zoom_factor
            pan_x = self.right_pan_x
            pan_y = self.right_pan_y
        
        canvas_x = int(img_x * self.scale_factor * zoom + pan_x)
        canvas_y = int(img_y * self.scale_factor * zoom + pan_y)
        return canvas_x, canvas_y
    
    def draw_temp_roi_on_canvas(self):
        """在左側Canvas上繪製臨時ROI選擇框"""
        if self.roi_start_point and self.roi_end_point:
            x1, y1 = self.image_to_canvas_coords(*self.roi_start_point, "left")
            x2, y2 = self.image_to_canvas_coords(*self.roi_end_point, "left")
            
            self.left_canvas.delete("roi_temp")
            self.left_canvas.create_rectangle(x1, y1, x2, y2, outline="lime", width=2, tags="roi_temp")
    
    def draw_roi_on_canvas(self, canvas, side):
        """在指定Canvas上繪製確定的ROI框"""
        global roi_rect
        
        canvas.delete("roi_rect")
        canvas.delete("roi_temp")
        
        if roi_rect:
            x, y, w, h = roi_rect
            x1, y1 = self.image_to_canvas_coords(x, y, side)
            x2, y2 = self.image_to_canvas_coords(x + w, y + h, side)
            
            canvas.create_rectangle(x1, y1, x2, y2, outline="red", width=3, tags="roi_rect")
    
    def update_image(self, image):
        """更新圖像並處理"""
        if image is None:
            return
        
        self.processed_images = process_image_pipeline(image, self)
        self.refresh_display()
    
    def refresh_display(self):
        """刷新左右兩側的圖像顯示"""
        if not self.processed_images:
            return
        
        # 更新左側圖像
        left_selection = self.left_image_var.get()
        left_image = self.get_image_by_selection(left_selection)
        if left_image is not None:
            self.update_canvas_image(self.left_canvas, left_image, "left")
        
        # 更新右側圖像
        right_selection = self.right_image_var.get()
        right_image = self.get_image_by_selection(right_selection)
        if right_image is not None:
            self.update_canvas_image(self.right_canvas, right_image, "right")
        
        # 更新特徵統計
        self.update_feature_stats()
        
        # 觸發分類評估
        if self.classification_enabled.get():
            self.evaluate_classification()
    
    def get_image_by_selection(self, selection):
        """根據選擇獲取對應的圖像"""
        mapping = {
            "原圖": "original",
            "原圖(含ROI框)": "original_with_roi",
            "灰階圖": "gray",
            "ROI區域": "roi",
            "高斯模糊": "blurred",
            "二值化": "binary",
            "Canny邊緣": "canny",
            "LBP紋理分析": "lbp",
            "輪廓粗糙度檢測": "contour_analysis",
            "亮度分布熱圖": "brightness_heatmap"
        }
        
        key = mapping.get(selection)
        if key and key in self.processed_images:
            return self.processed_images[key]
        return None
    
    def update_feature_stats(self):
        """更新特徵統計顯示"""
        if 'brightness_stats' not in self.processed_images:
            return
        
        stats = self.processed_images['brightness_stats']
        
        stats_text = f"""亮度統計:
平均值: {stats['mean']:.2f}
標準差: {stats['std']:.2f}
偏度: {stats['skewness']:.3f}
峰度: {stats['kurtosis']:.3f}

LBP參數:
半徑: {self.lbp_radius.get()}
點數: {self.lbp_points.get()}

處理參數:
高斯核: {self.gaussian_kernel.get()}
閾值模式: {'OTSU' if self.use_otsu.get() else 'Manual'}"""

        # 如果啟用分類，添加分類結果
        if self.classification_enabled.get() and hasattr(self, 'classification_result') and self.classification_result:
            result_text = f"ID: {self.classification_result['id']} ({self.classification_result['name']})"
            stats_text += f"\n\n分類結果: {result_text}"
        
        self.stats_text.delete("1.0", "end")
        self.stats_text.insert("1.0", stats_text)
    
    def update_canvas_image(self, canvas, image, side):
        """更新指定Canvas的圖像顯示"""
        try:
            if image is None:
                return
                
            # 處理圖像格式
            if len(image.shape) == 3:
                # 彩色圖像，轉換BGR到RGB
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                # 灰階圖像，轉換為RGB
                image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            
            # 取得Canvas尺寸和圖像尺寸
            canvas_width = canvas.winfo_width()
            canvas_height = canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                canvas_width, canvas_height = 400, 300
            
            img_height, img_width = image_rgb.shape[:2]
            
            # 計算基礎縮放比例（適應Canvas）
            scale_x = canvas_width / img_width
            scale_y = canvas_height / img_height
            self.scale_factor = min(scale_x, scale_y)
            
            # 取得當前側別的縮放和平移參數
            if side == "left":
                zoom = self.left_zoom_factor
                pan_x = self.left_pan_x
                pan_y = self.left_pan_y
            else:
                zoom = self.right_zoom_factor
                pan_x = self.right_pan_x
                pan_y = self.right_pan_y
            
            # 應用用戶縮放
            final_scale = self.scale_factor * zoom
            
            # 縮放圖像
            new_width = int(img_width * final_scale)
            new_height = int(img_height * final_scale)
            
            if new_width > 0 and new_height > 0:
                image_resized = cv2.resize(image_rgb, (new_width, new_height))
                
                # 轉換為PIL圖像
                pil_image = Image.fromarray(image_resized)
                tk_image = ImageTk.PhotoImage(pil_image)
                
                # 清除Canvas並顯示新圖像
                canvas.delete("all")
                # 應用平移偏移
                canvas.create_image(pan_x, pan_y, anchor="nw", image=tk_image)
                
                # 保存引用防止垃圾回收
                if side == "left":
                    self.left_tk_image = tk_image
                    # 在左側Canvas上顯示ROI框（僅原圖相關選項）
                    if self.left_image_var.get() in ["原圖", "原圖(含ROI框)"]:
                        self.draw_roi_on_canvas(canvas, side)
                else:
                    self.right_tk_image = tk_image
            
        except Exception as e:
            print(f"更新Canvas圖像錯誤 ({side}): {e}")
    
    def next_image(self):
        """下一張圖片按鈕"""
        next_image()
    
    def previous_image(self):
        """上一張圖片按鈕"""
        previous_image()
    
    def update_file_info(self, filename, current, total):
        """更新檔案資訊顯示"""
        info_text = f"檔案: {filename} ({current}/{total})"
        self.file_info_label.configure(text=info_text)
    
    def update_roi_values(self, roi_rect):
        """更新ROI數值"""
        if roi_rect:
            x, y, w, h = roi_rect
            self.x_var.set(x)
            self.y_var.set(y)
            self.w_var.set(w)
            self.h_var.set(h)
            self.status_label.configure(text=f"狀態: ROI已選擇 ({x}, {y}, {w}x{h})")
    
    def on_coord_changed(self, event=None):
        """座標輸入框變更時觸發"""
        self.apply_roi()
    
    def apply_roi(self):
        """套用ROI設定"""
        global roi_rect
        
        try:
            x = self.x_var.get()
            y = self.y_var.get()
            w = self.w_var.get()
            h = self.h_var.get()
            
            if w > 0 and h > 0:
                roi_rect = (x, y, w, h)
                self.status_label.configure(text=f"狀態: ROI已套用 ({x}, {y}, {w}x{h})")
                print(f"CTK ROI套用: x={x}, y={y}, w={w}, h={h}")
                
                # 重新處理圖像
                global current_image
                if current_image is not None:
                    self.processed_images = process_image_pipeline(current_image, self)
                    self.refresh_display()
            else:
                self.status_label.configure(text="錯誤: 寬度和高度必須大於0")
                
        except Exception as e:
            self.status_label.configure(text=f"錯誤: {str(e)}")
            print(f"CTK apply_roi錯誤: {e}")
    
    def clear_roi(self):
        """清除ROI設定"""
        global roi_rect, roi_start_point, roi_end_point
        
        try:
            roi_rect = None
            roi_start_point = None
            roi_end_point = None
            
            self.x_var.set(0)
            self.y_var.set(0)
            self.w_var.set(100)
            self.h_var.set(100)
            
            # 清除Canvas上的ROI框
            self.left_canvas.delete("roi_rect")
            self.left_canvas.delete("roi_temp")
            self.right_canvas.delete("roi_rect")
            self.right_canvas.delete("roi_temp")
            
            self.status_label.configure(text="狀態: ROI已清除")
            print("CTK ROI已清除")
            
            # 重新處理圖像
            global current_image
            if current_image is not None:
                self.processed_images = process_image_pipeline(current_image, self)
                self.refresh_display()
            
        except Exception as e:
            print(f"CTK clear_roi錯誤: {e}")
    
    def run(self):
        """啟動CTK視窗"""
        self.root.mainloop()

def start_roi_control_window():
    """在新線程中啟動ROI控制視窗"""
    global roi_control_window
    roi_control_window = ROIControlWindow()
    roi_control_window.run()

def main():
    """主函數"""
    global roi_rect, roi_start_point, roi_end_point, current_image, bmp_files, current_index, data_folder
    
    # 啟動ROI控制視窗線程
    roi_thread = threading.Thread(target=start_roi_control_window, daemon=True)
    roi_thread.start()
    
    # 設定dataFB資料夾路徑
    data_folder = "dataFB"
    
    # 檢查資料夾是否存在
    if not os.path.exists(data_folder):
        print(f"資料夾不存在: {data_folder}")
        return
    
    # 取得所有bmp檔案
    bmp_files = [f for f in os.listdir(data_folder) if f.lower().endswith('.bmp')]
    
    if not bmp_files:
        print("dataFB資料夾中沒有找到BMP檔案")
        return
    
    print(f"找到 {len(bmp_files)} 個BMP檔案")
    
    current_index = 0
    
    # 載入第一張圖片
    load_and_display_image(current_index)
    
    # 主循環，處理更新佇列
    try:
        while True:
            process_update_queue()
            # 小延遲避免CPU占用過高
            import time
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("程序結束")

if __name__ == "__main__":
    main()