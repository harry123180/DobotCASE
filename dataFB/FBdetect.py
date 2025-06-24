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
        
        self.setup_ui()
        
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
        
        prev_btn = ctk.CTkButton(nav_buttons, text="◀ 上一張", command=self.previous_image, width=100)
        prev_btn.pack(side="left", padx=5)
        
        next_btn = ctk.CTkButton(nav_buttons, text="下一張 ▶", command=self.next_image, width=100)
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
        
        # 圖像處理參數區域
        params_frame = ctk.CTkFrame(control_panel)
        params_frame.pack(fill="x", padx=10, pady=5)
        
        params_label = ctk.CTkLabel(params_frame, text="圖像處理參數", font=ctk.CTkFont(size=16, weight="bold"))
        params_label.pack(pady=5)
        
        # 參數控制區域分為多列
        params_grid = ctk.CTkFrame(params_frame)
        params_grid.pack()
        
        # 高斯模糊
        gaussian_frame = ctk.CTkFrame(params_grid)
        gaussian_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        
        ctk.CTkLabel(gaussian_frame, text="高斯核大小:").pack()
        self.gaussian_value_label = ctk.CTkLabel(gaussian_frame, text="5")
        self.gaussian_value_label.pack()
        self.gaussian_slider = ctk.CTkSlider(gaussian_frame, from_=1, to=15, number_of_steps=7, command=self.on_gaussian_changed)
        self.gaussian_slider.set(5)
        self.gaussian_slider.pack(fill="x", padx=5)
        
        # 閾值控制
        threshold_frame = ctk.CTkFrame(params_grid)
        threshold_frame.grid(row=0, column=1, padx=10, pady=5, sticky="ew")
        
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
        canny_frame.grid(row=0, column=2, padx=10, pady=5, sticky="ew")
        
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
        lbp_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        
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
        stats_frame = ctk.CTkFrame(params_grid)
        stats_frame.grid(row=1, column=1, columnspan=2, padx=10, pady=5, sticky="ew")
        
        ctk.CTkLabel(stats_frame, text="特徵統計", font=ctk.CTkFont(size=14, weight="bold")).pack()
        self.stats_text = ctk.CTkTextbox(stats_frame, height=100, width=300)
        self.stats_text.pack(fill="both", expand=True, padx=5, pady=5)
        
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
閾值模式: {'OTSU' if self.use_otsu.get() else 'Manual'}
"""
        
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