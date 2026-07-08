#!/usr/bin/env python3
"""
电阻矩阵可视化工具 — 8×8 热力图面板，支持串口实时数据。

数据帧格式 (258 字节):
  [0xFF] [0xAA] [64 个采样 × 4B: exc_id, adc_ch, val_hi, val_lo]
"""

from collections import deque
import math
import statistics
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

# ── 常量 ──────────────────────────────────────────────────────────────
EXC_COUNT   = 8
ADC_COUNT   = 8
SYNC_HI     = 0xFF
SYNC_LO     = 0xAA
FRAME_SIZE  = 2 + EXC_COUNT * ADC_COUNT * 4   # 258 字节
ADC_MAX_VAL = 4095                             # 12-bit ADC 满量程

# ── 滤波默认参数 ──────────────────────────────────────────────────────
DEFAULT_DEADZONE         = 100       # 死区阈值（低于此值的读数强制归零）
DEFAULT_SPATIAL_SIGMA    = 0.8       # 高斯模糊 σ（越大越模糊）
DEFAULT_DENOISE_SIGMA    = 2.5       # 椒盐去噪 σ 阈值（越小越激进，剔除越多）
DEFAULT_GHOST_THRESHOLD  = 80        # 去鬼影活性阈值（低于此值不参与连通域计算）


# 激励引脚标签（与 STM32 sm_exc_pins 顺序一致）
EXC_LABELS = ["PB7", "PB6", "PB5", "PB4", "PB0", "PB1", "PB10", "PB11"]

# ADC 通道标签
ADC_LABELS = ["PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7"]

# 热力图颜色渐变: 蓝(冷) → 青 → 绿 → 黄 → 红(热)
def heat_colour(norm: float):  # norm 范围 0..1
    """将 0–1 的值映射为热力图颜色 (#RRGGBB)。"""
    norm = max(0.0, min(1.0, norm))
    if norm < 0.25:
        t = norm / 0.25
        r, g, b = 0, int(255 * t), 255
    elif norm < 0.50:
        t = (norm - 0.25) / 0.25
        r, g, b = 0, 255, int(255 * (1 - t))
    elif norm < 0.75:
        t = (norm - 0.50) / 0.25
        r, g, b = int(255 * t), 255, 0
    else:
        t = (norm - 0.75) / 0.25
        r, g, b = 255, int(255 * (1 - t)), 0
    return f"#{r:02X}{g:02X}{b:02X}"


# ── 串口读取线程 ──────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    """后台线程：读取串口数据，同步帧，分发给 GUI。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self._running = False
        self.callback = None          # 回调参数为 8×8 int 矩阵

    def open(self, port: str, baud: int) -> bool:
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self._running = True
            return True
        except serial.SerialException as e:
            messagebox.showerror("串口错误", str(e))
            return False

    def close(self):
        self._running = False
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def run(self):
        buf = bytearray()
        while self._running:
            try:
                with self._lock:
                    if self.ser and self.ser.is_open:
                        chunk = self.ser.read(max(1, self.ser.in_waiting or 1))
                    else:
                        time.sleep(0.1)
                        continue
            except serial.SerialException:
                time.sleep(0.1)
                continue

            buf.extend(chunk)

            # 扫描同步头 0xFF 0xAA
            while len(buf) >= FRAME_SIZE:
                # 定位同步字
                sync_pos = -1
                for i in range(len(buf) - 1):
                    if buf[i] == SYNC_HI and buf[i + 1] == SYNC_LO:
                        sync_pos = i
                        break

                if sync_pos < 0:
                    buf.clear()
                    break

                # 丢弃同步字之前的字节
                if sync_pos > 0:
                    del buf[:sync_pos]

                if len(buf) < FRAME_SIZE:
                    break   # 等待更多数据

                # 提取一帧
                frame = buf[:FRAME_SIZE]
                del buf[:FRAME_SIZE]

                # 解析采样数据（跳过 2 字节同步头）
                matrix = self._parse(frame[2:])
                if matrix is not None and self.callback:
                    self.callback(matrix)

    @staticmethod
    def _parse(data: bytes):
        """解析 256 字节负载 → 8×8 int 矩阵，无效则返回 None。"""
        if len(data) != EXC_COUNT * ADC_COUNT * 4:
            return None
        matrix = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        for i in range(0, len(data), 4):
            exc  = data[i]
            adc  = data[i + 1]
            val  = (data[i + 2] << 8) | data[i + 3]
            # 合法性检查
            if exc >= EXC_COUNT or adc >= ADC_COUNT:
                return None
            matrix[exc][adc] = val
        return matrix


# ── 主应用程序 ─────────────────────────────────────────────────────────
class MatrixViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("电阻矩阵可视化 — 8×8 热力图")
        self.geometry("1280x1140")
        self.resizable(False, False)

        self.matrix = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        self.show_values = tk.BooleanVar(value=True)
        self._cells: list[list[tk.Label]] = []
        self._vmin = 0
        self._vmax = ADC_MAX_VAL
        self._auto_range = tk.BooleanVar(value=True)
        self._updating_slider = False   # 防递归标志

        # 行（激励）可见性 — 默认全部显示
        self._row_visible: list[tk.BooleanVar] = [
            tk.BooleanVar(value=True) for _ in range(EXC_COUNT)
        ]
        # 列（ADC）可见性 — 默认全部显示
        self._col_visible: list[tk.BooleanVar] = [
            tk.BooleanVar(value=True) for _ in range(ADC_COUNT)
        ]

        # 调零偏移量
        self._zero_offsets = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        self._has_zero = False

        # 单格响应增益（默认 1.0，点击上半部分 +0.1，下半部分 -0.1）
        self._gains: list[list[float]] = [[1.0] * ADC_COUNT for _ in range(EXC_COUNT)]
        self.show_gains = tk.BooleanVar(value=True)

        # 死区阈值
        self._deadzone = tk.IntVar(value=DEFAULT_DEADZONE)

        # 空间滤波状态
        self._spatial_mode = tk.StringVar(value="none")   # "none" | "gaussian" | "median"
        self._spatial_sigma = tk.DoubleVar(value=DEFAULT_SPATIAL_SIGMA)

        # 椒盐去噪状态（独立于空间滤波的后处理）
        self._use_denoise = tk.BooleanVar(value=False)
        self._denoise_sigma = tk.DoubleVar(value=DEFAULT_DENOISE_SIGMA)

        # 去鬼影状态（连通域分析，保留最大团）
        self._use_ghost_removal = tk.BooleanVar(value=False)
        self._ghost_threshold = tk.IntVar(value=DEFAULT_GHOST_THRESHOLD)
        self._ghost_cluster_count = 0      # 当前帧检测到的触摸群数量

        # 滑移检测状态
        self._cop_x = 3.5          # 压力中心 x（列，浮点）
        self._cop_y = 3.5          # 压力中心 y（行，浮点）
        self._cop_history: deque[tuple[float, float]] = deque(maxlen=8)
        self._slip_dx = 0.0        # 平滑后的移动方向 x
        self._slip_dy = 0.0        # 平滑后的移动方向 y
        self._slip_speed = 0.0     # 移动速率
        self._is_touching = False  # 当前是否有有效按压

        # 行列标签（可变副本，变换时会更新）
        self._row_labels = list(EXC_LABELS)
        self._col_labels = list(ADC_LABELS)

        # Checkbutton 引用（用于变换后更新标签文字）
        self._row_cbs: list[ttk.Checkbutton] = []
        self._col_cbs: list[ttk.Checkbutton] = []

        # 变换状态：记录已应用的操作序列，新数据到达时自动重放
        # 'T' = 转置, 'R' = 顺时针旋转90°
        self._transforms: list[str] = []

        self.reader = SerialReader()
        # 关键：串口线程回调 → 通过 after() 转到主线程更新 UI
        self.reader.callback = self._on_serial_data

        self._build_ui()
        self._draw_grid()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── 线程安全：串口数据 → 主线程 ──────────────────────────────────
    def _on_serial_data(self, matrix: list[list[int]]):
        """串口线程回调 — 调度到主线程执行 UI 更新。"""
        self.after(0, self._do_update_matrix, matrix)

    # ── UI 构建 ─────────────────────────────────────────────────────
    def _build_ui(self):
        # ---- 顶部栏：串口号选择 ----
        top = ttk.Frame(self)
        top.pack(pady=(10, 0), padx=10, fill=tk.X)

        ttk.Label(top, text="串口号:").pack(side=tk.LEFT)
        self._combo_port = ttk.Combobox(top, width=10, state="readonly")
        self._combo_port.pack(side=tk.LEFT, padx=5)

        ttk.Label(top, text="波特率:").pack(side=tk.LEFT, padx=(10, 0))
        self._combo_baud = ttk.Combobox(
            top, values=["9600","19200","38400","57600","115200"],
            width=8, state="readonly")
        self._combo_baud.set("115200")
        self._combo_baud.pack(side=tk.LEFT, padx=5)

        self._btn_conn = ttk.Button(top, text="连接", command=self._toggle_serial)
        self._btn_conn.pack(side=tk.LEFT, padx=10)

        self._lbl_status = ttk.Label(top, text="● 未连接", foreground="red")
        self._lbl_status.pack(side=tk.LEFT, padx=10)

        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)

        self._refresh_ports()

        # ---- 通道选择栏 ----
        ch_frame = ttk.LabelFrame(self, text="通道选择")
        ch_frame.pack(pady=(8, 0), padx=10, fill=tk.X)

        # -- 行选择（激励引脚） --
        row_line = ttk.Frame(ch_frame)
        row_line.pack(pady=(5, 2), padx=5, fill=tk.X)

        ttk.Label(row_line, text="激励(行):", width=9).pack(side=tk.LEFT)

        self._row_cbs.clear()
        for i, label in enumerate(self._row_labels):
            cb = ttk.Checkbutton(
                row_line, text=label, variable=self._row_visible[i],
                command=self._on_channel_toggle)
            cb.pack(side=tk.LEFT, padx=3)
            self._row_cbs.append(cb)

        ttk.Separator(row_line, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(row_line, text="全选", width=4,
                   command=self._select_all_rows).pack(side=tk.LEFT, padx=2)
        ttk.Button(row_line, text="全不选", width=5,
                   command=self._deselect_all_rows).pack(side=tk.LEFT, padx=2)
        ttk.Button(row_line, text="反选", width=4,
                   command=self._invert_rows).pack(side=tk.LEFT, padx=2)

        # -- 列选择（ADC 通道） --
        col_line = ttk.Frame(ch_frame)
        col_line.pack(pady=(2, 5), padx=5, fill=tk.X)

        ttk.Label(col_line, text="ADC(列):", width=9).pack(side=tk.LEFT)

        self._col_cbs.clear()
        for i, label in enumerate(self._col_labels):
            cb = ttk.Checkbutton(
                col_line, text=label, variable=self._col_visible[i],
                command=self._on_channel_toggle)
            cb.pack(side=tk.LEFT, padx=3)
            self._col_cbs.append(cb)

        ttk.Separator(col_line, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(col_line, text="全选", width=4,
                   command=self._select_all_cols).pack(side=tk.LEFT, padx=2)
        ttk.Button(col_line, text="全不选", width=5,
                   command=self._deselect_all_cols).pack(side=tk.LEFT, padx=2)
        ttk.Button(col_line, text="反选", width=4,
                   command=self._invert_cols).pack(side=tk.LEFT, padx=2)

        # ---- 主显示区域（网格 + 滑移面板）----
        main_area = ttk.Frame(self)
        main_area.pack(expand=True, fill=tk.BOTH, pady=5, padx=10)

        # 网格区域（左侧）
        self._grid_frame = ttk.Frame(main_area)
        self._grid_frame.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        # 滑移检测面板（右侧）
        slip_panel = ttk.LabelFrame(main_area, text="滑移检测")
        slip_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))

        self._slip_canvas = tk.Canvas(
            slip_panel, width=190, height=250,
            bg="#f0f0f0", highlightthickness=0)
        self._slip_canvas.pack(padx=12, pady=(12, 0))

        self._lbl_slip_dir = ttk.Label(
            slip_panel, text="等待按压…", anchor=tk.CENTER,
            font=("微软雅黑", 10, "bold"))
        self._lbl_slip_dir.pack(pady=(8, 2))

        self._lbl_slip_speed = ttk.Label(
            slip_panel, text="", anchor=tk.CENTER,
            font=("Consolas", 9), foreground="gray")
        self._lbl_slip_speed.pack(pady=(0, 10))

        # ---- 底部栏：控制 ----
        bottom = ttk.Frame(self)
        bottom.pack(side=tk.BOTTOM, fill=tk.X, pady=5, padx=10)

        # 第一行：复选框 + 数值输入 + 按钮
        ctrl_row1 = ttk.Frame(bottom)
        ctrl_row1.pack(fill=tk.X)

        ttk.Checkbutton(
            ctrl_row1, text="显示数值", variable=self.show_values,
            command=self._toggle_values).pack(side=tk.LEFT)

        ttk.Checkbutton(
            ctrl_row1, text="显示增益", variable=self.show_gains,
            command=self._toggle_gains_display).pack(side=tk.LEFT, padx=10)

        ttk.Checkbutton(
            ctrl_row1, text="自动量程", variable=self._auto_range,
            command=self._redraw_grid).pack(side=tk.LEFT, padx=10)

        ttk.Label(ctrl_row1, text="最小值:").pack(side=tk.LEFT)
        self._ent_vmin = ttk.Entry(ctrl_row1, width=5)
        self._ent_vmin.insert(0, "0")
        self._ent_vmin.pack(side=tk.LEFT, padx=2)
        self._ent_vmin.bind("<Return>", lambda _: self._entry_range_changed())
        self._ent_vmin.bind("<FocusOut>", lambda _: self._entry_range_changed())

        ttk.Label(ctrl_row1, text="最大值:").pack(side=tk.LEFT, padx=(5, 0))
        self._ent_vmax = ttk.Entry(ctrl_row1, width=5)
        self._ent_vmax.insert(0, str(ADC_MAX_VAL))
        self._ent_vmax.pack(side=tk.LEFT, padx=2)
        self._ent_vmax.bind("<Return>", lambda e: self._entry_range_changed())
        self._ent_vmax.bind("<FocusOut>", lambda e: self._entry_range_changed())

        ttk.Button(ctrl_row1, text="应用量程", command=self._manual_range).pack(
            side=tk.LEFT, padx=5)

        self._btn_zero = ttk.Button(
            ctrl_row1, text="一键调零", command=self._zero_calibrate)
        self._btn_zero.pack(side=tk.LEFT, padx=5)

        self._btn_unzero = ttk.Button(
            ctrl_row1, text="取消调零", command=self._cancel_zero, state=tk.DISABLED)
        self._btn_unzero.pack(side=tk.LEFT, padx=2)

        self._btn_reset_gains = ttk.Button(
            ctrl_row1, text="重置增益", command=self._reset_gains)
        self._btn_reset_gains.pack(side=tk.LEFT, padx=5)

        # 图例（第一行右侧）
        self._canvas_legend = tk.Canvas(
            ctrl_row1, width=200, height=18, highlightthickness=0)
        self._canvas_legend.pack(side=tk.RIGHT, padx=10)
        ttk.Label(ctrl_row1, text="冷").pack(side=tk.RIGHT)
        ttk.Label(ctrl_row1, text="热 ").pack(side=tk.RIGHT)
        self._draw_legend()

        # 第二行：颜色范围滑动条
        ctrl_row2 = ttk.Frame(bottom)
        ctrl_row2.pack(fill=tk.X, pady=(5, 0))

        ttk.Label(ctrl_row2, text="色阶下限:", width=9).pack(side=tk.LEFT)
        self._scale_vmin = ttk.Scale(
            ctrl_row2, from_=0, to=ADC_MAX_VAL, orient=tk.HORIZONTAL,
            command=self._on_slider_vmin)
        self._scale_vmin.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))

        ttk.Label(ctrl_row2, text="色阶上限:", width=9).pack(side=tk.LEFT)
        self._scale_vmax = ttk.Scale(
            ctrl_row2, from_=0, to=ADC_MAX_VAL, orient=tk.HORIZONTAL,
            command=self._on_slider_vmax)
        self._scale_vmax.set(ADC_MAX_VAL)
        self._scale_vmax.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # 第三行：矩阵方向变换按钮
        ctrl_row3 = ttk.Frame(bottom)
        ctrl_row3.pack(fill=tk.X, pady=(8, 0))

        ttk.Label(ctrl_row3, text="方向校正:").pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(ctrl_row3, text="转置（反转）",
                   command=self._transpose_matrix).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl_row3, text="旋转90°",
                   command=self._rotate90_matrix).pack(side=tk.LEFT, padx=2)

        # ---- 滤波设置 ----
        filt_frame = ttk.LabelFrame(bottom, text="滤波设置")
        filt_frame.pack(fill=tk.X, pady=(8, 0))

        # 第一行：死区阈值
        filt_row1 = ttk.Frame(filt_frame)
        filt_row1.pack(fill=tk.X, pady=(5, 2), padx=5)

        ttk.Label(filt_row1, text="死区阈值:").pack(side=tk.LEFT, padx=(0, 3))
        self._scale_deadzone = ttk.Scale(
            filt_row1, from_=0, to=500, orient=tk.HORIZONTAL, length=120,
            variable=self._deadzone, command=self._on_filter_param_change)
        self._scale_deadzone.pack(side=tk.LEFT)
        self._lbl_deadzone = ttk.Label(
            filt_row1, text=str(DEFAULT_DEADZONE), width=4, anchor=tk.W)
        self._lbl_deadzone.pack(side=tk.LEFT, padx=2)

        ttk.Label(filt_row1, text="低于阈值的读数强制归零",
                  foreground="gray").pack(side=tk.LEFT, padx=8)

        # 第二行：空间滤波
        filt_row2 = ttk.Frame(filt_frame)
        filt_row2.pack(fill=tk.X, pady=(2, 5), padx=5)

        ttk.Label(filt_row2, text="空间:").pack(side=tk.LEFT)
        for mode, label in [("none", "无"), ("gaussian", "高斯模糊"), ("median", "中值滤波")]:
            ttk.Radiobutton(
                filt_row2, text=label, variable=self._spatial_mode, value=mode,
                command=self._on_spatial_mode_change
            ).pack(side=tk.LEFT, padx=5)

        ttk.Separator(filt_row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        # 高斯 σ
        ttk.Label(filt_row2, text="σ(高斯):").pack(side=tk.LEFT, padx=(0, 3))
        self._scale_sigma = ttk.Scale(
            filt_row2, from_=30, to=250, orient=tk.HORIZONTAL, length=100,
            command=self._on_sigma_change)
        self._scale_sigma.set(int(DEFAULT_SPATIAL_SIGMA * 100))
        self._scale_sigma.pack(side=tk.LEFT)
        self._lbl_sigma = ttk.Label(
            filt_row2, text=f"{DEFAULT_SPATIAL_SIGMA:.2f}", width=4, anchor=tk.W)
        self._lbl_sigma.pack(side=tk.LEFT, padx=2)

        ttk.Separator(filt_row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(filt_row2, text="中值滤波固定 3×3 窗口",
                  foreground="gray").pack(side=tk.LEFT, padx=5)

        # 第三行：椒盐去噪（独立后处理）
        filt_row3 = ttk.Frame(filt_frame)
        filt_row3.pack(fill=tk.X, pady=(2, 5), padx=5)

        self._cb_denoise = ttk.Checkbutton(
            filt_row3, text="椒盐去噪", variable=self._use_denoise,
            command=self._on_denoise_toggle)
        self._cb_denoise.pack(side=tk.LEFT)

        ttk.Separator(filt_row3, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(filt_row3, text="σ 阈值:").pack(side=tk.LEFT, padx=(0, 3))
        self._scale_denoise = ttk.Scale(
            filt_row3, from_=10, to=60, orient=tk.HORIZONTAL, length=120,
            command=self._on_denoise_sigma_change)
        self._scale_denoise.set(int(DEFAULT_DENOISE_SIGMA * 10))
        self._scale_denoise.pack(side=tk.LEFT)
        self._lbl_denoise = ttk.Label(
            filt_row3, text=f"{DEFAULT_DENOISE_SIGMA:.1f}", width=4, anchor=tk.W)
        self._lbl_denoise.pack(side=tk.LEFT, padx=2)

        ttk.Label(filt_row3, text="MAD离群检测 · 3×3邻域",
                  foreground="gray").pack(side=tk.LEFT, padx=8)

        ttk.Label(filt_row3, text="σ 越小剔除越激进",
                  foreground="gray").pack(side=tk.RIGHT, padx=5)

        # 第四行：去鬼影（连通域分析）
        filt_row4 = ttk.Frame(filt_frame)
        filt_row4.pack(fill=tk.X, pady=(2, 5), padx=5)

        self._cb_ghost = ttk.Checkbutton(
            filt_row4, text="去鬼影", variable=self._use_ghost_removal,
            command=self._on_ghost_toggle)
        self._cb_ghost.pack(side=tk.LEFT)

        ttk.Separator(filt_row4, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(filt_row4, text="活性阈值:").pack(side=tk.LEFT, padx=(0, 3))
        self._scale_ghost = ttk.Scale(
            filt_row4, from_=20, to=500, orient=tk.HORIZONTAL, length=120,
            command=self._on_ghost_threshold_change)
        self._scale_ghost.set(DEFAULT_GHOST_THRESHOLD)
        self._scale_ghost.pack(side=tk.LEFT)
        self._lbl_ghost = ttk.Label(
            filt_row4, text=str(DEFAULT_GHOST_THRESHOLD), width=4, anchor=tk.W)
        self._lbl_ghost.pack(side=tk.LEFT, padx=2)

        ttk.Label(filt_row4, text="BFS 8-连通 · 只保留质量最大群",
                  foreground="gray").pack(side=tk.LEFT, padx=8)

        self._lbl_ghost_info = ttk.Label(
            filt_row4, text="", foreground="#2196F3")
        self._lbl_ghost_info.pack(side=tk.RIGHT, padx=5)

    def _draw_legend(self):
        w = 200
        self._canvas_legend.delete("all")
        for x in range(w):
            norm = x / (w - 1)
            c = heat_colour(norm)
            self._canvas_legend.create_line(x, 0, x, 20, fill=c, width=1)

    # ── 通道选择 ────────────────────────────────────────────────────
    def _get_visible_rows(self) -> list[int]:
        return [i for i in range(EXC_COUNT) if self._row_visible[i].get()]

    def _get_visible_cols(self) -> list[int]:
        return [i for i in range(ADC_COUNT) if self._col_visible[i].get()]

    def _on_channel_toggle(self):
        """通道勾选状态改变时重绘网格。"""
        self._draw_grid()

    # -- 行操作 --
    def _select_all_rows(self):
        for v in self._row_visible:
            v.set(True)
        self._draw_grid()

    def _deselect_all_rows(self):
        for v in self._row_visible:
            v.set(False)
        self._draw_grid()

    def _invert_rows(self):
        for v in self._row_visible:
            v.set(not v.get())
        self._draw_grid()

    # -- 列操作 --
    def _select_all_cols(self):
        for v in self._col_visible:
            v.set(True)
        self._draw_grid()

    def _deselect_all_cols(self):
        for v in self._col_visible:
            v.set(False)
        self._draw_grid()

    def _invert_cols(self):
        for v in self._col_visible:
            v.set(not v.get())
        self._draw_grid()

    # ── 矩阵方向变换 ────────────────────────────────────────────────
    @staticmethod
    def _apply_transpose(matrix: list[list[int]]) -> list[list[int]]:
        """返回转置后的新矩阵（纯函数）。"""
        new = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        for r in range(EXC_COUNT):
            for c in range(ADC_COUNT):
                new[c][r] = matrix[r][c]
        return new

    @staticmethod
    def _apply_rotate90(matrix: list[list[int]]) -> list[list[int]]:
        """返回顺时针旋转 90° 后的新矩阵（纯函数）。"""
        N = EXC_COUNT
        new = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        for r in range(N):
            for c in range(N):
                new[c][N - 1 - r] = matrix[r][c]
        return new

    def _apply_all_transforms(self, matrix: list[list[int]]) -> list[list[int]]:
        """将累积的变换操作依次应用到矩阵上。"""
        for op in self._transforms:
            if op == 'T':
                matrix = self._apply_transpose(matrix)
            elif op == 'R':
                matrix = self._apply_rotate90(matrix)
        return matrix

    def _transpose_matrix(self):
        """转置矩阵：行列互换（手动反转）。"""
        old_row_labels = list(self._row_labels)
        old_col_labels = list(self._col_labels)
        old_row_vis = [v.get() for v in self._row_visible]
        old_col_vis = [v.get() for v in self._col_visible]
        old_zero = [row[:] for row in self._zero_offsets]
        old_gains = [row[:] for row in self._gains]

        self.matrix = self._apply_transpose(self.matrix)
        self._zero_offsets = self._apply_transpose(old_zero)
        self._gains = self._apply_transpose(old_gains)
        self._row_labels = list(old_col_labels)
        self._col_labels = list(old_row_labels)
        for i in range(EXC_COUNT):
            self._row_visible[i].set(old_col_vis[i])
            self._col_visible[i].set(old_row_vis[i])

        # 记录操作（用于后续串口数据自动变换）
        self._transforms.append('T')
        self._sync_checkbutton_labels()
        self._draw_grid()

    def _rotate90_matrix(self):
        """顺时针旋转 90 度。"""
        old_row_labels = list(self._row_labels)
        old_col_labels = list(self._col_labels)
        old_row_vis = [v.get() for v in self._row_visible]
        old_col_vis = [v.get() for v in self._col_visible]
        old_zero = [row[:] for row in self._zero_offsets]
        old_gains = [row[:] for row in self._gains]

        self.matrix = self._apply_rotate90(self.matrix)
        self._zero_offsets = self._apply_rotate90(old_zero)
        self._gains = self._apply_rotate90(old_gains)
        self._col_labels = list(old_row_labels)
        N = EXC_COUNT
        for i in range(N):
            self._row_visible[i].set(old_col_vis[N - 1 - i])
            self._col_visible[i].set(old_row_vis[i])

        # 记录操作（用于后续串口数据自动变换）
        self._transforms.append('R')
        self._sync_checkbutton_labels()
        self._draw_grid()

    def _sync_checkbutton_labels(self):
        """根据当前行列标签更新 Checkbutton 文字。"""
        for i, cb in enumerate(self._row_cbs):
            cb.config(text=self._row_labels[i])
        for i, cb in enumerate(self._col_cbs):
            cb.config(text=self._col_labels[i])

    # ── 调零功能 ────────────────────────────────────────────────────
    def _zero_calibrate(self):
        """将当前读数保存为零点偏移量，后续读数将减去此偏移。"""
        self._zero_offsets = [row[:] for row in self.matrix]
        self._has_zero = True
        self._btn_zero.config(state=tk.DISABLED)
        self._btn_unzero.config(state=tk.NORMAL)
        self._apply_colours()

    def _cancel_zero(self):
        """取消调零，恢复原始读数。"""
        self._zero_offsets = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        self._has_zero = False
        self._btn_zero.config(state=tk.NORMAL)
        self._btn_unzero.config(state=tk.DISABLED)
        self._apply_colours()

    def _apply_zero(self, matrix: list[list[int]]) -> list[list[int]]:
        """对矩阵应用零点偏移，确保结果非负。"""
        if not self._has_zero:
            return matrix
        result = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        for r in range(EXC_COUNT):
            for c in range(ADC_COUNT):
                result[r][c] = max(0, matrix[r][c] - self._zero_offsets[r][c])
        return result

    def _apply_gains_and_zero(self, matrix: list[list[int]]) -> list[list[int]]:
        """先应用单格增益，再应用零点偏移，确保结果非负。"""
        result = [[0] * ADC_COUNT for _ in range(EXC_COUNT)]
        for r in range(EXC_COUNT):
            for c in range(ADC_COUNT):
                scaled = int(matrix[r][c] * self._gains[r][c])
                offset = self._zero_offsets[r][c] if self._has_zero else 0
                result[r][c] = max(0, scaled - offset)
        return result

    # ── 增益调节 ────────────────────────────────────────────────────
    def _on_gain_click(self, event: tk.Event):
        """点击格子上半部分 +0.1 增益，下半部分 -0.1。"""
        widget = event.widget
        r = getattr(widget, 'r', None)
        c = getattr(widget, 'c', None)
        if r is None or c is None:
            return

        delta = +0.1 if event.y < widget.winfo_height() / 2 else -0.1
        new_gain = round(self._gains[r][c] + delta, 1)
        new_gain = max(0.1, min(10.0, new_gain))

        if new_gain == self._gains[r][c]:
            return   # 已达边界，无变化

        self._gains[r][c] = new_gain

        # 视觉反馈：短暂凹陷效果
        widget.configure(relief=tk.SUNKEN)
        widget.after(150, lambda w=widget: w.configure(relief=tk.RIDGE))

        self._apply_colours()

    def _reset_gains(self):
        """重置所有格子的增益为 1.0。"""
        self._gains = [[1.0] * ADC_COUNT for _ in range(EXC_COUNT)]
        self._apply_colours()

    def _toggle_gains_display(self):
        """切换是否在格子上显示增益倍率。"""
        self._apply_colours()

    # ── 颜色范围滑动条 ──────────────────────────────────────────────
    def _on_slider_vmin(self, value):
        """下限滑动条拖动回调。"""
        if self._updating_slider:
            return
        v = int(float(value))
        if v >= self._vmax:
            v = self._vmax - 1
            self._updating_slider = True
            self._scale_vmin.set(v)
            self._updating_slider = False
        self._vmin = v
        self._auto_range.set(False)
        self._ent_vmin.delete(0, tk.END)
        self._ent_vmin.insert(0, str(v))
        self._apply_colours()

    def _on_slider_vmax(self, value):
        """上限滑动条拖动回调。"""
        if self._updating_slider:
            return
        v = int(float(value))
        if v <= self._vmin:
            v = self._vmin + 1
            self._updating_slider = True
            self._scale_vmax.set(v)
            self._updating_slider = False
        self._vmax = v
        self._auto_range.set(False)
        self._ent_vmax.delete(0, tk.END)
        self._ent_vmax.insert(0, str(v))
        self._apply_colours()

    def _sync_sliders_from_entries(self):
        """根据输入框的值同步滑动条位置。"""
        self._updating_slider = True
        self._scale_vmin.set(self._vmin)
        self._scale_vmax.set(self._vmax)
        self._updating_slider = False

    def _entry_range_changed(self):
        """输入框值改变时同步（用于 Return / FocusOut 事件）。"""
        self._manual_range()

    # ── 网格绘制 ────────────────────────────────────────────────────
    def _draw_grid(self):
        # 销毁旧控件
        for row in self._cells:
            for lbl in row:
                lbl.destroy()
        self._cells.clear()
        # 同时清除 grid_frame 中的列头/行头残余
        for w in self._grid_frame.winfo_children():
            w.destroy()

        vis_rows = self._get_visible_rows()
        vis_cols = self._get_visible_cols()

        if not vis_rows or not vis_cols:
            lbl = ttk.Label(self._grid_frame, text="请至少选择一行和一列通道",
                           font=("微软雅黑", 12), foreground="gray")
            lbl.grid(row=0, column=0, padx=20, pady=40)
            return

        # 通道索引 → 网格坐标的映射
        row_to_grid = {r: gi + 1 for gi, r in enumerate(vis_rows)}
        col_to_grid = {c: gj + 1 for gj, c in enumerate(vis_cols)}

        # 列标题（仅可见的 ADC 通道）
        for c in vis_cols:
            gj = col_to_grid[c]
            lbl = ttk.Label(self._grid_frame, text=self._col_labels[c],
                            font=("Consolas", 9, "bold"), anchor=tk.CENTER)
            lbl.grid(row=0, column=gj, sticky="nsew", padx=1, pady=1, ipadx=6, ipady=2)

        # 行
        for r in vis_rows:
            gr = row_to_grid[r]
            # 行标题
            lbl = ttk.Label(self._grid_frame, text=self._row_labels[r],
                            font=("Consolas", 9, "bold"), anchor=tk.CENTER,
                            width=6)
            lbl.grid(row=gr, column=0, sticky="nsew", padx=1, pady=1)

            row_cells = []
            for c in vis_cols:
                gj = col_to_grid[c]
                val = self.matrix[r][c]
                lbl = tk.Label(
                    self._grid_frame,
                    text=str(val),
                    font=("Consolas", 10, "bold"),
                    relief=tk.RIDGE,
                    borderwidth=2,
                    width=8, height=3,
                    anchor=tk.CENTER,
                )
                lbl.r = r   # 存储矩阵行索引
                lbl.c = c   # 存储矩阵列索引
                lbl.bind("<Button-1>", self._on_gain_click)
                lbl.grid(row=gr, column=gj, sticky="nsew", padx=1, pady=1)
                row_cells.append(lbl)
            self._cells.append(row_cells)

        self._apply_colours()

    def _apply_colours(self):
        """根据当前数据和量程更新单元格背景颜色。"""
        vmin, vmax = self._vmin, self._vmax
        rng = max(vmax - vmin, 1)

        vis_rows = self._get_visible_rows()
        vis_cols = self._get_visible_cols()

        if not vis_rows or not vis_cols or not self._cells:
            return

        display = self._apply_gains_and_zero(self.matrix)

        for gi, r in enumerate(vis_rows):
            for gj, c in enumerate(vis_cols):
                val = display[r][c]
                norm = (val - vmin) / rng
                fg = "white" if norm > 0.6 else "black"
                # 构建单元格文本：数值 + 可选增益行
                if self.show_values.get():
                    gain = self._gains[r][c]
                    if self.show_gains.get():
                        text = f"{val}\n×{gain:.1f}"
                    else:
                        text = str(val)
                else:
                    text = ""
                self._cells[gi][gj].configure(
                    bg=heat_colour(norm),
                    fg=fg,
                    text=text,
                )

    def _redraw_grid(self):
        self._apply_colours()

    # ── 数据处理（仅主线程调用）─────────────────────────────────────
    def _do_update_matrix(self, matrix: list[list[int]]):
        """在主线程中更新矩阵数据并刷新显示。"""
        # 流水线：原始 → 方向变换 → 空间滤波 → 椒盐去噪 → 去鬼影 → 滑移检测 → 死区 → 增益/调零 → 显示
        self.matrix = self._apply_all_transforms(matrix)
        self.matrix = self._apply_spatial_filter(self.matrix)
        if self._use_denoise.get():
            self.matrix = self._apply_salt_pepper_removal(self.matrix)
        if self._use_ghost_removal.get():
            self.matrix = self._apply_ghost_removal(self.matrix)
        self._update_slip(self.matrix)       # 基于去鬼影后的数据检测滑移
        self.matrix = self._apply_deadzone(self.matrix)
        if self._auto_range.get():
            display = self._apply_gains_and_zero(self.matrix)
            vis_rows = self._get_visible_rows()
            vis_cols = self._get_visible_cols()
            if vis_rows and vis_cols:
                flat = [display[r][c] for r in vis_rows for c in vis_cols]
                if flat:
                    self._vmin, self._vmax = min(flat), max(flat)
                    if self._vmin == self._vmax:
                        self._vmax += 1
            self._ent_vmin.delete(0, tk.END)
            self._ent_vmin.insert(0, str(self._vmin))
            self._ent_vmax.delete(0, tk.END)
            self._ent_vmax.insert(0, str(self._vmax))
            self._sync_sliders_from_entries()
        self._apply_colours()
        self._update_ghost_info_label()       # 更新去鬼影群数量标签

    def _toggle_values(self):
        self._apply_colours()

    def _manual_range(self):
        try:
            self._vmin = int(self._ent_vmin.get())
            self._vmax = int(self._ent_vmax.get())
            if self._vmin >= self._vmax:
                self._vmax = self._vmin + 1
                self._ent_vmax.delete(0, tk.END)
                self._ent_vmax.insert(0, str(self._vmax))
            self._auto_range.set(False)
            self._sync_sliders_from_entries()
            self._apply_colours()
        except ValueError:
            pass

    # ── 串口控制 ──────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._combo_port["values"] = ports
        if ports and not self._combo_port.get():
            self._combo_port.set(ports[0])

    def _toggle_serial(self):
        if self.reader.ser and self.reader.ser.is_open:
            self.reader.close()
            self._btn_conn.config(text="连接")
            self._lbl_status.config(text="● 未连接", foreground="red")
            return

        port = self._combo_port.get()
        if not port:
            messagebox.showwarning("未选择串口", "请先选择一个串口号。")
            return
        baud = int(self._combo_baud.get())
        if self.reader.open(port, baud):
            self.reader.start()
            self._btn_conn.config(text="断开")
            self._lbl_status.config(
                text=f"● 已连接  {port} @ {baud}", foreground="green")

    def _on_close(self):
        self.reader.close()
        self.destroy()

    # ── 空间滤波 ───────────────────────────────────────────────────
    def _make_gaussian_kernel(self, sigma: float) -> list[list[float]]:
        """生成 3×3 归一化高斯卷积核。"""
        kernel = [[0.0] * 3 for _ in range(3)]
        total = 0.0
        for i in range(3):
            for j in range(3):
                dx, dy = i - 1, j - 1
                kernel[i][j] = math.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
                total += kernel[i][j]
        # 归一化
        for i in range(3):
            for j in range(3):
                kernel[i][j] /= total
        return kernel

    def _apply_gaussian_blur(self, matrix: list[list[int]]) -> list[list[int]]:
        """3×3 高斯模糊 — 平滑压力分布，抑制孤立噪点。

        卷积核由 σ 动态生成。σ 越大邻格权重越高，模糊越强。
        """
        sigma = max(0.3, self._spatial_sigma.get())
        kernel = self._make_gaussian_kernel(sigma)
        N = EXC_COUNT
        result = [[0] * N for _ in range(N)]
        for r in range(N):
            for c in range(N):
                total, weight = 0.0, 0.0
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < N and 0 <= nc < N:
                            w = kernel[dr + 1][dc + 1]
                            total += matrix[nr][nc] * w
                            weight += w
                result[r][c] = int(total / weight) if weight > 0 else matrix[r][c]
        return result

    def _apply_median_spatial(self, matrix: list[list[int]]) -> list[list[int]]:
        """3×3 中值滤波 — 去除椒盐噪声（单格尖峰/凹陷）。

        用邻域中值替换中心值，对孤立异常点干净利落。
        例如：按压区域中出现一个 0 值或 ADC 毛刺尖峰，会被邻域中值覆盖。
        """
        N = EXC_COUNT
        result = [[0] * N for _ in range(N)]
        for r in range(N):
            for c in range(N):
                neighbors = []
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < N and 0 <= nc < N:
                            neighbors.append(matrix[nr][nc])
                result[r][c] = int(statistics.median(neighbors))
        return result

    def _apply_spatial_filter(self, matrix: list[list[int]]) -> list[list[int]]:
        """根据当前空间滤波模式处理矩阵。"""
        mode = self._spatial_mode.get()
        if mode == "none":
            return matrix
        elif mode == "gaussian":
            return self._apply_gaussian_blur(matrix)
        elif mode == "median":
            return self._apply_median_spatial(matrix)
        return matrix

    # ── 椒盐去噪 ───────────────────────────────────────────────────
    def _apply_salt_pepper_removal(self, matrix: list[list[int]]) -> list[list[int]]:
        """自适应椒盐去噪 — 检测并替换孤立的异常值。

        算法（MAD 离群检测）：
        1. 对每个单元格，取其 3×3 邻域（不含自身）
        2. 计算邻域中位数 med 和 MAD（中位绝对偏差）
        3. 若 |cell - med| > σ × MAD × 1.4826，则用 med 替换

        优势：只在检测到异常时才替换，正常数据原样保留。
        相比之下，中值滤波无条件替换每个格，会损失细节。
        """
        sigma = self._denoise_sigma.get()
        N = EXC_COUNT
        result = [row[:] for row in matrix]

        for r in range(N):
            for c in range(N):
                # 收集 3×3 邻域（排除自身）
                neighbors = []
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < N and 0 <= nc < N:
                            neighbors.append(matrix[nr][nc])

                if len(neighbors) < 3:
                    continue

                med = statistics.median(neighbors)
                abs_devs = [abs(n - med) for n in neighbors]
                mad = statistics.median(abs_devs)

                # MAD × 1.4826 ≈ 正态分布下的标准差
                mad_scaled = mad * 1.4826
                if mad_scaled < 1.0:
                    mad_scaled = 1.0   # 防止邻域全等导致除零等效

                deviation = abs(matrix[r][c] - med)
                if deviation > sigma * mad_scaled:
                    result[r][c] = int(med)   # 异常 → 中值替换

        return result

    # ── 去鬼影 ─────────────────────────────────────────────────────
    def _apply_ghost_removal(self, matrix: list[list[int]]) -> list[list[int]]:
        """去鬼影 — 连通域分析，只保留质量最大的触摸群。

        鬼影成因：电阻矩阵多触点时，电流走旁路产生虚假读数。

        算法（BFS 连通域标记）：
        1. 以活性阈值二值化矩阵
        2. BFS 搜索所有 8-连通区域（每个区域 = 一个触摸群）
        3. 计算每个群的总质量（群内所有格值的总和）
        4. 只保留质量最大的群，其余全部置零

        这样远处因旁路电流产生的鬼影团就会被剔除。
        """
        threshold = self._ghost_threshold.get()
        N = EXC_COUNT

        visited = [[False] * N for _ in range(N)]
        components: list[tuple[int, list[tuple[int, int]]]] = []

        # ---- BFS 扫描所有连通域 ----
        for r in range(N):
            for c in range(N):
                if matrix[r][c] < threshold or visited[r][c]:
                    continue

                # 发现新群，BFS 扩展开
                queue = deque()
                queue.append((r, c))
                visited[r][c] = True
                cells: list[tuple[int, int]] = []
                total_mass = 0

                while queue:
                    cr, cc = queue.popleft()
                    cells.append((cr, cc))
                    total_mass += matrix[cr][cc]

                    # 8-连通邻域
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr, nc = cr + dr, cc + dc
                            if (0 <= nr < N and 0 <= nc < N
                                    and matrix[nr][nc] >= threshold
                                    and not visited[nr][nc]):
                                visited[nr][nc] = True
                                queue.append((nr, nc))

                components.append((total_mass, cells))

        # ---- 无有效群则原样返回 ----
        if not components:
            self._ghost_cluster_count = 0
            return matrix

        # ---- 按总质量降序：最大群排第一 ----
        components.sort(key=lambda x: x[0], reverse=True)
        self._ghost_cluster_count = len(components)

        # ---- 只保留最大群的格值，其余置零 ----
        kept = set(components[0][1])
        result = [[0] * N for _ in range(N)]
        for r, c in kept:
            result[r][c] = matrix[r][c]
        return result

    # ── 滑移检测 ───────────────────────────────────────────────────
    def _compute_cop(self, matrix: list[list[int]]) -> tuple[float, float, float]:
        """计算压力中心 (CoP) 和总压力值。

        返回 (col_x, row_y, total_pressure)。
        压力中心 = Σ(pos × pressure) / Σ(pressure)
        """
        total = 0.0
        sum_x = 0.0
        sum_y = 0.0
        N = EXC_COUNT
        for r in range(N):
            for c in range(N):
                val = matrix[r][c]
                if val > 0:
                    sum_x += c * val
                    sum_y += r * val
                    total += val
        if total < 1.0:
            # 无有效按压，返回矩阵中心
            return (N / 2 - 0.5, N / 2 - 0.5, 0.0)
        return (sum_x / total, sum_y / total, total)

    def _update_slip(self, matrix: list[list[int]]):
        """从滤波后的矩阵更新滑移状态并重绘箭头。

        流程：
        1. 计算当前 CoP
        2. 判断是否有有效按压（总压力 > 阈值）
        3. 维护 CoP 历史轨迹
        4. 从轨迹拟合移动方向（最近 N 帧的位移向量平均）
        5. 对方向做 EMA 平滑
        """
        cop_x, cop_y, total = self._compute_cop(matrix)

        # 有效按压判断：至少 2 个格有读数且总压力足够
        active_cells = sum(1 for row in matrix for v in row if v > 0)
        min_total = 200  # 总压力阈值，避免噪声触发

        was_touching = self._is_touching
        self._is_touching = (active_cells >= 2 and total >= min_total)

        if not self._is_touching:
            if was_touching:
                # 刚松手，清空历史
                self._cop_history.clear()
                self._slip_dx = 0.0
                self._slip_dy = 0.0
                self._slip_speed = 0.0
            self._draw_slip_arrow()
            return

        # 记录当前 CoP
        self._cop_history.append((cop_x, cop_y))

        if len(self._cop_history) < 2:
            self._draw_slip_arrow()
            return

        # 从历史轨迹计算瞬时速度向量（最近几帧的位移）
        # 取最近的几个点做差分平均
        hist = list(self._cop_history)
        diffs = []
        for i in range(1, len(hist)):
            dx = hist[i][0] - hist[i - 1][0]
            dy = hist[i][1] - hist[i - 1][1]
            diffs.append((dx, dy))

        if not diffs:
            self._draw_slip_arrow()
            return

        # 平均速度（最近更多的权重）
        instant_dx = 0.0
        instant_dy = 0.0
        total_w = 0.0
        for i, (dx, dy) in enumerate(diffs):
            w = i + 1  # 越新的权重越高
            instant_dx += dx * w
            instant_dy += dy * w
            total_w += w
        instant_dx /= total_w
        instant_dy /= total_w

        speed = math.sqrt(instant_dx * instant_dx + instant_dy * instant_dy)

        # EMA 平滑方向（避免箭头抖动）
        alpha = 0.35
        self._slip_dx = alpha * instant_dx + (1.0 - alpha) * self._slip_dx
        self._slip_dy = alpha * instant_dy + (1.0 - alpha) * self._slip_dy
        self._slip_speed = speed

        self._cop_x = cop_x
        self._cop_y = cop_y
        self._draw_slip_arrow()

    def _draw_slip_arrow(self):
        """在滑移面板 Canvas 上绘制方向箭头。"""
        cw = self._slip_canvas
        cw.delete("all")

        w = 190
        h = 250
        cx, cy = w // 2, h // 2 - 15

        if not self._is_touching or self._slip_speed < 0.01:
            # 空闲状态：灰色圆 + 中心点
            r = 45
            cw.create_oval(cx - r, cy - r, cx + r, cy + r,
                           outline="#cccccc", width=3, dash=(4, 4))
            cw.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                           fill="#cccccc", outline="")
            return

        # 方向向量
        mag = math.sqrt(self._slip_dx * self._slip_dx +
                        self._slip_dy * self._slip_dy)
        if mag < 0.005:
            # 有按压但几乎无移动
            r = 45
            cw.create_oval(cx - r, cy - r, cx + r, cy + r,
                           outline="#aaaaaa", width=3)
            cw.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                           fill="#4CAF50", outline="")
            cw.create_text(cx, cy - r - 15, text="静止",
                           fill="#888888", font=("微软雅黑", 9))
            return

        dx = self._slip_dx / mag
        dy = self._slip_dy / mag

        # 箭头长度：速度越快越长，但有上下限
        arrow_len = max(18, min(55, self._slip_speed * 30 + 15))

        # 速度等级决定颜色（慢→绿，中→橙，快→红）
        speed_norm = min(1.0, self._slip_speed / 2.0)
        if speed_norm < 0.33:
            color = "#4CAF50"      # 绿
        elif speed_norm < 0.66:
            color = "#FF9800"      # 橙
        else:
            color = "#F44336"      # 红

        # 外圆
        r = 45
        cw.create_oval(cx - r, cy - r, cx + r, cy + r,
                       outline="#dddddd", width=2, fill="#fafafa")

        # 十字参考线
        cw.create_line(cx - 35, cy, cx + 35, cy, fill="#e0e0e0", width=1)
        cw.create_line(cx, cy - 35, cx, cy + 35, fill="#e0e0e0", width=1)

        # 主方向箭头
        ex = cx + dx * arrow_len
        ey = cy + dy * arrow_len
        cw.create_line(cx, cy, ex, ey,
                       arrow=tk.LAST, arrowshape=(14, 18, 7),
                       fill=color, width=5, capstyle=tk.ROUND)

        # 起点圆点
        cw.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                       fill=color, outline="")

        # 轨迹尾巴（显示最近的移动路径）
        hist = list(self._cop_history)
        if len(hist) >= 3:
            # 在箭头坐标系中缩放历史点
            scale = 10.0  # 每格 = 10 像素
            points = []
            for hx, hy in hist[-8:]:
                px = cx + (hx - self._cop_x) * scale
                py = cy + (hy - self._cop_y) * scale
                # 裁剪到圆内
                dist = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
                if dist > r - 5:
                    px = cx + (px - cx) * (r - 5) / dist
                    py = cy + (py - cy) * (r - 5) / dist
                points.extend([px, py])
            if len(points) >= 4:
                cw.create_line(*points, fill="#bbbbbb", width=2, smooth=True)

        self._update_slip_labels()

    def _update_slip_labels(self):
        """更新滑移面板的文字标签。"""
        if not self._is_touching:
            self._lbl_slip_dir.config(text="等待按压…", foreground="gray")
            self._lbl_slip_speed.config(text="")
            return

        mag = math.sqrt(self._slip_dx * self._slip_dx +
                        self._slip_dy * self._slip_dy)
        if mag < 0.005:
            self._lbl_slip_dir.config(text="● 静止", foreground="#888888")
            self._lbl_slip_speed.config(text="")
            return

        # 方向文字
        dx = self._slip_dx / mag
        dy = self._slip_dy / mag
        angle = math.degrees(math.atan2(-dy, dx))  # 上为负

        if -22.5 <= angle < 22.5:
            dtext = "→ 右"
        elif 22.5 <= angle < 67.5:
            dtext = "↗ 右上"
        elif 67.5 <= angle < 112.5:
            dtext = "↑ 上"
        elif 112.5 <= angle < 157.5:
            dtext = "↖ 左上"
        elif angle >= 157.5 or angle < -157.5:
            dtext = "← 左"
        elif -157.5 <= angle < -112.5:
            dtext = "↙ 左下"
        elif -112.5 <= angle < -67.5:
            dtext = "↓ 下"
        else:
            dtext = "↘ 右下"

        self._lbl_slip_dir.config(text=dtext, foreground="#333333")
        self._lbl_slip_speed.config(
            text=f"速率 {self._slip_speed:.2f} 格/帧")

    def _apply_deadzone(self, matrix: list[list[int]]) -> list[list[int]]:
        """死区阈值 — 低于阈值的读数强制归零，消除底噪伪触点。"""
        th = self._deadzone.get()
        if th <= 0:
            return matrix
        return [
            [val if val >= th else 0 for val in row]
            for row in matrix
        ]

    # ── 滤波 UI 回调 ───────────────────────────────────────────────
    def _on_filter_param_change(self, *_):
        self._update_deadzone_label()
        self._apply_colours()

    def _update_deadzone_label(self):
        self._lbl_deadzone.config(text=str(self._deadzone.get()))

    def _on_spatial_mode_change(self):
        """空间滤波模式切换时立即刷新显示。"""
        self._apply_colours()

    def _on_denoise_toggle(self):
        """椒盐去噪开关切换。"""
        self._apply_colours()

    def _on_denoise_sigma_change(self, value):
        """椒盐去噪 σ 阈值滑动条回调。"""
        sigma = max(1.0, int(float(value)) / 10.0)
        self._denoise_sigma.set(sigma)
        self._lbl_denoise.config(text=f"{sigma:.1f}")
        self._apply_colours()

    def _on_ghost_toggle(self):
        """去鬼影开关切换。"""
        self._apply_colours()

    def _on_ghost_threshold_change(self, value):
        """去鬼影活性阈值滑动条回调。"""
        th = int(float(value))
        self._ghost_threshold.set(th)
        self._lbl_ghost.config(text=str(th))
        self._apply_colours()

    def _update_ghost_info_label(self):
        """更新去鬼影信息标签 — 显示检测到的群数量。"""
        if not self._use_ghost_removal.get():
            self._lbl_ghost_info.config(text="")
            return
        n = self._ghost_cluster_count
        if n == 0:
            self._lbl_ghost_info.config(text="未检测到触摸群", foreground="gray")
        elif n == 1:
            self._lbl_ghost_info.config(text="✓ 1 个群（无鬼影）", foreground="#4CAF50")
        else:
            self._lbl_ghost_info.config(
                text=f"⚠ {n} 个群 → 已剔除 {n - 1} 个鬼影",
                foreground="#FF5722")

    def _on_sigma_change(self, value):
        """高斯 σ 滑动条回调。"""
        sigma = max(0.3, int(float(value)) / 100.0)
        self._spatial_sigma.set(sigma)
        self._lbl_sigma.config(text=f"{sigma:.2f}")
        self._apply_colours()


# ── 入口 ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = MatrixViewer()
    app.mainloop()
