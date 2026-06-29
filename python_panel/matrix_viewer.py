#!/usr/bin/env python3
"""
电阻矩阵可视化工具 — 8×8 热力图面板，支持串口实时数据。

数据帧格式 (258 字节):
  [0xFF] [0xAA] [64 个采样 × 4B: exc_id, adc_ch, val_hi, val_lo]
"""

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
        self.geometry("780x880")
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

        for i, label in enumerate(EXC_LABELS):
            cb = ttk.Checkbutton(
                row_line, text=label, variable=self._row_visible[i],
                command=self._on_channel_toggle)
            cb.pack(side=tk.LEFT, padx=3)

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

        for i, label in enumerate(ADC_LABELS):
            cb = ttk.Checkbutton(
                col_line, text=label, variable=self._col_visible[i],
                command=self._on_channel_toggle)
            cb.pack(side=tk.LEFT, padx=3)

        ttk.Separator(col_line, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(col_line, text="全选", width=4,
                   command=self._select_all_cols).pack(side=tk.LEFT, padx=2)
        ttk.Button(col_line, text="全不选", width=5,
                   command=self._deselect_all_cols).pack(side=tk.LEFT, padx=2)
        ttk.Button(col_line, text="反选", width=4,
                   command=self._invert_cols).pack(side=tk.LEFT, padx=2)

        # ---- 网格区域 ----
        self._grid_frame = ttk.Frame(self)
        self._grid_frame.pack(expand=True, pady=5)

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
            lbl = ttk.Label(self._grid_frame, text=ADC_LABELS[c],
                            font=("Consolas", 9, "bold"), anchor=tk.CENTER)
            lbl.grid(row=0, column=gj, sticky="nsew", padx=1, pady=1, ipadx=6, ipady=2)

        # 行
        for r in vis_rows:
            gr = row_to_grid[r]
            # 行标题
            lbl = ttk.Label(self._grid_frame, text=EXC_LABELS[r],
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

        display = self._apply_zero(self.matrix)

        for gi, r in enumerate(vis_rows):
            for gj, c in enumerate(vis_cols):
                val = display[r][c]
                norm = (val - vmin) / rng
                fg = "white" if norm > 0.6 else "black"
                self._cells[gi][gj].configure(
                    bg=heat_colour(norm),
                    fg=fg,
                    text=str(val) if self.show_values.get() else "",
                )

    def _redraw_grid(self):
        self._apply_colours()

    # ── 数据处理（仅主线程调用）─────────────────────────────────────
    def _do_update_matrix(self, matrix: list[list[int]]):
        """在主线程中更新矩阵数据并刷新显示。"""
        self.matrix = matrix
        if self._auto_range.get():
            display = self._apply_zero(matrix)
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


# ── 入口 ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = MatrixViewer()
    app.mainloop()
