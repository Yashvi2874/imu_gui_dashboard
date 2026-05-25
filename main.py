import sys
import math
import random
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QPushButton, QSizePolicy, QSpacerItem,
    QDialog, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QTabWidget,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPainter, QPen, QBrush, QPalette, QIcon, QPixmap

import pyqtgraph as pg
import numpy as np
from trajectory import TrajectoryPage

BG_DEEP     = "#0A0E1A"
BG_CARD     = "#111827"
BG_CARD2    = "#161D2E"
BORDER      = "#1F2D45"
ACCENT_CYAN = "#00D4FF"
ACCENT_TEAL = "#00FFC8"
ACCENT_RED  = "#FF4560"
ACCENT_YEL  = "#FEB019"
ACCENT_PUR  = "#775DD0"
ACCENT_ORG  = "#FF8C42"
TEXT_PRI    = "#E8F4FD"
TEXT_SEC    = "#6B8CAE"

PLOT_COLORS = {
    "X": ACCENT_CYAN, "Y": ACCENT_TEAL, "Z": ACCENT_RED,
    "Roll": ACCENT_CYAN, "Pitch": ACCENT_YEL, "Yaw": ACCENT_PUR,
}

HISTORY   = 200
UPDATE_MS = 40


def scaled(base, w, h):
    return max(1, int(base * min(w / 1400, h / 860)))


class IMUSimulator:
    def __init__(self):
        self.t    = 0.0
        self._yaw = 0.0
        self.position_history = deque(maxlen=5000)
        self.x = 0.0
        self.y = 0.0
        self.velocity = 0.0

    def step(self, dt=0.04):
        self.t += dt
        t = self.t

        accel = {
            "X": 1.5 * math.sin(0.8 * t) + 0.4 * math.sin(3.1 * t + 0.5) + random.gauss(0, 0.08),
            "Y": 1.2 * math.cos(0.6 * t + 0.3) + 0.3 * math.sin(2.5 * t) + random.gauss(0, 0.08),
            "Z": 9.81 + 0.25 * math.sin(1.2 * t) + random.gauss(0, 0.05),
        }
        gyro = {
            "X": 8.0 * math.sin(0.5 * t + 1.0) + 2.0 * math.sin(2.0 * t) + random.gauss(0, 0.3),
            "Y": 6.0 * math.cos(0.7 * t) + 1.5 * math.sin(3.0 * t + 0.8) + random.gauss(0, 0.3),
            "Z": 4.0 * math.sin(0.3 * t + 2.0) + 1.0 * math.cos(1.8 * t) + random.gauss(0, 0.2),
        }
        # Magnetometer — Earth field ~50µT, slowly varying heading + noise
        mag = {
            "X": 28.0 * math.cos(self._yaw * math.pi / 180) + 2.0 * math.sin(0.3 * t) + random.gauss(0, 0.5),
            "Y": 28.0 * math.sin(self._yaw * math.pi / 180) + 2.0 * math.cos(0.3 * t) + random.gauss(0, 0.5),
            "Z": -38.0 + 1.5 * math.sin(0.15 * t) + random.gauss(0, 0.3),
        }
        self._yaw = (self._yaw + gyro["Z"] * dt * 0.6) % 360.0
        orient = {
            "Roll":  25.0 * math.sin(0.4 * t) + random.gauss(0, 0.2),
            "Pitch": 15.0 * math.cos(0.55 * t + 0.5) + random.gauss(0, 0.2),
            "Yaw":   self._yaw,
        }

        # Linear acceleration magnitude (gravity removed)
        lin_accel = math.sqrt(
            (accel["X"]) ** 2 +
            (accel["Y"]) ** 2 +
            (accel["Z"] - 9.81) ** 2
        )

        # Velocity magnitude — leaky integrator on lin accel
        if not hasattr(self, "_vel"):
            self._vel = 0.0
        self._vel = self._vel * 0.98 + lin_accel * dt

        # Onboard temperature (°C) — IMU chip heats up slightly over time
        temp = 28.0 + 6.0 * math.sin(0.05 * t) + random.gauss(0, 0.2)

        # Barometric pressure (hPa)
        pressure = 1013.25 + 8.0 * math.sin(0.08 * t) + random.gauss(0, 0.15)

        return accel, gyro, mag, orient, lin_accel, self._vel, temp, pressure

    def get_position(self, dt=0.04):
        if not hasattr(self, 'position_history'):
            self.position_history = deque(maxlen=5000)

        t = self.t

        # Reset downrange cache on restart
        if t < 0.1:
            if hasattr(self, '_downrange'):
                delattr(self, '_downrange')

        BURN_END  = 10.0
        COAST_END = 25.0
        APOGEE_T  = 27.0
        BURN_PEAK = 2800.0    # altitude at end of burn
        APOGEE_ALT = 3450.0   # peak altitude

        if t <= BURN_END:
            # Smooth power curve from 0 to BURN_PEAK
            frac = t / BURN_END
            alt = BURN_PEAK * (frac ** 0.55)
            vertical_speed = (BURN_PEAK * 0.55 / BURN_END) * max(frac, 0.01) ** (-0.45)
            accel_z = 140.0
            phase = "BURN"

        elif t <= COAST_END:
            # Smooth continuation — cosine interpolation from BURN_PEAK to APOGEE_ALT
            coast_t = t - BURN_END
            coast_dur = COAST_END - BURN_END
            frac = coast_t / coast_dur
            # Cosine easing: starts fast, ends slow (approaching apogee)
            ease = (1 - math.cos(frac * math.pi * 0.5))
            burn_alt = BURN_PEAK  # alt at end of burn = BURN_PEAK
            alt = burn_alt + (APOGEE_ALT - burn_alt) * ease
            vertical_speed = (APOGEE_ALT - burn_alt) * math.pi * 0.5 / coast_dur * math.sin(frac * math.pi * 0.5)
            accel_z = -9.81
            phase = "COAST"

        elif t <= APOGEE_T:
            alt = APOGEE_ALT
            vertical_speed = 0.0
            accel_z = -9.81
            phase = "APOGEE"

        else:
            descent_t = t - APOGEE_T
            alt = max(0.0, APOGEE_ALT - 0.5 * 18.0 * descent_t ** 2)
            vertical_speed = -(18.0 * descent_t)
            accel_z = -9.81
            if alt <= 0:
                alt = 0.0
                vertical_speed = 0.0
                accel_z = 0.0
                phase = "LANDED"
            else:
                phase = "DESCENT"

        # Horizontal downrange
        if phase in ("DESCENT", "LANDED"):
            if not hasattr(self, '_downrange'):
                self._downrange = 0.5 * APOGEE_T
            self._downrange += 2.5 * dt
            downrange = self._downrange
        else:
            self._downrange = 2.5 * t + 4.0 * math.sin(0.08 * t)
            downrange = self._downrange

        alt = max(0.0, alt + random.gauss(0, 1.5))

        pos_dict = {
            "x":             float(downrange),
            "y":             float(alt),
            "alt":           float(alt),
            "heading":       float(self._yaw),
            "speed":         float(abs(vertical_speed)),
            "vertical_speed": float(vertical_speed),
            "accel_z":       float(accel_z),
            "phase":         phase,
            "t":             float(t),
            "apogee":        float(APOGEE_ALT),
        }
        self.position_history.append(pos_dict)
        return pos_dict


def make_separator(vertical=False):
    sep = QFrame()
    sep.setFrameShape(QFrame.VLine if vertical else QFrame.HLine)
    sep.setStyleSheet(f"color: {BORDER}; background: {BORDER};")
    sep.setFixedWidth(1) if vertical else sep.setFixedHeight(1)
    return sep


class ValueCard(QWidget):
    def __init__(self, axis, unit, color, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(62)
        self.setStyleSheet(f"""
            background: {BG_CARD2};
            border: 1px solid {color}33;
            border-left: 3px solid {color};
            border-radius: 6px;
        """)
        row = QHBoxLayout(self)
        row.setContentsMargins(10, 0, 10, 0)
        row.setSpacing(6)

        self.axis_lbl = QLabel(axis)
        self.axis_lbl.setFont(QFont("Courier New", 9, QFont.Bold))
        self.axis_lbl.setStyleSheet(f"color: {color}; background: transparent; border: none;")
        self.axis_lbl.setFixedWidth(22)

        self.unit_lbl = QLabel(unit)
        self.unit_lbl.setFont(QFont("Courier New", 8))
        self.unit_lbl.setStyleSheet(f"color: {TEXT_SEC}; background: transparent; border: none;")

        self.val_lbl = QLabel("0.000")
        self.val_lbl.setFont(QFont("Courier New", 14, QFont.Bold))
        self.val_lbl.setStyleSheet(f"color: {TEXT_PRI}; background: transparent; border: none;")
        self.val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        row.addWidget(self.axis_lbl)
        row.addWidget(self.unit_lbl)
        row.addStretch()
        row.addWidget(self.val_lbl)

    def update_fonts(self, w, h):
        fs = scaled(9, w, h)
        self.axis_lbl.setFont(QFont("Courier New", fs, QFont.Bold))
        self.unit_lbl.setFont(QFont("Courier New", max(1, fs - 1)))
        self.val_lbl.setFont(QFont("Courier New", scaled(14, w, h), QFont.Bold))

    def update_value(self, v):
        self.val_lbl.setText(f"{v:+.3f}" if abs(v) < 100 else f"{v:+.1f}")


class WideValueCard(QWidget):
    """Single-value card with a wider label — for scalar readings like temp, pressure, magnitude."""
    def __init__(self, label, unit, color, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"""
            background: {BG_CARD2};
            border: 1px solid {color}33;
            border-left: 3px solid {color};
            border-radius: 6px;
        """)
        row = QHBoxLayout(self)
        row.setContentsMargins(10, 0, 10, 0)
        row.setSpacing(6)

        self.lbl = QLabel(label)
        self.lbl.setFont(QFont("Courier New", 11, QFont.Bold))
        self.lbl.setStyleSheet(f"color: {color}; background: transparent; border: none;")

        self.unit_lbl = QLabel(unit)
        self.unit_lbl.setFont(QFont("Courier New", 9))
        self.unit_lbl.setStyleSheet(f"color: {TEXT_SEC}; background: transparent; border: none;")

        self.val_lbl = QLabel("0.000")
        self.val_lbl.setFont(QFont("Courier New", 15, QFont.Bold))
        self.val_lbl.setStyleSheet(f"color: {TEXT_PRI}; background: transparent; border: none;")
        self.val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        row.addWidget(self.lbl)
        row.addWidget(self.unit_lbl)
        row.addStretch()
        row.addWidget(self.val_lbl)

    def update_value(self, v):
        self.val_lbl.setText(f"{v:+.3f}" if abs(v) < 100 else f"{v:+.1f}")


class ArcGauge(QWidget):
    def __init__(self, label, min_val, max_val, color, parent=None):
        super().__init__(parent)
        self.label   = label
        self.min_val = min_val
        self.max_val = max_val
        self.color   = QColor(color)
        self._value  = 0.0
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v):
        self._value = max(self.min_val, min(self.max_val, v))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        size = min(w, h) - 10
        x = (w - size) // 2
        y = (h - size) // 2
        pen_w = max(3, size // 16)

        painter.setPen(QPen(QColor(BORDER), pen_w, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(x, y, size, size, -210 * 16, -300 * 16)

        norm = (self._value - self.min_val) / (self.max_val - self.min_val)
        painter.setPen(QPen(self.color, pen_w, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(x, y, size, size, -210 * 16, int(-norm * 300 * 16))

        val_fs = max(7, size // 9)
        painter.setPen(QPen(QColor(TEXT_PRI)))
        painter.setFont(QFont("Courier New", val_fs, QFont.Bold))
        painter.drawText(0, 0, w, h, Qt.AlignCenter, f"{self._value:.1f}°")

        lbl_fs = max(6, size // 14)
        painter.setPen(QPen(self.color))
        painter.setFont(QFont("Courier New", lbl_fs, QFont.Bold))
        painter.drawText(0, h - lbl_fs * 2 - 2, w, lbl_fs * 2, Qt.AlignCenter, self.label)


class BarIndicator(QWidget):
    def __init__(self, label, unit, min_v, max_v, warn, crit, color, parent=None):
        super().__init__(parent)
        self.label      = label
        self.unit       = unit
        self.min_v      = min_v
        self.max_v      = max_v
        self.warn       = warn
        self.crit       = crit
        self.base_color = QColor(color)
        self._value     = min_v
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v):
        self._value = max(self.min_v, min(self.max_v, v))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        lbl_w = max(44, int(w * 0.17))
        val_w = max(52, int(w * 0.22))
        bar_x = lbl_w + 4
        bar_w = w - lbl_w - val_w - 8
        bar_h = max(5, int(h * 0.25))
        bar_y = (h - bar_h) // 2
        fs    = max(6, int(h * 0.16))

        painter.setPen(QPen(QColor(TEXT_SEC)))
        painter.setFont(QFont("Courier New", fs, QFont.Bold))
        painter.drawText(0, 0, lbl_w, h, Qt.AlignVCenter | Qt.AlignLeft, self.label)

        painter.setBrush(QBrush(QColor(BORDER)))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(bar_x, bar_y, bar_w, bar_h, 3, 3)

        norm  = (self._value - self.min_v) / (self.max_v - self.min_v)
        fill_w = int(norm * bar_w)
        fill_color = (QColor(ACCENT_RED) if self._value >= self.crit
                      else QColor(ACCENT_YEL) if self._value >= self.warn
                      else self.base_color)
        if fill_w > 0:
            painter.setBrush(QBrush(fill_color))
            painter.drawRoundedRect(bar_x, bar_y, fill_w, bar_h, 3, 3)

        painter.setPen(QPen(QColor(TEXT_PRI)))
        painter.setFont(QFont("Courier New", fs, QFont.Bold))
        painter.drawText(w - val_w, 0, val_w, h, Qt.AlignVCenter | Qt.AlignRight,
                         f"{self._value:.1f}{self.unit}")


def styled_plot(title, x_label="Time (s)", y_label=""):
    pw = pg.PlotWidget()
    pw.setBackground(BG_CARD)
    pw.showGrid(x=True, y=True, alpha=0.08)
    pw.getPlotItem().hideAxis("top")
    pw.getPlotItem().hideAxis("right")
    pw.getAxis("bottom").setPen(pg.mkPen(BORDER))
    pw.getAxis("left").setPen(pg.mkPen(BORDER))
    pw.getAxis("bottom").setTextPen(pg.mkPen(TEXT_SEC))
    pw.getAxis("left").setTextPen(pg.mkPen(TEXT_SEC))
    pw.getAxis("bottom").setLabel(x_label, color=TEXT_SEC)
    pw.getAxis("left").setLabel(y_label, color=TEXT_SEC)
    pw.getPlotItem().layout.addItem(pg.LabelItem(title, color=TEXT_SEC, size="9pt"), 0, 1)
    pw.setMouseEnabled(x=False, y=False)
    pw.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    return pw


class AircraftAttitudeIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(160, 160)

    def set_attitude(self, roll, pitch, yaw):
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw
        self.update()

    def paintEvent(self, event):
        from PyQt5.QtGui import QPainterPath, QPolygonF
        from PyQt5.QtCore import QPointF

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx = w / 2
        strip_h = max(28, int(h * 0.12))
        ball_h  = h - strip_h - 4
        cy      = ball_h / 2
        r       = min(w, ball_h) / 2 - 14

        clip = QPainterPath()
        clip.addEllipse(cx - r, cy - r, r * 2, r * 2)
        painter.setClipPath(clip)

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)
        pitch_px = (self.pitch / 90.0) * r

        painter.setBrush(QBrush(QColor("#1B4F8A")))
        painter.setPen(Qt.NoPen)
        painter.drawRect(int(-r * 2), int(-r * 2 + pitch_px), int(r * 4), int(r * 2))
        painter.setBrush(QBrush(QColor("#6B3A1F")))
        painter.drawRect(int(-r * 2), int(pitch_px), int(r * 4), int(r * 2))

        for deg in range(-60, 61, 10):
            if deg == 0:
                continue
            y_off  = pitch_px - (deg / 90.0) * r
            line_w = r * (0.38 if deg % 20 == 0 else 0.22)
            painter.setPen(QPen(QColor(255, 255, 255, 60), 1))
            painter.drawLine(int(-line_w), int(y_off), int(line_w), int(y_off))
            if deg % 20 == 0:
                painter.setPen(QPen(QColor(220, 220, 220, 180)))
                painter.setFont(QFont("Courier New", max(5, int(r * 0.09))))
                painter.drawText(int(line_w + 3), int(y_off + 4), f"{abs(deg)}")

        painter.setPen(QPen(QColor("#FFFFFF"), 3))
        painter.drawLine(int(-r), int(pitch_px), int(r), int(pitch_px))
        painter.restore()
        painter.setClipping(False)

        painter.setPen(QPen(QColor("#2A4060"), 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(int(cx - r), int(cy - r), int(r * 2), int(r * 2))

        painter.setPen(QPen(QColor(TEXT_SEC), 1))
        for deg in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            angle_rad = math.radians(deg - 90)
            tick_len  = r * (0.09 if deg % 30 == 0 else 0.05)
            x1 = cx + (r - 1) * math.cos(angle_rad)
            y1 = cy + (r - 1) * math.sin(angle_rad)
            x2 = cx + (r - 1 - tick_len) * math.cos(angle_rad)
            y2 = cy + (r - 1 - tick_len) * math.sin(angle_rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)
        tri = r * 0.11
        poly = QPolygonF([QPointF(0, -(r - 3)),
                          QPointF(-tri, -(r - 3 - tri * 1.8)),
                          QPointF( tri, -(r - 3 - tri * 1.8))])
        painter.setBrush(QBrush(QColor(ACCENT_YEL)))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(poly)
        painter.restore()

        ac     = r * 0.52
        pen_ac = QPen(QColor("#FFFFFF"), max(2, int(r * 0.05)), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(pen_ac)
        painter.drawLine(int(cx), int(cy - ac * 0.25), int(cx), int(cy + ac * 0.25))
        painter.drawLine(int(cx - ac), int(cy), int(cx + ac), int(cy))
        painter.drawLine(int(cx - ac), int(cy), int(cx - ac * 0.72), int(cy - ac * 0.15))
        painter.drawLine(int(cx + ac), int(cy), int(cx + ac * 0.72), int(cy - ac * 0.15))
        tw = ac * 0.32
        painter.drawLine(int(cx - tw), int(cy + ac * 0.22), int(cx + tw), int(cy + ac * 0.22))
        painter.setBrush(QBrush(QColor("#FFFFFF")))
        painter.setPen(Qt.NoPen)
        dr = max(3, int(r * 0.055))
        painter.drawEllipse(int(cx - dr), int(cy - dr), dr * 2, dr * 2)

        sy = int(ball_h + 4)
        painter.setBrush(QBrush(QColor(BG_CARD2)))
        painter.setPen(QPen(QColor(BORDER), 1))
        painter.drawRoundedRect(0, sy, w, strip_h, 4, 4)
        fs    = max(7, int(strip_h * 0.38))
        third = w // 3
        painter.setFont(QFont("Courier New", fs, QFont.Bold))
        painter.setPen(QPen(QColor(ACCENT_CYAN)))
        painter.drawText(0, sy, third, strip_h, Qt.AlignCenter, f"R {self.roll:+.1f}°")
        painter.setPen(QPen(QColor(ACCENT_YEL)))
        painter.drawText(third, sy, third, strip_h, Qt.AlignCenter, f"P {self.pitch:+.1f}°")
        painter.setPen(QPen(QColor(ACCENT_PUR)))
        painter.drawText(third * 2, sy, third, strip_h, Qt.AlignCenter, f"Y {self.yaw:.1f}°")


class TelemetryTableDialog(QDialog):
    COLS = [
        "T (s)",
        "Ax (m/s²)", "Ay (m/s²)", "Az (m/s²)",
        "Gx (°/s)",  "Gy (°/s)",  "Gz (°/s)",
        "Mx (µT)",   "My (µT)",   "Mz (µT)",
        "Roll (°)",  "Pitch (°)", "Yaw (°)",
    ]

    def __init__(self, records_ref, parent=None):
        super().__init__(parent)
        self._records_ref = records_ref
        self.setWindowTitle("Telemetry Log — 1 Hz")
        self.resize(1100, 600)
        self.setStyleSheet(f"background: {BG_DEEP}; color: {TEXT_PRI};")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        self.title_lbl = QLabel()
        self.title_lbl.setFont(QFont("Courier New", 11, QFont.Bold))
        self.title_lbl.setStyleSheet(f"color: {ACCENT_CYAN};")
        layout.addWidget(self.title_lbl)

        self.table = QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setStyleSheet(f"""
            QTableWidget {{
                background: {BG_CARD}; color: {TEXT_PRI};
                gridline-color: {BORDER}; border: 1px solid {BORDER};
                font-family: 'Courier New'; font-size: 9pt;
            }}
            QHeaderView::section {{
                background: {BG_CARD2}; color: {ACCENT_CYAN};
                font-family: 'Courier New'; font-size: 9pt; font-weight: bold;
                border: 1px solid {BORDER}; padding: 4px;
            }}
            QTableWidget::item:selected {{ background: {ACCENT_CYAN}33; }}
        """)
        layout.addWidget(self.table)

        close_btn = QPushButton("CLOSE")
        close_btn.setFixedHeight(30)
        close_btn.setStyleSheet(f"""
            QPushButton {{
                background: {BG_CARD2}; color: {ACCENT_RED};
                border: 1px solid {ACCENT_RED}55; border-radius: 4px;
                font-family: 'Courier New'; font-size: 10pt; font-weight: bold;
                padding: 0 16px;
            }}
            QPushButton:hover {{ background: {ACCENT_RED}22; }}
        """)
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn, alignment=Qt.AlignRight)

        # Refresh every second while open
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start(1000)
        self._refresh()

    def _refresh(self):
        records = self._records_ref
        self.title_lbl.setText(f"HISTORICAL TELEMETRY  —  {len(records)} samples @ 1 Hz")
        self.table.setRowCount(len(records))
        for row, rec in enumerate(records):
            for col, val in enumerate(rec):
                item = QTableWidgetItem(f"{val:.3f}" if isinstance(val, float) else str(val))
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(row, col, item)
        # Auto-scroll to latest
        if records:
            self.table.scrollToBottom()

    def closeEvent(self, event):
        self._refresh_timer.stop()
        super().closeEvent(event)


class IMUDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Synodic Space Labs — IMU Dashboard")
        self.resize(1400, 860)
        self.setStyleSheet(f"QMainWindow {{ background: {BG_DEEP}; }}")

        self.ts = deque([0.0] * HISTORY, maxlen=HISTORY)
        self.buffers = {
            "accel":  {k: deque([0.0] * HISTORY, maxlen=HISTORY) for k in ["X", "Y", "Z"]},
            "gyro":   {k: deque([0.0] * HISTORY, maxlen=HISTORY) for k in ["X", "Y", "Z"]},
            "mag":    {k: deque([0.0] * HISTORY, maxlen=HISTORY) for k in ["X", "Y", "Z"]},
            "orient": {k: deque([0.0] * HISTORY, maxlen=HISTORY) for k in ["Roll", "Pitch", "Yaw"]},
        }
        self.simulator    = IMUSimulator()
        self.elapsed      = 0.0
        self.running      = True
        self._plots_built = False
        self._telemetry   = []          # 1 Hz log records
        self._last_log_t  = -1.0        # last logged second

        # Create QTabWidget and style it
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: none;
                background: {BG_DEEP};
            }}
            QTabBar {{
                background: {BG_DEEP};
            }}
            QTabBar::tab {{
                background: {BG_DEEP};
                color: #4A5568;
                height: 38px;
                padding: 0 20px;
                min-width: 260px;
                font-family: 'Courier New';
                font-size: 12pt;
                font-weight: bold;
                border: none;
            }}
            QTabBar::tab:selected {{
                color: #00E5FF;
                background: {BG_DEEP};
            }}
            QTabBar::tab:hover {{
                color: #00E5FF;
            }}
        """)
        self.setCentralWidget(self.tabs)

        self._build_ui()
        self._build_plots()

        self.trajectory_page = TrajectoryPage(self.simulator)
        self.tabs.addTab(self.trajectory_page, "  Trajectory  ")

        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.timeout.connect(self.trajectory_page.update)
        self.timer.start(UPDATE_MS)

    def _section_label(self, text):
        lbl = QLabel(text)
        lbl.setFont(QFont("Courier New", 13, QFont.Bold))
        lbl.setStyleSheet(f"color: {TEXT_SEC}; letter-spacing: 2px; background: transparent;")
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lbl.setFixedHeight(22)
        return lbl

    def _build_ui(self):
        root = QWidget()
        root.setStyleSheet(f"background: {BG_DEEP};")
        self.tabs.addTab(root, "  IMU Dashboard  ")

        master = QVBoxLayout(root)
        master.setContentsMargins(12, 8, 12, 8)
        master.setSpacing(6)

        header = QHBoxLayout()
        self.logo = QLabel("◈ SYNODIC SPACE LABS")
        self.logo.setFont(QFont("Courier New", 13, QFont.Bold))
        self.logo.setStyleSheet(f"color: {ACCENT_CYAN}; letter-spacing: 3px;")

        self.status_dot = QLabel("● LIVE")
        self.status_dot.setFont(QFont("Courier New", 9, QFont.Bold))
        self.status_dot.setStyleSheet(f"color: {ACCENT_TEAL};")

        self.time_lbl = QLabel("T+00:00:00")
        self.time_lbl.setFont(QFont("Courier New", 10))
        self.time_lbl.setStyleSheet(f"color: {TEXT_SEC};")

        self.pause_btn = QPushButton("⏸ PAUSE")
        self.pause_btn.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.pause_btn.setFixedHeight(36)
        self.pause_btn.setStyleSheet(f"""
            QPushButton {{
                background: {BG_CARD2}; color: {ACCENT_YEL};
                border: 1px solid {ACCENT_YEL}55; border-radius: 4px;
                font-family: 'Courier New'; font-size: 13pt; font-weight: bold;
                padding: 0 16px;
            }}
            QPushButton:hover {{ background: {ACCENT_YEL}22; }}
        """)
        self.pause_btn.clicked.connect(self._toggle_pause)

        self.telem_btn = QPushButton("📋 TELEMETRY")
        self.telem_btn.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.telem_btn.setFixedHeight(36)
        self.telem_btn.setStyleSheet(f"""
            QPushButton {{
                background: {BG_CARD2}; color: {ACCENT_CYAN};
                border: 1px solid {ACCENT_CYAN}55; border-radius: 4px;
                font-family: 'Courier New'; font-size: 13pt; font-weight: bold;
                padding: 0 16px;
            }}
            QPushButton:hover {{ background: {ACCENT_CYAN}22; }}
        """)
        self.telem_btn.clicked.connect(self._show_telemetry)

        header.addWidget(self.logo)
        header.addSpacerItem(QSpacerItem(10, 0, QSizePolicy.Expanding))
        header.addWidget(self.status_dot)
        header.addSpacing(16)
        header.addWidget(self.time_lbl)
        header.addSpacing(12)
        header.addWidget(self.telem_btn)
        header.addSpacing(8)
        header.addWidget(self.pause_btn)
        master.addLayout(header)
        master.addWidget(make_separator())

        body = QHBoxLayout()
        body.setSpacing(10)
        body.addLayout(self._col_left(),   stretch=3)
        body.addWidget(make_separator(vertical=True))
        body.addLayout(self._col_centre(), stretch=4)
        body.addWidget(make_separator(vertical=True))
        body.addLayout(self._col_right(),  stretch=3)
        master.addLayout(body, stretch=1)

    def _col_left(self):
        col = QVBoxLayout()
        col.setSpacing(6)

        col.addWidget(self._section_label("ACCELEROMETER  (m/s²)"))
        self.accel_cards = {}
        for ax, clr in [("X", ACCENT_CYAN), ("Y", ACCENT_TEAL), ("Z", ACCENT_RED)]:
            card = ValueCard(ax, "m/s²", clr)
            self.accel_cards[ax] = card
            col.addWidget(card, stretch=1)

        col.addWidget(self._section_label("GYROSCOPE  (°/s)"))
        self.gyro_cards = {}
        for ax, clr in [("X", ACCENT_CYAN), ("Y", ACCENT_TEAL), ("Z", ACCENT_RED)]:
            card = ValueCard(ax, "°/s", clr)
            self.gyro_cards[ax] = card
            col.addWidget(card, stretch=1)

        col.addWidget(self._section_label("MAGNETOMETER  (µT)"))
        self.mag_cards = {}
        for ax, clr in [("X", ACCENT_CYAN), ("Y", ACCENT_TEAL), ("Z", ACCENT_ORG)]:
            card = ValueCard(ax, "µT", clr)
            self.mag_cards[ax] = card
            col.addWidget(card, stretch=1)

        col.addWidget(self._section_label("LINEAR ACCELERATION  (m/s²)"))
        self.lin_card = WideValueCard("LIN ACCEL", "m/s²", ACCENT_CYAN)
        col.addWidget(self.lin_card, stretch=1)

        col.addWidget(self._section_label("LINEAR VELOCITY  (m/s)"))
        self.vel_card = WideValueCard("LIN VEL", "m/s", ACCENT_TEAL)
        col.addWidget(self.vel_card, stretch=1)

        col.addWidget(self._section_label("STATISTICS (LIVE)"))
        self.stats_lbl = QLabel()
        self.stats_lbl.setFont(QFont("Courier New", 11))
        self.stats_lbl.setStyleSheet(
            f"color: {TEXT_SEC}; background: {BG_CARD2}; "
            f"border: 1px solid {BORDER}; border-radius: 4px; padding: 8px;"
        )
        self.stats_lbl.setWordWrap(True)
        self.stats_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        col.addWidget(self.stats_lbl, stretch=3)

        return col

    def _col_centre(self):
        col = QVBoxLayout()
        col.setSpacing(4)

        col.addWidget(self._section_label("ACCELEROMETER HISTORY"))
        self.accel_plot = styled_plot("Accel (m/s²)", y_label="Acceleration (m/s²)")
        col.addWidget(self.accel_plot, stretch=1)

        col.addWidget(self._section_label("GYROSCOPE HISTORY"))
        self.gyro_plot = styled_plot("Gyro (°/s)", y_label="Angular Rate (°/s)")
        col.addWidget(self.gyro_plot, stretch=1)

        col.addWidget(self._section_label("MAGNETOMETER HISTORY"))
        self.mag_plot = styled_plot("Mag (µT)", y_label="Field Strength (µT)")
        col.addWidget(self.mag_plot, stretch=1)

        col.addWidget(self._section_label("ORIENTATION HISTORY"))
        self.orient_plot = styled_plot("Orient (°)", y_label="Angle (°)")
        col.addWidget(self.orient_plot, stretch=1)
        return col

    def _col_right(self):
        col = QVBoxLayout()
        col.setSpacing(4)

        col.addWidget(self._section_label("SYSTEM HEALTH"))
        self.bar_cpu  = BarIndicator("CPU",    "%",   0,   100,  70,  90,  ACCENT_TEAL)
        self.bar_bat  = BarIndicator("BATT",   "%",   0,   100,  30,  15,  ACCENT_CYAN)
        self.bar_temp = BarIndicator("TEMP",   "°C",  20,  90,   60,  75,  ACCENT_YEL)
        self.bar_sig  = BarIndicator("SIGNAL", "%",   0,   100,  30,  15,  ACCENT_PUR)
        self.bar_baro = BarIndicator("BARO",   "hPa", 900, 1100, 999, 980, ACCENT_TEAL)
        health_layout = QVBoxLayout()
        health_layout.setSpacing(0)
        health_layout.setContentsMargins(0, 0, 0, 0)
        for bar in [self.bar_cpu, self.bar_bat, self.bar_temp, self.bar_sig, self.bar_baro]:
            bar.setFixedHeight(58)
            health_layout.addWidget(bar)
        col.addLayout(health_layout)

        col.addSpacing(4)
        col.addWidget(self._section_label("ORIENTATION  (ARC)"))
        col.addSpacing(0)
        gauge_row = QHBoxLayout()
        gauge_row.setSpacing(3)
        self.gauges = {}
        for name, mn, mx, clr in [("Roll", -90, 90, ACCENT_CYAN),
                                   ("Pitch", -90, 90, ACCENT_YEL),
                                   ("Yaw", 0, 360, ACCENT_PUR)]:
            g = ArcGauge(name, mn, mx, clr)
            self.gauges[name] = g
            gauge_row.addWidget(g)
        col.addLayout(gauge_row, stretch=1)

        col.addSpacing(40)
        col.addWidget(self._section_label("ATTITUDE INDICATOR"))
        col.addSpacing(12)
        self.attitude = AircraftAttitudeIndicator()
        self.attitude.setMaximumHeight(360)
        col.addWidget(self.attitude, stretch=3)
        col.addSpacing(40)

        col.addWidget(self._section_label("TEMPERATURE  /  PRESSURE"))
        self.temp_card = WideValueCard("TEMPERATURE", "°C",  ACCENT_YEL)
        self.pres_card = WideValueCard("PRESSURE",    "hPa", ACCENT_PUR)
        col.addWidget(self.temp_card, stretch=0)
        col.addWidget(self.pres_card, stretch=0)
        return col

    def _build_plots(self):
        lw = 1.5
        # Add legend once per plot
        for plot in [self.accel_plot, self.gyro_plot, self.orient_plot, self.mag_plot]:
            lg = plot.addLegend(offset=(-10, 10))
            lg.setLabelTextColor(TEXT_SEC)

        self.accel_curves  = {ax: self.accel_plot.plot(pen=pg.mkPen(PLOT_COLORS[ax], width=lw), name=ax) for ax in ["X", "Y", "Z"]}
        self.gyro_curves   = {ax: self.gyro_plot.plot(pen=pg.mkPen(PLOT_COLORS[ax], width=lw), name=ax) for ax in ["X", "Y", "Z"]}
        self.orient_curves = {k: self.orient_plot.plot(pen=pg.mkPen(PLOT_COLORS[k], width=lw), name=k) for k in ["Roll", "Pitch", "Yaw"]}
        mag_colors = {"X": ACCENT_CYAN, "Y": ACCENT_TEAL, "Z": ACCENT_ORG}
        self.mag_curves    = {ax: self.mag_plot.plot(pen=pg.mkPen(mag_colors[ax], width=lw), name=ax) for ax in ["X", "Y", "Z"]}

    def resizeEvent(self, event):
        super().resizeEvent(event)
        w, h = self.width(), self.height()
        self.logo.setFont(QFont("Courier New", max(8, scaled(13, w, h)), QFont.Bold))
        self.time_lbl.setFont(QFont("Courier New", max(7, scaled(10, w, h))))
        for cards in [getattr(self, "accel_cards", {}),
                      getattr(self, "gyro_cards", {}),
                      getattr(self, "mag_cards", {})]:
            for card in cards.values():
                card.update_fonts(w, h)

    def _tick(self):
        if not self.running:
            return

        dt = UPDATE_MS / 1000.0
        self.elapsed += dt
        accel, gyro, mag, orient, lin_accel, vel, temp, pressure = self.simulator.step(dt)

        self.ts.append(self.elapsed)
        for ax in ["X", "Y", "Z"]:
            self.buffers["accel"][ax].append(accel[ax])
            self.buffers["gyro"][ax].append(gyro[ax])
            self.buffers["mag"][ax].append(mag[ax])
        for k in ["Roll", "Pitch", "Yaw"]:
            self.buffers["orient"][k].append(orient[k])

        ts = np.array(self.ts)

        for ax in ["X", "Y", "Z"]:
            self.accel_cards[ax].update_value(accel[ax])
            self.gyro_cards[ax].update_value(gyro[ax])
            self.mag_cards[ax].update_value(mag[ax])
            self.accel_curves[ax].setData(ts, np.array(self.buffers["accel"][ax]))
            self.gyro_curves[ax].setData(ts, np.array(self.buffers["gyro"][ax]))
            self.mag_curves[ax].setData(ts, np.array(self.buffers["mag"][ax]))

        self.lin_card.update_value(lin_accel)
        self.vel_card.update_value(vel)
        self.temp_card.update_value(temp)
        self.pres_card.update_value(pressure)


        for k in ["Roll", "Pitch", "Yaw"]:
            self.gauges[k].set_value(orient[k])
            self.orient_curves[k].setData(ts, np.array(self.buffers["orient"][k]))

        self.attitude.set_attitude(orient["Roll"], orient["Pitch"], orient["Yaw"])

        # 1 Hz telemetry log
        current_sec = int(self.elapsed)
        if current_sec != self._last_log_t:
            self._last_log_t = current_sec
            self._telemetry.append((
                float(current_sec),
                accel["X"], accel["Y"], accel["Z"],
                gyro["X"],  gyro["Y"],  gyro["Z"],
                mag["X"],   mag["Y"],   mag["Z"],
                orient["Roll"], orient["Pitch"], orient["Yaw"],
            ))

        cpu  = 40 + 30 * abs(math.sin(0.2 * self.elapsed)) + random.gauss(0, 2)
        bat  = max(0, 85 - self.elapsed * 0.05)
        temp = 38 + 12 * abs(math.sin(0.1 * self.elapsed)) + random.gauss(0, 0.5)
        sig  = 70 + 20 * math.sin(0.15 * self.elapsed) + random.gauss(0, 1)
        baro = 1013 + 8 * math.sin(0.08 * self.elapsed) + random.gauss(0, 0.3)

        self.bar_cpu.set_value(cpu)
        self.bar_bat.set_value(bat)
        self.bar_temp.set_value(temp)
        self.bar_sig.set_value(sig)
        self.bar_baro.set_value(baro)

        m, s = divmod(int(self.elapsed), 60)
        h, m = divmod(m, 60)
        self.time_lbl.setText(f"T+{h:02d}:{m:02d}:{s:02d}")

        # Update window title with real-time speed and altitude
        if hasattr(self, "trajectory_page") and self.trajectory_page.last_position:
            pos = self.trajectory_page.last_position
            self.setWindowTitle(f"GCS Dashboard | {pos['speed']:.1f} m/s | Alt: {pos['alt']:.1f} m")

        ax_data = np.array(self.buffers["accel"]["X"])
        mag_mag = np.sqrt(
            np.array(self.buffers["accel"]["X"]) ** 2 +
            np.array(self.buffers["accel"]["Y"]) ** 2 +
            np.array(self.buffers["accel"]["Z"]) ** 2
        )
        mag_field = math.sqrt(mag["X"]**2 + mag["Y"]**2 + mag["Z"]**2)
        heading   = math.degrees(math.atan2(mag["Y"], mag["X"])) % 360

        self.stats_lbl.setText(
            f"│ ACCEL-X  μ={np.mean(ax_data):+.3f}  σ={np.std(ax_data):.3f}\n"
            f"│ |ACCEL|  {np.mean(mag_mag):.3f} m/s²\n"
            f"│ |MAG|    {mag_field:.1f} µT\n"
            f"│ HEADING  {heading:.1f}°\n"
            f"│ YAW      {orient['Yaw']:.1f}°\n"
            f"│ SAMPLES  {int(self.elapsed / dt):,}"
        )

    def _show_telemetry(self):
        dlg = TelemetryTableDialog(self._telemetry, parent=self)
        dlg.show()  # non-blocking so live updates keep flowing

    def _toggle_pause(self):
        self.running = not self.running
        if self.running:
            self.pause_btn.setText("⏸ PAUSE")
            self.status_dot.setText("● LIVE")
            self.status_dot.setStyleSheet(f"color: {ACCENT_TEAL};")
        else:
            self.pause_btn.setText("▶ RESUME")
            self.status_dot.setText("● PAUSED")
            self.status_dot.setStyleSheet(f"color: {ACCENT_YEL};")


def main():
    pg.setConfigOptions(antialias=True, foreground=TEXT_SEC, background=BG_CARD)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(BG_DEEP))
    pal.setColor(QPalette.WindowText,      QColor(TEXT_PRI))
    pal.setColor(QPalette.Base,            QColor(BG_CARD))
    pal.setColor(QPalette.AlternateBase,   QColor(BG_CARD2))
    pal.setColor(QPalette.Text,            QColor(TEXT_PRI))
    pal.setColor(QPalette.ButtonText,      QColor(TEXT_PRI))
    pal.setColor(QPalette.Highlight,       QColor(ACCENT_CYAN))
    pal.setColor(QPalette.HighlightedText, QColor(BG_DEEP))
    app.setPalette(pal)

    win = IMUDashboard()
    win.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
