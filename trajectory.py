from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QGraphicsPolygonItem, QCheckBox, QGroupBox, QLabel, QPushButton, QGraphicsEllipseItem, QFileDialog, QMessageBox, QScrollArea, QFrame
from PyQt5.QtGui import QPolygonF, QPen, QBrush, QColor, QFont, QPixmap
from PyQt5.QtCore import pyqtSlot, QPointF, Qt, QRectF, QTimer
import pyqtgraph as pg
import math
import csv
import time

class TrajectoryPage(QWidget):
    def __init__(self, simulator, parent=None):
        super().__init__(parent)
        self.simulator = simulator
        self.last_position = None
        self.max_speed = 0.0
        self.flight_log = []
        
        self._last_draw_count = 0

        # Smooth marker animation state
        self.prev_position = None
        self.curr_position = None
        self.anim_start_time = 0.0
        self.anim_timer = QTimer(self)
        self.anim_timer.setInterval(10)  # ~100Hz animation ticks
        self.anim_timer.timeout.connect(self.animate_marker)
        self.anim_timer.start()
        
        self.setup_ui()

    def create_telemetry_row(self, key_text):
        row_widget = QWidget()
        row_widget.setFixedHeight(46)
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(10, 9, 10, 9)
        row_layout.setSpacing(8)
        
        key_label = QLabel(key_text)
        key_label.setStyleSheet("color: #8892A4; font-family: 'Courier New'; font-size: 18px; font-weight: bold; background: transparent; border: none;")
        
        val_label = QLabel("0.0")
        val_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        val_label.setStyleSheet("color: #FFFFFF; font-family: 'Courier New'; font-size: 19px; font-weight: bold; background: transparent; border: none;")
        
        row_layout.addWidget(key_label)
        row_layout.addStretch()
        row_layout.addWidget(val_label)
        return row_widget, val_label

    def setup_ui(self):
        # Main layout is vertical
        main_vertical_layout = QVBoxLayout(self)
        main_vertical_layout.setContentsMargins(10, 10, 10, 10)
        main_vertical_layout.setSpacing(10)

        # CONTROLS Header label
        controls_label = QLabel("CONTROLS")
        controls_label.setStyleSheet("""
            color: #4FC3F7;
            font-family: 'Courier New';
            font-size: 18px;
            font-weight: bold;
            letter-spacing: 2px;
            background: transparent;
            border: none;
            margin-top: 4px;
            margin-bottom: 2px;
        """)
        main_vertical_layout.addWidget(controls_label)

        # Style definitions
        btn_style = """
            QPushButton {
                background-color: #1C2A3A;
                color: #00E5FF;
                border: 1px solid #00E5FF;
                border-radius: 4px;
                padding: 8px 18px;
                font-family: 'Courier New';
                font-size: 16px;
                font-weight: bold;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #00E5FF;
                color: #1C2A3A;
            }
            QPushButton:pressed {
                background-color: #00B2CC;
                border-color: #00B2CC;
            }
        """

        chk_style = """
            QCheckBox {
                color: #00E5FF;
                font-family: 'Courier New';
                font-size: 15px;
                font-weight: bold;
                background: transparent;
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border: 1px solid #00E5FF;
                border-radius: 3px;
                background: #111827;
            }
            QCheckBox::indicator:checked {
                background: #00E5FF;
                border-color: #00E5FF;
            }
        """

        self.btn_centre = QPushButton("Centre View")
        self.btn_centre.setStyleSheet(btn_style)
        self.btn_centre.clicked.connect(self.centre_view)

        self.btn_clear = QPushButton("Clear Trail")
        self.btn_clear.setStyleSheet(btn_style)
        self.btn_clear.clicked.connect(self.clear_trail)

        self.btn_zoom_in = QPushButton("Zoom In")
        self.btn_zoom_in.setStyleSheet(btn_style)
        self.btn_zoom_in.clicked.connect(self.zoom_in)

        self.btn_zoom_out = QPushButton("Zoom Out")
        self.btn_zoom_out.setStyleSheet(btn_style)
        self.btn_zoom_out.clicked.connect(self.zoom_out)

        # Record button (green inactive style by default)
        rec_style = """
            QPushButton {
                background-color: #1C2A3A;
                color: #00FF88;
                border: 1px solid #00FF88;
                border-radius: 4px;
                padding: 8px 18px;
                font-family: 'Courier New';
                font-size: 16px;
                font-weight: bold;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #00FF88;
                color: #1C2A3A;
            }
        """
        self.btn_record = QPushButton("● Record")
        self.btn_record.setCheckable(True)
        self.btn_record.setStyleSheet(rec_style)
        self.btn_record.clicked.connect(self.toggle_recording)

        # Export CSV button
        self.btn_export = QPushButton("Export CSV")
        self.btn_export.setStyleSheet(btn_style)
        self.btn_export.clicked.connect(self.export_csv)

        # Snapshot button
        self.btn_snapshot = QPushButton("Snapshot")
        self.btn_snapshot.setStyleSheet(btn_style)
        self.btn_snapshot.clicked.connect(self.take_snapshot)

        self.lock_follow_checkbox = QCheckBox("Lock Follow")
        self.lock_follow_checkbox.setStyleSheet(chk_style)

        self.show_prediction_checkbox = QCheckBox("Prediction")
        self.show_prediction_checkbox.setChecked(True)
        self.show_prediction_checkbox.setStyleSheet(chk_style)
        self.show_prediction_checkbox.stateChanged.connect(self.toggle_prediction)

        self.pause_checkbox = QCheckBox("Pause")
        self.pause_checkbox.setStyleSheet(chk_style)

        # Controls panel frame
        controls_frame = QFrame()
        controls_frame.setStyleSheet("""
            QFrame {
                background-color: #111827;
                border: 1px solid #1F2D45;
                border-radius: 6px;
                padding: 2px 6px;
            }
        """)
        controls_layout = QHBoxLayout(controls_frame)
        controls_layout.setContentsMargins(6, 4, 6, 4)
        controls_layout.setSpacing(6)

        # Group 1 (Navigation/View)
        controls_layout.addWidget(self.btn_centre)
        controls_layout.addWidget(self.btn_clear)
        controls_layout.addWidget(self.btn_zoom_in)
        controls_layout.addWidget(self.btn_zoom_out)

        # Separator 1
        sep1 = QFrame()
        sep1.setFixedWidth(2)
        sep1.setFixedHeight(34)
        sep1.setContentsMargins(4, 0, 4, 0)
        sep1.setStyleSheet("background: #1F2D45;")
        controls_layout.addWidget(sep1)

        # Group 2 (Recording/Actions)
        controls_layout.addWidget(self.btn_record)
        controls_layout.addWidget(self.btn_export)
        controls_layout.addWidget(self.btn_snapshot)

        # Separator 2
        sep2 = QFrame()
        sep2.setFixedWidth(2)
        sep2.setFixedHeight(34)
        sep2.setContentsMargins(4, 0, 4, 0)
        sep2.setStyleSheet("background: #1F2D45;")
        controls_layout.addWidget(sep2)

        # Group 3 (Checkboxes)
        controls_layout.addWidget(self.lock_follow_checkbox)
        controls_layout.addWidget(self.show_prediction_checkbox)
        controls_layout.addWidget(self.pause_checkbox)

        controls_layout.addStretch()

        main_vertical_layout.addWidget(controls_frame)

        # Body layout (plot/alt stack on left, sidebar on right)
        body_layout = QHBoxLayout()
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(10)

        # Left side container layout (vertical stack for 2D plot + Alt plot)
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(5)

        # 1. Main Trajectory PlotWidget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#0A0E1A')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.getAxis('bottom').setTickPen(pg.mkPen('#1C2333'))
        self.plot_widget.getAxis('left').setTickPen(pg.mkPen('#1C2333'))
        self.plot_widget.getAxis('bottom').setPen(pg.mkPen('#1F2D45'))
        self.plot_widget.getAxis('left').setPen(pg.mkPen('#1F2D45'))
        self.plot_widget.getAxis('bottom').setTextPen(pg.mkPen('#6B8CAE'))
        self.plot_widget.getAxis('left').setTextPen(pg.mkPen('#6B8CAE'))
        self.plot_widget.setLabel('bottom', "Downrange (m)")
        self.plot_widget.setLabel('left', "Altitude (m)")
        self.plot_widget.getPlotItem().layout.addItem(
            pg.LabelItem("ROCKET TRAJECTORY", color='#6B8CAE', size='9pt'), 0, 1)
        self.plot_widget.setXRange(-5, 140, padding=0.02)
        self.plot_widget.setYRange(0, 3800, padding=0.02)
        self.plot_widget.enableAutoRange(axis='x', enable=False)
        self.plot_widget.enableAutoRange(axis='y', enable=False)
        self._last_range_update = 0.0



        # Flat trail curve (used when gradient is disabled)
        self.trail_curve = pg.PlotCurveItem(pen=pg.mkPen('#00E5FF', width=2.5))
        self.plot_widget.addItem(self.trail_curve)

        # Phase-colored curves
        self.burn_trail = pg.PlotCurveItem(pen=pg.mkPen('#FF4400', width=2.5))
        self.coast_trail = pg.PlotCurveItem(pen=pg.mkPen('#FEB019', width=2.5))
        self.descent_trail = pg.PlotCurveItem(pen=pg.mkPen('#00FF88', width=2.5))
        self.plot_widget.addItem(self.burn_trail)
        self.plot_widget.addItem(self.coast_trail)
        self.plot_widget.addItem(self.descent_trail)

        # Predictive ghost trail (5-second dashed curve, magenta)
        ghost_color = QColor('#FF00FF')
        ghost_color.setAlpha(120)
        self.ghost_trail = pg.PlotCurveItem(pen=pg.mkPen(ghost_color, width=1, style=Qt.DashLine))
        self.plot_widget.addItem(self.ghost_trail)

        # Predicted endpoint marker
        self.ghost_endpoint = pg.ScatterPlotItem(size=8, brush=pg.mkBrush(ghost_color), pen=pg.mkPen(None), symbol='o')
        self.plot_widget.addItem(self.ghost_endpoint)

        # Custom QGraphicsPolygonItem for the rocket marker (taller, narrower rocket silhouette pointing upward)
        poly = QPolygonF([
            QPointF(0, -16),    # nose tip
            QPointF(5, -4),     # right shoulder
            QPointF(4, 8),      # right body
            QPointF(7, 14),     # right fin tip
            QPointF(2, 8),      # right fin root
            QPointF(-2, 8),     # left fin root
            QPointF(-7, 14),    # left fin tip
            QPointF(-4, 8),     # left body
            QPointF(-5, -4),    # left shoulder
        ])
        self.pos_marker = QGraphicsPolygonItem(poly)
        self.pos_marker.setFlag(QGraphicsPolygonItem.ItemIgnoresTransformations, True)
        self.pos_marker.setBrush(QBrush(QColor('#FF6600')))
        self.pos_marker.setPen(QPen(QColor('#FFFFFF'), 1))
        self.plot_widget.addItem(self.pos_marker)

        # Launch point marker (green dot at origin)
        self.launch_marker = pg.ScatterPlotItem(
            x=[0], y=[0], size=14, symbol='t1',
            brush=pg.mkBrush('#00FF88'), pen=pg.mkPen('#FFFFFF', width=1)
        )
        self.plot_widget.addItem(self.launch_marker)
        launch_label = pg.TextItem("LAUNCH", color='#00FF88', anchor=(0, 1))
        launch_label.setFont(QFont('Courier New', 9, QFont.Bold))
        launch_label.setPos(2, 50)
        self.plot_widget.addItem(launch_label)

        # Apogee marker (cyan star — updates position when phase becomes APOGEE or DESCENT)
        self.apogee_marker = pg.ScatterPlotItem(
            x=[], y=[], size=16, symbol='star',
            brush=pg.mkBrush('#00E5FF'), pen=pg.mkPen('#FFFFFF', width=1)
        )
        self.apogee_marker.setVisible(False)
        self.plot_widget.addItem(self.apogee_marker)

        self.apogee_label = pg.TextItem("", color='#00E5FF', anchor=(0, 1))
        self.apogee_label.setFont(QFont('Courier New', 9, QFont.Bold))
        self.apogee_label.setVisible(False)
        self.plot_widget.addItem(self.apogee_label)

        self.apogee_recorded = False

        # Phase transition lines and labels
        self.burnout_line = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('#FF4400', width=1, style=Qt.DashLine))
        self.plot_widget.addItem(self.burnout_line)
        self.burnout_label = pg.TextItem("BURNOUT", color='#FF4400', anchor=(0.5, 1.5))
        self.burnout_label.setFont(QFont('Courier New', 8, QFont.Bold))
        self.plot_widget.addItem(self.burnout_label)
        self.burnout_recorded = False

        self.burnout_line.setValue(-9999)
        self.burnout_label.setPos(-9999, 0)

        # Add vertical crosshair line to main plot
        self.v_line_main = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('#FF8C00', width=1, style=Qt.DashLine))
        self.plot_widget.addItem(self.v_line_main)

        left_layout.addWidget(self.plot_widget, stretch=1)

        # Altitude Profile Section Header label
        self.alt_header = QLabel("ALTITUDE PROFILE")
        self.alt_header.setStyleSheet("""
            color: #4FC3F7;
            font-family: 'Courier New';
            font-size: 11px;
            font-weight: bold;
            letter-spacing: 2px;
            background: transparent;
            border: none;
            margin-top: 8px;
            margin-bottom: 2px;
        """)
        left_layout.addWidget(self.alt_header)

        # 2. Altitude PlotWidget (120px fixed height)
        self.alt_plot = pg.PlotWidget()
        self.alt_plot.setYRange(0, 500, padding=0)
        self.alt_plot.getAxis('left').setRange(0, 500)
        self.alt_plot.getAxis('left').setWidth(55)
        self.alt_plot.setBackground('#0A0E1A')
        self.alt_plot.setFixedHeight(160)
        self.alt_plot.showGrid(x=True, y=True)
        self.alt_plot.getAxis('bottom').setTickPen(pg.mkPen('#1C2333'))
        self.alt_plot.getAxis('left').setTickPen(pg.mkPen('#1C2333'))
        self.alt_plot.getAxis('bottom').setPen(pg.mkPen('#1F2D45'))
        self.alt_plot.getAxis('left').setPen(pg.mkPen('#1F2D45'))
        self.alt_plot.getAxis('bottom').setTextPen(pg.mkPen('#6B8CAE'))
        self.alt_plot.getAxis('left').setTextPen(pg.mkPen('#6B8CAE'))
        self.alt_plot.getAxis('left').setStyle(tickTextWidth=50)
        self.alt_plot.setLabel('bottom', "Time (s)")
        self.alt_plot.setLabel('left', "Alt (m)", color='#6B8CAE')

        # Altitude curve (orange line with FillBetween fill)
        self.alt_curve = pg.PlotCurveItem(pen=pg.mkPen('#FF8C00', width=2))
        self.alt_plot.addItem(self.alt_curve)
        self.alt_baseline = pg.PlotCurveItem([0],[0])
        self.alt_fill = pg.FillBetweenItem(
            self.alt_curve,
            self.alt_baseline,
            brush=pg.mkBrush(255, 140, 0, 35)
        )
        self.alt_plot.addItem(self.alt_fill)

        # Add vertical crosshair line to altitude plot
        self.v_line_alt = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('#FF8C00', width=1, style=Qt.DashLine))
        self.alt_plot.addItem(self.v_line_alt)

        left_layout.addWidget(self.alt_plot)

        body_layout.addLayout(left_layout, 70)

        # Right side: sidebar layout
        self.sidebar_layout = QVBoxLayout()
        self.sidebar_layout.setSpacing(12)
        self.sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_widget = QWidget()
        sidebar_widget.setLayout(self.sidebar_layout)
        sidebar_widget.setFixedWidth(360)
        body_layout.addWidget(sidebar_widget)



        self.phase_label = QLabel("● BURN")
        self.phase_label.setAlignment(Qt.AlignCenter)
        self.phase_label.setStyleSheet("""
            color: #FF4400;
            font-family: 'Courier New';
            font-size: 20px;
            font-weight: bold;
            background-color: rgba(255, 68, 0, 15);
            border: 1px solid #FF4400;
            border-radius: 4px;
            padding: 12px;
            margin-bottom: 8px;
        """)
        self.sidebar_layout.addWidget(self.phase_label)

        # Breach Flashing Indicator (hidden by default)
        self.breach_label = QLabel("⚠ GEO-FENCE BREACH")
        self.breach_label.setAlignment(Qt.AlignCenter)
        self.breach_label.setStyleSheet("""
            color: #FF2244;
            font-family: 'Courier New';
            font-size: 18px;
            font-weight: bold;
            background-color: rgba(255, 34, 68, 15);
            border: 1px solid #FF2244;
            border-radius: 4px;
            padding: 12px;
            margin-bottom: 6px;
            letter-spacing: 1px;
        """)
        self.breach_label.setVisible(False)
        self.sidebar_layout.addWidget(self.breach_label)

        # Anomaly Flashing Indicator (hidden by default)
        self.anomaly_label = QLabel("⚠ ANOMALY DETECTED")
        self.anomaly_label.setAlignment(Qt.AlignCenter)
        self.anomaly_label.setStyleSheet("""
            color: #FF8C00;
            font-family: 'Courier New';
            font-size: 18px;
            font-weight: bold;
            background-color: rgba(255, 140, 0, 15);
            border: 1px solid #FF8C00;
            border-radius: 4px;
            padding: 12px;
            margin-bottom: 6px;
            letter-spacing: 1px;
        """)
        self.anomaly_label.setVisible(False)
        self.sidebar_layout.addWidget(self.anomaly_label)

        self.anomaly_flash_timer = QTimer(self)
        self.anomaly_flash_timer.setInterval(300)
        self.anomaly_flash_timer.timeout.connect(self.flash_anomaly)
        self.anomaly_visible = False

        self.last_speed = None
        self.last_alt = None

        # Gradient trail toggle checkbox in the sidebar
        self.gradient_checkbox = QCheckBox("Gradient trail")
        self.gradient_checkbox.setChecked(True)
        self.gradient_checkbox.setStyleSheet("""
            QCheckBox {
                color: #6B8CAE;
                font-family: 'Courier New';
                font-size: 15px;
                font-weight: bold;
                background: transparent;
                margin-bottom: 0px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border: 1px solid #1F2D45;
                border-radius: 3px;
                background: #111827;
            }
            QCheckBox::indicator:checked {
                background: #00E5FF;
                border-color: #00E5FF;
            }
        """)
        trail_options_frame = QFrame()
        trail_options_frame.setStyleSheet("""
            QFrame {
                background: #111827;
                border: 1px solid #1F2D45;
                border-radius: 6px;
                padding: 4px;
            }
        """)
        trail_frame_layout = QVBoxLayout(trail_options_frame)
        trail_frame_layout.setContentsMargins(8, 6, 8, 6)
        trail_label = QLabel("TRAIL OPTIONS")
        trail_label.setStyleSheet("color: #00FFC8; font-family: 'Courier New'; font-size: 14px; font-weight: bold; letter-spacing: 2px; border: none; background: transparent;")
        trail_frame_layout.addWidget(trail_label)
        trail_frame_layout.addWidget(self.gradient_checkbox)
        self.sidebar_layout.addWidget(trail_options_frame)

        # Group box styling matching dark theme
        group_style = """
            QGroupBox {
                background-color: #111827;
                border: 1px solid #1F2D45;
                border-radius: 6px;
                margin-top: 26px;
                font-family: 'Courier New';
                font-size: 17px;
                font-weight: bold;
                color: #00FFC8;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
            }
        """

        # 1. "Live Position" group
        self.pos_group = QGroupBox("Live Position")
        self.pos_group.setStyleSheet(group_style)
        pos_layout = QVBoxLayout(self.pos_group)
        pos_layout.setContentsMargins(10, 15, 10, 10)
        
        row_x_widget, self.lbl_x = self.create_telemetry_row("X (m)")
        row_y_widget, self.lbl_y = self.create_telemetry_row("Y (m)")
        row_alt_widget, self.lbl_alt = self.create_telemetry_row("Altitude (m)")
        
        pos_layout.addWidget(row_x_widget)
        pos_layout.addWidget(row_y_widget)
        pos_layout.addWidget(row_alt_widget)
        self.sidebar_layout.addWidget(self.pos_group, 1)

        # 2. "Flight Dynamics" group
        self.dyn_group = QGroupBox("Flight Dynamics")
        self.dyn_group.setStyleSheet(group_style)
        dyn_layout = QVBoxLayout(self.dyn_group)
        dyn_layout.setContentsMargins(10, 15, 10, 10)
        
        row_speed_widget, self.lbl_speed = self.create_telemetry_row("Speed (m/s)")
        row_accel_widget, self.lbl_heading = self.create_telemetry_row("Accel Z (m/s²)")
        row_vspeed_widget, self.lbl_vspeed = self.create_telemetry_row("V. Speed (m/s)")
        
        dyn_layout.addWidget(row_speed_widget)
        dyn_layout.addWidget(row_accel_widget)
        dyn_layout.addWidget(row_vspeed_widget)
        self.sidebar_layout.addWidget(self.dyn_group, 1)

        # 3. "Trail Stats" group
        self.stats_group = QGroupBox("Trail Stats")
        self.stats_group.setStyleSheet(group_style)
        stats_layout = QVBoxLayout(self.stats_group)
        stats_layout.setContentsMargins(10, 15, 10, 10)
        
        row_dist_widget, self.lbl_dist = self.create_telemetry_row("Max Altitude (m)")
        row_max_speed_widget, self.lbl_max_speed = self.create_telemetry_row("Max Speed")
        row_time_widget, self.lbl_time = self.create_telemetry_row("Flight Time (s)")
        
        stats_layout.addWidget(row_dist_widget)
        stats_layout.addWidget(row_max_speed_widget)
        stats_layout.addWidget(row_time_widget)
        self.sidebar_layout.addWidget(self.stats_group, 1)

        # Phase Legend Section
        legend_group = QGroupBox("Phase Legend")
        legend_group.setStyleSheet(group_style)
        legend_layout = QVBoxLayout(legend_group)
        legend_layout.setContentsMargins(10, 18, 10, 10)
        legend_layout.setSpacing(10)

        phases_info = [
            ("#FF4400", "Burn     (0 – 10s)"),
            ("#FEB019", "Coast    (10 – 25s)"),
            ("#00E5FF", "Apogee   (25 – 27s)"),
            ("#00FF88", "Descent  (27s+)"),
        ]
        for color, label in phases_info:
            row = QWidget()
            row_lay = QHBoxLayout(row)
            row_lay.setContentsMargins(0, 0, 0, 0)
            row_lay.setSpacing(10)
            dot = QLabel("●")
            dot.setStyleSheet(f"color: {color}; font-size: 16px; background: transparent; border: none;")
            dot.setFixedWidth(20)
            txt = QLabel(label)
            txt.setStyleSheet("color: #8892A4; font-family: 'Courier New'; font-size: 14px; background: transparent; border: none;")
            row_lay.addWidget(dot)
            row_lay.addWidget(txt)
            legend_layout.addWidget(row)

        self.sidebar_layout.addWidget(legend_group)

        main_vertical_layout.addLayout(body_layout)

        # Signal proxies for linked crosshair lines on mouse hover
        self.proxy_main = pg.SignalProxy(
            self.plot_widget.scene().sigMouseMoved,
            rateLimit=60,
            slot=self.mouse_moved_main
        )
        self.proxy_alt = pg.SignalProxy(
            self.alt_plot.scene().sigMouseMoved,
            rateLimit=60,
            slot=self.mouse_moved_alt
        )

    def toggle_prediction(self, state):
        visible = self.show_prediction_checkbox.isChecked()
        self.ghost_trail.setVisible(visible)
        self.ghost_endpoint.setVisible(visible)

    def animate_marker(self):
        if self.prev_position is None or self.curr_position is None:
            return
        
        elapsed = time.time() - self.anim_start_time
        t = elapsed / 0.040
        if t > 1.0:
            t = 1.0
        elif t < 0.0:
            t = 0.0
            
        px, py, p_rot = self.prev_position
        cx, cy, c_rot = self.curr_position
        
        # Linear interpolation for x and y
        x = px + t * (cx - px)
        y = py + t * (cy - py)
        
        # Shortest path angle interpolation for rocket rotation (pointing up 0° or down 180°)
        diff = (c_rot - p_rot) % 360.0
        if diff > 180.0:
            diff -= 360.0
        rot = (p_rot + t * diff) % 360.0
        
        self.pos_marker.setPos(x, y)
        self.pos_marker.setRotation(rot)

    def trigger_anomaly(self):
        if self.anomaly_label.isHidden():
            self.anomaly_label.setVisible(True)
            self.anomaly_visible = True
            self.anomaly_flash_timer.start()
            QTimer.singleShot(3000, self.hide_anomaly)

    def flash_anomaly(self):
        self.anomaly_visible = not self.anomaly_visible
        if self.anomaly_visible:
            self.anomaly_label.setStyleSheet("""
                color: #FF8C00;
                font-family: 'Courier New';
                font-size: 18px;
                font-weight: bold;
                background-color: rgba(255, 140, 0, 15);
                border: 1px solid #FF8C00;
                border-radius: 4px;
                padding: 12px;
                margin-bottom: 6px;
                letter-spacing: 1px;
            """)
        else:
            self.anomaly_label.setStyleSheet("""
                color: transparent;
                font-family: 'Courier New';
                font-size: 18px;
                font-weight: bold;
                background-color: transparent;
                border: 1px solid transparent;
                border-radius: 4px;
                padding: 12px;
                margin-bottom: 6px;
                letter-spacing: 1px;
            """)

    def hide_anomaly(self):
        self.anomaly_flash_timer.stop()
        self.anomaly_label.setVisible(False)

    def toggle_recording(self):
        if self.btn_record.isChecked():
            self.btn_record.setText("■ Stop Rec")
            self.flight_log.clear()
            
            self.rec_blink_timer = QTimer(self)
            self.rec_blink_timer.setInterval(600)
            self._rec_blink_state = True
            def blink():
                self._rec_blink_state = not self._rec_blink_state
                col = "#FF2244" if self._rec_blink_state else "#881122"
                self.btn_record.setStyleSheet(f"""
                    QPushButton {{
                        background-color: #2D0D14;
                        color: {col};
                        border: 2px solid {col};
                        border-radius: 4px;
                        padding: 8px 18px;
                        font-family: 'Courier New';
                        font-size: 16px;
                        font-weight: bold;
                        min-height: 40px;
                    }}
                """)
            self.rec_blink_timer.timeout.connect(blink)
            self.rec_blink_timer.start()
            blink()
        else:
            if hasattr(self, 'rec_blink_timer'):
                self.rec_blink_timer.stop()
            self.btn_record.setText("● Record")
            self.btn_record.setStyleSheet("""
                QPushButton {
                    background-color: #1C2A3A;
                    color: #00FF88;
                    border: 1px solid #00FF88;
                    border-radius: 4px;
                    padding: 8px 18px;
                    font-family: 'Courier New';
                    font-size: 16px;
                    font-weight: bold;
                    min-height: 40px;
                }
                QPushButton:hover {
                    background-color: #00FF88;
                    color: #1C2A3A;
                }
            """)

    def export_csv(self):
        if not self.flight_log:
            QMessageBox.warning(self, "Export CSV", "No telemetry data recorded to export.")
            return

        options = QFileDialog.Options()
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export CSV File", "", "CSV Files (*.csv)", options=options
        )
        if filepath:
            try:
                with open(filepath, mode='w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=["t", "x", "y", "alt", "heading", "speed"])
                    writer.writeheader()
                    for row in self.flight_log:
                        writer.writerow({
                            "t": row["t"],
                            "x": row["x"],
                            "y": row["y"],
                            "alt": row["alt"],
                            "heading": row["heading"],
                            "speed": row["speed"]
                        })
                QMessageBox.information(self, "Export Success", f"Saved to {filepath}")
            except Exception as e:
                QMessageBox.critical(self, "Export Failed", f"Could not write CSV file:\n{str(e)}")

    def take_snapshot(self):
        options = QFileDialog.Options()
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Snapshot", "", "PNG Files (*.png)", options=options
        )
        if filepath:
            try:
                pixmap = self.plot_widget.grab()
                pixmap.save(filepath)
                QMessageBox.information(self, "Snapshot Saved", f"Saved to {filepath}")
            except Exception as e:
                QMessageBox.critical(self, "Snapshot Failed", f"Could not save snapshot image:\n{str(e)}")

    def centre_view(self):
        self.plot_widget.setXRange(-5, 140, padding=0.02)
        self.plot_widget.setYRange(0, 3800, padding=0.02)
        self.alt_plot.autoRange()

    def clear_trail(self):
        self.simulator.position_history.clear()
        # Reset simulator physics so rocket relaunches from scratch
        self.simulator._vz       = 0.0
        self.simulator._alt      = 0.0
        self.simulator._landed   = False
        self.simulator._apogee_alt = 0.0
        self.simulator._downrange  = 0.0
        self.simulator.t           = 0.0
        self.simulator.x           = 0.0
        self.simulator.y           = 0.0
        self.simulator._vel        = 0.0

        self.max_speed = 0.0
        self._last_range_update = 0.0
        self._last_draw_count = 0
        self.prev_position = None
        self.curr_position = None
        self.last_speed = None
        self.last_alt = None

        self.breach_label.setVisible(False)
        self.anomaly_label.setVisible(False)
        self.anomaly_flash_timer.stop()
        self.phase_label.setText("● BURN")
        self.phase_label.setStyleSheet("""
            color: #FF4400;
            font-family: 'Courier New';
            font-size: 20px;
            font-weight: bold;
            background-color: rgba(255, 68, 0, 15);
            border: 1px solid #FF4400;
            border-radius: 4px;
            padding: 12px;
            margin-bottom: 8px;
        """)
        self.apogee_recorded = False
        self.apogee_marker.setVisible(False)
        self.apogee_label.setVisible(False)
        self.trail_curve.setData([], [])
        self.burn_trail.setData([], [])
        self.coast_trail.setData([], [])
        self.descent_trail.setData([], [])
        self.burnout_recorded = False
        self.burnout_line.setValue(-9999)
        self.burnout_label.setPos(-9999, 0)
        self.alt_curve.setData([], [])
        self.alt_baseline.setData([], [])
        self.alt_plot.setYRange(0, 500, padding=0)
        self.alt_plot.setXRange(0, 10, padding=0)
        self.ghost_trail.setData([], [])
        self.ghost_endpoint.setData([], [])
        self.v_line_main.setValue(0.0)
        self.v_line_alt.setValue(0.0)
        self.lbl_x.setText("0.00")
        self.lbl_y.setText("0.00")
        self.lbl_alt.setText("0.00")
        self.lbl_speed.setText("0.00")
        self.lbl_heading.setText("0.00")
        self.lbl_vspeed.setText("0.00")
        self.lbl_dist.setText("0.0")
        self.lbl_max_speed.setText("0.00")
        self.lbl_time.setText("0.0")

    def zoom_in(self):
        self.plot_widget.getViewBox().scaleBy(s=(0.7, 0.7))

    def zoom_out(self):
        self.plot_widget.getViewBox().scaleBy(s=(1.3, 1.3))

    def mouse_moved_main(self, evt):
        pos = evt[0]
        if self.plot_widget.sceneBoundingRect().contains(pos):
            mouse_point = self.plot_widget.getViewBox().mapSceneToView(pos)
            x_east = mouse_point.x()
            history = self.simulator.position_history
            if len(history) > 0:
                # Find closest history record based on East coordinate
                closest_pos = min(history, key=lambda pos_item: abs(pos_item["x"] - x_east))
                self.v_line_main.setValue(closest_pos["x"])
                self.v_line_alt.setValue(closest_pos["t"])

    def mouse_moved_alt(self, evt):
        pos = evt[0]
        if self.alt_plot.sceneBoundingRect().contains(pos):
            mouse_point = self.alt_plot.getViewBox().mapSceneToView(pos)
            t_sec = mouse_point.x()
            history = self.simulator.position_history
            if len(history) > 0:
                # Find closest history record based on Time coordinate
                closest_pos = min(history, key=lambda pos_item: abs(pos_item["t"] - t_sec))
                self.v_line_main.setValue(closest_pos["x"])
                self.v_line_alt.setValue(closest_pos["t"])

    @pyqtSlot()
    def update(self):
        if self.pause_checkbox.isChecked():
            return

        self.last_position = self.simulator.get_position()
        
        xs = [pos["x"] for pos in self.simulator.position_history]
        ys = [pos["y"] for pos in self.simulator.position_history]
        ts = [pos["t"] for pos in self.simulator.position_history]
        alts = [pos["alt"] for pos in self.simulator.position_history]

        history_len = len(self.simulator.position_history)
        if self.gradient_checkbox.isChecked():
            # Clear/hide flat curve
            self.trail_curve.setData([], [])

            # Phase-colored curves
            burn_xs, burn_ys = [], []
            coast_xs, coast_ys = [], []
            descent_xs, descent_ys = [], []
            
            for p in self.simulator.position_history:
                phase_name = p["phase"]
                px, py = p["x"], p["alt"]
                if phase_name == "BURN":
                    burn_xs.append(px)
                    burn_ys.append(py)
                elif phase_name == "COAST":
                    if len(burn_xs) > 0 and len(coast_xs) == 0:
                        burn_xs.append(px)
                        burn_ys.append(py)
                    coast_xs.append(px)
                    coast_ys.append(py)
                else: # APOGEE or DESCENT
                    if len(coast_xs) > 0 and len(descent_xs) == 0:
                        coast_xs.append(px)
                        coast_ys.append(py)
                    descent_xs.append(px)
                    descent_ys.append(py)

            self.burn_trail.setData(burn_xs, burn_ys)
            self.coast_trail.setData(coast_xs, coast_ys)
            self.descent_trail.setData(descent_xs, descent_ys)
        else:
            # Flat cyan mode: draw full curve, clear phase segments
            self.trail_curve.setData(xs, ys)
            self.burn_trail.setData([], [])
            self.coast_trail.setData([], [])
            self.descent_trail.setData([], [])

        # Update secondary altitude profile plot
        self.alt_curve.setData(ts, alts)
        if ts:
            self.alt_baseline.setData(ts, [0.0] * len(ts))
        if alts:
            self.alt_plot.setYRange(0, max(max(alts) * 1.1, 100), padding=0)
            self.alt_plot.getAxis('left').setTickSpacing(
                levels=[(1000, 0), (500, 0)]
            )
        if ts:
            self.alt_plot.setXRange(min(ts), max(ts) + 2, padding=0)

        if self.last_position:
            x = self.last_position["x"]
            y = self.last_position["y"]
            heading = self.last_position["heading"]
            speed = self.last_position["speed"]
            alt = self.last_position["alt"]
            t = self.last_position["t"]
            
            # Phase display logic
            phase = self.last_position.get("phase", "BURN")
            phase_colors = {
                "BURN":    ("#FF4400", "rgba(255,68,0,15)"),
                "COAST":   ("#FEB019", "rgba(254,176,25,15)"),
                "APOGEE":  ("#00E5FF", "rgba(0,229,255,15)"),
                "DESCENT": ("#00FF88", "rgba(0,255,136,15)"),
                "LANDED":  ("#FFFFFF", "rgba(255,255,255,10)")
            }
            if phase == "LANDED":
                col, bg = phase_colors["LANDED"]
                self.phase_label.setText("● LANDED")
                self.phase_label.setStyleSheet(f"""
                    color: {col};
                    font-family: 'Courier New';
                    font-size: 20px;
                    font-weight: bold;
                    background-color: {bg};
                    border: 1px solid {col};
                    border-radius: 4px;
                    padding: 12px;
                    margin-bottom: 8px;
                """)
                self.lbl_speed.setText("0.00")
                self.lbl_vspeed.setText("0.00")
                self.lbl_alt.setText("0.00")
                self.prev_position = (x, 0.0, 180.0)
                self.curr_position = (x, 0.0, 180.0)
                self.anim_start_time = time.time()
                return  # freeze display

            col, bg = phase_colors.get(phase, ("#FFFFFF", "transparent"))
            self.phase_label.setText(f"● {phase}")
            self.phase_label.setStyleSheet(f"""
                color: {col};
                font-family: 'Courier New';
                font-size: 20px;
                font-weight: bold;
                background-color: {bg};
                border: 1px solid {col};
                border-radius: 4px;
                padding: 12px;
                margin-bottom: 8px;
            """)
            
            # Dynamic phase line recording
            if phase == "COAST" and not self.burnout_recorded:
                self.burnout_recorded = True
                self.burnout_line.setValue(x)
                self.burnout_label.setPos(x, 150)
                


            # Track and display apogee
            if not self.apogee_recorded and phase in ("DESCENT", "LANDED"):
                self.apogee_recorded = True
                apogee_alt = self.last_position.get("apogee", 3450.0)
                apogee_x   = self.last_position["x"]
                self.apogee_marker.setData(x=[apogee_x], y=[apogee_alt])
                self.apogee_marker.setVisible(True)
                self.apogee_label.setPos(apogee_x + 2, apogee_alt + 80)
                self.apogee_label.setText(f"APOGEE {apogee_alt:.0f}m")
                self.apogee_label.setVisible(True)
            
            # Anomaly detection: speed jump > 5 m/s or alt jump > 10m
            if self.last_speed is not None and self.last_alt is not None:
                current_phase = self.last_position.get("phase", "BURN")
                prev_phase = getattr(self, "_last_phase", current_phase)
                self._last_phase = current_phase
                
                phase_just_changed = (current_phase != prev_phase)
                
                if not phase_just_changed:
                    if abs(speed - self.last_speed) > 5.0 or abs(alt - self.last_alt) > 10.0:
                        self.trigger_anomaly()
            self.last_speed = speed
            self.last_alt = alt

            # Set target positions for animation interpolation
            v_speed = self.last_position.get("vertical_speed", 0.0)
            target_rot = 0.0 if v_speed >= 0.0 else 180.0
            
            if self.curr_position is not None:
                self.prev_position = self.curr_position
            else:
                self.prev_position = (x, y, target_rot)
            self.curr_position = (x, y, target_rot)
            self.anim_start_time = time.time()

            # Record position if recording is active
            if self.btn_record.isChecked():
                self.flight_log.append(self.last_position)



            # Extrapolate 5-second predictive ghost trail (10 segments spaced 0.5s apart)
            if self.show_prediction_checkbox.isChecked():
                # True signed vertical speed from the simulator (noise-free)
                vy = self.last_position.get("vertical_speed", 0.0)
                
                # True horizontal speed from simulator's downrange derivative
                vx = 0.25 * math.cos(0.05 * t) + 0.5
                
                # Determine vertical acceleration based on phase
                phase = self.last_position.get("phase", "BURN")
                accel_z = self.last_position.get("accel_z", 0.0)
                if phase == "BURN":
                    ay = accel_z
                elif phase == "DESCENT":
                    ay = -15.0
                else:
                    ay = -9.81
                
                pred_xs = []
                pred_ys = []
                for i in range(11):  # 11 points total including current position (t=0)
                    t_future = i * 0.5
                    pred_x = x + vx * t_future
                    pred_y = y + vy * t_future + 0.5 * ay * (t_future ** 2)
                    if pred_y < 0:
                        pred_y = 0.0
                    pred_xs.append(pred_x)
                    pred_ys.append(pred_y)
                
                self.ghost_trail.setData(pred_xs, pred_ys)
                self.ghost_endpoint.setData([pred_xs[-1]], [pred_ys[-1]])
            else:
                self.ghost_trail.setData([], [])
                self.ghost_endpoint.setData([], [])



            # Update Max Speed
            if self.last_position.get("t", 0) < 0.1:
                self.max_speed = 0.0
                self._last_range_update = 0.0



            history = self.simulator.position_history

            # Reset max speed if trail is cleared
            if len(history) == 0:
                self.max_speed = 0.0

            # Vertical Speed (using noise-free value from simulator for smooth display)
            v_speed = self.last_position.get("vertical_speed", 0.0)

            # Total Distance
            total_dist = 0.0
            for i in range(1, len(history)):
                dx = history[i]["x"] - history[i-1]["x"]
                dy = history[i]["y"] - history[i-1]["y"]
                total_dist += math.sqrt(dx*dx + dy*dy)

            # Flight Time
            if len(history) >= 1:
                flight_time = history[-1]["t"] - history[0]["t"]
            else:
                flight_time = 0.0

            # Auto-centre if Lock Follow is active (using current view dimensions to preserve zoom/aspect ratio)
            if self.lock_follow_checkbox.isChecked():
                view_box = self.plot_widget.getViewBox()
                x_range, y_range = view_box.viewRange()
                x_width = x_range[1] - x_range[0]
                y_height = y_range[1] - y_range[0]
                view_box.setRange(
                    xRange=(x - x_width / 2.0, x + x_width / 2.0),
                    yRange=(y - y_height / 2.0, y + y_height / 2.0),
                    padding=0
                )
            else:
                # Smart range auto-scaling (updates every 2 seconds to avoid jumping)
                current_t = self.last_position.get("t", 0)
                if current_t - self._last_range_update > 2.0:
                    self._last_range_update = current_t
                    history = list(self.simulator.position_history)
                    if len(history) > 1:
                        xs = [p["x"] for p in history]
                        ys = [p["y"] for p in history]
                        x_pad = max(20, (max(xs) - min(xs)) * 0.15)
                        y_pad = max(200, max(ys) * 0.1)
                        self.plot_widget.setXRange(min(xs) - x_pad, max(xs) + x_pad, padding=0)
                        self.plot_widget.setYRange(0, max(ys) + y_pad, padding=0)

            # Update UI labels
            self.lbl_x.setText(f"{x:+.2f}")
            self.lbl_y.setText(f"{y:+.2f}")
            self.lbl_alt.setText(f"{alt:.2f}")

            phase = self.last_position.get("phase", "BURN")
            v_speed_abs = abs(self.last_position.get("vertical_speed", 0.0))
            
            if phase == "BURN":
                if v_speed_abs > self.max_speed:
                    self.max_speed = v_speed_abs
            
            self.lbl_max_speed.setText(f"{self.max_speed:.2f}")

            self.lbl_heading.setText(f"{self.last_position.get('accel_z', 0):.2f}")

            if phase == "LANDED":
                self.lbl_speed.setText("0.00")
                self.lbl_vspeed.setText("0.00")
                self.lbl_alt.setText("0.00")
            else:
                self.lbl_speed.setText(f"{speed:.2f}")
                v_speed_val = self.last_position.get("vertical_speed", 0.0)
                self.lbl_vspeed.setText(f"{v_speed_val:+.2f}")

            self.lbl_dist.setText(f"{max(p['alt'] for p in self.simulator.position_history):.1f}")
            self.lbl_time.setText(f"{flight_time:.1f}")
        else:
            # Clear predictions if no position
            self.ghost_trail.setData([], [])
            self.ghost_endpoint.setData([], [])
