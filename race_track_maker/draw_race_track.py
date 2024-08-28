import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, QLineEdit, QLabel
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout
from PyQt5.QtWidgets import QCheckBox, QGraphicsScene, QGraphicsPathItem, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtGui import QPainterPath, QPen, QPainter, QColor
from PyQt5.QtCore import Qt, QPointF, QRectF

from race_track_maker import TrackGenerator
from scipy.ndimage import gaussian_filter1d
import numpy as np
import matplotlib.pyplot as plt

def gaussian_smooth(x, y, sigma):
    smoothed_x = gaussian_filter1d(x, sigma=sigma)
    smoothed_y = gaussian_filter1d(y, sigma=sigma)
    return smoothed_x, smoothed_y

class GridGraphicsView(QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.grid_step = 40  # Distance between grid lines
        self.bold_pen = QPen(QColor(255, 182, 193), 2, Qt.SolidLine)  # Light red color
        self.normal_pen = QPen(Qt.lightGray, 0.5, Qt.DotLine)  # Pen for normal grid lines
        self.text_pen = QPen(Qt.black)  # Pen for text labels
        self.text_offset_x = 20  # Offset for x-axis text positioning
        self.text_offset_y = 15  # Offset for y-axis text positioning

    def drawBackground(self, painter: QPainter, rect: QRectF):
        super().drawBackground(painter, rect)
        
        # Get scene rectangle bounds
        left = int(rect.left())
        right = int(rect.right())
        top = int(rect.top())
        bottom = int(rect.bottom())
        
        # Draw normal grid lines
        painter.setPen(self.normal_pen)
        x = left - (left % self.grid_step)
        while x < right:
            painter.drawLine(x, top, x, bottom)
            x += self.grid_step
        
        y = top - (top % self.grid_step)
        while y < bottom:
            painter.drawLine(left, y, right, y)
            y += self.grid_step

        # Draw bold center lines
        painter.setPen(self.bold_pen)
        # Vertical center line
        painter.drawLine(0, top, 0, bottom)
        # Horizontal center line
        painter.drawLine(left, 0, right, 0)

        # Draw numbers along the x-axis (bottom edge)
        painter.setPen(self.text_pen)
        x = left - (left % self.grid_step)
        while x < right:
            # if x != 0:  # Avoid drawing 0 at the center
            grid_x = x // self.grid_step
            painter.drawText(x + self.text_offset_x, bottom - self.text_offset_y, f"{grid_x}")
            x += self.grid_step

        # Draw numbers along the y-axis (right edge)
        y = top - (top % self.grid_step)
        while y < bottom:
            # if y != 0:  # Avoid drawing 0 at the center
            grid_y = -y // self.grid_step  # Invert y-axis for correct coordinates
            painter.drawText(-right , y - self.text_offset_y, f"{grid_y}")
            y += self.grid_step

class SplineDrawer(QMainWindow):
    def __init__(self):
        super().__init__()
        # Create an instance of TrackGenerator
        self.scaler = 40
        self.track_width = 4
        self.track_height = 1
        self.track_generator = TrackGenerator(track_width=self.track_width)
        self.file_name = "race_track"

        self.setWindowTitle("Race Track Maker")
        self.setGeometry(100, 100, 800, 600)

        # Set up the graphics view and scene
        self.view = GridGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setRenderHint(QPainter.SmoothPixmapTransform)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Center the view at (0,0)
        self.view.setSceneRect(-self.width()//2, -self.height()//2, self.width(), self.height())
        self.view.centerOn(0, 0)

        self.path = QPainterPath()
        self.pen = QPen(Qt.black, 2)
        self.path_item = QGraphicsPathItem()
        self.path_item.setPen(self.pen)
        self.scene.addItem(self.path_item)

        self.points = []

        # Create labels and text fields
        self.scaler_label = QLabel("Scaler:")
        self.scaler_input = QLineEdit()
        self.scaler_input.setText(str(self.scaler))
        
        self.track_width_label = QLabel("Track Width:")
        self.track_width_input = QLineEdit()
        self.track_width_input.setText(str(self.track_width))

        self.track_height_label = QLabel("Height Width:")
        self.track_height_input = QLineEdit()
        self.track_height_input.setText(str(self.track_height))

        self.file_name_label = QLabel("file name")
        self.file_name_input = QLineEdit()
        self.file_name_input.setText(str(self.file_name))

        # Create buttons
        self.clear_button = QPushButton("Clear Spline")
        self.print_button = QPushButton("Generate path")
        self.close_loop_checkbox = QCheckBox("Close Loop tracks")

        # Connect buttons
        self.clear_button.clicked.connect(self.clear_spline)
        self.print_button.clicked.connect(self.generate_path)


        # Create vertical layouts for labels and text fields
        self.scaler_layout = QVBoxLayout()
        self.scaler_layout.addWidget(self.scaler_label)
        self.scaler_layout.addWidget(self.scaler_input)

        self.track_width_layout = QVBoxLayout()
        self.track_width_layout.addWidget(self.track_width_label)
        self.track_width_layout.addWidget(self.track_width_input)

        self.track_height_layout = QVBoxLayout()
        self.track_height_layout.addWidget(self.track_height_label)
        self.track_height_layout.addWidget(self.track_height_input)


        self.file_name_layout = QVBoxLayout()
        self.file_name_layout.addWidget(self.file_name_label)
        self.file_name_layout.addWidget(self.file_name_input)

        # Create a horizontal layout to hold the vertical layouts
        self.input_layout = QHBoxLayout()
        self.input_layout.addLayout(self.scaler_layout)
        self.input_layout.addLayout(self.track_width_layout)
        self.input_layout.addLayout(self.track_height_layout)
        self.input_layout.addLayout(self.file_name_layout)

        # Create horizontal layout for buttons
        self.button_layout = QHBoxLayout()
        self.button_layout.addWidget(self.clear_button)
        self.button_layout.addWidget(self.print_button)
        self.button_layout.addWidget(self.close_loop_checkbox)
        

        # Set up layout
        layout = QVBoxLayout()
        layout.addWidget(self.view)
        layout.addLayout(self.input_layout)  # Add the input_layout here
        # layout.addWidget(self.close_loop_checkbox)
        layout.addLayout(self.button_layout)  # Add the button_layout here

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Connect mouse events
        self.view.mousePressEvent = self.mouse_press_event
        self.view.mouseMoveEvent = self.mouse_move_event
        self.view.mouseReleaseEvent = self.mouse_release_event

    def mouse_press_event(self, event):
        if event.button() == Qt.LeftButton:
            point = self.view.mapToScene(event.pos())
            self.points.append(point)
            if len(self.points) > 1:
                self.update_path()

    def mouse_move_event(self, event):
        if event.buttons() & Qt.LeftButton:
            point = self.view.mapToScene(event.pos())
            self.points.append(point)
            self.update_path()

    def mouse_release_event(self, event):
        if event.button() == Qt.LeftButton:
            self.update_path()

    def update_path(self):
        self.path = QPainterPath()
        if len(self.points) > 1:
            self.path.moveTo(self.points[0])
            for i in range(1, len(self.points) - 1):
                mid_point = (self.points[i] + self.points[i + 1]) / 2
                self.path.quadTo(self.points[i], mid_point)
            if len(self.points) > 2:
                self.path.lineTo(self.points[-1])

            self.path_item.setPath(self.path)
        self.scene.update()

    def clear_spline(self):
        self.points = []
        self.path_item.setPath(QPainterPath())
        self.scene.update()

    def generate_path(self):
        # Generate the Gazebo world file with the walls
        point_list = []
        x_ = []
        y_ = []
        scaler = float(self.scaler_input.text())
        track_width_ = float(self.track_width_input.text())
        track_height_ = float(self.track_height_input.text())
        self.file_name = self.file_name_input.text()
        for i, point in enumerate(self.points):
            # print(f"({point.x()}, {point.y()})")
            x_.append(point.x()/scaler)
            y_.append((point.y()/scaler)*-1)
        x_smooth, y_smooth = gaussian_smooth(x_, y_, 7)  # s=0 ensures interpolation through all points
        for i, point in enumerate(x_smooth):
            point_list.append((x_smooth[i],y_smooth[i]))
        
        close_loop_status = self.close_loop_checkbox.isChecked()
        self.track_generator.generate_track_world(point_list, self.file_name, close_loop_status,track_width_, track_height_)

        print("Path generated")
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SplineDrawer()
    window.show()
    sys.exit(app.exec_())
