import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QSlider, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import cv2

def create_USB_Camera_Control_GUI(camera_index, motor_speed_range, detected_part_display_size, button_text, window_title):
    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle(window_title)
    
    # Initialize Camera
    cap = cv2.VideoCapture(camera_index)
    
    # Button for On/Off System
    toggle_button = QPushButton(button_text)
    toggle_button.clicked.connect(lambda: toggle_system(cap))  # Assuming toggle_system is defined elsewhere
    
    # Trackbar for Motor Speed
    motor_speed_slider = QSlider(Qt.Horizontal)
    motor_speed_slider.setMinimum(motor_speed_range['min'])
    motor_speed_slider.setMaximum(motor_speed_range['max'])
    motor_speed_slider.valueChanged.connect(lambda: adjust_motor_speed(motor_speed_slider.value()))  # Assuming adjust_motor_speed is defined elsewhere
    
    # Main Image Display
    main_image_label = QLabel()
    
    # Detected Part Display
    detected_part_label = QLabel()
    detected_part_label.setFixedSize(detected_part_display_size['width'], detected_part_display_size['height'])
    
    # Layout Setup
    main_layout = QVBoxLayout()
    controls_layout = QHBoxLayout()
    controls_layout.addWidget(toggle_button)
    controls_layout.addWidget(motor_speed_slider)
    main_layout.addLayout(controls_layout)
    main_layout.addWidget(main_image_label)
    main_layout.addWidget(detected_part_label)
    window.setLayout(main_layout)
    
    # Function to update the GUI with camera feed
    def update_window():
        ret, frame = cap.read()
        if ret:
            # Convert to QImage and display
            height, width, channel = frame.shape
            bytesPerLine = 3 * width
            qImg = QImage(frame.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
            main_image_label.setPixmap(QPixmap.fromImage(qImg).scaled(640, 480, Qt.KeepAspectRatio))
            
            # Example: Display detected part (replace with actual detection logic)
            detected_part = frame[100:300, 100:300]  # Example: Crop a part of the frame
            height, width, channel = detected_part.shape
            bytesPerLine = 3 * width
            qDetectedImg = QImage(detected_part.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
            detected_part_label.setPixmap(QPixmap.fromImage(qDetectedImg))
        
        # Repeat
        QTimer.singleShot(1, update_window)
    
    update_window()
    window.show()
    sys.exit(app.exec_())

# Example usage based on the JSON parameters
create_USB_Camera_Control_GUI(
    camera_index=0,
    motor_speed_range={"min": 0, "max": 100},
    detected_part_display_size={"width": 200, "height": 150},
    button_text="Toggle System",
    window_title="USB Camera Controller"
)