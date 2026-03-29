#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QComboBox
)
from PyQt5.QtCore import QTimer, Qt


# ================= ROS NODE =================
class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.lift_pub = self.create_publisher(String, '/lift_cmd', 10)
        self.nav_pub = self.create_publisher(String, '/table_cmd', 10)

        self.status_text = "IDLE"
        self.battery_percent = 100


# ================= MAIN WINDOW =================
class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle("GASTROBOT")
        self.showFullScreen()
        self.setFocusPolicy(Qt.StrongFocus)

        self.setStyleSheet("""
        QWidget {
            background-color: #1e1e1e;
            color: #39FF14;
            font-family: Arial;
        }
        """)

        main_layout = QVBoxLayout()

        # ================= TOP BAR =================
        top_bar = QHBoxLayout()

        self.status_label = QLabel("STATUS: IDLE")
        self.status_label.setStyleSheet("font-size: 20px; font-weight: bold;")

        self.battery_label = QLabel("BATTERY: 100%")
        self.battery_label.setStyleSheet("font-size: 20px; font-weight: bold;")

        top_bar.addWidget(self.status_label)
        top_bar.addStretch()
        top_bar.addWidget(self.battery_label)

        main_layout.addLayout(top_bar)

        # ================= FLEX SPACE ABOVE TITLE =================
        main_layout.addStretch()

        # ================= TITLE (CENTERED) =================
        self.title = QLabel("GASTROBOT CONTROL")
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet("font-size: 40px; font-weight: bold; padding: 10px;")

        main_layout.addWidget(self.title)

        # ================= FLEX SPACE BETWEEN TITLE & BUTTONS =================
        main_layout.addStretch()

        # ================= BUTTON SECTION =================
        content_layout = QVBoxLayout()

        # ===== TABLE =====
        table_row = QHBoxLayout()

        self.table_dropdown = QComboBox()
        for i in range(1, 11):
            self.table_dropdown.addItem(f"Table {i}")

        self.table_dropdown.setStyleSheet("""
            font-size: 32px;
            padding: 20px;
        """)

        self.go_button = QPushButton("GO")
        self.go_button.setStyleSheet("""
            QPushButton {
                background-color: #39FF14;
                color: black;
                font-size: 48px;
                padding: 35px;
                border-radius: 15px;
            }
        """)

        self.go_button.clicked.connect(self.send_table)

        table_row.addWidget(self.table_dropdown)
        table_row.addWidget(self.go_button)

        content_layout.addLayout(table_row)

        # ===== LIFT =====
        lift_row = QHBoxLayout()

        self.lift_up = QPushButton("SERVE")
        self.lift_stop = QPushButton("STOP")
        self.lift_home = QPushButton("HOME")

        btn_style = "font-size: 28px; padding: 25px; border-radius: 10px;"

        self.lift_up.setStyleSheet("background-color: #39FF14; " + btn_style)
        self.lift_stop.setStyleSheet("background-color: orange; " + btn_style)
        self.lift_home.setStyleSheet("background-color: #666; " + btn_style)

        self.lift_up.clicked.connect(lambda: self.send_lift("UP"))
        self.lift_stop.clicked.connect(lambda: self.send_lift("STOP"))
        self.lift_home.clicked.connect(lambda: self.send_lift("HOME"))

        lift_row.addWidget(self.lift_up)
        lift_row.addWidget(self.lift_stop)
        lift_row.addWidget(self.lift_home)

        content_layout.addLayout(lift_row)

        # ===== E-STOP =====
        self.estop = QPushButton("EMERGENCY STOP")
        self.estop.setStyleSheet("""
            QPushButton {
                background-color: red;
                color: white;
                font-size: 40px;
                padding: 30px;
                border-radius: 15px;
            }
        """)

        content_layout.addWidget(self.estop)

        # ===== POWER =====
        power_row = QHBoxLayout()

        self.shutdown_btn = QPushButton("SHUTDOWN")
        self.restart_btn = QPushButton("RESTART")

        self.shutdown_btn.setStyleSheet("background-color: red; font-size: 20px; padding: 15px;")
        self.restart_btn.setStyleSheet("background-color: orange; font-size: 20px; padding: 15px;")

        power_row.addWidget(self.shutdown_btn)
        power_row.addWidget(self.restart_btn)

        content_layout.addLayout(power_row)

        # ================= ADD BUTTONS TO MAIN =================
        main_layout.addLayout(content_layout)

        self.setLayout(main_layout)

        # ================= TIMER =================
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    # ================= FUNCTIONS =================

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

    def send_table(self):
        msg = String()
        msg.data = self.table_dropdown.currentText()
        self.node.nav_pub.publish(msg)

    def send_lift(self, cmd):
        msg = String()
        msg.data = cmd
        self.node.lift_pub.publish(msg)

    def update_ui(self):
        self.status_label.setText(f"STATUS: {self.node.status_text}")
        self.battery_label.setText(f"BATTERY: {self.node.battery_percent}%")

        rclpy.spin_once(self.node, timeout_sec=0)


# ================= MAIN =================
def main():
    rclpy.init()
    node = GuiNode()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
