from PyQt5.QtWidgets import QWidget, QSlider, QLabel, QVBoxLayout, QApplication, QHBoxLayout
from PyQt5.QtCore import Qt
import sys
import numpy as np
import Robot
import socket
import time
import MyRobotMath as math

class DragUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR5 Position Controller")
        self.resize(300, 200)

        layout = QVBoxLayout()
        self.sliders = {}
        self.labels = {}

        for axis in ['X', 'Y', 'Z',]:
            hlayout = QHBoxLayout()
            label = QLabel(f"{axis}  : 0")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-500)
            slider.setMaximum(500)
            slider.setValue(0)
            slider.setSingleStep(1)

            slider.valueChanged.connect(self.on_value_changed)

            self.labels[axis] = label
            self.sliders[axis] = slider

            hlayout.addWidget(label)
            hlayout.addWidget(slider)
            layout.addLayout(hlayout)

        self.setLayout(layout)

        for axis in ['RX', 'RY', 'RZ',]:
            hlayout = QHBoxLayout()
            label = QLabel(f"{axis}: 0")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.setSingleStep(1)

            slider.valueChanged.connect(self.on_value_changed)

            self.labels[axis] = label
            self.sliders[axis] = slider

            hlayout.addWidget(label)
            hlayout.addWidget(slider)
            layout.addLayout(hlayout)

        self.setLayout(layout)

    def on_value_changed(self):
        x = self.sliders['X'].value()
        y = self.sliders['Y'].value()
        z = self.sliders['Z'].value()
        rx = self.sliders['RX'].value()
        ry = self.sliders['RY'].value()
        rz = self.sliders['RZ'].value()
        self.labels['X'].setText(f"X  : {x}")
        self.labels['Y'].setText(f"Y  : {y}")
        self.labels['Z'].setText(f"Z  : {z}")
        self.labels['RX'].setText(f"RX: {rx}")
        self.labels['RY'].setText(f"RY: {ry}")
        self.labels['RZ'].setText(f"RZ: {rz}")

        # 여기서 실시간 trajectory 생성 및 전송
        pose = [x, y, z, rx, ry, rz]
        print("→ 드래그 위치:", pose)


        # TODO: math.IK + trajectory + socket.send()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DragUI()
    window.show()
    sys.exit(app.exec_())