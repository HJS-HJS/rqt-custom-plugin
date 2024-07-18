import rospy
import roslib

from .heart_beat_indicator import HeartBeatIndicator
from .topic_list_indicator import TopicListIndicator

from python_qt_binding.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QComboBox,
    QGroupBox,
)
from python_qt_binding.QtCore import QBasicTimer

class HeartBeatWidget(QWidget):
    def __init__(self):
        super(HeartBeatWidget, self).__init__()
        # Give QObjects reasonable names
        self.setWindowTitle("HeartBeatWidget")
        # Process standalone plugin command-line arguments
        self.init_ui()

    def init_ui(self):
        # create a vertical layout
        vbox = QVBoxLayout()
        vbox.addWidget(TopicListIndicator())

        # labels to show the status of each node
        heat_beat_group = QGroupBox("Heart Beat")
        heat_beat_group_layout = QVBoxLayout()

        self.system_manager_label = HeartBeatIndicator(
            "System Manager", "/unld/heartbeat/system_manager"
        )
        heat_beat_group_layout.addWidget(self.system_manager_label)

        self.ensenso_camera_node_label = HeartBeatIndicator(
            "Camera Driver", "/unld/heartbeat/ensenso_camera_node"
        )
        heat_beat_group_layout.addWidget(self.ensenso_camera_node_label)

        self.cam_extrinsic_publisher_label = HeartBeatIndicator(
            "Camera Extrinsic Publisher", "/unld/heartbeat/cam_extrinsic_publisher"
        )
        heat_beat_group_layout.addWidget(self.cam_extrinsic_publisher_label)

        self.object_detector_2d_label = HeartBeatIndicator(
            "Object Detector 2D", "/unld/heartbeat/object_detector_2d"
        )
        heat_beat_group_layout.addWidget(self.object_detector_2d_label)

        self.obb_calculator_label = HeartBeatIndicator(
            "OBB Calculator", "/unld/heartbeat/obb_calculator"
        )
        heat_beat_group_layout.addWidget(self.obb_calculator_label)

        self.mask_visualizer_label = HeartBeatIndicator(
            "Mask Visualizer", "/unld/heartbeat/mask_visualizer"
        )
        heat_beat_group_layout.addWidget(self.mask_visualizer_label)

        self.pilot_task_planner_label = HeartBeatIndicator(
            "Pilot Task Planner", "/unld/heartbeat/pilot_task_planner"
        )
        heat_beat_group_layout.addWidget(self.pilot_task_planner_label)

        heat_beat_group.setLayout(heat_beat_group_layout)
        vbox.addWidget(heat_beat_group)

        self.setLayout(vbox)

        self.timer = QBasicTimer()
        self.timer.start(1000, self)
        
    def timerEvent(self, event):
        # gui update in timer event, 30 fps
        self.system_manager_label.on_timer()
        self.ensenso_camera_node_label.on_timer()
        self.cam_extrinsic_publisher_label.on_timer()
        self.object_detector_2d_label.on_timer()
        self.obb_calculator_label.on_timer()
        self.mask_visualizer_label.on_timer()
        self.pilot_task_planner_label.on_timer()
        self.timer.start(33, self)
        pass

    def shutdown_plugin(self):
        del self.system_manager_label
        del self.ensenso_camera_node_label
        del self.cam_extrinsic_publisher_label
        del self.object_detector_2d_label
        del self.obb_calculator_label
        del self.mask_visualizer_label
        del self.pilot_task_planner_label
        pass