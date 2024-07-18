import rospy
import roslib

from .heart_beat_indicator import HeartBeatIndicator
from .topic_list_indicator import TopicListIndicator

from python_qt_binding.QtWidgets import QWidget,QVBoxLayout,QGroupBox
from python_qt_binding.QtCore import QBasicTimer

class HeartBeatWidget(QWidget):
    def __init__(self):
        super(HeartBeatWidget, self).__init__()
        # Give QObjects reasonable names
        self.setWindowTitle("HeartBeatWidget")
        # Process standalone plugin command-line arguments
        self.topic_label_list=[]
        self.topic_label_list=[
            HeartBeatIndicator("System Manager", "/unld/heartbeat/system_manager"),
            # HeartBeatIndicator("Camera Driver", "/unld/heartbeat/ensenso_camera_node"),
            # HeartBeatIndicator("Camera Extrinsic Publisher", "/unld/heartbeat/cam_extrinsic_publisher"),
            # HeartBeatIndicator("Object Detector 2D", "/unld/heartbeat/object_detector_2d"),
            # HeartBeatIndicator("OBB Calculator", "/unld/heartbeat/obb_calculator"),
            # HeartBeatIndicator("Mask Visualizer", "/unld/heartbeat/mask_visualizer"),
            # HeartBeatIndicator("Pilot Task Planner", "/unld/heartbeat/pilot_task_planner"),
        ]

        self.init_ui()

    def init_ui(self):
        # create a vertical layout
        self.topic_list = TopicListIndicator("")
        

        # labels to show the status of each node
        heat_beat_group = QGroupBox("Heart Beat")
        self.heart_beat_group_layout = QVBoxLayout()

        for label in self.topic_label_list:
            self.heart_beat_group_layout.addWidget(label)

        heat_beat_group.setLayout(self.heart_beat_group_layout)
        
        vbox = QVBoxLayout()
        vbox.addLayout(self.topic_list)
        vbox.addWidget(heat_beat_group)
        self.setLayout(vbox)

        self.timer = QBasicTimer()
        self.timer.start(1000, self)

    def update_ui(self):
        # print('self.layout', self.layout)
        # print('self.heart_beat_layout', self.heart_beat_group_layout)
        # self.heart_beat_group_layout = QVBoxLayout()
        # self.heart_beat_group_layout.removeWidget()
        pass
    
        
    def timerEvent(self, event):
        # gui update in timer event, 30 fps
        for label in self.topic_label_list:
            label.on_timer()
        self.timer.start(33, self)
        self.update_ui()
        pass

    def shutdown_plugin(self):
        del self.system_manager_label
        del self.ensenso_camera_node_label
        del self.cam_extrinsic_publisher_label
        del self.object_detector_2d_label
        del self.obb_calculator_label
        del self.mask_visualizer_label
        del self.pilot_task_planner_label
        self.timer.stop()
        pass