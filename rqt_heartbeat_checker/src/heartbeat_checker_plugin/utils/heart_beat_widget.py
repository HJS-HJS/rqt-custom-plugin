from typing import List, Tuple

from .topic_list_indicator import TopicListIndicator

from python_qt_binding.QtWidgets import QWidget, QVBoxLayout
from python_qt_binding.QtCore import QBasicTimer


class HeartBeatWidget(QWidget):
    def __init__(self):
        """Basic setting for widget.
        """
        super(HeartBeatWidget, self).__init__()
        # Give QObjects reasonable names
        self.setWindowTitle("HeartBeatWidget")
        # Generate GUI
        self.init_ui()
        # Start event timer with period 100 [ms]
        self.timer = QBasicTimer()
        self.timer.start(100, self)

    def init_ui(self):
        """Generate GUI include each topic label.
        """
        # Create widgets and labels.
        self.vbox = QVBoxLayout()                     # Layout that contain every widget, label created in this funtion.
        self.topic_indicator = TopicListIndicator()   # Layout to search, add, remove topics.
        self.heart_beat_group_layout = QVBoxLayout()  # Layout to show the status of every topic node.

        # Set the relationship between each layout.
        self.vbox.addLayout(self.topic_indicator)
        self.vbox.addLayout(self.heart_beat_group_layout)
        # Apply layout to widget itself.
        self.setLayout(self.vbox)

    def timerEvent(self, event):
        """Update ui and check each labels heartbeat.
        """
        # GUI update in timer event only when timer is running
        if self.timer.timerId():
            # Update GUI.
            self.update_ui()
            # Check every heartbeat.
            for topic_name in self.topic_indicator.label_list:
                self.topic_indicator.label_list[topic_name].on_timer()

    def update_ui(self):
        """Update topic labels in self.heart_beat_group_layout when topic label list is changed.
        """
        if self.topic_indicator.is_changed:
            # update removable topic list in self.topic_indicator
            self.topic_indicator.update_removable_topics()
            # add label from label list in self.topic_indicator. Duplicate labels are not automatically added again.
            for topic_name in self.topic_indicator.label_list:
                self.heart_beat_group_layout.addWidget(self.topic_indicator.label_list[topic_name])

    def shutdown_plugin(self):
        """Shutdown plugin when this custom rqt plugin is down
        """
        # delete every labels not to subscribe topics
        for topic_name in self.topic_indicator.topic_list:
            del self.topic_indicator.label_list[topic_name]
        # stop timer event
        self.timer.stop()

    def save_settings(self) -> Tuple[List[str], List[str], List[float]]:
        """When the plugin is terminated, the current state is saved. Save the topic, label name, and hz.

        Returns:
            Tuple[List[str], List[str], List[float]]: topic list, label name list, hz list
        """
        return self.topic_indicator.topic_list, self.topic_indicator.label_name_list, self.topic_indicator.topic_hz_list
    
    def apply_settings(self, saved_topics):
        """When the plugin starts anew, uses the previously saved topic, label, and hz.
        """
        if saved_topics is not None:
            for i in range(len(saved_topics[0])):
                self.topic_indicator.add_topic(saved_topics[0][i], saved_topics[1][i], saved_topics[2][i])