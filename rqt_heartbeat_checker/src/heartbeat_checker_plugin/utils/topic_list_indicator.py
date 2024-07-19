import rospy
import roslib
import rosmsg
from python_qt_binding.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QMenu, QAction
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtCore import qWarning

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

from .ros_data import ROSData

class TopicListIndicator(QHBoxLayout):
    def __init__(self, initial_topics=None):
        super(TopicListIndicator, self).__init__()
        self.setObjectName('TopicListIndicator')

        # topic list
        self._rosdata = {}

        # create widgets
        self.text = QLabel("Topic")
        self.search_box = QLineEdit("")
        self.add_button = QPushButton()
        self.add_button.setIcon(QIcon.fromTheme('add'))
        self.add_button.setFixedSize(40,40)
        self.del_button = QPushButton()
        self.del_button.setIcon(QIcon.fromTheme('remove'))
        self.del_button.setFixedSize(40,40)

        # add widgets to layout
        self.addWidget(self.text)
        self.addWidget(self.search_box)
        self.addWidget(self.add_button)
        self.addWidget(self.del_button)

        # add button setting
        self.add_button.setEnabled(True)
        self.del_button.setEnabled(True)

        # topic search box setting
        self._topic_completer = TopicCompleter(self.search_box)
        self._topic_completer.update_topics()
        self.search_box.setCompleter(self._topic_completer)

        # user custom qt slot
        # about add topic
        self.add_button.clicked.connect(self.add_topic)
        self.search_box.returnPressed.connect(self.add_topic)
        # about remove topic
        self.menu = QMenu(self.del_button)
        self.del_button.clicked.connect(self.show_removable_topics)

    def add_topic(self):
        topic_name = str(self.search_box.text())
        topic_type, real_topic, _ = topic_helpers.get_topic_type(topic_name)
        print(topic_name, topic_type, real_topic)
        print(self._rosdata)
        if topic_name in self._rosdata:
            qWarning('Topic already subscribed: %s' % topic_name)
        else:
            self._rosdata[topic_name] = ROSData(topic_name)


    def show_removable_topics(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)
        
        self.menu.clear()
        action = QAction("aaa", self.menu)
        action.triggered.connect(make_remove_topic_function("aaa"))

        for topic in sorted(self._rosdata.keys()):
            action = QAction(topic, self.menu)
            action.triggered.connect(make_remove_topic_function(topic))

        self.menu.addAction(action)
        self.del_button.setMenu(self.menu)
        pass

    def remove_topic(self, topic_name):
        print("clicked\t", topic_name)
        pass

    def _topic_list(self):
        return self.topic_list

    def __del__(self):
        pass