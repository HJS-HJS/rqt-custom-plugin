import rospy
import roslib
import rosmsg
from python_qt_binding.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QMenu,
    QAction,
)
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtCore import qWarning

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

from .ros_label import ROSLabel


class TopicListIndicator(QHBoxLayout):
    def __init__(self, initial_topics=None):
        super(TopicListIndicator, self).__init__()
        self.setObjectName("TopicListIndicator")

        # topic list
        self._roslabel = {}
        self.is_changed = False

        # create widgets
        self.text = QLabel("Topic")
        self.search_box = QLineEdit("")
        self.add_button = QPushButton()
        self.add_button.setIcon(QIcon.fromTheme("add"))
        self.add_button.setFixedSize(40, 40)
        self.del_button = QPushButton()
        self.del_button.setIcon(QIcon.fromTheme("remove"))
        self.del_button.setFixedSize(40, 40)

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
        self.del_button.setMenu(self.menu)
        self.del_button.clicked.connect(self.update_removable_topics)

    def add_topic(self):
        topic_name = str(self.search_box.text())
        if topic_name in self._roslabel:
            qWarning("Topic already subscribed: %s" % topic_name)
        else:
            self._roslabel[topic_name] = ROSLabel(topic_name)
        self.is_changed = True

    def update_removable_topics(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self.menu.clear()

        for topic in sorted(self._roslabel.keys()):
            action = QAction(topic, self.menu)
            action.triggered.connect(make_remove_topic_function([topic]))
            self.menu.addAction(action)

        if len(self._roslabel) > 0:
            action = QAction("All", self.menu)
            action.triggered.connect(
                make_remove_topic_function(sorted(self._roslabel.keys()))
            )
            self.menu.addAction(action)
        else:
            action = QAction("None", self.menu)
            action.setEnabled(False)
            self.menu.addAction(action)

        self.del_button.setMenu(self.menu)
        self.is_changed = False
        pass

    def remove_topic(self, topic_name):
        for topic in topic_name:
            self._roslabel[topic].setParent(None)
            del self._roslabel[topic]
        self.is_changed = True
        pass

    def topic_list(self):
        return self._roslabel

    def topic_num(self):
        return len(self._roslabel.keys())

    def topic_name_list(self):
        return sorted(self._roslabel.keys())

    def __del__(self):
        pass
