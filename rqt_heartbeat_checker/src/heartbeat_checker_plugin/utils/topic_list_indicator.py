from typing import List

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
        self.del_button = QPushButton()

        # add widgets to layout
        self.addWidget(self.text)
        self.addWidget(self.search_box)
        self.addWidget(self.add_button)
        self.addWidget(self.del_button)

        # button widget setting 
        self.add_button.setIcon(QIcon.fromTheme("add"))
        self.add_button.setFixedSize(40, 40)
        self.del_button.setIcon(QIcon.fromTheme("remove"))
        self.del_button.setFixedSize(40, 40)
        self.add_button.setEnabled(True)
        self.del_button.setEnabled(True)

        # topic search widget setting
        self._topic_completer = TopicCompleter(self.search_box)
        self._topic_completer.update_topics()
        self.search_box.setCompleter(self._topic_completer)

        # user custom qt slot
        # when add button clicked, add topic
        self.add_button.clicked.connect(self.add_topic)
        # when press enter, add topic
        self.search_box.returnPressed.connect(self.add_topic)
        # when del button clicked, show removable topic list
        self.menu = QMenu(self.del_button)
        self.del_button.setMenu(self.menu)

    def add_topic(self):
        """generate ros topic qlabel and add to the self._roslabel list
        """
        # save topic name as text in search_box
        topic_name = str(self.search_box.text())
        # check for duplicate topics
        if topic_name in self._roslabel:
            qWarning("Topic already subscribed: %s" % topic_name)
        else:
            self._roslabel[topic_name] = ROSLabel(topic_name)
        # check for changed topic
        self.is_changed = True

    def remove_topic(self, topic_list: List[str]):
        """remove ros topic qlabel from the self._roslabel list and delete it

        Args:
            topic_list (List[str]): List of topic names want to remove.
        """
        for topic_name in topic_list:
            # delete each topic label from its parent widget
            self._roslabel[topic_name].setParent(None)
            # delete each topic label
            del self._roslabel[topic_name]
        # check for changed topic
        self.is_changed = True

    def update_removable_topics(self):
        """ update deletable topics from del_button menu
        """
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        # initialize menu
        self.menu.clear()

        # connect topic names and remove actions in menu
        for topic in sorted(self._roslabel.keys()):
            action = QAction(topic, self.menu)
            action.triggered.connect(make_remove_topic_function([topic]))
            self.menu.addAction(action)

        # add remove all topic menu when its numeber is more than 0
        if len(self._roslabel) > 0:
            action = QAction("All", self.menu)
            action.triggered.connect(
                make_remove_topic_function(sorted(self._roslabel.keys()))
            )
            self.menu.addAction(action)
        # add none when topic not exists
        else:
            action = QAction("None", self.menu)
            action.setEnabled(False)
            self.menu.addAction(action)

        # set updated menu
        self.del_button.setMenu(self.menu)
        # check for changed topic
        self.is_changed = False

    @property
    def topic_list(self):
        return sorted(self._roslabel.keys())
    
    @property
    def label_list(self):
        return self._roslabel

    @property
    def topic_num(self):
        return len(self._roslabel.keys())

    def __del__(self):
        pass
