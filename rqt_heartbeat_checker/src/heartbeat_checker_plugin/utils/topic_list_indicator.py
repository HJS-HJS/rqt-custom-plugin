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
        self.text = QLabel("Topic")     # text widget
        self.search_box = QLineEdit("") # topic search box
        self.rel_button = QPushButton() # reload topic button
        self.add_button = QPushButton() # add topic as label button
        self.del_button = QPushButton() # delete label button

        # add widgets to layout
        self.addWidget(self.text)
        self.addWidget(self.search_box)
        self.addWidget(self.rel_button)
        self.addWidget(self.add_button)
        self.addWidget(self.del_button)

        # button widget setting 
        self.rel_button.setFixedSize(40, 40)
        self.add_button.setFixedSize(40, 40)
        self.del_button.setFixedSize(40, 40)
        self.rel_button.setIcon(QIcon.fromTheme("reload"))
        self.add_button.setIcon(QIcon.fromTheme("add"))
        self.del_button.setIcon(QIcon.fromTheme("remove"))
        self.rel_button.setEnabled(True)
        self.add_button.setEnabled(True)
        self.del_button.setEnabled(True)

        # topic search widget setting
        self._topic_completer = TopicCompleter(self.search_box)
        # update topic list menu
        self._topic_completer.update_topics()
        self.search_box.setCompleter(self._topic_completer)

        # user custom qt slot
        # reset topics
        self.rel_button.clicked.connect(self.reload_topic)      # when reload button clicked
        # add topic
        self.add_button.clicked.connect(self.add_topic)         # when add button clicked
        self.search_box.returnPressed.connect(self.add_topic)   # when press enter at search box
        # show removable topic list
        self.menu = QMenu(self.del_button)                      # set removable topic menu to del button
        self.del_button.setMenu(self.menu)                      # when del button clicked

    def add_topic(self, topic_name:str=None, label_name:str=None, hz:float=1.0):
        """generate ros topic qlabel and add to the self._roslabel list
        Args:
            topic_name (str, optional): name of topic to add. Defaults to None.
            label_name (str, optional): name of topic label to apply. Defaults to None.
            hz (float, optional): hz of topic to add.
        """
        # save topic name as text in search_box
        topic_name = str(self.search_box.text()) if topic_name is None else topic_name
        # check for duplicate topics
        if topic_name in self._roslabel:
            qWarning("Topic already subscribed: %s" % topic_name)
        else:
            self._roslabel[topic_name] = ROSLabel(topic_name, label_name, hz)
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

    def reload_topic(self):
        # update topic list menu
        self._topic_completer.update_topics()
        # update remove button menu
        self.update_removable_topics()


    def update_removable_topics(self):
        """ update deletable topics from del_button menu
        """
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        # initialize menu
        self.menu.clear()

        # connect topic names and remove actions in menu
        for topic in sorted(self._roslabel.keys()):
            action = QAction(self._roslabel[topic].label_name, self.menu)
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

        # update del button menu
        self.del_button.setMenu(self.menu)
        # update topic list menu
        self._topic_completer.update_topics()
        # check for changed topic
        self.is_changed = False

    @property
    def topic_list(self):
        return sorted(self._roslabel.keys())
    
    @property
    def label_list(self):
        return self._roslabel
    
    @property
    def label_name_list(self):
        return [self._roslabel[x].label_name for x in self._roslabel]

    @property
    def topic_hz_list(self):
        return [self._roslabel[x].hz for x in self._roslabel]
    
    @property
    def topic_num(self):
        return len(self._roslabel.keys())

    def __del__(self):
        pass
