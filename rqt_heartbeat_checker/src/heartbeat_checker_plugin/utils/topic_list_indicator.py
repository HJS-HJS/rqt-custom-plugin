import rospy
import roslib
import rosmsg


from python_qt_binding.QtWidgets import (
    QComboBox,
)

class TopicListIndicator(QComboBox):
    
    def __init__(self, topic_list=None):
        super(TopicListIndicator, self).__init__()
        self.topic_list = topic_list



    def _add_message(self):
        if self._msgs_combo.count() == 0:
            return
        msg = (self._package_combo.currentText() +
               '/' + self._msgs_combo.currentText())
        print(msg)

        rospy.logdebug('_add_message msg={}'.format(msg))

        msg_class = roslib.message.get_message_class(msg)()
        text_tree_root = 'Msg Root'
        self._messages_tree.model().add_message(msg_class,
                                        self.tr(text_tree_root), msg, msg)

        self._messages_tree._recursive_set_editable(
            self._messages_tree.model().invisibleRootItem(), False)


    def _topic_list(self):
        return self.topic_list

    def __del__(self):
        pass