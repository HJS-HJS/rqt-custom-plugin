import threading

import rospy
import roslib.message
import roslib.names
from rqt_py_common import topic_helpers


class ROSData(object):
    """
    Subscriber to ROS topic that buffers incoming data
    """

    def __init__(self, topic_name):
        self.topic = topic_name
        self.title = topic_name

        self.lock = threading.Lock()
        self.buff_x = []
        self.buff_y = []
        self.start_time = rospy.get_time()

        topic_type, _, _ = topic_helpers.get_topic_type(self.topic)

        if topic_type is not None:
            data_class = roslib.message.get_message_class(topic_type)
            self.sub = rospy.Subscriber(self.topic, data_class, self._ros_cb)
        else:
            self.sub = None

    def close(self):
        self.sub.unregister()

    def _ros_cb(self, msg):
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        try:
            self.lock.acquire()
            try:
                self.buff_y.append(self._get_data(msg))
                # #944: use message header time if present
                if msg.__class__._has_header:
                    self.buff_x.append(msg.header.stamp.to_sec() - self.start_time)
                else:
                    self.buff_x.append(rospy.get_time() - self.start_time)
                # self.axes[index].plot(datax, buff_y)
            except AttributeError:
                self.error = RosPlotException("Invalid topic spec [%s]" % (self.name))
        finally:
            self.lock.release()

    def next(self):
        """
        Get the next data in the series

        :returns: [xdata], [ydata]
        """
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            buff_x = self.buff_x
            buff_y = self.buff_y
            self.buff_x = []
            self.buff_y = []
        finally:
            self.lock.release()
        return buff_x, buff_y

    def update_topic(self):
        if self.sub is None:
            topic_type, _, _ = topic_helpers.get_topic_type(self.topic)
            if topic_type is not None:
                data_class = roslib.message.get_message_class(topic_type)
                self.sub = rospy.Subscriber(self.topic, data_class, self._ros_cb)
            else:
                self.sub = None

    def _get_data(self, msg):
        pass

    def label_name(self):
        return self.title

    def set_label_name(self, label_name):
        self.title = label_name


class RosPlotException(Exception):
    pass
