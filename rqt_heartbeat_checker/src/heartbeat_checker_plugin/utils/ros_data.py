import string
import sys
import threading
import time

import rosgraph
import roslib.message
import roslib.names
import rospy

class ROSData(object):
    """
    Subscriber to ROS topic that buffers incoming data
    """

    def __init__(self, topic, start_time):
        self.name = topic
        self.start_time = start_time
        self.error = None

        self.lock = threading.Lock()
        self.buff_x = []
        self.buff_y = []

        topic_type, real_topic, fields = get_topic_type(topic)
        if topic_type is not None:
            self.field_evals = generate_field_evals(fields)
            data_class = roslib.message.get_message_class(topic_type)
            self.sub = rospy.Subscriber(real_topic, data_class, self._ros_cb)
        else:
            self.error = RosPlotException("Can not resolve topic type of %s" % topic)

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
                #self.axes[index].plot(datax, buff_y)
            except AttributeError, e:
                self.error = RosPlotException("Invalid topic spec [%s]: %s" % (self.name, str(e)))
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

    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return float(val)
            for f in self.field_evals:
                val = f(val)
            return float(val)
        except IndexError:
            self.error = RosPlotException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException("[%s] value was not numeric: %s" % (self.name, val))