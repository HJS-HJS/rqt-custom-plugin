from .utils.heart_beat_widget import HeartBeatWidget
from qt_gui.plugin import Plugin

class HeartBeatPlugin(Plugin):
    def __init__(self, context):
        super(HeartBeatPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("HeartBeatPlugin")

        # Create QWidget
        self._widget = HeartBeatWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
        self._widget.close()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass