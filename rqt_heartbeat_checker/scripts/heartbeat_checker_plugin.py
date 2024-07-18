#!/usr/bin/env python

import sys

from heartbeat_checker_plugin.heartbeat_checker_plugin import HeartBeatPlugin
from rqt_gui.main import Main

plugin = "heartbeat_checker_plugin"
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
