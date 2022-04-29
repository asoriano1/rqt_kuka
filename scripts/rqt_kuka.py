#!/usr/bin/env python

import sys

from rqt_gui.main import Main

main = Main()
sys.exit(main.main(sys.argv, standalone='rqt_kuka'))

##
###!/usr/bin/env python
##
##import sys
##
##from rqt_kuka.robotnik_rqt_kuka_node import RqtKuka
##from rqt_gui.main import Main
##
##plugin = 'rqt_kuka'
##main = Main(filename=plugin)
##sys.exit(main.main(sys.argv, standalone=plugin))
##

