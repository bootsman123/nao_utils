#!/usr/bin/env python

PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

import sys;

from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

from NaoExperimentModel import NaoExperimentModel;
from NaoExperimentView import NaoExperimentView;
from NaoExperimentPresenter import NaoExperimentPresenter;

if( __name__ == '__main__' ):
    App = QtGui.QApplication( sys.argv );
    rospy.init_node( 'nao_experiment' );
    
    NaoExperimentModel = NaoExperimentModel();
    NaoExperimentView = NaoExperimentView();
    NaoExperimentPresenter = NaoExperimentPresenter( model = NaoExperimentModel,
                                                     view = NaoExperimentView );
    
    sys.exit( App.exec_() );