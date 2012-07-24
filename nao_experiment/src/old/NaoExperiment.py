PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

from nao_experiment.states.StateStart import StateStart;
from nao_experiment.states.StateIntroduction import StateIntroduction;
from nao_experiment.states.StateCollectingObjects import StateCollectingObjects;

from nao_experiment.NaoExperimentUtils import NaoExperimentUtils;

# http://wiki.python-ogre.org/index.php/Game_State_Management
class NaoExperiment( QtCore.QObject ):
    """
    NaoExperiment
    """
    def __init__( self ):
        QtCore.QObject.__init__( self );
        
        # Create states.
        self.__stateStart = StateStart( experiment = self );
        self.__stateIntroduction = StateIntroduction( experiment = self );
        self.__stateCollectingObjects = StateCollectingObjects( experiment = self );

        # Setup state machine.
        self.__stateMachine = QtCore.QStateMachine();
        self.__stateMachine.addState( self.__stateStart );
        self.__stateMachine.addState( self.__stateIntroduction );
        self.__stateMachine.addState( self.__stateCollectingObjects );
        
        # Add transitions.
        self.__stateStart.addTransition( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.__stateIntroduction );
        self.__stateIntroduction.addTransition( self.__stateIntroduction, QtCore.SIGNAL( 'finished()' ), self.__stateCollectingObjects );

        self.__stateMachine.setInitialState( self.__stateStart );
        
        # Setup utils.
        self.__utils = NaoExperimentUtils();
        
        # Initialize.
        self.__isRunning = False;
        self.__isPaused = False;

    def start( self ):
        self.__isRunning = True;
        self.__stateMachine.start();
    
    def pause( self ):
        self.__isPaused = True;
    
    def resume( self ):
        self.__isPaused = False;
    
    def stop( self ):
        self.__isRunning = True;
        self.__stateMachine.stop();
    
    def isRunning( self ):
        return self.__isRunning;
    
    def isPaused( self ):
        return self.__isPaused;
        
    def getUtils( self ):
        return self.__utils;
    
    '''  
        #self.__stateMachine.addTransition( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.__stateIntroduction );
        #self.__stateMachine.addTransition( self.__stateStart, QtCore.SIGNAL( 'exited()' ), self.__stateIntroduction );

        #QtCore.QObject.connect( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.onFinishedStateStart );

        #self.__stateStart.finished.connect( self.onFinishedStateStart );
        #self.__stateStart.test.connect( self.onTestStateStart );
        
        #self.__stateStart.connect( self, QtCore.SIGNAL( 'finished()' ), QtCore.SLOT( 'onFinishedStateStart()' ) );
    '''