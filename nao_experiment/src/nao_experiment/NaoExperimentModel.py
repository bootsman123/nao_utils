PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from cv_bridge import CvBridge, CvBridgeError;
import cv;
import os;
import yaml;

from rosgraph_msgs.msg import Log;
from sensor_msgs.msg import Image;
from cmvision.msg import Blob, Blobs;

from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
from nao_experiment.states.StateStart import StateStart;
from nao_experiment.states.StateIntroduction import StateIntroduction;
from nao_experiment.states.StateCollectingObjects import StateCollectingObjects;
    
class NaoExperimentModel( QtCore.QObject ):
    """
    NaoExperimentModel
    """
    imageChanged = QtCore.pyqtSignal( QtGui.QImage );
    blobsChanged = QtCore.pyqtSignal( Blobs );
    logChanged = QtCore.pyqtSignal( Log );
    objectsChanged = QtCore.pyqtSignal( [] )
    
    def __init__( self ):
        QtCore.QObject.__init__( self );
        
        # Setup OpenCV.
        self.__cvBridge = CvBridge();
        
        # Setup subscribers.
        self.__logTopic = rospy.get_param( 'logTopic', '/rosout' );
        self.__logSubscriber = rospy.Subscriber( self.__logTopic, Log, self.onLogReceived );
        
        self.__cameraTopic = rospy.get_param( 'cameraTopic', '/nao_camera_processed/image_raw' );
        self.__cameraSubscriber = rospy.Subscriber( self.__cameraTopic, Image, self.onImageReceived );
        
        self.__blobsTopic = rospy.get_param( 'blobsTopic', '/blobs' );
        self.__blobsSubscriber = rospy.Subscriber( self.__blobsTopic, Blobs, self.onBlobsReceived );
        
        # Hold a list of all the log messages.
        self.__log = [];
        
        # Load objects.
        objectsListPath = os.path.join( roslib.packages.get_pkg_dir( PACKAGE_NAME ), 'cfg' );
        objectsListFilePath = os.path.join( objectsListPath, 'config.yaml' );
        
        with open( objectsListFilePath, 'r' ) as objectsListFile:
            self.__objectsList = yaml.load( objectsListFile );
        
        self.__objectsMapper = QtCore.QSignalMapper( self );
        
        for index, o in enumerate( self.__objectsList ):
            #self.__objectsMapper.setMapping( o, index );
            o.changed.connect( self.__objectsMapper.map );
        
        rospy.sleep( 1 );
        
        # Initialize.
        self.setupExperiment();

    def __del__( self ):
        self.__logSubscriber.unregister();
        self.__cameraSubscriber.unregister();
        self.__blobsSubscriber.unregister();
    
    def setupExperiment( self ):
        # Initialize.
        self.__isRunning = False;
        self.__isPaused = False;
        
         # Create states.
        self.__stateStart = StateStart( model = self );
        #self.__stateIntroduction = StateIntroduction( model = self );
        self.__stateCollectingObjects = StateCollectingObjects( model = self );

        # Setup state machine.
        self.__stateMachine = QtCore.QStateMachine();
        self.__stateMachine.addState( self.__stateStart );
        #self.__stateMachine.addState( self.__stateIntroduction );
        self.__stateMachine.addState( self.__stateCollectingObjects );
        
        # Add transitions.
        #self.__stateStart.addTransition( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.__stateIntroduction );
        #self.__stateIntroduction.addTransition( self.__stateIntroduction, QtCore.SIGNAL( 'finished()' ), self.__stateCollectingObjects );

        self.__stateStart.addTransition( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.__stateCollectingObjects );

        self.__stateMachine.setInitialState( self.__stateStart );
        
    def start( self ):
        self.__isRunning = True;
        self.__stateMachine.start();
    
    def pause( self ):
        self.__isPaused = True;
    
    def resume( self ):
        self.__isPaused = False;
    
    def stop( self ):
        self.__isRunning = False;
        self.__stateMachine.stop();
    
    def isRunning( self ):
        return self.__isRunning;
    
    def isPaused( self ):
        return self.__isPaused;
        
    def onLogReceived( self, data ):
        """
        onLogReceived
        @param: data
        """
        # We only want to capture nodes from the experiment.
        if( data.name == rospy.get_name() ):
            self.__log.append( data );
            self.logChanged.emit( data );

    def onImageReceived( self, data ):
        """
        onImageReceived
        @param: data
        """
        openCVMatrix = self.__cvBridge.imgmsg_to_cv( data );
        
        # Convert from BGR to RGB.
        cv.CvtColor( openCVMatrix, openCVMatrix, cv.CV_BGR2RGB );

        # Keep a local reference to the data.
        self.__imageStringData = openCVMatrix.tostring();
        
        # Create QImage.
        self.__qImage = QtGui.QImage( self.__imageStringData,
                               data.width,
                               data.height,
                               openCVMatrix.step,
                               QtGui.QImage.Format_RGB888 );
 
        self.imageChanged.emit( self.__qImage );
        
    def onBlobsReceived( self, data ):
        """
        onImageReceived
        @param: data
        """
        self.blobsChanged.emit( data );
    