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
#from nao_experiment.states.StateIntroduction import StateIntroduction;
from nao_experiment.states.StateCollectingPieces import StateCollectingPieces;
from nao_experiment.states.StateEvaluatingPieces import StateEvaluatingPieces;
from nao_experiment.states.StateEnd import StateEnd;

from utils.Rectangle import Rectangle;
from pieces.Piece import Piece;
from pieces.PieceRectangular import PieceRectangular;
    
class NaoExperimentModel( QtCore.QObject ):
    """
    NaoExperimentModel
    """
    imageChanged = QtCore.pyqtSignal( QtGui.QImage );
    blobsChanged = QtCore.pyqtSignal( Blobs );
    logChanged = QtCore.pyqtSignal( Log );
    piecesChanged = QtCore.pyqtSignal( list );
    
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
        
        # Initialize variables.
        self.__log = [];
        self.__blobs = [];
        self.__pieces = [];
        self.__image = None;
        
        self.__isRunning = False;
        self.__isPaused = False;
        
        # Setup.
        self.setupPieces();
        self.setupStateMachine();

    def __del__( self ):
        self.__logSubscriber.unregister();
        self.__cameraSubscriber.unregister();
        self.__blobsSubscriber.unregister();
        
    def setupPieces( self ):
        """
        setupPieces
        """
        piecesPath = os.path.join( roslib.packages.get_pkg_dir( PACKAGE_NAME ), 'cfg' );
        piecesFilePath = os.path.join( piecesPath, 'pieces.yaml' );
        
        with open( piecesFilePath, 'r' ) as pieces:
            piecesData = yaml.load( pieces );
        
        for pieceData in piecesData:
            piece = PieceRectangular( x = pieceData[ 'x' ],
                                      y = pieceData[ 'y' ],
                                      width = pieceData[ 'width' ],
                                      height = pieceData[ 'height' ] );
                                      
            self.__pieces.append( piece );
    
    def setupStateMachine( self ):
        """
        setupStateMachine
        """
         # Create states.
        self.__stateStart = StateStart( model = self );
        #self.__stateIntroduction = StateIntroduction( model = self );
        self.__stateCollectingPieces = StateCollectingPieces( model = self );
        self.__stateEvaluatingPieces = StateEvaluatingPieces( model = self );
        self.__stateEnd = StateEnd( model = self );

        # Setup state machine.
        self.__stateMachine = QtCore.QStateMachine();
        self.__stateMachine.addState( self.__stateStart );
        #self.__stateMachine.addState( self.__stateIntroduction );
        self.__stateMachine.addState( self. __stateCollectingPieces );
        self.__stateMachine.addState( self. __stateEvaluatingPieces );
        self.__stateMachine.addState( self. __stateEnd );
        
        # Add transitions.
        #self.__stateStart.addTransition( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.__stateIntroduction );
        #self.__stateIntroduction.addTransition( self.__stateIntroduction, QtCore.SIGNAL( 'finished()' ), self.__stateCollectingPieces );

        self.__stateStart.addTransition( self.__stateStart, QtCore.SIGNAL( 'finished()' ), self.__stateCollectingPieces );
        self.__stateCollectingPieces.addTransition( self.__stateCollectingPieces, QtCore.SIGNAL( 'finished()' ), self.__stateEvaluatingPieces );
        self.__stateEvaluatingPieces.addTransition( self.__stateEvaluatingPieces, QtCore.SIGNAL( 'finished()' ), self.__stateEnd );

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
    
    def getPieces( self ):
        """
        Returns the list of all the pieces.
        @return: A list of all the pieces.
        """
        return self.__pieces;
    
    def getBlobs( self ):
        """
        Returns the latest received list of blobs.
        @return: A list of blobs.
        """
        return self.__blobs;
    
    def getImage( self ):
        """
        Return the latest received image.
        @return: An image.
        """
        return self.__image;
    
    def detectPieces( self ):
        if( len( self.getPieces() ) == 0 ):
            return;
        
        if( len( self.getBlobs() ) == 0 ):
            return;
        
        # Loop over all the pieces.
        for piece in self.getPieces():
            isDetected = False;
            
            # Loop over all the blobs.
            for blob in self.getBlobs():
                rectangle = Rectangle( top = blob.top,
                                       left = blob.left,
                                       bottom = blob.bottom,
                                       right = blob.right );

                if( piece.doesIntersectsWithRectangle( rectangle ) ):
                    isDetected = True;
                    break;
                
            # Set whether the object is detected or not.
            if( isDetected ):
                piece.addStatus( Piece.DETECTED );
            else:
                piece.removeStatus( Piece.DETECTED );
            
        # Notify when all pieces have been changed.
        self.piecesChanged.emit( self.getPieces() );
        
    def onLogReceived( self, log ):
        """
        onLogReceived
        @param: data
        """
        # We only want to capture nodes from the experiment.
        if( log.name == rospy.get_name() ):
            self.__log.append( log );
            self.logChanged.emit( log );

    def onImageReceived( self, image ):
        """
        onImageReceived
        @param: data
        """
        openCVMatrix = self.__cvBridge.imgmsg_to_cv( image );
        
        # Convert from BGR to RGB.
        cv.CvtColor( openCVMatrix, openCVMatrix, cv.CV_BGR2RGB );

        # Keep a local reference to the data.
        self.__imageStringData = openCVMatrix.tostring();
        
        # Create QImage.
        self.__image = QtGui.QImage( self.__imageStringData,
                                    image.width,
                                    image.height,
                                    openCVMatrix.step,
                                    QtGui.QImage.Format_RGB888 );
 
        self.imageChanged.emit( self.__image );
        
    def onBlobsReceived( self, blobs ):
        """
        onImageReceived
        @param: blobs
        """
        self.__blobs = blobs.blobs;
        self.detectPieces();
        
        self.blobsChanged.emit( blobs );