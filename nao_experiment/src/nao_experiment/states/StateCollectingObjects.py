PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

import os;
import yaml;

from cmvision.msg import Blobs;

from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

from utils.Rectangle import Rectangle;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

class StateCollectingObjects( State ):
    """
    StateCollectingObjects
    """
    piecesChanged = QtCore.pyqtSignal( list );
    
    def __init__( self, model ):
        State.__init__( self, model = model );
        
    def onEntry( self, event ):
        
        self.__blobs = [];
        
        self.__blobsTopic = rospy.get_param( 'blobsTopic', '/blobs' );
        self.__blobsSubscriber = rospy.Subscriber( self.__blobsTopic, Blobs, self.onBlobsReceived );
        
        # Look down at the pieces.
        Utils.getInstance().enableBodyStiffness();
        Utils.getInstance().setBodyPose( 'headDown' );
        
        State.onEntry( self, event );
        
    def onExit( self, event ):
        self.__blobsSubscriber.unregister();
        Utils.getInstance().disableBodyStiffness();
        
        State.onExit( self, event );
          
    def onRun( self ):
        if( not( self.shouldRun() ) ):
            return;
                
        # Detect pieces.
        self.detectPieces( self.getModel().getPieces(), self.__blobs );
                    
        # Say the message.
        speechMessage = 'Collecting objects';
        #Utils.getInstance().say( speechMessage );
        
        #self.finished.emit();
        
    def detectPieces( self, pieces, blobs ):
        # Loop over all the pieces.
        for piece in pieces:
            isDetected = False;
            
            rectangle = None;
            
            # Loop over all the blobs.
            for blob in blobs:
                rectangle = Rectangle( top = blob.top,
                                       left = blob.left,
                                       bottom = blob.bottom,
                                       right = blob.right );

                if( piece.doesIntersectsWithRectangle( rectangle ) ):
                    isDetected = True;
                    break;
                
            # Set whether the object is detected or not.
            piece.setDetected( isDetected );
            
        self.piecesChanged.emit( pieces );

    @QtCore.pyqtSlot( Blobs )
    def onBlobsReceived( self, blobs ):
        self.__blobs = blobs.blobs;