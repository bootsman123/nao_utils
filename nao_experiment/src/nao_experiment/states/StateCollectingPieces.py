PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

import os;
import yaml;
import random;

from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

from pieces.Piece import Piece;

class StateCollectingPieces( State ):
    """
    StateCollectingPieces
    """
    def __init__( self, model ):
        State.__init__( self, model = model );
        
        self.setWorker( StateCollectingPieces.StateCollectingPiecesWorker( self ) );
    
    class StateCollectingPiecesWorker( State.Worker ):
        """
        StateCollectingPiecesWorker
        """
        NUMBER_OF_COLLECTABLE_PIECES = 4;
        ACTION_INTERVAL = rospy.Duration( 10, 0 );
    
        def __init__( self, state ):
            State.Worker.__init__( self, state );
          
        def onRun( self ):
            # Pick NUMBER_OF_PIECES pieces.
            collectablePieces = random.sample( self.getModel().getPieces(), self.NUMBER_OF_COLLECTABLE_PIECES );
            currentCollectablePiece = None;
            
            lastActionTime = None;
                    
            Utils.getInstance().enableBodyStiffness();
            
            while( self.isRunning() ):
                while( self.isPaused() ):
                    rospy.sleep( self.SLEEP_TIME );
                    
                # Check if we are done.
                if( len( collectablePieces ) == 0 and 
                    currentCollectablePiece is None ):
                    break;
                
                # Select a new piece, if there is none.
                if( currentCollectablePiece is None ):
                    currentCollectablePiece = collectablePieces.pop();
                    rospy.loginfo( 'Piece X has to be collected.' );
                
                # Handle the piece.
                if( lastActionTime is None ):
                    currentCollectablePiece.changeStatus( Piece.NONE, Piece.HANDLING );
                    
                    # Look down at the pieces.
                    #Utils.getInstance().enableBodyStiffness();
                    Utils.getInstance().setBodyPose( 'headDown' );
                    #Utils.getInstance().disableBodyStiffness();
                    
                    # Tell the user to pick up the piece and point towards it.
                    rospy.loginfo( 'Telling the user to pick up piece X.' );
                    
                    # Determine to where to point.
                    # This is really fugly, but for now sufficient.
                    laneWidth = self.getModel().getImage().width() / 5;
                    x = currentCollectablePiece.x();
                    
                    bodyPose = None;
                    
                    if( x < laneWidth ):
                        bodyPose = 'laneOne';
                    elif( x < 2 * laneWidth ):
                        bodyPose = 'laneTwo';
                    elif( x < 3 * laneWidth ):
                        bodyPose = 'laneThree';
                    elif( x < 4 * laneWidth ):
                        bodyPose = 'laneFour';
                    else:
                        bodyPose = 'laneFive';

                    rospy.loginfo( bodyPose );
                    
                    # Point.
                    Utils.getInstance().setBodyPose( bodyPose );
                    Utils.getInstance().say( 'Pick up the piece.' );
                    rospy.sleep( 3 * self.SLEEP_TIME );
                    
                    # Move back to the original position.
                    Utils.getInstance().setBodyPose( 'sitWithHeadDown' );
                    rospy.sleep( 3 * self.SLEEP_TIME );
                    
                    lastActionTime = rospy.Time.now();
                    
                elif( ( lastActionTime + self.ACTION_INTERVAL ) < rospy.Time.now() ):
                    rospy.loginfo( 'The user failed to grab the object with N seconds.' );
                    Utils.getInstance().say( 'You did not pick the correct piece.' );
                    
                    rospy.sleep( 5 * self.SLEEP_TIME );
                else:
                    # Check if the user has take the object.
                    if( not( currentCollectablePiece.getStatus() & Piece.DETECTED ) ):
                        rospy.loginfo( 'Piece X has been collected.' );
                        Utils.getInstance.say( 'Well done.' );
                        
                        currentCollectablePiece.changeStatus( Piece.HANDLING, Piece.HANDLED );
                        currentCollectablePiece = None;
                        lastActionTime = None;
                        
                        rospy.sleep( 2 * self.SLEEP_TIME );

            Utils.getInstance().disableBodyStiffness();
    
            self.finished.emit();