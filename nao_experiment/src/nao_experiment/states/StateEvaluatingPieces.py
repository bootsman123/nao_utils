PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

class StateEvaluatingPieces( State ):
    """
    StateEvaluatingPieces
    """    
    def __init__( self, model ):
        State.__init__( self, model = model );
        
        self.setWorker( StateEvaluatingPieces.StateEvaluatingPiecesWorker( self ) );
        
    class StateEvaluatingPiecesWorker( State.Worker ):
        """
        StateEvaluatingPiecesWorker
        """
        def __init__( self, state ):
            State.Worker.__init__( self, state );
           
        def onRun( self ):
            Utils.getInstance().enableBodyStiffness();
            
            Utils.getInstance().setBodyPose( 'headInitial' );
            Utils.getInstance().say( 'Make something your like out of the pieces you just picked.' );
            
            rospy.sleep( 2 * self.SLEEP_TIME );
            
            Utils.getInstance().setBodyPose( 'headDown' );
            
            rospy.sleep( 8 * self.SLEEP_TIME );
            
            Utils.getInstance().say( 'Lets see what you have made.' );
            
            rospy.sleep( 2 * self.SLEEP_TIME );
            
            Utils.getInstance().setBodyPose( 'headInitial' );
            Utils.getInstance().say( 'It looks interesting, but I think you can do better.' );
            
            Utils.getInstance().disableBodyStiffness();
            
            self.finished.emit();