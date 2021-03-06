PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

class StateIntroduction( State ):
    """
    StateIntroduction
    """    
    def __init__( self, model ):
        State.__init__( self, model = model );
        
        self.setWorker( StateIntroduction.StateIntroductionWorker( self ) );
        
    class StateIntroductionWorker( State.Worker ):
        """
        StateIntroductionWorker
        """
        def __init__( self, state ):
            State.Worker.__init__( self, state );
           
        def onRun( self ):
            Utils.enableBodyStiffness();
            Utils.getInstance().setBodyPose( 'headInitial' );
            Utils.disableBodyStiffness();
            
            # Sleep if paused.
            while( self.isPaused() ):
                rospy.sleep( self.SLEEP_TIME );
                    
            # Say the message.
            Utils.getInstance().say( 'Introduction.' );
            
            rospy.sleep( 3 * self.SLEEP_TIME );
            
            self.finished.emit();