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
        SLEEP_TIME = 1;
        
        def __init__( self, state ):
            State.Worker.__init__( self, state );
           
        def onRun( self ):
            Utils.getInstance().setBodyPose( 'headInitial' );
            
            # Sleep (will only sleep when needed).
            while( self.isPaused() ):
                rospy.sleep( self.SLEEP_TIME );
                    
            # Say the message.
            speechMessage = 'Introduction.';
            Utils.getInstance().say( speechMessage );
            
            rospy.sleep( 3 * self.SLEEP_TIME );
            
            self.finished.emit();