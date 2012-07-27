PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

class StateEnd( State ):
    """
    StateEnd
    """    
    def __init__( self, model ):
        State.__init__( self, model = model );
        
        self.setWorker( StateEnd.StateEndWorker( self ) );
        
    class StateEndWorker( State.Worker ):
        """
        StateEndWorker
        """
        def __init__( self, state ):
            State.Worker.__init__( self, state );
           
        def onRun( self ):
            Utils.enableBodyStiffness();
            
            Utils.getInstance().setBodyPose( 'headInitial' );
            Utils.getInstance().say( 'Thank you for the experiment.' );
            
            Utils.disableBodyStiffness();
            
            self.finished.emit();