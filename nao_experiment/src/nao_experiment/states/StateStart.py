PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

import nao_sensors.msg;

class StateStart( State ):
    """
    StateStart
    """    
    def __init__( self, model ):
        State.__init__( self, model = model );
        
        self.setWorker( StateStart.StateStartWorker( self ) );
    
    class StateStartWorker( State.Worker ):
        SLEEP_TIME = 1;
        
        def __init__( self, state ):
            State.Worker.__init__( self, state );
            
            # Initialize.
            self.__headHasBeenTouched = False;
            
        def onRun( self ):         
            # Setup subscribers.
            sensorsTactileButtonTopic = rospy.get_param( 'sensorsTopic', '/nao_sensors_tactile_button' );
            sensorsTactileButtonSubscriber = rospy.Subscriber( sensorsTactileButtonTopic, nao_sensors.msg.TactileButton, self.onTactileButtonPressed );         
            
            speechMessage = 'Start.';
            #self.__speechMessage.data = 'Hi, I am Naomi. If you want to start the experiment touch my head.';

            speechInterval = rospy.Duration( 15, 0 );
            speechTime = rospy.Time.now() - speechInterval; # So that Naomi will immediatly start talking, instead of X seconds later.
            
            # Sit down.
            #Utils.getInstance().enableBodyStiffness();
            #Utils.getInstance().setBodyPose( 'sit' );
            
            while( self.isRunning() ):
                # Sleep (will only sleep when needed).
                while( self.isPaused() ):
                    rospy.sleep( self.SLEEP_TIME );
                    
                # Say the message.
                if( ( speechTime + speechInterval ) < rospy.Time.now() ):
                    Utils.getInstance().say( speechMessage );
                    speechTime = rospy.Time.now();
                
                # Check if the head has been touched.
                if( self.__headHasBeenTouched ):
                    break;
                
                rospy.sleep( self.SLEEP_TIME );
                
            # Clean up.
            sensorsTactileButtonSubscriber.unregister();
            #Utils.getInstance().disableBodyStiffness();
                
            self.finished.emit();
                
        def onTactileButtonPressed( self, data ):
            """
            onTactileButtonPressed
            @param: data
            """
            if( not( self.isPaused() ) ):
                self.__headHasBeenTouched = True;