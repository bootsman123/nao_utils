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
        """
        StateStartWorker
        """
        SPEECH_INTERVAL = rospy.Duration( 7, 0 );
        
        def __init__( self, state ):
            State.Worker.__init__( self, state );
            
            # Initialize.
            self.__headHasBeenTouched = False;
            
        def onRun( self ):         
            # Setup subscribers.
            sensorsTactileButtonTopic = rospy.get_param( 'sensorsTopic', '/nao_sensors_tactile_button' );
            sensorsTactileButtonSubscriber = rospy.Subscriber( sensorsTactileButtonTopic, nao_sensors.msg.TactileButton, self.onTactileButtonPressed );         
            
            speechMessage = 'Hi, I am Naomi. Touch my head to start the experiment.';
            lastSpeechTime = rospy.Time.now() - self.SPEECH_INTERVAL; # So that Naomi will immediatly start talking, instead of X seconds later.
            
            # Sit down.
            Utils.getInstance().enableBodyStiffness();
            Utils.getInstance().setBodyPose( 'sit' );
            Utils.getInstance().disableBodyStiffness();
            
            while( self.isRunning() ):
                while( self.isPaused() ):
                    rospy.sleep( self.SLEEP_TIME );
                    
                # Say the message.
                if( ( lastSpeechTime + self.SPEECH_INTERVAL ) < rospy.Time.now() ):
                    Utils.getInstance().say( speechMessage );
                    rospy.loginfo( 'Said: {speechMessage}'.format( speechMessage = speechMessage ) );
                    
                    lastSpeechTime = rospy.Time.now();
                
                # Check if the head has been touched.
                if( self.__headHasBeenTouched ):
                    break;
                
                rospy.sleep( self.SLEEP_TIME );
                
            # Clean up.
            sensorsTactileButtonSubscriber.unregister();
                
            self.finished.emit();
                
        def onTactileButtonPressed( self, data ):
            """
            onTactileButtonPressed
            @param: data
            """
            if( not( self.isPaused() ) ):
                self.__headHasBeenTouched = True;