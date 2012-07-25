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

        # Initialize.
        self.__headHasBeenTouched = False;
        
    def onEntry( self, event ):
        # Setup subscribers.
        self.__sensorsTactileButtonTopic = rospy.get_param( 'sensorsTopic', '/nao_sensors_tactile_button' );
        self.__sensorsTactileButtonSubscriber = rospy.Subscriber( self.__sensorsTactileButtonTopic, nao_sensors.msg.TactileButton, self.onTactileButtonPressed );         
        
        self.__speechMessage = 'Start.';
        #self.__speechMessage.data = 'Hi, I am Naomi. If you want to start the experiment touch my head.';

        self.__speechInterval = rospy.Duration( 5, 0 );
        self.__speechTime = rospy.Time.now();
        
        # Sit down.
        Utils.getInstance().enableBodyStiffness();
        Utils.getInstance().setBodyPose( 'sit' );
        
        State.onEntry( self, event );
    
    def onExit( self, event ):
        self.__sensorsTactileButtonSubscriber.unregister();
        
        Utils.getInstance().disableBodyStiffness();
        
        State.onExit( self, event );
            
    def onRun( self ):
        if( not( self.shouldRun() ) ):
            return;
                
        # Say the message.
        if( ( self.__speechTime + self.__speechInterval ) < rospy.Time.now() ):
            Utils.getInstance().say( self.__speechMessage );
            rospy.loginfo( 'Said: {speechMessage}'.format( speechMessage = self.__speechMessage ) );
            
            self.__speechTime = rospy.Time.now();
        
        # Check if the head has been touched.
        if( self.__headHasBeenTouched ):
            self.finished.emit();
            return;
            
    def onTactileButtonPressed( self, data ):
        """
        onTactileButtonPressed
        @param: data
        """
        if( not( self.isPaused() ) ):
            self.__headHasBeenTouched = True;