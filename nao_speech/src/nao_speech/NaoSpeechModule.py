#!/usr/bin/env python

import roslib;
roslib.load_manifest( 'nao_speech' );
import rospy;

from nao_core.NaoNode import NaoNode;
from std_msgs.msg import String;

class NaoSpeechModule( NaoNode ):
    """
    NaoSpeechModule
    """
    def __init__( self ):
        NaoNode.__init__( self );
        
        # Initialize ROS node.
        rospy.init_node( 'nao_speech' );
        
        self.__textToSpeechProxy = self.getProxy( 'ALTextToSpeech' );
        
        # Create subscribers.
        self.__speechSubscriber = rospy.Subscriber( '/nao_speech', String, self.onSpeechReceived );

    def shutdown( self ):
        pass;
    
    def onSpeechReceived( self, data ):
        """
        onSpeechReceived
        """
        self.__textToSpeechProxy.post.say( data.data );
        
if( __name__ == '__main__' ):
    NaoSpeech = NaoSpeechModule();
    rospy.spin();
    
    NaoSpeech.shutdown();
    rospy.loginfo( 'NaoSpeech has shutdown.' );