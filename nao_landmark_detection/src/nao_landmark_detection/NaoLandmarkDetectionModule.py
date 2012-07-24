#!/usr/bin/env python

import roslib;
roslib.load_manifest( 'nao_landmark_detection' );
import rospy;

from nao_core.NaoNode import NaoNode;
from naoqi import ALModule;

from nao_landmark_detection.msg import LandmarkDetection, Info;

NaoLandmarkDetection = None;

class NaoLandmarkDetectionModule( NaoNode, ALModule ):
    """
    NaoLandmarkDetectionModule
    """
    LANDMARK_DETECTION_PERIOD = 1000;
    LANDMARK_DETECTION_PRECISION = 1.0;
    
    def __init__( self, name ):
        NaoNode.__init__( self );
        ALModule.__init__( self, name );
        
        self.name = name;
        
        # Initialize ROS node.
        rospy.init_node( 'nao_landmark_detection' );
        
        self.memoryProxy = self.getProxy( 'ALMemory' );
        self.landmarkDetectionProxy = self.getProxy( 'ALLandMarkDetection' );
        self.subscribe();
        
        # Create publishers.
        self.landmarkDetectionPublisher = rospy.Publisher( 'nao_landmark_detection', LandmarkDetection );
        
        # Create messages.
        self.LandmarkDetectionMessage = LandmarkDetection();
        
    def shutdown( self ):
        self.unsubscribe();
    
    def subscribe( self ):
        self.landmarkDetectionProxy.subscribe( 'LandmarkDetection', self.LANDMARK_DETECTION_PERIOD, self.LANDMARK_DETECTION_PRECISION );
        self.memoryProxy.subscribeToEvent( 'LandmarkDetected', self.name, 'onLandmarkChanged' );
    
    def unsubscribe( self ):
        self.memoryProxy.unsubscribeToEvent( 'LandmarkDetected', self.name );
        self.landmarkDetectionProxy.unsubscribe( 'LandmarkDetection' );
    
    def onLandmarkChanged( self, name, value, message ):
        """
        onLandmarkChanged
        """
        # Check if a valid landmark has been detected. Sometimes this is not the case.
        if( not value ):
            return;
        
        timestamp = value[ 0 ];
        landmarks = value[ 1 ];
        # There are two more values which we do not use.
        
        # Build the message.
        self.LandmarkDetectionMessage.header.stamp.secs = timestamp[ 0 ];
        self.LandmarkDetectionMessage.header.stamp.nsecs = timestamp[ 1 ] * 1000;
        self.LandmarkDetectionMessage.info = [];
        
        for landmark in landmarks:
            info = Info();
            info.alpha = landmark[ 0 ][ 0 ];
            info.beta = landmark[ 0 ][ 1 ];
            info.sizeX = landmark[ 0 ][ 2 ];
            info.sizeY = landmark[ 0 ][ 3 ];
            info.heading = landmark[ 0 ][ 4 ];
            info.id = landmark[ 1 ][ 0 ];
            
            self.LandmarkDetectionMessage.info.append( info );
            
        self.landmarkDetectionPublisher.publish( self.LandmarkDetectionMessage );
    
if( __name__ == '__main__' ):
    NaoLandmarkDetection = NaoLandmarkDetectionModule( 'NaoLandmarkDetection' );
    rospy.spin();
    
    NaoLandmarkDetection.shutdown();
    rospy.loginfo( 'NaoLandmarkDetection has shutdown.' );