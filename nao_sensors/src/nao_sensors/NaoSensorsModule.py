#!/usr/bin/env python

import roslib;
roslib.load_manifest( 'nao_sensors' );
import rospy;

from nao_core.NaoNode import NaoNode;
from naoqi import ALModule;

from nao_sensors.msg import TactileButton, BumperButton, ChestButton;

NaoSensors = None;

class NaoSensorsModule( NaoNode, ALModule ):
    """
    NaoSensorsModule
    """
    
    def __init__( self, name ):
        NaoNode.__init__( self );
        ALModule.__init__( self, name );
        
        self.name = name;
        
        # Initialize ROS node.
        rospy.init_node( 'nao_sensors' );
        
        self.memoryProxy = self.getProxy( 'ALMemory' );
        self.subscribe();
        
        # Create publishers.
        self.tactileButtonPublisher = rospy.Publisher( 'nao_sensors_tactile_button', TactileButton );
        self.bumperButtonPublisher = rospy.Publisher( 'nao_sensors_bumper_button', BumperButton );
        self.chestButtonPublisher = rospy.Publisher( 'nao_sensors_chest_button', ChestButton );
        
        # Create messages.
        self.TactileButtonMessage = TactileButton();
        self.BumperButtonMessage = BumperButton();
        self.ChestButtonMessage = ChestButton();

    def shutdown( self ):
        self.unsubscribe();
        
    def subscribe( self ):
        self.memoryProxy.subscribeToEvent( 'FrontTactilTouched', self.name, 'onTactileButtonChanged' );
        self.memoryProxy.subscribeToEvent( 'MiddleTactilTouched', self.name, 'onTactileButtonChanged' );
        self.memoryProxy.subscribeToEvent( 'RearTactilTouched', self.name, 'onTactileButtonChanged' );
        self.memoryProxy.subscribeToEvent( 'LeftBumperPressed', self.name, 'onBumperButtonChanged' );
        self.memoryProxy.subscribeToEvent( 'RightBumperPressed', self.name, 'onBumperButtonChanged' );
        self.memoryProxy.subscribeToEvent( 'ChestButtonPressed', self.name, 'onChestButtonChanged' );
    
    def unsubscribe( self ):
        self.memoryProxy.unsubscribeToEvent( 'FrontTactilTouched', self.name );
        self.memoryProxy.unsubscribeToEvent( 'MiddleTactilTouched', self.name );
        self.memoryProxy.unsubscribeToEvent( 'RearTactilTouched', self.name );
        self.memoryProxy.unsubscribeToEvent( 'LeftBumperPressed', self.name );
        self.memoryProxy.unsubscribeToEvent( 'RightBumperPressed', self.name );
        self.memoryProxy.unsubscribeToEvent( 'ChestButtonPressed', self.name );
    
    def onTactileButtonChanged( self, name, value, message ):
        """
        onTactileButtonChanged
        """
        if( name == "FrontTactilTouched" ):
            self.TactileButtonMessage.button = self.TactileButtonMessage.BUTTON_FRONT;
        elif( name == "MiddleTactilTouched" ):
            self.TactileButtonMessage.button = self.TactileButtonMessage.BUTTON_MIDDLE;
        elif( name == "RearTactilTouched" ):
            self.TactileButtonMessage.button = self.TactileButtonMessage.BUTTON_REAR;
            
        self.TactileButtonMessage.state = int( value );
        
        self.tactileButtonPublisher.publish( self.TactileButtonMessage );
        rospy.logdebug( 'onTactileButtonChanged: name={name}, value={value}, message={message}'.format( name = name, value = value, message = message ) );
    
    def onBumperButtonChanged( self, name, value, message ):
        """
        onBumperButtonChanged
        """
        if( name == "RightBumperPressed" ):
            self.BumperButtonMessage.button = self.BumperButtonMessage.BUTTON_RIGHT;
        elif( name == "LeftBumperPressed" ):
            self.BumperButtonMessage.button = self.BumperButtonMessage.BUTTON_LEFT;
        
        self.BumperButtonMessage.state = int( value );
        
        self.bumperButtonPublisher.publish( self.BumperButtonMessage );
        rospy.logdebug( 'onBumperButtonChanged: name={name}, value={value}, message={message}'.format( name = name, value = value, message = message ) );
    
        
    def onChestButtonChanged( self, name, value, message ):
        """
        onChestButtonChanged
        """
        self.ChestButtonMessage.state = int( value );
        
        self.chestButtonPublisher.publish( self.ChestButtonMessage );
        rospy.logdebug( 'onChestButtonChanged: name={name}, value={value}, message={message}'.format( name = name, value = value, message = message ) );
        
if( __name__ == '__main__' ):
    NaoSensors = NaoSensorsModule( 'NaoSensors' );
    rospy.spin();
    
    NaoSensors.shutdown();
    rospy.loginfo( 'NaoSensors has shutdown.' );