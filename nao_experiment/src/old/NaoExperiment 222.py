#!/usr/bin/env python

PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;
import actionlib;

from rospy.exceptions import ROSException, ROSInterruptException;

import os;
import sys;

import yaml;

from nao_core.NaoNode import NaoNode;
from bins.BinRectangular import BinRectangular;

from nao_sensors.msg import TactileButton;
from nao_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, BodyPoseAction, BodyPoseGoal;
from trajectory_msgs.msg import JointTrajectoryPoint;
import sensor_msgs.msg;
import std_srvs.srv;

class NaoExperiment( NaoNode ):
    def __init__( self ):
        NaoNode.__init__( self );
        
        rospy.init_node( 'nao_experiment' );

        # Connect to the server.
        self.__actionClient = actionlib.SimpleActionClient( 'joint_trajectory', JointTrajectoryAction );
        
        if( not self.__actionClient.wait_for_server( rospy.Duration( 10 ) ) ):
            rospy.logerr( 'Unable to establish a connection to the "joint_trajectory" server.' );
            sys.exit( 1 );
            
        # Subscribe to the blob detection.

        '''
        self.setStiffness( [ 'HeadYaw', 'HeadPitch', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand' ], [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ] );
        self.setPose( 'headAndArmsInitial' );
        
        self.laneOne();
        rospy.sleep( 2 );
        
        self.setPose( 'headAndArmsInitial' );
        
        self.laneTwo();
        rospy.sleep( 2 );
        
        self.setPose( 'headAndArmsInitial' );
        
        self.laneThree();
        rospy.sleep( 2 );
        
        self.setPose( 'headAndArmsInitial' );
        
        self.laneFour();
        rospy.sleep( 2 );
        
        self.setPose( 'headAndArmsInitial' );
        
        self.laneFive();
        rospy.sleep( 2 );
        
        self.setPose( 'headAndArmsInitial' );
        self.setStiffness( [ 'HeadYaw', 'HeadPitch', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand' ], [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] );
        '''
            
        # Sit down.
        #self.enableBodyStiffness();
        #self.setPose( 'sit' );
        #self.disableBodyStiffness();
        
        #print( self.getProxy( 'ALMotion' ).getSummary() );
        
        # Read in bins.
        path = os.path.join( roslib.packages.get_pkg_dir( PACKAGE_NAME ), 'cfg' );
        fileName = os.path.join( path, 'config.yaml' );
        
        file = open( fileName, 'r' );
        data = yaml.load( file );
        
        print( 'File name: {fileName}'.format( fileName = fileName ) );
        print( data );
        
    def enableBodyStiffness( self ):
        rospy.wait_for_service( '/body_stiffness/enable' );
        body_stiffness_enable = rospy.ServiceProxy( '/body_stiffness/enable', std_srvs.srv.Empty );
        body_stiffness_enable();
        
    def disableBodyStiffness( self ):
        rospy.wait_for_service( '/body_stiffness/disable' );
        body_stiffness_disable = rospy.ServiceProxy( '/body_stiffness/disable', std_srvs.srv.Empty );
        body_stiffness_disable();
        
    def setStiffness( self, name = None, effort = 1.0 ):
        """
        Set the stiffness of one or more joints.
        Should be used for any joint except when full body stiffness is needed.
        """
        # One second should be enough time for the controller to hook up.
        self.__stiffnessPublisher = rospy.Publisher( '/joint_stiffness', sensor_msgs.msg.JointState );
        rospy.sleep( rospy.Duration( 1 ) );
        
        self.__stiffnessPublisher.publish( sensor_msgs.msg.JointState( name = list( name ), effort = list( effort ) ) );
        rospy.sleep( rospy.Duration( 1 ) );
            
    def setPose( self, name = None ):
        # Connect to the server.
        client = actionlib.SimpleActionClient( 'body_pose', BodyPoseAction );
        
        if( not client.wait_for_server( rospy.Duration( 3.0 ) ) ):
            rospy.logerr( 'Unable to establish a connection to the "body_pose" server.' );
            sys.exit( 1 );
            
        client.send_goal_and_wait( BodyPoseGoal( pose_name = name ) );
    
    def laneOne( self ):
        goal = JointTrajectoryGoal();
        goal.trajectory.joint_names = [ 'HeadYaw', 'HeadPitch', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand' ];
        goal.trajectory.points = [];
        goal.trajectory.points.append( JointTrajectoryPoint( time_from_start = rospy.Duration( 2 ), positions = [ -0.250084, 0.514872, 0.529272, -0.408086, -0.061402, 0.569156, 0.194776, 1.0 ] ) );
        
        self.__actionClient.send_goal_and_wait( goal );
        
    def laneTwo( self ):
        goal = JointTrajectoryGoal();
        goal.trajectory.joint_names = [ 'HeadYaw', 'HeadPitch', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand' ];
        goal.trajectory.points = [];
        goal.trajectory.points.append( JointTrajectoryPoint( time_from_start = rospy.Duration( 2 ), positions = [ -0.204064, 0.514872, 0.507796, -0.276162, -0.265424, 0.635118, 0.271476, 1.0 ] ) );
        
        self.__actionClient.send_goal_and_wait( goal );
        
    def laneThree( self ):
        goal = JointTrajectoryGoal();
        goal.trajectory.joint_names = [ 'HeadYaw', 'HeadPitch', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand' ];
        goal.trajectory.points = [];
        goal.trajectory.points.append( JointTrajectoryPoint( time_from_start = rospy.Duration( 1.5 ), positions = [ 0.0, 0.514872, 0.566088, -0.009246, -0.378940, 0.628982, 0.351244, 1.0 ] ) );
        
        self.__actionClient.send_goal_and_wait( goal );
        
    def laneFour( self ):
        # Move to inital position.
        goal = JointTrajectoryGoal();
        goal.trajectory.joint_names = [ 'HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand' ];
        goal.trajectory.points = [];
        goal.trajectory.points.append( JointTrajectoryPoint( time_from_start = rospy.Duration( 2.0 ), positions = [ 0.090464, 0.514872, 0.549130, 0.111940, -0.001576, -0.257670, -0.061402, 1.0 ] ) );
        
        self.__actionClient.send_goal_and_wait( goal );
        
    def laneFive( self ):
        # Move to inital position.
        goal = JointTrajectoryGoal();
        goal.trajectory.joint_names = [ 'HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand' ];
        goal.trajectory.points = [];
        goal.trajectory.points.append( JointTrajectoryPoint( time_from_start = rospy.Duration( 2.0 ), positions = [ 0.207048, 0.514872, 0.584412, 0.288350, 0.007628, -0.248466, -0.061402, 1.0 ] ) );
        
        self.__actionClient.send_goal_and_wait( goal );

if( __name__ == '__main__' ):
    NaoExperiment();
    rospy.spin();