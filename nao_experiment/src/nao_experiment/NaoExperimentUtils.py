PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

import sys;
from utils.Singleton import Singleton;

import actionlib;

import std_msgs.msg;
import std_srvs.srv;

from nao_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, BodyPoseAction, BodyPoseGoal;
from trajectory_msgs.msg import JointTrajectoryPoint;

class NaoExperimentUtils( Singleton ):
    """
    NaoExperimentUtils
    """
    TIMEOUT_TIME = 5;
    
    def __init__( self ):
        Singleton.__init__( self );

        # Connect to the server.
        self.__actionClientBodyPose = actionlib.SimpleActionClient( 'body_pose', BodyPoseAction );
        
        if( not( self.__actionClientBodyPose.wait_for_server( rospy.Duration( self.TIMEOUT_TIME ) ) ) ):
            rospy.logerr( 'Unable to establish a connection to the "body_pose" server.' );
            sys.exit( 1 );

        # Setup publishers
        self.__speechTopic = rospy.get_param( 'speechTopic', '/nao_speech' );
        self.__speechPublisher = rospy.Publisher( self.__speechTopic, std_msgs.msg.String );
        
    def __del__( self ):
        self.__speechPublisher.unregister();
            
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
            
    def setBodyPose( self, name = None ):
        self.__actionClientBodyPose.send_goal_and_wait( BodyPoseGoal( pose_name = name ) );
        
    def say( self, data ):
        message = std_msgs.msg.String();
        message.data = data;
        self.__speechPublisher.publish( message );
        
    '''
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
        '''