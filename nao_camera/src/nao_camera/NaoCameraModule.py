#!/usr/bin/env python

import roslib;
roslib.load_manifest( 'nao_camera' );
import rospy;

from nao_core.NaoNode import NaoNode;
import vision_definitions;

from std_msgs.msg import Header, Time;
from sensor_msgs.msg import Image, CameraInfo;

'''
From vision_definitions.py:
# Image format
k960p = 3                # 1280*960
k4VGA = 3                # 1280*960
kVGA = 2                 # 640*480
kQVGA = 1                # 320*240
kQQVGA = 0               # 160*120

# Color Space
kYuvColorSpace = 0
kyUvColorSpace = 1
kyuVColorSpace = 2
kRgbColorSpace = 3
krGbColorSpace = 4
krgBColorSpace = 5
kHsvColorSpace = 6
khSvColorSpace = 7
khsVColorSpace = 8
kYUV422InterlacedColorSpace = 9  #deprecated
kYUV422ColorSpace = 9
kYUVColorSpace = 10
kRGBColorSpace = 11
kHSVColorSpace = 12
kBGRColorSpace = 13
kYYCbCrColorSpace = 14
kH2RGBColorSpace = 15
kHSMixedColorSpace = 16
'''

class NaoCameraModule( NaoNode ):
    """
    NaoCameraModule
    """
    def __init__( self ):
        NaoNode.__init__( self );
        
        # Initialize ROS node.
        rospy.init_node( 'nao_camera' );
        
        self.__videoDeviceProxy = self.getProxy( 'ALVideoDevice' );
        
        # Get options.
        self.__options = {};
        self.__options[ 'resolution' ] = rospy.get_param( '~resolution', vision_definitions.kVGA );
        self.__options[ 'colorSpace' ] = rospy.get_param( '~color_space', vision_definitions.kBGRColorSpace );
        self.__options[ 'fps' ] = rospy.get_param( '~fps', 15 );
        self.__options[ 'camera' ] = rospy.get_param( '~camera', 0 );
        
        self.subscribe();
        
        # Create publishers.
        self.__imagePublisher = rospy.Publisher( '/nao_camera/image_raw', Image );
        self.__cameraInfoPublisher = rospy.Publisher( '/nao_camera/camera_info', CameraInfo );

        # Publish images.
        rate = rospy.Rate( self.__options[ 'fps' ] );
        
        while( not rospy.is_shutdown() ):
            self.publish();
            rate.sleep();

    def shutdown( self ):
        self.unsubscribe();
        
    def subscribe( self ):
        # Unsubscribe zombie instances.
        self.__videoDeviceProxy.unsubscribeAllInstances( 'nao_image' );
        
        # Set the camera.   
        self.__videoDeviceProxyName = self.__videoDeviceProxy.subscribe( 'nao_image', self.__options[ 'resolution' ], self.__options[ 'colorSpace' ], self.__options[ 'fps' ] );
        self.__videoDeviceProxy.setParam( vision_definitions.kCameraSelectID, self.__options[ 'camera' ] );
    
    def unsubscribe( self ):
        self.__videoDeviceProxy.unsubscribe( self.__videoDeviceProxyName );
        
    def publish( self ):
        # Get the image.
        image = self.__videoDeviceProxy.getImageRemote( self.__videoDeviceProxyName );
            
        # Create Image message.
        ImageMessage = Image();
        ImageMessage.header.stamp.secs = image[ 5 ];
        ImageMessage.width = image[ 0 ];
        ImageMessage.height = image[ 1 ];
        ImageMessage.step = image[ 2 ] * image[ 0 ];
        ImageMessage.is_bigendian = False;
        ImageMessage.encoding = 'bgr8';
        ImageMessage.data = image[ 6 ];
        
        self.__imagePublisher.publish( ImageMessage );
        
        # Create CameraInfo message.
        # Data from the calibration phase is hard coded for now.
        CameraInfoMessage = CameraInfo();
        CameraInfoMessage.header.stamp.secs = image[ 5 ];
        CameraInfoMessage.width = image[ 0 ];
        CameraInfoMessage.height = image[ 1 ];
        CameraInfoMessage.D = [ -0.0769218451517258, 0.16183180613612602, 0.0011626049774280595, 0.0018733894100460534, 0.0 ];
        CameraInfoMessage.K = [ 581.090096189648, 0.0, 341.0926325830606, 0.0, 583.0323248080421, 241.02441593704128, 0.0, 0.0, 1.0 ];
        CameraInfoMessage.R = [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ];
        CameraInfoMessage.P = [ 580.5918359588198, 0.0, 340.76398441334106, 0.0, 0.0, 582.4042541534321, 241.04182225157172, 0.0, 0.0, 0.0, 1.0, 0.0] ;
        CameraInfoMessage.distortion_model = 'plumb_bob';
        
        #CameraInfoMessage.roi.x_offset = self.__ROIXOffset;
        #CameraInfoMessage.roi.y_offset = self.__ROIYOffset;
        #CameraInfoMessage.roi.width = self.__ROIWidth;
        #CameraInfoMessage.roi.height = self.__ROIHeight;
        #CameraInfoMessage.roi.do_rectify = self.__ROIDoRectify;
        
        self.__cameraInfoPublisher.publish( CameraInfoMessage );

if( __name__ == '__main__' ):
    NaoCamera = NaoCameraModule();
    rospy.spin();
    
    NaoCamera.shutdown();
    rospy.loginfo( 'NaoCamera has shutdown.' );
