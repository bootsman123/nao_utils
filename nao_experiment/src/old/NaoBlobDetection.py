#!/usr/bin/env python

PACKAGE_NAME = 'nao_blob_detection';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;
import dynamic_reconfigure.server;

import sys;

import cv;
import yaml;

from nao_core.NaoNode import NaoNode;

from cmvision.msg import Blob, Blobs;
from sensor_msgs.msg import Image;
from cv_bridge import CvBridge, CvBridgeError;
from nao_blob_detection.cfg import NaoBlobDetectionConfig;

# For cropping (Region Of Interest (ROI)) it could be interesting to take a look at:
# http://www.ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html
class NaoBlobDetectionModule( NaoNode ):
    """
    NaoBlobDetectionModule
    """
    def __init__( self ):
        NaoNode.__init__( self );
        
        # Initialize ROS node.
        rospy.init_node( PACKAGE_NAME );       
        
        # Create dynamic configure server.
        self.__configService = dynamic_reconfigure.server.Server( NaoBlobDetectionConfig, self.onConfigReceived );
        
        # Initialize
        self.initialize();
        
        # Setup OpenCV.
        self.__cvBridge = CvBridge();
        
        # Create publishers.
        self.__cameraPublisher = rospy.Publisher( self.__topicCameraPublisher, Image );
        
        # Create subscribers.
        self.__cameraSubscriber = rospy.Subscriber( self.__topicCameraSubscriber, Image, self.onImageReceived );
        self.__blobsSubscriber = rospy.Subscriber( self.__topicBlobsSubscriber, Blobs, self.onBlobsReceived );

    def shutdown( self ):
        pass;
    
    def initialize( self ):
        if( not( rospy.has_param( '~xOffset' ) ) ):
            rospy.logerr( 'The parameter "xOffset" is not available.' );
            sys.exit( -1 );
            
        if( not( rospy.has_param( '~yOffset' ) ) ):
            rospy.logerr( 'The parameter "yOffset" is not available.' );
            sys.exit( -1 );
            
        if( not( rospy.has_param( '~width' ) ) ):
            rospy.logerr( 'The parameter "width" is not available.' );
            sys.exit( -1 );
            
        if( not( rospy.has_param( '~height' ) ) ):
            rospy.logerr( 'The parameter "height" is not available.' );
            sys.exit( -1 );
        
        # Setup defaults (could be made accessible via dynamic reconfigure).
        self.__topicCameraPublisher= '/nao_camera_processed/image_raw';
        self.__topicCameraSubscriber = '/nao_camera/image_raw';
        self.__topicBlobsSubscriber = '/blobs';
                
        # Set the default cropping rectangle (just an arbitrary value different from None and an list).
        self.__croppingRectangle = -1;
    
    def calculateCroppingRectangle( self, imageWidth, imageHeight ):
        """
        It will calculate the cropping rectangle based on the set configuration and the 
        image width and image height. If no cropping has to occur, than it will return None, 
        a rectangle other.
        
        @param imageWidth: Width of the image.
        @param imageHeight: Height of the image.
        @return: Either None if no cropping has to occur or a rectangle otherwise.
        """
        # Check if the cropping has already been calculated.
        if( not( self.__croppingRectangle == -1 ) ):
            return self.__croppingRectangle;
        
        self.__croppingRectangle = None;
        
        # If xOffset = yOffset = width = height = 0, than no cropping has to occur.
        if( self.__config[ 'xOffset' ] != 0 or
            self.__config[ 'yOffset' ] != 0 or
            self.__config[ 'width' ] != 0 or
            self.__config[ 'height' ] != 0 ):
            # Set x and y offsets.
            xOffset = self.__config[ 'xOffset' ];
            yOffset = self.__config[ 'yOffset' ];
            
            # Determine the width.
            width = imageWidth;
            
            if( self.__config[ 'width' ] != 0 ):
                width = self.__config[ 'width' ];
                
            if( width == imageWidth ):
                width -= xOffset;
                
            # Determine the height.
            height = imageHeight;
            
            if( self.__config[ 'height' ] != 0 ):
                height = self.__config[ 'height' ];
                
            if( height == imageHeight ):                
                height -= yOffset;
            
            # Set the cropping rectangle.
            self.__croppingRectangle = ( xOffset, yOffset, width, height );
            
        return self.__croppingRectangle;

    def onImageReceived( self, data ):
        """
        onImageReceived
        """
        try:
            openCVMatrix = self.__cvBridge.imgmsg_to_cv( data, 'rgb8' );
            openCVImage = cv.GetImage( openCVMatrix );
            
            # Determine cropping rectangle.
            croppingRectangle = self.calculateCroppingRectangle( openCVImage.width, openCVImage.height );
   
            # Crop the image, if needed.
            if( croppingRectangle is not None ):
                openCVImage = cv.GetSubRect( openCVImage, croppingRectangle );
            
            # Set the message.
            ImageMessage = self.__cvBridge.cv_to_imgmsg( openCVImage, 'rgb8' );
            ImageMessage.header.stamp.secs = data.header.stamp.secs;
            
            #print( 'Width: {w}'.format( ImageMessage.width ) );
            #print( 'Height: {h}'.format( ImageMessage.height ) );
                
            # Publish the image.
            self.__cameraPublisher.publish( ImageMessage );
        except CvBridgeError, e:
            rospy.logerr( 'OpenCV exception: {message}'.format( message = e ) );
        
    def onBlobsReceived( self, data ):
        """
        onBlobsReceived
        """
        print( 'Received {numberOfBlobs} blobs.'.format( numberOfBlobs = len( data.blobs ) ) );
        
    def onConfigReceived( self, config, level ):
        """
        onConfigReceived
        """
        self.__config = config;

        # Reset cropping rectangle.
        self.__croppingRectangle = -1;
        
        return config;
        

if( __name__ == '__main__' ):
    NaoBlobDetection = NaoBlobDetectionModule();
    rospy.spin();
    
    NaoBlobDetection.shutdown();
    rospy.loginfo( 'NaoBlobDetection has shutdown.' );
    
'''
/* convert to grayscale manually */
int i, j, r, g, b, byte;
for( i = 0 ; i < height ; i++ ) {
    for( j = 0 ; j < width ; j++ ) {
        r = data[i*step + j*nchannels + 0];
        g = data[i*step + j*nchannels + 1];
        b = data[i*step + j*nchannels + 2];
       
        byte = ( r + g + b ) / 3;
       
        data[i*step + j*nchannels + 0] = byte;
        data[i*step + j*nchannels + 1] = byte;
        data[i*step + j*nchannels + 2] = byte;
    }
}


            # K-means clustering.
            col = cv.Reshape( openCVImageCropped, 3, openCVImageCropped.width * openCVImageCropped.height );
            samples = cv.CreateMat( col.height, 1, cv.CV_32FC3 );
            cv.Scale( col, samples );
            labels = cv.CreateMat( col.height, 1, cv.CV_32SC1 );
            
            crit = ( cv.CV_TERMCRIT_EPS + cv.CV_TERMCRIT_ITER, 3, 1.0 );
            cv.KMeans2( samples, 128, labels, crit );
            
            clusters = {};
            
            for i in xrange( col.rows ):
                b,g,r,_ = cv.Get1D( samples, i );
                lbl,_,_,_ = cv.Get1D( labels, i );
                try:
                    clusters[lbl].append((b,g,r))
                except KeyError:
                    clusters[lbl] = [ (b,g,r) ]
                    
            means = {};
            for c in clusters:
                b,g,r = zip(*clusters[c]);
                means[c]=(sum(b)/len(b), sum(g)/len(g), sum(r)/len(r), _);
                
            for  i in xrange( col.rows ):
                lbl,_,_,_ = cv.Get1D( labels, i );
                cv.Set1D( col, i, means[ lbl ] );
                
    
        
        # The following does not work, it screws over the image.
        cv.SetImageROI( openCVImage, ( x, y, width, height ) );
        
        # Determine ROI
        width = int( openCVImage.width * self.CROPPED_WIDTH );
        height = int( openCVImage.height * self.CROPPED_HEIGHT );
        x = int( ( openCVImage.width - width ) / 2 ); # Width has to be in the middle.
        y = int( openCVImage.height - height );
        rect = ( x, y, width, height ); # cv.Rect() is not known appearently.
        
                #openCVImageCropped = cv.CreateImage( ( width, height ), openCVImage.depth, openCVImage.nChannels );
                #openCVImage = cv.GetSubRect( openCVImage, roi );
                #cv.Copy( openCVImageROIRect, openCVImage );
        
            r = cv.CreateImage( ( width, height ), cv.IPL_DEPTH_8U, 1 );
            g = cv.CreateImage( ( width, height ), cv.IPL_DEPTH_8U, 1 );
            b = cv.CreateImage( ( width, height ), cv.IPL_DEPTH_8U, 1 );
            
            cv.Split( openCVImageCropped, r, g, b, None );
            

            cv.Threshold( r, r, 128, 255, cv.CV_THRESH_BINARY );
            cv.Threshold( g, g, 128, 255, cv.CV_THRESH_BINARY );
            cv.Threshold( b, b, 150, 255, cv.CV_THRESH_BINARY );
            
            cv.Merge( r, g, b, None, openCVImageCropped );
            
            
                #openCVImage = cv.GetSubRect( openCVImage, croppingRectangle );
            
            # Process image further.
            #cv.CvtColor( openCVImageCropped, openCVImageCropped, cv.CV_RGB2HSV );
            #cv.Smooth( openCVImage, openCVImage, cv.CV_MEDIAN , 11, 11 );
            
                        
                
                openCVImageCropped = cv.CreateImage( ( croppingRectangle[ 2 ], croppingRectangle[ 3 ] ), openCVImage.depth, openCVImage.nChannels );
                cv.SetImageROI( openCVImage, croppingRectangle );
                cv.Copy( openCVImage, openCVImageCropped );
                cv.ResetImageROI( openCVImage );

'''
