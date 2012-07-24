#!/usr/bin/env python

import roslib;
roslib.load_manifest( 'nao_experiment' );
import rospy;

#from cmvision.msg import Blob, Blobs;
from sensor_msgs.msg import Image;

from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
from bins.BinList import BinList;
from bins.RectangleBin import RectangleBin;

class NaoExperimentSetupFrame( wx.Frame ):
    TITLE = 'Experiment Setup GUI';
    
    STATE_NONE = 0;
    STATE_CROPPING = 1;
    STATE_COLOR_PICKING = 2;
    STATE_BIN_PICKING = 3;
    
    def __init__( self, parent ):
        wx.Frame.__init__( self, parent );
        
        rospy.init_node( 'experiment_setup_gui' );
        
        # Create subscribers.
        self.cameraSubscriber = rospy.Subscriber( 'nao_camera', Image, self.onImageReceived );
        
        # Setup the GUI.
        self.SetTitle( self.TITLE );
        self.SetMinSize( ( 200, 200 ) );
        self.CreateStatusBar();
        
        # Create layout.
        Sizer = wx.BoxSizer( wx.HORIZONTAL );
        
        # Create components.
        # Menu.
        self.Menu = wx.Panel( self.GetParent() );
        
        # Canvas.
        self.Canvas = wx.Bitmap( self.GetParent() );
        
        # Add components and set layout.
        Sizer.Add( self.Menu, True, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.LEFT, 20 );
        Sizer.Add( self.Canvas, True, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.LEFT, 20 );
        
        self.SetSizer( Sizer );
        
    def onImageReceived( self, data ):
        """
        onImageReceived
        @param: data
        """
        
if( __name__ == '__main__' ):
    App = wx.App();
    NaoExperimentSetupFrame = NaoExperimentSetupFrame( None );
    App.MainLoop();