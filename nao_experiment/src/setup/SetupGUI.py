#!/usr/bin/env python

PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;
import dynamic_reconfigure.client;
import actionlib;

import cv;
import yaml;

import sys;
import os;

from cv_bridge import CvBridge, CvBridgeError;

#from cmvision.msg import Blob, Blobs;
from sensor_msgs.msg import Image;
from nao_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal;
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState;

from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
from setup.SetupGUIImageWidget import SetupGUIImageWidget;
from setup.SetupGUIListModel import SetupGUIListModel;
from setup.SetupGUIDialog import SetupGUIDialog;

from colors.ColorBlob import ColorBlob;
from pieces.PieceRectangular import PieceRectangular;

class SetupGUI( QtGui.QMainWindow ):
    """
    SetupGUI
    """
    STATE_NONE = 0;
    STATE_CROPPING = 1;
    STATE_COLOR_PICKING = 2;
    STATE_BIN_PICKING = 3;
    
    def __init__( self ):
        """
        Constructor.
        """
        QtGui.QMainWindow.__init__( self );
        
        rospy.init_node( 'nao_experiment_setup_gui' );
        
        # Setup the interface.
        self.setupUi( self );

        # Initialize.
        self.__state = self.STATE_NONE;
        
        self.ColorListModel = SetupGUIListModel();
        self.ColorList.setModel( self.ColorListModel );
        
        self.BinListModel = SetupGUIListModel();
        self.BinList.setModel( self.BinListModel );
        
        # Set the head correctly.
        self.setupHead();
        
        # Setup OpenCV.
        self.cvBridge = CvBridge();
        
        # Create subscribers.
        self.cameraSubscriber = rospy.Subscriber( '/nao_camera_processed/image_raw', Image, self.onImageReceived );
        
        # Add handlers.
        self.ButtonImageCrop.clicked.connect( self.buttonImageCropClicked );
        self.ButtonImageReset.clicked.connect( self.buttonImageResetClicked );
        self.ButtonColorAdd.clicked.connect( self.buttonColorAddClicked );
        self.ButtonColorDelete.clicked.connect( self.buttonColorDeleteClicked );
        self.ButtonBinAdd.clicked.connect( self.buttonBinAddClicked );
        self.ButtonBinDelete.clicked.connect( self.buttonBinDeleteClicked );
        self.ActionSave.triggered.connect( self.actionSaveTriggered );
        self.ActionQuit.triggered.connect( self.actionQuitTriggered );
        
        # Fugly, but it works for now.
        self.ImageWidget.mousePressEvent = self.imageWidgetMousePressEvent;
        self.ImageWidget.mouseMoveEvent = self.imageWidgetMouseMoveEvent;
        self.ImageWidget.mouseReleaseEvent = self.imageWidgetMouseReleaseEvent;
        
        # Show.
        self.show();
        
    def actionSaveTriggered( self ):
        """
        actionSaveTriggered
        """
        path = os.path.join( roslib.packages.get_pkg_dir( PACKAGE_NAME ), 'cfg' );
        
        # Save color blob file.
        if( len( self.ColorListModel.items() ) > 0 ):
            filePath = os.path.join( path, 'colors.txt' );
            
            with open( fileName, 'w' ) as file:
                colors = '';
                thresholds = '';
                
                for colorBlob in self.ColorListModel.items():
                    colors += colorBlob.formatColor() + '\n';
                    thresholds += colorBlob.formatThreshold() + '\n';
                
                # Write data.
                file.write( '[colors]\n' );
                file.write( colors );
                file.write( '\n' );
                file.write( '[thresholds]\n' );
                file.write( thresholds );
            
            rospy.loginfo( 'Saved color blob information to: {filePath}'.format( filePath = filePath ) );
        
        # Save bin file.
        if( len( self.BinListModel.items() ) > 0 ):
            filePath = os.path.join( path, 'config.yaml' );
            
            with open( filePath, 'w' ) as file:
                yaml.dump( self.BinListModel.items(), file );
            
            rospy.loginfo( 'Saved bin settings to: {filePath}'.format( filePath = filePath ) );
        
    def actionQuitTriggered( self ):
        """
        actionQuitTriggered
        """
        pass;
        
    def setupHead( self ):
        # Connect to the server.
        self.actionClient = actionlib.SimpleActionClient( 'joint_trajectory', JointTrajectoryAction );
        
        if( not self.actionClient.wait_for_server( rospy.Duration( 10 ) ) ):
            rospy.logerr( 'Unable to establish a connection to the "joint_trajectory" server.' );
            sys.exit( 1 );
            
        # Set the stiffness to the head.
        self.stiffnessPublisher = rospy.Publisher( '/joint_stiffness', JointState );
        rospy.sleep( rospy.Duration( 1 ) );
        
        self.stiffnessPublisher.publish( JointState( name = [ 'HeadYaw', 'HeadPitch' ], effort = [ 1.0, 1.0 ] ) );
        rospy.sleep( rospy.Duration( 1 ) );
        
        # Set the correct position of the head.
        goal = JointTrajectoryGoal();
        goal.trajectory.joint_names = [ 'HeadYaw', 'HeadPitch' ];
        goal.trajectory.points = [];
        goal.trajectory.points.append( JointTrajectoryPoint( time_from_start = rospy.Duration( 2 ), positions = [ 0.0, 0.5149 ] ) ); #0.5149
        
        self.actionClient.send_goal_and_wait( goal );
        
    def setStiffness( self, name = None, effort = 1.0 ):
        """
        Set the stiffness of one or more joints.
        Should be used for any joint except when full body stiffness is needed.
        """
        # One second should be enough time for the controller to hook up.
        self.stiffnessPublisher = rospy.Publisher( '/joint_stiffness', JointState );
        rospy.sleep( rospy.Duration( 1 ) );
        
        self.stiffnessPublisher.publish( JointState( name = list( name ), effort = list( effort ) ) );
        rospy.sleep( rospy.Duration( 1 ) );
        
    def handleImageCrop( self, p1, p2 ):
        # Calculate settings.
        xOffset = min( p1.x(), p2.x() );
        yOffset = min( p1.y(), p2.y() );
        width = max( p1.x(), p2.x() ) - min( p1.x(), p2.x() );
        height = max( p1.y(), p2.y() ) - min( p1.y(), p2.y() );
        
        client = dynamic_reconfigure.client.Client( 'nao_blob_detection' );
        client.update_configuration( { 'xOffset': xOffset, 'yOffset': yOffset, 'width': width, 'height': height } );
     
        self.ImageWidget.clearRectangle();
        self.reset();
        
    def handleColorAdd( self ):
        setupGUIDialog = SetupGUIDialog( self );
        if( setupGUIDialog.exec_() == QtGui.QDialog.Accepted ):
            name = setupGUIDialog.getName();
            color = setupGUIDialog.getColor();
            
            colorBlob = ColorBlob( name, color, self.__colors );
            self.ColorListModel.addItem( colorBlob );
        
        self.reset();
        
    def handleBinAdd( self ):
        # Add the bin to the list.
        objectRectangular = ObjectRectangular.fromCenter( self.__position.x(), self.__position.y() );
        self.BinListModel.addItem( objectRectangular );
        
        # Redraw.
        self.ImageWidget.drawBins( self.BinListModel );
        self.reset();
        
    def reset( self ):
        self.__state = self.STATE_NONE;
        self.ButtonImageCrop.setDown( False );
        self.ButtonColorAdd.setDown( False );
        self.ButtonBinAdd.setDown( False );
        self.StatusBar.clearMessage();
        
    def buttonImageCropClicked( self ):
        """
        buttonImageCropClicked
        """
        if( self.__state == self.STATE_NONE ):
            self.__state = self.STATE_CROPPING;
            self.__position = None;
            self.ButtonImageCrop.setDown( True );
        
    def buttonImageResetClicked( self ):
        """
        buttonImageResetClicked
        """
        # Create a service.
        client = dynamic_reconfigure.client.Client( 'nao_blob_detection' );
        client.update_configuration( { 'xOffset': 0, 'yOffset': 0, 'width': 0, 'height': 0 } );
         
        self.__state = self.STATE_NONE;
        
    def buttonColorAddClicked( self ):
        """
        buttonColorAddClicked
        """
        if( self.__state == self.STATE_NONE ):
            self.__state = self.STATE_COLOR_PICKING;
            self.__colors = [];
            self.ButtonColorAdd.setDown( True );
        elif( self.__state == self.STATE_COLOR_PICKING ):
            self.handleColorAdd();
            
    def buttonColorDeleteClicked( self ):
        """
        buttonColorDeleteClicked
        """
        # Get the selected indices.
        selectedIndices = self.ColorList.selectedIndexes();
        
        for selectedIndex in selectedIndices:
            self.ColorListModel.deleteItem( selectedIndex );
        
    def buttonBinAddClicked( self ):
        """
        buttonBinAddClicked
        """
        if( self.__state == self.STATE_NONE ):
            self.__state = self.STATE_BIN_PICKING;
            self.__position = None;
            self.ButtonBinAdd.setDown( True );
        elif( self.__state == self.STATE_BIN_PICKING ):
            self.handleBinAdd();
            
    def buttonBinDeleteClicked( self ):
        """
        buttonBinDeleteClicked
        """
        # Get the selected indices.
        selectedIndices = self.BinList.selectedIndexes();
        
        for selectedIndex in selectedIndices:
            self.BinListModel.deleteItem( selectedIndex );
        
        # Redraw.
        self.ImageWidget.drawBins( self.BinListModel );
        
    def imageWidgetMousePressEvent( self, event ):
        """
        imageWidgetMousePressEvent
        """
        # When a right mouse button occurs, things have to be finished and then cleaned up.
        if( event.button() == QtCore.Qt.RightButton ):
            if( self.__state == self.STATE_COLOR_PICKING ):
                self.handleColorAdd();
            elif( self.__state == self.STATE_BIN_PICKING ):
                self.handleBinAdd();
            
            # Reset.
            self.reset();
            return;
        
        # None.
        if( self.__state == self.STATE_NONE ):
            pass;
        # Cropping.
        elif( self.__state == self.STATE_CROPPING ):
            self.__position = event.pos();
        # Color picking.
        elif( self.__state == self.STATE_COLOR_PICKING ):
            # Locate the color at the current position.
            rgb = cv.Get2D( self.__openCVMatrix, event.pos().y(), event.pos().x() );
            self.__colors.append( rgb );
        # Bin picking.
        elif( self.__state == self.STATE_BIN_PICKING ):
            self.__position = event.pos();
            self.handleBinAdd();
            
    def imageWidgetMouseMoveEvent( self, event ):
        """
        imageWidgetMouseMoveEvent
        """
        if( self.__state == self.STATE_CROPPING ):
            if( not( self.__position is None ) ):
                self.ImageWidget.drawRectangle( self.__position, event.pos() );
            else:
                self.StatusBar.showMessage( 'Position: ({x}, {y})'.format( x = event.pos().x(), y = event.pos().y() ) );
        elif( self.__state == self.STATE_COLOR_PICKING ):
            rgb = cv.Get2D( self.__openCVMatrix, event.pos().y(), event.pos().x() );
            self.StatusBar.showMessage( 'R:{r} G:{g} B:{b}'.format( r = rgb[ 0 ], g = rgb[ 1 ], b = rgb[ 2 ] ) );
        elif( self.__state == self.STATE_BIN_PICKING ):
            self.StatusBar.showMessage( 'Position: ({x}, {y})'.format( x = event.pos().x(), y = event.pos().y() ) );

    def imageWidgetMouseReleaseEvent( self, event ):
        """
        imageWidgetMouseReleaseEvent
        """
        if( self.__state == self.STATE_CROPPING ):
            self.handleImageCrop( self.__position, event.pos() );
            
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(996, 803)
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Experiment Setup", None, QtGui.QApplication.UnicodeUTF8))
        self.CentralWidget = QtGui.QWidget(MainWindow)
        self.CentralWidget.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.CentralWidget.setObjectName(_fromUtf8("CentralWidget"))
        self.CentralWidgetLayout = QtGui.QHBoxLayout(self.CentralWidget)
        self.CentralWidgetLayout.setObjectName(_fromUtf8("CentralWidgetLayout"))
        self.MenuWidget = QtGui.QWidget(self.CentralWidget)
        self.MenuWidget.setMaximumSize(QtCore.QSize(220, 16777215))
        self.MenuWidget.setObjectName(_fromUtf8("MenuWidget"))
        self.MenuWidgetLayout = QtGui.QVBoxLayout(self.MenuWidget)
        self.MenuWidgetLayout.setMargin(0)
        self.MenuWidgetLayout.setObjectName(_fromUtf8("MenuWidgetLayout"))
        self.ImageSettingsWidget = QtGui.QWidget(self.MenuWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ImageSettingsWidget.sizePolicy().hasHeightForWidth())
        self.ImageSettingsWidget.setSizePolicy(sizePolicy)
        self.ImageSettingsWidget.setObjectName(_fromUtf8("ImageSettingsWidget"))
        self.ImageSettingsWidgetLayout = QtGui.QVBoxLayout(self.ImageSettingsWidget)
        self.ImageSettingsWidgetLayout.setMargin(0)
        self.ImageSettingsWidgetLayout.setObjectName(_fromUtf8("ImageSettingsWidgetLayout"))
        self.ImageSettingsLabel = QtGui.QLabel(self.ImageSettingsWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.ImageSettingsLabel.setFont(font)
        self.ImageSettingsLabel.setText(QtGui.QApplication.translate("MainWindow", "Image settings", None, QtGui.QApplication.UnicodeUTF8))
        self.ImageSettingsLabel.setObjectName(_fromUtf8("ImageSettingsLabel"))
        self.ImageSettingsWidgetLayout.addWidget(self.ImageSettingsLabel)
        self.ButtonImageCrop = QtGui.QPushButton(self.ImageSettingsWidget)
        self.ButtonImageCrop.setText(QtGui.QApplication.translate("MainWindow", "C&rop", None, QtGui.QApplication.UnicodeUTF8))
        self.ButtonImageCrop.setObjectName(_fromUtf8("ButtonImageCrop"))
        self.ImageSettingsWidgetLayout.addWidget(self.ButtonImageCrop)
        self.ButtonImageReset = QtGui.QPushButton(self.ImageSettingsWidget)
        self.ButtonImageReset.setText(QtGui.QApplication.translate("MainWindow", "Reset", None, QtGui.QApplication.UnicodeUTF8))
        self.ButtonImageReset.setObjectName(_fromUtf8("ButtonImageReset"))
        self.ImageSettingsWidgetLayout.addWidget(self.ButtonImageReset)
        self.MenuWidgetLayout.addWidget(self.ImageSettingsWidget)
        self.ColorSettingsWidget = QtGui.QWidget(self.MenuWidget)
        self.ColorSettingsWidget.setObjectName(_fromUtf8("ColorSettingsWidget"))
        self.ColorSettingsWidgetLayout = QtGui.QVBoxLayout(self.ColorSettingsWidget)
        self.ColorSettingsWidgetLayout.setMargin(0)
        self.ColorSettingsWidgetLayout.setObjectName(_fromUtf8("ColorSettingsWidgetLayout"))
        self.ColorSettingsLabel = QtGui.QLabel(self.ColorSettingsWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.ColorSettingsLabel.setFont(font)
        self.ColorSettingsLabel.setText(QtGui.QApplication.translate("MainWindow", "Color settings", None, QtGui.QApplication.UnicodeUTF8))
        self.ColorSettingsLabel.setObjectName(_fromUtf8("ColorSettingsLabel"))
        self.ColorSettingsWidgetLayout.addWidget(self.ColorSettingsLabel)
        self.ColorList = QtGui.QListView(self.ColorSettingsWidget)
        self.ColorList.setObjectName(_fromUtf8("ColorList"))
        self.ColorSettingsWidgetLayout.addWidget(self.ColorList)
        self.ColorSettingsButtonLayout = QtGui.QHBoxLayout()
        self.ColorSettingsButtonLayout.setObjectName(_fromUtf8("ColorSettingsButtonLayout"))
        self.ButtonColorAdd = QtGui.QPushButton(self.ColorSettingsWidget)
        self.ButtonColorAdd.setText(QtGui.QApplication.translate("MainWindow", "Add &color", None, QtGui.QApplication.UnicodeUTF8))
        self.ButtonColorAdd.setObjectName(_fromUtf8("ButtonColorAdd"))
        self.ColorSettingsButtonLayout.addWidget(self.ButtonColorAdd)
        self.ButtonColorDelete = QtGui.QPushButton(self.ColorSettingsWidget)
        self.ButtonColorDelete.setText(QtGui.QApplication.translate("MainWindow", "Delete color", None, QtGui.QApplication.UnicodeUTF8))
        self.ButtonColorDelete.setObjectName(_fromUtf8("ButtonColorDelete"))
        self.ColorSettingsButtonLayout.addWidget(self.ButtonColorDelete)
        self.ColorSettingsWidgetLayout.addLayout(self.ColorSettingsButtonLayout)
        self.MenuWidgetLayout.addWidget(self.ColorSettingsWidget)
        self.BinSettingsWidget = QtGui.QWidget(self.MenuWidget)
        self.BinSettingsWidget.setObjectName(_fromUtf8("BinSettingsWidget"))
        self.BinSettingsLayout = QtGui.QVBoxLayout(self.BinSettingsWidget)
        self.BinSettingsLayout.setMargin(0)
        self.BinSettingsLayout.setObjectName(_fromUtf8("BinSettingsLayout"))
        self.BinSettingsLabel = QtGui.QLabel(self.BinSettingsWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.BinSettingsLabel.setFont(font)
        self.BinSettingsLabel.setText(QtGui.QApplication.translate("MainWindow", "Bin settings", None, QtGui.QApplication.UnicodeUTF8))
        self.BinSettingsLabel.setObjectName(_fromUtf8("BinSettingsLabel"))
        self.BinSettingsLayout.addWidget(self.BinSettingsLabel)
        self.BinList = QtGui.QListView(self.BinSettingsWidget)
        self.BinList.setObjectName(_fromUtf8("BinList"))
        self.BinSettingsLayout.addWidget(self.BinList)
        self.BinSettingsButtonLayout = QtGui.QHBoxLayout()
        self.BinSettingsButtonLayout.setObjectName(_fromUtf8("BinSettingsButtonLayout"))
        self.ButtonBinAdd = QtGui.QPushButton(self.BinSettingsWidget)
        self.ButtonBinAdd.setText(QtGui.QApplication.translate("MainWindow", "Add &bin", None, QtGui.QApplication.UnicodeUTF8))
        self.ButtonBinAdd.setObjectName(_fromUtf8("ButtonBinAdd"))
        self.BinSettingsButtonLayout.addWidget(self.ButtonBinAdd)
        self.ButtonBinDelete = QtGui.QPushButton(self.BinSettingsWidget)
        self.ButtonBinDelete.setText(QtGui.QApplication.translate("MainWindow", "Delete bin", None, QtGui.QApplication.UnicodeUTF8))
        self.ButtonBinDelete.setObjectName(_fromUtf8("ButtonBinDelete"))
        self.BinSettingsButtonLayout.addWidget(self.ButtonBinDelete)
        self.BinSettingsLayout.addLayout(self.BinSettingsButtonLayout)
        self.MenuWidgetLayout.addWidget(self.BinSettingsWidget)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.MenuWidgetLayout.addItem(spacerItem)
        self.CentralWidgetLayout.addWidget(self.MenuWidget)
        self.ImageWidget = SetupGUIImageWidget(self.CentralWidget) #QtGui.QWidget(self.CentralWidget)
        self.ImageWidget.setMouseTracking(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ImageWidget.sizePolicy().hasHeightForWidth())
        self.ImageWidget.setSizePolicy(sizePolicy)
        self.ImageWidget.setObjectName(_fromUtf8("ImageWidget"))
        self.CentralWidgetLayout.addWidget(self.ImageWidget)
        MainWindow.setCentralWidget(self.CentralWidget)
        self.MenuBar = QtGui.QMenuBar(MainWindow)
        self.MenuBar.setGeometry(QtCore.QRect(0, 0, 996, 21))
        self.MenuBar.setObjectName(_fromUtf8("MenuBar"))
        self.MenuFile = QtGui.QMenu(self.MenuBar)
        self.MenuFile.setTitle(QtGui.QApplication.translate("MainWindow", "File", None, QtGui.QApplication.UnicodeUTF8))
        self.MenuFile.setObjectName(_fromUtf8("MenuFile"))
        MainWindow.setMenuBar(self.MenuBar)
        self.StatusBar = QtGui.QStatusBar(MainWindow)
        self.StatusBar.setObjectName(_fromUtf8("StatusBar"))
        MainWindow.setStatusBar(self.StatusBar)
        self.ActionSave = QtGui.QAction(MainWindow)
        self.ActionSave.setText(QtGui.QApplication.translate("MainWindow", "Save", None, QtGui.QApplication.UnicodeUTF8))
        self.ActionSave.setShortcut(QtGui.QApplication.translate("MainWindow", "Ctrl+S", None, QtGui.QApplication.UnicodeUTF8))
        self.ActionSave.setObjectName(_fromUtf8("ActionSave"))
        self.ActionQuit = QtGui.QAction(MainWindow)
        self.ActionQuit.setText(QtGui.QApplication.translate("MainWindow", "Quit", None, QtGui.QApplication.UnicodeUTF8))
        self.ActionQuit.setShortcut(QtGui.QApplication.translate("MainWindow", "Ctrl+Q", None, QtGui.QApplication.UnicodeUTF8))
        self.ActionQuit.setObjectName(_fromUtf8("ActionQuit"))
        self.MenuFile.addAction(self.ActionSave)
        self.MenuFile.addSeparator()
        self.MenuFile.addAction(self.ActionQuit)
        self.MenuBar.addAction(self.MenuFile.menuAction())
         
    def onImageReceived( self, data ):
        """
        onImageReceived
        @param: data
        """
        '''
        openCVMatrix = self.cvBridge.imgmsg_to_cv( data, 'rgb8' );
        #qtImage = QtGui.QImage( openCVMatrix.tostring(), data.width, data.height, QtGui.QImage.Format_RGB888 );
        qtPixmap = QtGui.QPixmap();
        qtPixmap.loadFromData( openCVMatrix.tostring() );
        
        self.ImageWidget.setMinimumSize( data.width, data.height );
        self.ImageWidget.resize( data.width, data.height );
        self.ImageWidget.drawImage( qtPixmap );
        '''
        
        self.__openCVMatrix = self.cvBridge.imgmsg_to_cv( data, 'rgb8' );
        qImage = QtGui.QImage( self.__openCVMatrix.tostring(), data.width, data.height, self.__openCVMatrix.step, QtGui.QImage.Format_RGB888 );
        
        self.ImageWidget.setMinimumSize( data.width, data.height );
        self.ImageWidget.resize( data.width, data.height );
        self.ImageWidget.drawImage( qImage );
        
if( __name__ == '__main__' ):
    App = QtGui.QApplication( sys.argv );
    SetupGUI = SetupGUI();
    
    sys.exit( App.exec_() );