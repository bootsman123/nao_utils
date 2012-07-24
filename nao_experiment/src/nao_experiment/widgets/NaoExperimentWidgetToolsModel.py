PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

import dynamic_reconfigure.client;

from nao_experiment.widgets.WidgetListModel import WidgetListModel;

class NaoExperimentWidgetToolsModel( object ):
    """
    NaoExperimentWidgetToolsModel
    """
    cameraCrop = QtCore.pyqtSlot();
    
    def __init__( self ):
        self.__ListColorsModel = WidgetListModel();
        self.__ListObjectsModel = WidgetListModel();
        
        self.__dynamicReconfigureClient = dynamic_reconfigure.client.Client( 'nao_blob_detection' );
        
    def cameraCrop( self, xOffset = 0, yOffset = 0, width = 0, height = 0 ):
        self.cameraCrop.emit( { 'xOffset': xOffset,
                                'yOffset': yOffset,
                                'width': width,
                                'height': height } );
        
    def cameraCrop( self, xOffset = 0, yOffset = 0, width = 0, height = 0 ):
        self.__dynamicReconfigureClient.update_configuration( { 'xOffset': xOffset,
                                                                'yOffset': yOffset,
                                                                'width': width,
                                                                'height': height } );

    def cameraReset( self ):
        self.cameraCrop();
