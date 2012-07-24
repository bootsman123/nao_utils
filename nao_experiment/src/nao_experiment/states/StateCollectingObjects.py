PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

import os;
import yaml;

from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

from utils.Rectangle import Rectangle;

from nao_experiment.states.State import State;
from nao_experiment.NaoExperimentUtils import NaoExperimentUtils as Utils;

class StateCollectingObjects( State ):
    """
    StateCollectingObjects
    """     
    def __init__( self, model ):
        State.__init__( self, model = model );
        
        self.setWorker( StateCollectingObjects.StateCollectingObjectsWorker( self ) );
    
    class StateCollectingObjectsWorker( State.Worker ):
        SLEEP_TIME = 1;
        
        def __init__( self, state ):
            State.Worker.__init__( self, state );
            
            self.__blobs = [];
            self.__objects = [];
          
        def onRun( self ):
            self.getState().getModel().blobsChanged.connect( self.onBlobsChanged );
            
            '''
            # Load the objects.
            configPath = os.path.join( roslib.packages.get_pkg_dir( PACKAGE_NAME ), 'cfg' );
            filePath = os.path.join( configPath, 'config.yaml' );
            
            with open( filePath ) as file:
                objectsListModel = yaml.load( file );
                self.__objects = objectsListModel.items();
                
            # Attach signal mapper.
            self.__signalMapper = QtCore.QSignalMapper();
            
            for index, item in enumerate( self.__objects ):
                #self.__signalMapper.setMapping( item, index );
                item.changed.connect( self.__signalMapper.map );
                
            #self.__signalMapper.mapped.connect( );
            '''
            
            # Look down at objects.
            #Utils.getInstance().enableBodyStiffness();
            #Utils.getInstance().setBodyPose( 'headDown' );
            
            while( self.isRunning() ):
                # Sleep (will only sleep when needed).
                while( self.isPaused() ):
                    rospy.sleep( self.SLEEP_TIME );
                    
                # Detect objects.
                self.detectObjects( self.__objects, self.__blobs );
                        
                # Say the message.
                speechMessage = 'Collecting objects';
                Utils.getInstance().say( speechMessage );
                
                rospy.sleep( 10 * self.SLEEP_TIME );
            
            #Utils.getInstance().disableBodyStiffness();
            
            self.finished.emit();
            
        def detectObjects( self, objects, blobs ):
            # Loop over all the objects.
            for o in self.__objects:
                isDetected = False;
                
                # Loop over all the blobs.
                for blob in blobs.blobs:
                    r = Rectangle( top = blob.top,
                                   left = blob.left,
                                   bottom = blob.bottom,
                                   right = blob.right );

                    if( o.doesIntersectsWithRectangle( r ) ):
                        isDetected = True;
                        break;
                    
                # Set whether the object is detected or not.
                o.isDetected( isDetected );
                rospy.loginfo( 'Detected object {o}.'.format( o = o ) );

        def onBlobsChanged( self, blobs ):
            if( self.isRunning() ):
                self.__blobs = blobs.blobs;
                
                
                #print( self.__objects );
                #rospy.loginfo( 'Gotten {c} blobs.'.format( c = blobs.blob_count ) );