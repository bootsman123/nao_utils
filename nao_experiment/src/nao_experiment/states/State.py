PACKAGE_NAME = 'nao_experiment';

import roslib;
roslib.load_manifest( PACKAGE_NAME );
import rospy;

from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
import re;

class State( QtCore.QState ):
    """
    State
    """
    SLEEP_TIME = 1;
    
    finished = QtCore.pyqtSignal();
    
    def __init__( self, parent = None, model = None ):
        QtCore.QState.__init__( self, parent );
        
        self.setModel( model );
    
    def onEntry( self, event ):
        rospy.loginfo( 'Entering "{name}".'.format( name = re.sub( '(?<!^)(?=[A-Z])', ' ', self.__class__.__name__ ).upper() ) );
        
        
        self.__timer = QtCore.QTimer( self );
        self.__timer.timeout.connect( self.onRun );
        self.__timer.destroyed.connect( self.onFinished );
        
        self.finished.connect( self.__timer.stop );
        self.finished.connect( self.__timer.deleteLater );
        
        self.__timer.start( 50 );
        
    def onExit( self, event ):
        rospy.loginfo( 'Exiting "{name}".'.format( name = re.sub( '(?<!^)(?=[A-Z])', ' ', self.__class__.__name__ ).upper() ) );
      
    def shouldRun( self ):
        return ( self.__timer.isActive() and
                 self.isRunning() and
                 not( self.isPaused() ) );
      
    @QtCore.pyqtSlot()  
    def onFinished( self ):
        self.emit( QtCore.SIGNAL( 'finished()' ) );
    
    def setModel( self, model ):
        self.__model = model;
        
    def getModel( self ):
        return self.__model;
    
    def isRunning( self ):
        return self.getModel().isRunning();
    
    def isPaused( self ):
        return self.getModel().isPaused();