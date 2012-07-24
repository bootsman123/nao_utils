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
    class Worker( QtCore.QObject ):
        """
        Worker
        """
        run = QtCore.pyqtSignal();
        finished = QtCore.pyqtSignal();
        
        def __init__( self, state ):
            QtCore.QObject.__init__( self );
            
            self.setState( state );
        
        @QtCore.pyqtSlot()
        def onRun( self ):
            pass;
        
        def setState( self, state ):
            self.__state = state;
            
        def getState( self ):
            return self.__state;
        
        def getModel( self ):
            return self.getState().getModel();
        
        def isRunning( self ):
            return self.getModel().isRunning();
        
        def isPaused( self ):
            return self.getModel().isPaused();
        
    def __init__( self, parent = None, model = None ):
        QtCore.QState.__init__( self, parent );
        
        self.setModel( model );
        
        self.setWorker( None );
        self.setThread( None );
    
    def onEntry( self, event ):
        rospy.loginfo( 'Entering "{name}".'.format( name = re.sub( '(?<!^)(?=[A-Z])', ' ', self.__class__.__name__ ).upper() ) );
        
        # Create the thread and move the worker to the thread.
        self.setThread( QtCore.QThread() );
        self.getWorker().moveToThread( self.__thread );
        
        # Connect signals.
        self.getWorker().finished.connect( self.__thread.quit );
        
        self.getThread().started.connect( self.__worker.onRun );
        self.getThread().finished.connect( self.onFinished );
        self.getThread().finished.connect( self.__thread.deleteLater );
        
        # Start the thread and the worker.
        self.getThread().start();
        
    def onExit( self, event ):
        rospy.loginfo( 'Exiting "{name}".'.format( name = re.sub( '(?<!^)(?=[A-Z])', ' ', self.__class__.__name__ ).upper() ) );
      
    @QtCore.pyqtSlot()  
    def onFinished( self ):
        self.emit( QtCore.SIGNAL( 'finished()' ) );
    
    def setModel( self, model ):
        self.__model = model;
        
    def getModel( self ):
        return self.__model;
    
    def setThread( self, thread ):
        self.__thread = thread;
        
    def getThread( self ):
        return self.__thread;
    
    def setWorker( self, worker ):
        self.__worker = worker;
        
    def getWorker( self ):
        return self.__worker;