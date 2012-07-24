from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

class Worker( QtCore.QObject ):
    """
    Worker
    """
    run = QtCore.pyqtSignal();
    finished = QtCore.pyqtSignal();
    
    def __init__( self, state ):
        QtCore.QObject.__init__( self );
        
        self.__state = state;
    
    @QtCore.pyqtSlot()
    def run( self ):
        pass;
    
    def isPaused( self ):
        return self.__state.isPaused();