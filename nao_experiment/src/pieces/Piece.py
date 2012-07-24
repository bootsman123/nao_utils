from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
class Piece( QtCore.QObject ):
    pass;

class Piece( QtCore.QObject ):
    changed = QtCore.pyqtSignal();
    
    def __init__( self ):
        QtCore.QObject.__init__( self );
        
        self.__isDetected = False;
        
    def setDetected( self, isDetected ):
        self.__isDetected = isDetected;
        self.changed.emit();
    
    def isDetected( self ):
        return self.__isDetected;
    
    def __str__( self ):
        pass;
    
    def doesIntersectsWithRectangle( self, rectangle ):
        pass;