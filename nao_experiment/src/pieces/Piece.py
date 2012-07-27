from PyQt4 import QtCore;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
class Piece( QtCore.QObject ):
    NONE = 1;
    DETECTED = 2;
    HANDLING = 4;
    HANDLED = 8;
    
    def __init__( self ):
        QtCore.QObject.__init__( self );
        
        self.__status = self.NONE;
        self.__color = None;

    def addStatus( self, status = 0 ):
        self.__status = self.__status | status;
        
    def removeStatus( self, status = 0 ):
        self.__status = self.__status & ~status;
        
    def changeStatus( self, statusFrom = 0, statusTo = 0 ):
        self.removeStatus( statusFrom );
        self.addStatus( statusTo );
        
    def getStatus( self ):
        return self.__status;
    
    def setColor( self, color ):
        self.__color = color;
        
    def getColor( self ):
        return self.__color;
    
    def __str__( self ):
        pass;
    
    def draw( self, painter ):
        """
        Draw the piece using the painter.
        @param painter: The painter to draw with.
        """
        pass;
    
    def doesIntersectsWithRectangle( self, rectangle ):
        pass;