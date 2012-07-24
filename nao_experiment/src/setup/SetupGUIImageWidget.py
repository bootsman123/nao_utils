from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

class SetupGUIImageWidget( QtGui.QWidget ):
    """
    SetupGUIImageWidget
    """
    RECTANGLE_COLOR = '#FF0000';
    BIN_COLOR = '#000000'; #'#FFFFFF';
    
    def __init__( self, parent = None ):
        """
        Constructor.
        @param parent: Parent of this widget.
        """
        QtGui.QWidget.__init__( self, parent );
        
        self.__image = None;
        self.__rectangle = None;
        self.__ColorListModel = None;
        self.__BinListModel = None;
        
    def paintEvent( self, event ):
        Painter = QtGui.QPainter( self );
        
        # Draw image.
        if( self.__image is not None ):
            Painter.drawImage( QtCore.QPoint( 0, 0 ), self.__image );
    
        # Draw cropping rectangle.
        if( self.__rectangle is not None ):
            Painter.setPen( QtGui.QColor( self.RECTANGLE_COLOR ) );
            Painter.drawRect( self.__rectangle );              
            
        # Draw bins.
        if( self.__BinListModel is not None ):
            Painter.setPen( QtGui.QColor( self.BIN_COLOR ) );                
            
            for b in self.__BinListModel.items():
                rect = QtCore.QRect( b.x(), b.y(), b.width(), b.height() );
                Painter.drawRect( rect );
                
    def drawImage( self, image ):
        self.__image = image;
        self.update();
    
    def drawRectangle( self, p1, p2 ):
        self.__rectangle = QtCore.QRect( p1, p2 );
        self.update();
        
    def clearRectangle( self ):
        self.__rectangle = None;
        self.update();
            
    def drawBins( self, BinListModel ):
        self.__BinListModel = BinListModel;
        self.update();