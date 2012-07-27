from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

from cmvision.msg import Blobs;
from pieces.Piece import Piece;

class NaoExperimentWidgetImage( QtGui.QWidget ):
    """
    NaoExperimentWidgetImage
    """
    def __init__( self, parent = None ):
        """
        Constructor.
        @param parent: Parent of this widget.
        """
        QtGui.QWidget.__init__( self, parent );
        
        # Initialize
        self.__image = None;
        self.__blobs = None;
        self.__pieces = [];
   
    def paintEvent( self, event ):
        self.paintImage();
        self.paintBlobs();
        self.paintPieces();
        
    def paintImage( self ):
        """
        Paint the image.
        """
        if( self.__image is None ):
            return;
        
        if( self.__image.isNull() ):
            return;

        painter = QtGui.QPainter();
        painter.begin( self );
        
        painter.drawImage( QtCore.QPoint( 0, 0 ), self.__image );
        
        painter.end();
        
    def paintBlobs( self ):
        """
        Paint all the blobs.
        """
        if( self.__blobs is None ):
            return;
        
        if( self.__blobs.blob_count == 0 ):
            return;
        
        painter = QtGui.QPainter();
        painter.begin( self );
        
        for blob in self.__blobs.blobs:
            painter.setPen( QtGui.QPen( QtGui.QColor( blob.red, blob.green, blob.blue ) ) );
            painter.drawRect( blob.left,
                              blob.top,
                              abs( blob.left - blob.right ),
                              abs( blob.top - blob.bottom ) );
                              
        painter.end();
        
    def paintPieces( self ):
        """
        Paint all the pieces.
        """
        if( len( self.__pieces ) == 0 ):
            return;

        painter = QtGui.QPainter();
        painter.begin( self );

        painter.setPen( QtGui.QColor( 255, 255, 255, alpha = 255 ) );       
        
        # Set default brushes. 
        brushNone = QtCore.Qt.NoBrush;
        brushDetected = QtGui.QBrush( QtGui.QColor( 255, 255, 255, alpha = 150 ) );
        brushHandling = QtGui.QBrush( QtGui.QColor( 255, 255, 255, alpha = 150 ), style = QtCore.Qt.Dense5Pattern );
        brushHandled = QtGui.QBrush( QtGui.QColor( 255, 255, 255, alpha = 150 ), style = QtCore.Qt.DiagCrossPattern )
        
        for piece in self.__pieces:
            if( piece.getStatus() & Piece.HANDLED ):
                painter.setBrush( brushHandled );
            elif( piece.getStatus() & Piece.HANDLING ):
                painter.setBrush( brushHandling );
            elif( piece.getStatus() & Piece.DETECTED ):
                painter.setBrush( brushDetected );
            else:
                painter.setBrush( brushNone );

            painter.drawRect( piece.x(),
                              piece.y(),
                              piece.width(),
                              piece.height() );

        painter.end();
        
    @QtCore.pyqtSlot( QtGui.QImage )
    def onImageChanged( self, image ):
        self.__image = image;
        
        self.setMinimumSize( self.__image.width(), self.__image.height() );
        self.resize( self.__image.width(), self.__image.height() );
        
        self.update();
        
    @QtCore.pyqtSlot( Blobs ) 
    def onBlobsChanged( self, blobs ):
        self.__blobs = blobs;
        self.update();
    
    @QtCore.pyqtSlot( list )
    def onPiecesChanged( self, pieces ):
        self.__pieces = pieces;
        self.update();