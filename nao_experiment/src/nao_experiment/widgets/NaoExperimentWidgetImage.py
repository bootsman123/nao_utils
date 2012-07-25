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
        self.__qImage = None;
        self.__blobs = None;
        self.__pieces = [];
   
    def paintEvent( self, event ):
        self.paintImage();
        self.paintBlobs();
        self.paintPieces();
        
    def paintImage( self ):
        """
        paintImage
        @param: painter
        """
        if( self.__qImage is None ):
            return;

        painter = QtGui.QPainter();
        painter.begin( self );
        
        painter.drawImage( QtCore.QPoint( 0, 0 ), self.__qImage );
        
        painter.end();
        
    def paintBlobs( self ):
        """
        paintBlobs
        @param: painter
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
        paintPieces
        @param: painter
        """
        if( len( self.__pieces ) == 0 ):
            return;
        
        painter = QtGui.QPainter();
        painter.begin( self );

        painter.setPen( QtGui.QColor( 255, 255, 255, alpha = 255 ) );

        for piece in self.__pieces:
            if( piece.isDetected() ):
                painter.setBrush( QtGui.QColor( 255, 255, 255, alpha = 200 ) );
            else:
                painter.setBrush( QtCore.Qt.NoBrush );
            
            painter.drawRect( piece.x(),
                              piece.y(),
                              piece.width(),
                              piece.height() );

        painter.end();
        
    @QtCore.pyqtSlot( QtGui.QImage )
    def onImageChanged( self, qImage ):
        if( qImage.isNull() ):
            return;
        
        self.__qImage = qImage;
        
        self.setMinimumSize( qImage.width(), qImage.height() );
        self.resize( qImage.width(), qImage.height() );
        
        self.update();
        
    @QtCore.pyqtSlot( Blobs ) 
    def onBlobsChanged( self, blobs ):
        self.__blobs = blobs;
        self.update();
    
    @QtCore.pyqtSlot( list )
    def onPiecesChanged( self, pieces ):
        self.__pieces = pieces;
        self.update();
    
    '''
    @QtCore.pyqtSlot( Piece )
    def onObjectChanged( self, piece ):
        self.__pieces.append( piece );
        self.update();
    '''