from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

class NaoExperimentPresenter( QtCore.QObject ):
    """
    NaoExperimentPresenter
    """
    def __init__( self, model, view ):
        QtCore.QObject.__init__( self );
        self.__model = model;
        self.__view = view;
        
        self.__model.imageChanged.connect( self.__view.WidgetImage.onImageChanged );
        self.__model.blobsChanged.connect( self.__view.WidgetImage.onBlobsChanged );
        self.__model.piecesChanged.connect( self.__view.WidgetImage.onPiecesChanged );
        self.__model.logChanged.connect( self.__view.onLogChanged );

        # Attach to buttons.
        self.__view.ButtonStart.clicked.connect( self.onButtonStartClicked );
        self.__view.ButtonPauseResume.toggled.connect( self.onButtonPauseResumeToggled );
        self.__view.ButtonStop.clicked.connect( self.onButtonStopClicked );
 
    def onButtonStartClicked( self ):
        if( not( self.__model.isRunning() ) ):
            self.__model.start();
        
    def onButtonPauseResumeToggled( self, checked ):
        if( not( self.__model.isRunning() ) ):
            return;
        
        if( self.__model.isPaused() ):
            self.__model.resume();
        else:
            self.__model.pause();
        
    def onButtonStopClicked( self ):
        if( self.__model.isRunning() ):
            self.__model.stop();