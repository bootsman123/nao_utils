from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;

class NaoExperimentWidgetToolsPresenter( QtCore.QObject ):
    """
    NaoExperimentWidgetToolsPresenter
    """
    def __init__( self, parent = parent, model = model, view = view ):
        QtCore.QObject.__init__( self );
        
        self.__parent = parent;
        self.__model = model;
        self.__view = view;
        
        self.__view.setAllowedAreas( QtCore.Qt.RightDockWidgetArea );
        self.__view.setTitleBarWidget( QtGui.QWidget( self.NaoExperimentWidgetTools ) );
        
        self.__model.cameraCrop.connect( self.__parent.__view.ImageWidget );
        
        # Attach to buttons.
        self.__view.ButtonCameraCrop.toggled.connect( self.onButtonCameraCropToggled );
        self.__view.ButtonCameraReset.clicked.connect( self.onButtonCameraResetClicked );
        #self.__view.ButtonColorAdd;
        #self.__view.ButtonShapeTriangleAdd;
        #self.__view.ButtonShapeRectangleAdd;
        #self.__view.ButtonShapeEllipseAdd;
        
    def onButtonCameraCropToggled( self, checked ):
        if( checked ):
            pass;
        else:
            pass;
    
    def onButtonCameraResetClicked( self ):
        self.__model.cameraReset();
        