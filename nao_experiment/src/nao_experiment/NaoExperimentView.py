from rosgraph_msgs.msg import Log;

from PyQt4 import QtCore, QtGui
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
from resources import resources;
    
from nao_experiment.widgets.NaoExperimentWidgetImage import NaoExperimentWidgetImage;

from datetime import datetime;

#http://rafaelbarreto.com/2011/08/27/a-pyqt-widget-for-opencv-camera-preview/

class NaoExperimentView( QtGui.QMainWindow ):
    def __init__( self ):
        QtGui.QMainWindow.__init__( self );
        
        # Setup the interface.
        self.setupUi( self );
        
        self.ButtonStart.clicked.connect( self.onButtonStartClicked );
        self.ButtonPauseResume.toggled.connect( self.onButtonPauseResumeToggled );
        self.ButtonStop.clicked.connect( self.onButtonStopClicked );

        # Show.
        self.show();
        
    @QtCore.pyqtSlot( Log )
    def onLogChanged( self, log ):
        # Determine the color.
        color = '#000000';
        
        if( log.level is log.DEBUG ):
            color = '#0000FF'; 
        elif( log.level is log.INFO ):
            pass;
        elif( log.level is log.WARN ):
            color = '#FF8000';
        elif( log.level is log.ERROR ):
            color = '#FF0000';
        elif( log.level is log.FATAL ):
            color = '#00FF00';
    
        # Create a HTML string.
        html = '<span style="color:{color};">[{date}]: {message}</span>'.format( color = color,
                                                                           date = datetime.fromtimestamp( log.header.stamp.secs ),
                                                                           message = log.msg );
        self.TextEditLog.append( html );
        #http://stackoverflow.com/questions/3120258/qtextedit-inserthtml-is-very-slow

    def onButtonStartClicked( self ):
        if( self.ButtonStart.isEnabled() ):
            self.ButtonStart.setEnabled( False );
            self.ButtonPauseResume.setEnabled( True );
            self.ButtonStop.setEnabled( True );

    def onButtonPauseResumeToggled( self, checked ):
        if( checked ):
            self.ButtonPauseResume.setText( _fromUtf8( 'Resume' ) );
        else:
            self.ButtonPauseResume.setText( _fromUtf8( 'Pause' ) );
            
    def onButtonStopClicked( self ):
        if( self.ButtonStop.isEnabled() ):
            self.ButtonStart.setEnabled( True );
            self.ButtonPauseResume.setEnabled( False );
            self.ButtonPauseResume.setText( _fromUtf8( 'Pause' ) );
            self.ButtonStop.setEnabled( False );

    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1024, 768)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Nao Experiment", None, QtGui.QApplication.UnicodeUTF8))
        self.WidgetContainer = QtGui.QWidget(MainWindow)
        self.WidgetContainer.setObjectName(_fromUtf8("WidgetContainer"))
        self.WidgetContainerLayout = QtGui.QVBoxLayout(self.WidgetContainer)
        self.WidgetContainerLayout.setSpacing(6)
        self.WidgetContainerLayout.setMargin(0)
        self.WidgetContainerLayout.setObjectName(_fromUtf8("WidgetContainerLayout"))
        self.WidgetButtons = QtGui.QWidget(self.WidgetContainer)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WidgetButtons.sizePolicy().hasHeightForWidth())
        self.WidgetButtons.setSizePolicy(sizePolicy)
        self.WidgetButtons.setObjectName(_fromUtf8("WidgetButtons"))
        self.WidgetButtonsLayout = QtGui.QHBoxLayout(self.WidgetButtons)
        self.WidgetButtonsLayout.setMargin(0)
        self.WidgetButtonsLayout.setObjectName(_fromUtf8("WidgetButtonsLayout"))
        self.LabelExperiment = QtGui.QLabel(self.WidgetButtons)
        self.LabelExperiment.setText(QtGui.QApplication.translate("MainWindow", "Experiment:", None, QtGui.QApplication.UnicodeUTF8))
        self.LabelExperiment.setObjectName(_fromUtf8("LabelExperiment"))
        self.WidgetButtonsLayout.addWidget(self.LabelExperiment)
        self.ButtonStart = QtGui.QPushButton(self.WidgetButtons)
        self.ButtonStart.setText(QtGui.QApplication.translate("MainWindow", "Start", None, QtGui.QApplication.UnicodeUTF8))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/control.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonStart.setIcon(icon)
        self.ButtonStart.setObjectName(_fromUtf8("ButtonStart"))
        self.WidgetButtonsLayout.addWidget(self.ButtonStart)
        self.ButtonPauseResume = QtGui.QPushButton(self.WidgetButtons)
        self.ButtonPauseResume.setCheckable(True)
        self.ButtonPauseResume.setEnabled(False)
        self.ButtonPauseResume.setText(QtGui.QApplication.translate("MainWindow", "Pause", None, QtGui.QApplication.UnicodeUTF8))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/control-pause.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonPauseResume.setIcon(icon1)
        self.ButtonPauseResume.setObjectName(_fromUtf8("ButtonPauseResume"))
        self.WidgetButtonsLayout.addWidget(self.ButtonPauseResume)
        self.ButtonStop = QtGui.QPushButton(self.WidgetButtons)
        self.ButtonStop.setEnabled(False)
        self.ButtonStop.setText(QtGui.QApplication.translate("MainWindow", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/control-stop-square.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonStop.setIcon(icon2)
        self.ButtonStop.setObjectName(_fromUtf8("ButtonStop"))
        self.WidgetButtonsLayout.addWidget(self.ButtonStop)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.WidgetButtonsLayout.addItem(spacerItem)
        self.ButtonTools = QtGui.QPushButton(self.WidgetButtons)
        self.ButtonTools.setCheckable(True)
        self.ButtonTools.setText(QtGui.QApplication.translate("MainWindow", "Enable Edit Mode", None, QtGui.QApplication.UnicodeUTF8))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/toolbox.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonTools.setIcon(icon3)
        self.ButtonTools.setObjectName(_fromUtf8("ButtonTools"))
        self.WidgetButtonsLayout.addWidget(self.ButtonTools)
        self.WidgetContainerLayout.addWidget(self.WidgetButtons)
        self.WidgetMain = QtGui.QWidget(self.WidgetContainer)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WidgetMain.sizePolicy().hasHeightForWidth())
        self.WidgetMain.setSizePolicy(sizePolicy)
        self.WidgetMain.setObjectName(_fromUtf8("WidgetMain"))
        self.WidgetMainLayout = QtGui.QVBoxLayout(self.WidgetMain)
        self.WidgetMainLayout.setMargin(0)
        self.WidgetMainLayout.setMargin(0)
        self.WidgetMainLayout.setObjectName(_fromUtf8("WidgetMainLayout"))
        self.WidgetImage = NaoExperimentWidgetImage(self.WidgetMain)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WidgetImage.sizePolicy().hasHeightForWidth())
        self.WidgetImage.setSizePolicy(sizePolicy)
        self.WidgetImage.setObjectName(_fromUtf8("WidgetImage"))
        self.WidgetMainLayout.addWidget(self.WidgetImage)
        self.WidgetLog = QtGui.QWidget(self.WidgetMain)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WidgetLog.sizePolicy().hasHeightForWidth())
        self.WidgetLog.setSizePolicy(sizePolicy)
        self.WidgetLog.setObjectName(_fromUtf8("WidgetLog"))
        self.WidgetLogLayout = QtGui.QVBoxLayout(self.WidgetLog)
        self.WidgetLogLayout.setSpacing(0)
        self.WidgetLogLayout.setContentsMargins(0, 9, 0, 0)
        self.WidgetLogLayout.setObjectName(_fromUtf8("WidgetLogLayout"))
        self.WidgetLogOptions = QtGui.QWidget(self.WidgetLog)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WidgetLogOptions.sizePolicy().hasHeightForWidth())
        self.WidgetLogOptions.setSizePolicy(sizePolicy)
        self.WidgetLogOptions.setObjectName(_fromUtf8("WidgetLogOptions"))
        self.WidgetLogOptionsLayout = QtGui.QHBoxLayout(self.WidgetLogOptions)
        self.WidgetLogOptionsLayout.setContentsMargins(-1, 0, -1, -1)
        self.WidgetLogOptionsLayout.setObjectName(_fromUtf8("WidgetLogOptionsLayout"))
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.WidgetLogOptionsLayout.addItem(spacerItem1)
        self.LabelLogType = QtGui.QLabel(self.WidgetLogOptions)
        self.LabelLogType.setText(QtGui.QApplication.translate("MainWindow", "Log Type:", None, QtGui.QApplication.UnicodeUTF8))
        self.LabelLogType.setObjectName(_fromUtf8("LabelLogType"))
        self.WidgetLogOptionsLayout.addWidget(self.LabelLogType)
        self.ComboBoxLogType = QtGui.QComboBox(self.WidgetLogOptions)
        self.ComboBoxLogType.setObjectName(_fromUtf8("ComboBoxLogType"))
        self.ComboBoxLogType.addItem(_fromUtf8(""))
        self.ComboBoxLogType.setItemText(0, QtGui.QApplication.translate("MainWindow", "Any", None, QtGui.QApplication.UnicodeUTF8))
        self.ComboBoxLogType.addItem(_fromUtf8(""))
        self.ComboBoxLogType.setItemText(1, QtGui.QApplication.translate("MainWindow", "Debug", None, QtGui.QApplication.UnicodeUTF8))
        self.ComboBoxLogType.addItem(_fromUtf8(""))
        self.ComboBoxLogType.setItemText(2, QtGui.QApplication.translate("MainWindow", "Info", None, QtGui.QApplication.UnicodeUTF8))
        self.ComboBoxLogType.addItem(_fromUtf8(""))
        self.ComboBoxLogType.setItemText(3, QtGui.QApplication.translate("MainWindow", "Warn", None, QtGui.QApplication.UnicodeUTF8))
        self.ComboBoxLogType.addItem(_fromUtf8(""))
        self.ComboBoxLogType.setItemText(4, QtGui.QApplication.translate("MainWindow", "Error", None, QtGui.QApplication.UnicodeUTF8))
        self.ComboBoxLogType.addItem(_fromUtf8(""))
        self.ComboBoxLogType.setItemText(5, QtGui.QApplication.translate("MainWindow", "Fatal", None, QtGui.QApplication.UnicodeUTF8))
        self.WidgetLogOptionsLayout.addWidget(self.ComboBoxLogType)
        self.WidgetLogLayout.addWidget(self.WidgetLogOptions)
        self.TextEditLog = QtGui.QTextEdit(self.WidgetLog)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.TextEditLog.sizePolicy().hasHeightForWidth())
        self.TextEditLog.setSizePolicy(sizePolicy)
        self.TextEditLog.setFocusPolicy(QtCore.Qt.NoFocus)
        self.TextEditLog.setLineWrapMode(QtGui.QTextEdit.NoWrap)
        self.TextEditLog.setTextInteractionFlags(QtCore.Qt.TextSelectableByKeyboard|QtCore.Qt.TextSelectableByMouse)
        self.TextEditLog.setObjectName(_fromUtf8("TextEditLog"))
        self.WidgetLogLayout.addWidget(self.TextEditLog)
        self.WidgetMainLayout.addWidget(self.WidgetLog)
        self.WidgetContainerLayout.addWidget(self.WidgetMain)
        MainWindow.setCentralWidget(self.WidgetContainer)
        self.MenuBar = QtGui.QMenuBar(MainWindow)
        self.MenuBar.setGeometry(QtCore.QRect(0, 0, 1024, 21))
        self.MenuBar.setObjectName(_fromUtf8("MenuBar"))
        MainWindow.setMenuBar(self.MenuBar)
        self.StatusBar = QtGui.QStatusBar(MainWindow)
        self.StatusBar.setObjectName(_fromUtf8("StatusBar"))
        MainWindow.setStatusBar(self.StatusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        pass