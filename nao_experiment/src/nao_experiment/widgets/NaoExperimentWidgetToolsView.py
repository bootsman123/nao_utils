from PyQt4 import QtCore, QtGui;

try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
from resources import resources;

class NaoExperimentWidgetToolsView( QtGui.QDockWidget ):
    """
    NaoExperimentWidgetToolsView
    """
    def __init__( self, parent = None ):
        QtGui.QDockWidget.__init__( self, parent );
        
        self.setupUi( self );
    
    def setupUi(self, WidgetDock):
        WidgetDock.setObjectName(_fromUtf8("WidgetDock"))
        WidgetDock.resize(280, 676)
        WidgetDock.setWindowTitle(QtGui.QApplication.translate("WidgetDock", "Nao Experiment Settings", None, QtGui.QApplication.UnicodeUTF8))
        self.WidgetDockContents = QtGui.QWidget()
        self.WidgetDockContents.setObjectName(_fromUtf8("WidgetDockContents"))
        self.WidgetDockLayout = QtGui.QVBoxLayout(self.WidgetDockContents)
        self.WidgetDockLayout.setObjectName(_fromUtf8("WidgetDockLayout"))
        self.WidgetCameraSettings = QtGui.QWidget(self.WidgetDockContents)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WidgetCameraSettings.sizePolicy().hasHeightForWidth())
        self.WidgetCameraSettings.setSizePolicy(sizePolicy)
        self.WidgetCameraSettings.setObjectName(_fromUtf8("WidgetCameraSettings"))
        self.WidgetCameraSettingsLayout = QtGui.QHBoxLayout(self.WidgetCameraSettings)
        self.WidgetCameraSettingsLayout.setMargin(0)
        self.WidgetCameraSettingsLayout.setObjectName(_fromUtf8("WidgetCameraSettingsLayout"))
        self.LabelCameraSettings = QtGui.QLabel(self.WidgetCameraSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.LabelCameraSettings.sizePolicy().hasHeightForWidth())
        self.LabelCameraSettings.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.LabelCameraSettings.setFont(font)
        self.LabelCameraSettings.setText(QtGui.QApplication.translate("WidgetDock", "Camera settings", None, QtGui.QApplication.UnicodeUTF8))
        self.LabelCameraSettings.setObjectName(_fromUtf8("LabelCameraSettings"))
        self.WidgetCameraSettingsLayout.addWidget(self.LabelCameraSettings)
        self.ButtonCameraCrop = QtGui.QPushButton(self.WidgetCameraSettings)
        self.ButtonCameraCrop.setCheckable(True)
        self.ButtonCameraCrop.setText(QtGui.QApplication.translate("WidgetDock", "Crop", None, QtGui.QApplication.UnicodeUTF8))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/map-resize-actual.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonCameraCrop.setIcon(icon)
        self.ButtonCameraCrop.setObjectName(_fromUtf8("ButtonCameraCrop"))
        self.WidgetCameraSettingsLayout.addWidget(self.ButtonCameraCrop)
        self.ButtonCameraReset = QtGui.QPushButton(self.WidgetCameraSettings)
        self.ButtonCameraReset.setText(QtGui.QApplication.translate("WidgetDock", "Reset", None, QtGui.QApplication.UnicodeUTF8))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/cross-button.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonCameraReset.setIcon(icon1)
        self.ButtonCameraReset.setObjectName(_fromUtf8("ButtonCameraReset"))
        self.WidgetCameraSettingsLayout.addWidget(self.ButtonCameraReset)
        self.WidgetDockLayout.addWidget(self.WidgetCameraSettings)
        self.WidgetColorSettings = QtGui.QWidget(self.WidgetDockContents)
        self.WidgetColorSettings.setObjectName(_fromUtf8("WidgetColorSettings"))
        self.WidgetColorSettingsLayout = QtGui.QVBoxLayout(self.WidgetColorSettings)
        self.WidgetColorSettingsLayout.setMargin(0)
        self.WidgetColorSettingsLayout.setObjectName(_fromUtf8("WidgetColorSettingsLayout"))
        self.LabelColorSettings = QtGui.QLabel(self.WidgetColorSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.LabelColorSettings.sizePolicy().hasHeightForWidth())
        self.LabelColorSettings.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.LabelColorSettings.setFont(font)
        self.LabelColorSettings.setText(QtGui.QApplication.translate("WidgetDock", "Color settings", None, QtGui.QApplication.UnicodeUTF8))
        self.LabelColorSettings.setObjectName(_fromUtf8("LabelColorSettings"))
        self.WidgetColorSettingsLayout.addWidget(self.LabelColorSettings)
        self.ListColors = QtGui.QListView(self.WidgetColorSettings)
        self.ListColors.setObjectName(_fromUtf8("ListColors"))
        self.WidgetColorSettingsLayout.addWidget(self.ListColors)
        self.WidgetColorSettingsButtonLayout = QtGui.QHBoxLayout()
        self.WidgetColorSettingsButtonLayout.setObjectName(_fromUtf8("WidgetColorSettingsButtonLayout"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.WidgetColorSettingsButtonLayout.addItem(spacerItem)
        self.ButtonColorAdd = QtGui.QPushButton(self.WidgetColorSettings)
        self.ButtonColorAdd.setText(QtGui.QApplication.translate("WidgetDock", "Color", None, QtGui.QApplication.UnicodeUTF8))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/pipette.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonColorAdd.setIcon(icon2)
        self.ButtonColorAdd.setObjectName(_fromUtf8("ButtonColorAdd"))
        self.WidgetColorSettingsButtonLayout.addWidget(self.ButtonColorAdd)
        self.WidgetColorSettingsLayout.addLayout(self.WidgetColorSettingsButtonLayout)
        self.WidgetDockLayout.addWidget(self.WidgetColorSettings)
        self.WidgetShapeSettings = QtGui.QWidget(self.WidgetDockContents)
        self.WidgetShapeSettings.setObjectName(_fromUtf8("WidgetShapeSettings"))
        self.WidgetShapeSettingsLayout = QtGui.QVBoxLayout(self.WidgetShapeSettings)
        self.WidgetShapeSettingsLayout.setMargin(0)
        self.WidgetShapeSettingsLayout.setObjectName(_fromUtf8("WidgetShapeSettingsLayout"))
        self.LabelShapeSettings = QtGui.QLabel(self.WidgetShapeSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.LabelShapeSettings.sizePolicy().hasHeightForWidth())
        self.LabelShapeSettings.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.LabelShapeSettings.setFont(font)
        self.LabelShapeSettings.setText(QtGui.QApplication.translate("WidgetDock", "Shape settings", None, QtGui.QApplication.UnicodeUTF8))
        self.LabelShapeSettings.setObjectName(_fromUtf8("LabelShapeSettings"))
        self.WidgetShapeSettingsLayout.addWidget(self.LabelShapeSettings)
        self.ListShapes = QtGui.QListView(self.WidgetShapeSettings)
        self.ListShapes.setObjectName(_fromUtf8("ListShapes"))
        self.WidgetShapeSettingsLayout.addWidget(self.ListShapes)
        self.WidgetShapeSettingsButtonLayout = QtGui.QHBoxLayout()
        self.WidgetShapeSettingsButtonLayout.setObjectName(_fromUtf8("WidgetShapeSettingsButtonLayout"))
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.WidgetShapeSettingsButtonLayout.addItem(spacerItem1)
        self.ButtonShapeTriangleAdd = QtGui.QPushButton(self.WidgetShapeSettings)
        self.ButtonShapeTriangleAdd.setText(QtGui.QApplication.translate("WidgetDock", "Triangle", None, QtGui.QApplication.UnicodeUTF8))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/layer-shape-polygon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonShapeTriangleAdd.setIcon(icon3)
        self.ButtonShapeTriangleAdd.setObjectName(_fromUtf8("ButtonShapeTriangleAdd"))
        self.WidgetShapeSettingsButtonLayout.addWidget(self.ButtonShapeTriangleAdd)
        self.ButtonShapeRectangleAdd = QtGui.QPushButton(self.WidgetShapeSettings)
        self.ButtonShapeRectangleAdd.setText(QtGui.QApplication.translate("WidgetDock", "Rectangle", None, QtGui.QApplication.UnicodeUTF8))
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/layer-shape.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonShapeRectangleAdd.setIcon(icon4)
        self.ButtonShapeRectangleAdd.setObjectName(_fromUtf8("ButtonShapeRectangleAdd"))
        self.WidgetShapeSettingsButtonLayout.addWidget(self.ButtonShapeRectangleAdd)
        self.ButtonShapeEllipseAdd = QtGui.QPushButton(self.WidgetShapeSettings)
        self.ButtonShapeEllipseAdd.setText(QtGui.QApplication.translate("WidgetDock", "Ellipse", None, QtGui.QApplication.UnicodeUTF8))
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/layer-shape-ellipse.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.ButtonShapeEllipseAdd.setIcon(icon5)
        self.ButtonShapeEllipseAdd.setObjectName(_fromUtf8("ButtonShapeEllipseAdd"))
        self.WidgetShapeSettingsButtonLayout.addWidget(self.ButtonShapeEllipseAdd)
        self.WidgetShapeSettingsLayout.addLayout(self.WidgetShapeSettingsButtonLayout)
        self.WidgetDockLayout.addWidget(self.WidgetShapeSettings)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.WidgetDockLayout.addItem(spacerItem2)
        WidgetDock.setWidget(self.WidgetDockContents)

        self.retranslateUi(WidgetDock)
        QtCore.QMetaObject.connectSlotsByName(WidgetDock)

    def retranslateUi(self, WidgetDock):
        pass