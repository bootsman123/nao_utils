from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class SetupGUIDialog( QtGui.QDialog ):
    """
    SetupGUIDialog
    """
    def __init__( self, parent = None ):
        """
        Constructor.
        @param parent: Parent of this widget.
        """
        QtGui.QDialog.__init__( self, parent );
        
        self.__name = 'Black';
        self.__color = '(0, 0, 0)';
        
        self.setupUi( self );
    
    def setupUi( self, ColorDialog ):
        ColorDialog.setObjectName(_fromUtf8("ColorDialog"))
        ColorDialog.resize(174, 1)
        ColorDialog.setWindowTitle(QtGui.QApplication.translate("ColorDialog", "Add color...", None, QtGui.QApplication.UnicodeUTF8))
        self.ColorDialogLayout = QtGui.QVBoxLayout(ColorDialog)
        self.ColorDialogLayout.setObjectName(_fromUtf8("ColorDialogLayout"))
        self.labelName = QtGui.QLabel(ColorDialog)
        self.labelName.setText(QtGui.QApplication.translate("ColorDialog", "Name:", None, QtGui.QApplication.UnicodeUTF8))
        self.labelName.setObjectName(_fromUtf8("labelName"))
        self.ColorDialogLayout.addWidget(self.labelName)
        self.inputName = QtGui.QLineEdit(ColorDialog)
        self.inputName.setText(_fromUtf8(self.__name));
        self.inputName.setObjectName(_fromUtf8("inputName"))
        self.ColorDialogLayout.addWidget(self.inputName)
        self.labelColor = QtGui.QLabel(ColorDialog)
        self.labelColor.setText(QtGui.QApplication.translate("ColorDialog", "Color:", None, QtGui.QApplication.UnicodeUTF8))
        self.labelColor.setObjectName(_fromUtf8("labelColor"))
        self.ColorDialogLayout.addWidget(self.labelColor)
        self.inputColor = QtGui.QLineEdit(ColorDialog)
        self.inputColor.setText(_fromUtf8(self.__color))
        self.inputColor.setObjectName(_fromUtf8("inputColor"))
        self.ColorDialogLayout.addWidget(self.inputColor)
        self.buttonBox = QtGui.QDialogButtonBox(ColorDialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName(_fromUtf8("buttonBox"))
        self.ColorDialogLayout.addWidget(self.buttonBox)

        self.retranslateUi(ColorDialog)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("accepted()")), ColorDialog.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("rejected()")), ColorDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(ColorDialog)
        
    def accept( self ):
        self.__name = self.inputName.text();
        self.__color = self.inputColor.text();
        
        QtGui.QDialog.accept( self );
        
    def reject( self ):
        QtGui.QDialog.reject( self );
    
    def getName( self ):
        return self.__name;
    
    def getColor( self ):
        return self.__color;

    def retranslateUi(self, ColorDialog):
        pass
