from PyQt4 import QtCore, QtGui;
try:
    _fromUtf8 = QtCore.QString.fromUtf8;
except AttributeError:
    _fromUtf8 = lambda s: s;
    
class WidgetListModel( QtCore.QAbstractListModel ):
    def __init__( self, parent = None ):
        QtCore.QAbstractListModel.__init__( self, parent );
        
        self.__items = [];
        
    def addItem( self, item ):
        self.beginInsertRows( QtCore.QModelIndex(), self.rowCount(), self.rowCount() );
        self.__items.append( item );
        self.endInsertRows();
        
    def deleteItem( self, index ):
        if( index.row() < 0 or index.row() > self.rowCount() ):
            return;
        
        self.beginRemoveRows( index.parent(), index.row(), index.row() );
        self.__items.pop( index.row() );
        self.endRemoveRows();
        
    def items( self ):
        return self.__items;
         
    def rowCount( self, index = QtCore.QModelIndex() ):
        return len( self.__items );
    
    def data( self, index, role = QtCore.Qt.DisplayRole ):
        if( index.row() < 0 or index.row() >= len( self.__items ) ):
            return QtCore.QVariant();
        elif( role == QtCore.Qt.DisplayRole ):
            return QtCore.QString( str( self.__items[ index.row() ] ) );
        else:
            return QtCore.QVariant();