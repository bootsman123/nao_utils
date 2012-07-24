class BinList( object ):
    def __init__( self ):
        """
        Constructor.
        """
        self.__bins = [];
    
    def add( self, bin ):
        """
        Add a bin to the list.
        @param bin: The bin to be added.
        """
        self.__bins.append( bin );
        
    def remove( self, bin ):
        """
        Remove a bin from the list.
        @param bin: The bin to be removed.
        """
        self.__bins.remove( bin );
        
    def get( self ):
        """
        Returns list of all the bins.
        @return: A list of all the bins.
        """
        return self.__bins;