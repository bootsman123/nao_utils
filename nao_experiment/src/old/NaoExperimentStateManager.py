class NaoExperimentStateManager( object ):
    """
    NaoExperimentStateManager
    """
    def __init__( self ):
        """
        Constructor.
        """
        self.__states = [];
        
    def states( self ):
        """
        Returns a list of all the states.
        @return: A list of all the states.
        """
        return self.__states;
    
    def changeState( self, name ):
        """
        Change to the given state.
        @param state: The given state.
        """
        if( len( self.__states ) > 0 ):
            oldState = self.__states.pop();
            oldState.stop();
        
        state = name( self );
        self.__states.append( state );
        state.start();
 
    def pushState( self, state ):
        """
        Pause the current state and enter the given state.
        @param state: The given state.
        """
        if( len( self.__states ) > 0 ):
            self.__states[ -1 ].pause();
        
        self.__states.append( state );
        state.start();
 
    def popState( self ):
        """
        Exits the current state an resumes the previous one.
        """
        if( len( self.__states ) > 0 ):
            oldState = self.__states.pop();
            oldState.stop();
        
        if( len( self.__states ) > 0 ):
            self.__states[ -1 ].resume();
    