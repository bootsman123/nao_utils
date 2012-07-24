class Rectangle( object ):
    def __init__( self, top = 0, left = 0, bottom = 0, right = 0 ):
        self.__top = top;
        self.__left = left;
        self.__bottom = bottom;
        self.__right = right;
        
    def doesIntersects( self, rectangle ):
        return not( rectangle.__left > self.__right or 
                    rectangle.__right > self.__left or
                    rectangle.__top > self.__bottom or
                    rectangle.__bottom > self.__top );