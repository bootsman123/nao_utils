class Block( object ):
    TYPES = [ 'square', 'circle', 'triangle', 'trapezoid' ];
    COLOURS = [ 'red', 'green', 'blue' ];
    
    def __init__( self, type, colour, coord ):
        self._type = type;
        self._colour = colour;
        self._coord = coord;
        
    def type( self ):
        return self._type;
    
    def colour( self ):
        return self._colour;
    
    def coord( self ):
        return self._coord;
    
    def __str__( self ):
        str = '';
        
        # Colour.
        str += self.colour();
        str += ' ';
        str += self.type();
        
        return str;
        