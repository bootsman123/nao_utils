from pieces.Piece import Piece;
from utils.Rectangle import Rectangle;

class PieceRectangular( Object ):
    """
    PieceRectangular
    """
    WIDTH = 40;
    HEIGHT = 40;
    
    def __init__( self, x, y, width = WIDTH, height = HEIGHT ):
        """
        Constructor.
        @param x: X-coordinate of the top left corner of the object.
        @param y: Y-coordinate of the top left corner of the object.
        @param width: Width of the object.
        @param height: Height of the object.
        """
        Object.__init__( self );
        
        self.__x = x;
        self.__y = y;
        self.__width = width;
        self.__height = height;
        
        # Create a rectangle.
        self.__rectangle = Rectangle( top = self.__y,
                                      left = self.__x,
                                      bottom = self.__y + self.__height,
                                      right = self.__x + self.__width );
    
    @staticmethod    
    def fromCenter( x, y, width = WIDTH, height = HEIGHT ):
        """
        Returns a rectangular object from the center coordinates instead of the top left coordinates.
        @param x: X-coordinate of the center of the object.
        @param y: Y-coordinate of the center corner of the object.
        @param width: Width of the object.
        @param height: Height of the object.
        """
        topLeftX = x - width / 2;
        topLeftY = y - height / 2;
        
        return ObjectRectangular( topLeftX, topLeftY, width, height );
        
    def get( self ):
        """
        Returns the rectangle which describes the object.
        @return: The rectangle which describes the object.
        """
        return self.__rectangle;
    
    def x( self ):
        """
        Returns the x-coordinate of the object.
        @return: The x-coordinate of the object.
        """
        return self.__x;
    
    def y( self ):
        """
        Returns the y-coordinate of the object.
        @return: The y-coordinate of the object.
        """
        return self.__y;
    
    def width( self ):
        """
        Returns the width of the object.
        @return: The width of the object.
        """
        return self.__width;
    
    def height( self ):
        """
        Returns the height of the object.
        @return: The height of the object.
        """
        return self.__height;
        
    def doesIntersectsWithRectangle( self, rectangle ):
        """
        Returns true if the object intersects with another rectangle.
        @param rectangle: The rectangle to check the intersection with.
        @return: True if the object intersects with the rectangle, false if not.
        """
        return self.__rectangle.doesIntersects( rectangle );
    
    def __str__( self ):
        return '({x}, {y}) w={width},h={height}'.format( x = self.__x,
                                                         y = self.__y,
                                                         width = self.__width,
                                                         height = self.__height );
    '''
    def __repr__( self ):
        return '{className}(x={x}, y={y}, width={width}, height={height})'.format( className = self.__class__.__name__,
                                                                                   x = self.__x,
                                                                                   y = self.__y,
                                                                                   width = self.__width,
                                                                                   height = self.__height );
    '''