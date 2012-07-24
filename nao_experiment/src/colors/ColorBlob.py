from utils.Color import Color;

class ColorBlob( object ):
    MIN_YUV = 0;
    MAX_YUV = 255;
    
    OFFSET_Y = 25;
    OFFSET_U = 25;
    OFFSET_V = 25;
    
    def __init__( self, name, color, rgbValues ):
        self.__name = name;
        self.__color = color;
        self.__rgbValues = rgbValues;
        
        # Parse the list of RGB-values.
        minR = minG = minB = 255;
        maxR = maxG = maxB = 0;
        
        # Loop over the list.
        for rgb in rgbValues:
            minR = min( rgb[ 0 ], minR );
            maxR = max( rgb[ 0 ], maxR );
            
            minG = min( rgb[ 1 ], minG );
            maxG = max( rgb[ 1 ], maxG );
            
            minB = min( rgb[ 2 ], minB );
            maxB = max( rgb[ 2 ], maxB );
            
        self.__minRgb = Color( ( minR, minG, minB ) );
        self.__maxRgb = Color( ( maxR, maxG, maxB ) );
        
    def calculateYuv( self, rgb, offset ):
        yuv = rgb.toYuv();
        
        y = max( min( yuv[ 0 ] + OFFSET, MAX_YUV ), MIN_YUV );
        u = max( min( yuv[ 1 ] + OFFSET, MAX_YUV ), MIN_YUV );
        v = max( min( yuv[ 2 ] + OFFSET, MAX_YUV ), MIN_YUV );
        
        return ( y, u, v );
        
    def formatColor( self ):
        return '{color} {merge} {expected} {name}'.format( color = self.__color,
                                                           merge = '0.000000',
                                                           expected = 1,
                                                           name = self.__name );
        
    def formatThreshold( self ):
        minYuv = self.__minRgb.toYuv();
        minY = int( round( max( min( minYuv[ 0 ] - self.OFFSET_Y, self.MAX_YUV ), self.MIN_YUV ) ) );
        minU = int( round( max( min( minYuv[ 1 ] - self.OFFSET_U, self.MAX_YUV ), self.MIN_YUV ) ) );
        minV = int( round( max( min( minYuv[ 2 ] - self.OFFSET_V, self.MAX_YUV ), self.MIN_YUV ) ) );
        
        maxYuv = self.__maxRgb.toYuv();
        maxY = int( round( max( min( maxYuv[ 0 ] + self.OFFSET_Y, self.MAX_YUV ), self.MIN_YUV ) ) );
        maxU = int( round( max( min( maxYuv[ 1 ] + self.OFFSET_U, self.MAX_YUV ), self.MIN_YUV ) ) );
        maxV = int( round( max( min( maxYuv[ 2 ] + self.OFFSET_V, self.MAX_YUV ), self.MIN_YUV ) ) );
        
        return '({minY}:{maxY}, {minU}:{maxU}, {minV}:{maxV})'.format( minY = minY,
                                                                       maxY = maxY,
                                                                       minU = minU,
                                                                       maxU = maxU,
                                                                       minV = minV,
                                                                       maxV = maxV );
    
    def __str__( self ):
        return '{name} ({n} samples)'.format( name = self.__name,
                                              n = len( self.__rgbValues ) );
                
            