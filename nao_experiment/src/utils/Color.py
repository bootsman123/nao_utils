class Color( object ):
    def __init__( self, rgb ):
        self.__rgb = rgb;
        
    def __str__( self ):
        return '({r}, {g}, {b})'.format( r = self.__rgb[ 0 ],
                                         g = self.__rgb[ 1 ],
                                         b = self.__rgb[ 2 ] );

    def toYuv( self ):
        r = self.__rgb[ 0 ];
        g = self.__rgb[ 1 ];
        b = self.__rgb[ 2 ];
        
        y = ( 0.257 * r ) + ( 0.504 * g ) + ( 0.098 * b ) + 16;
        u = -( 0.148 * r ) - ( 0.291 * g ) + ( 0.439 * b ) + 128
        v =  ( 0.439 * r ) - ( 0.368 * g ) - ( 0.071 * b ) + 128
        
        return ( y, u, v );

'''


    # Constants to convert between RGB and YUV.
    W_R = 0.299;
    W_B = 0.144;
    W_G = 0.587;  # 1 - W_R - W_B
    U_MAX = 0.436;
    V_MAX = 0.615;

    def toYuv2( self ):
        r = self.__rgb[ 0 ];
        g = self.__rgb[ 1 ];
        b = self.__rgb[ 2 ];
        
        y = self.W_R * r + self.W_G * g + self.W_B * b;
        u = self.U_MAX * ( ( b - y ) / ( 1 - self.W_B ) );
        v = self.V_MAX * ( ( r - y ) / ( 1 - self.W_R ) );
        
        return ( y, u, v );    
'''      