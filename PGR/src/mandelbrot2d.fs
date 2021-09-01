#version 130

#define MReMin -2.0f
#define MReMax  1.0f
#define MImMin -1.0f
#define MImMax  1.0f

// rozmery okna
uniform vec2 window;

// centrum zoom 
uniform vec2 center;

//zoom ratio
uniform float zoom;

// maximalni pocet iteraci
uniform int maxIter;

// barva fragmentu
out vec4 fragColor;

void main() 
{ 
    //z=z*z + c //komplexni cisla
    //cx,zx, zx2 - relna cast
    //cy,zy, zy2 - imaginarni cast
	
    //actual pixel (gl_FragCoord.x) - x cord scale to x <-2, 1> / zoom + move in X
    float cx = (gl_FragCoord.x *  ((MReMax - MReMin) / window.x) + MReMin) / zoom + center.x;

    //aktulani pixel (gl_FragCoord.x) - y cord scale to y <-1, 1>  / zoom + move in Y
    float cy = (gl_FragCoord.y * ((MImMax - MImMin) / window.y) + MImMin) / zoom + center.y;
    
    float zx = 0.0f;
    float zy = 0.0f;
	
    float zx2 = 0.0f;
    float zy2 = 0.0f;

    int iter = 0;

    // escape(bailout) condition and max num. of iterations checked
    while ((zx2 + zy2 <= 4.0) && iter < maxIter) 
    {
		//calculate real part of z into temp var
        float x =( zx2 - zy2) + cx;
		//calculate imaginary part of z into temp var
        float y = (2.0 * zx * zy) + cy;

		//rewrite calculated values
        zx = x; 
        zy = y;

		//used twice, calculated once
        zx2 = zx*zx;
        zy2 = zy*zy;

        iter = iter + 1;
    }
    

	    //color by iteration. coloring algorithm. Ratios are picked randomly with correction from output
        float r = (1.0 * iter / (maxIter / 3 ));
        float g = 0.0;
        float b = 0.4-(0.4 * iter / (maxIter / 4 ));
		
		//set output collor for fragment
        fragColor = vec4((iter == maxIter ? vec3(0.0) : vec3(r, g, b)),1.0);
}



