#include "pgr.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Data
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int maxIter = 100;
float windowWidth = 800.0f;
float windowHeight = 600.0f;
float zoom = 1.0;
float zoomX = 1.0;
int pointIndex = 0;
bool presetPoints = true;

//basic primitive
const float rectPoints[][2] = {
    { -1.0, -1.0 }, { 1.0, -1.0 }, { 1.0, 1.0},
    { 1.0, 1.0 }, { -1.0, 1.0 }, { -1.0, -1.0}
};
//c - interesting points
const float cPoints[][2] = {
    { -1.9, 0.0  },
    { -1.8, 0.0  },
    { -1.7, 0.0  },
    { -1.6, 0.0  },
    { -1.4015, 0.0  },
    { -1.4012, 0.0  },
    { -0.8, 0.0  },
    { -0.72890, 0.173532  },
    { -0.603796, 0.616074 },
    { -0.517273, 0.667849 },
    { -0.44341 , 0.668955 },
    { -0.37462 , 0.668272 },
    { -0.310137, 0.667546 },
    { -0.249673, 0.666776 },
    { -0.219701, 0.732691 },
    { -0.241047, 0.783066 },
    { -0.249673, 0.666776 },
    { -0.254955, 0.767824 },
    { -0.235296, 0.800975 },
    { -0.000093, 0.722835 },
    {  0.30093, 0.622835 },
    {  0.318, 0.422535 },
    {  0.60093, 0.3822835 },
};
const int cPointSize = 23;
GLuint pointsVBO;
GLuint positionAttrib;
GLuint maxIterUniform;
GLuint windowUniform;
GLuint zoomUniform;
GLuint centerUniform;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Shaders
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

GLuint vertexShader, fragmentShader, shaderProgram;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Event handlers
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
* Init
*/
void onInit()
{
    // Compile shaders
    fragmentShader = compileShader(GL_FRAGMENT_SHADER, loadFile("mandelbrot2d.fs").c_str());
    vertexShader = compileShader(GL_VERTEX_SHADER, loadFile("mandelbrot2d.vs").c_str());
    // Link shaders
    shaderProgram = linkShader(2, vertexShader, fragmentShader);

    positionAttrib = glGetAttribLocation(shaderProgram, "position");

    // Get location of "color" uniform
    windowUniform = glGetUniformLocation(shaderProgram, "window");
    centerUniform = glGetUniformLocation(shaderProgram, "center");
    zoomUniform = glGetUniformLocation(shaderProgram, "zoom");
    maxIterUniform = glGetUniformLocation(shaderProgram, "maxIter");
    // Copy points to graphics card
    glGenBuffers(1, &pointsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectPoints), rectPoints, GL_STATIC_DRAW);
}

// Called when window needs to be redrawn
void onWindowRedraw()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shaderProgram);

    glEnableVertexAttribArray(positionAttrib);

    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glVertexAttribPointer(positionAttrib, 2, GL_FLOAT, GL_FALSE, 0, NULL);

    //set window Width, Height
    glUniform2f(windowUniform, windowWidth, windowHeight);

    //Set center to zoom in - c points
    glUniform2fv(centerUniform, 1, cPoints[pointIndex]);
    //Set zoom ratio
    glUniform1f(zoomUniform, zoom);
    //Set max iterations
    glUniform1i(maxIterUniform, maxIter);

    // Draw selected primitive
    glDrawArrays(GL_TRIANGLES, 0, sizeof(rectPoints) / sizeof(*rectPoints));

    //continues zoom - changing step with zoomX
    zoom += (0.001f * zoomX) ;
    if(zoom > 0xFFFAFFFF)
    {
        zoom = 1.0;
        zoomX = 1.0;
        maxIter = 100;
    }
    //if set higher than 1000 iterations, than decrement
    if (maxIter < 1000)
        maxIter += 1;
    else
        maxIter -= 1;

    SDL_GL_SwapBuffers();
}
/**
 *resize okna
 */
void onWindowResized(int width, int height)
{
    glViewport(0, 0, width, height);
}
/**
 * Input keyboard
 */
void onKeyDown(SDLKey key, Uint16 /*mod*/)
{
    switch (key)
    {
        //exit
    case SDLK_ESCAPE : quit(); break;
        //iteration
    case SDLK_SPACE : zoomX = 1.0; zoom = 1.0 ;break;
    case SDLK_1 : maxIter += 10; break;
    case SDLK_2 : maxIter -= 10; break;
    case SDLK_3: maxIter += 100; break;
    case SDLK_4: maxIter -= 100; break;
        //parametr info
    case SDLK_5: 
        std::cout << "------------------------------------------------------------------------------------------ " << maxIter << std::endl;
        std::cout << "zoom : " << zoom << " zoomX : " << + zoomX * 0.001 << " maxiter : " << maxIter << std::endl;
        std::cout << "cx : " << cPoints[pointIndex][0] << " cy : " << cPoints[pointIndex][1]  << std::endl;
        std::cout <<" maxiter : " << maxIter << std::endl;
        std::cout << "------------------------------------------------------------------------------------------ " << maxIter << std::endl;
        break;
        //zoom
    case SDLK_6: zoomX *= 10.0; break;
    case SDLK_7: zoomX /= 10.0; break;
        //posun - c 
    case SDLK_8: pointIndex = (pointIndex + 1) % cPointSize; break;
    case SDLK_9:
        if (pointIndex - 1 < 0)
            pointIndex = cPointSize;
        pointIndex--;
        break;
    default : break;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//unused input handlers
void onKeyUp(SDLKey /*key*/, Uint16 /*mod*/)
{
}

void onMouseMove(unsigned /*x*/, unsigned /*y*/, int /*xrel*/, int /*yrel*/, Uint8 /*buttons*/)
{
}

void onMouseDown(Uint8 /*button*/, unsigned /*x*/, unsigned /*y*/)
{
}

void onMouseUp(Uint8 /*button*/, unsigned /*x*/, unsigned /*y*/)
{
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int /*argc*/, char ** /*argv*/)
{
    try 
    {
        // Init SDL - only video subsystem will be used
        if(SDL_Init(SDL_INIT_VIDEO) < 0) throw SDL_Exception();

        // Shutdown SDL when program ends
        atexit(SDL_Quit);

        // init SDL , glew
        init(windowWidth, windowHeight, 32, 0, 0);

        mainLoop(33); // 33ms cca 30fps
    } 
    catch(exception & ex) 
    {
        cout << "ERROR : " << ex.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}