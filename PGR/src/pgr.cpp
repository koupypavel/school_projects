#include "pgr.h"

#include <vector>
#include <fstream>
#include <iterator>

#ifdef _MSC_VER
# pragma comment(lib, "SDL.lib")
# pragma comment(lib, "OpenGL32.lib")
# ifndef USE_GLEE
#  pragma comment(lib, "glew32.lib")
# endif//USE_GLEE
#endif//_MSC_VER

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Error handling
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const char * getGlErrorString(GLenum error)
{
#define ERROR(e) case e : return #e
    switch(error)
    {
        ERROR(GL_NO_ERROR);
        ERROR(GL_INVALID_ENUM);
        ERROR(GL_INVALID_VALUE);
        ERROR(GL_INVALID_OPERATION);
        ERROR(GL_INVALID_FRAMEBUFFER_OPERATION);
        ERROR(GL_OUT_OF_MEMORY);
    default : 
        return "UNKNOWN_GL_ERROR";
    }
#undef ERROR
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Shaders
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Load whole file and return it as std::string
std::string loadFile(const char * const file)
{
    std::ifstream stream(file);
    if(stream.fail()) throw std::runtime_error(std::string("Can't open \'") + file + "\'");
    return std::string(std::istream_iterator<char>(stream >> std::noskipws), std::istream_iterator<char>());
}

// Common shader log code
#ifndef USE_GLEE
    std::string getInfoLog(GLuint id, PFNGLGETSHADERIVPROC getLen, PFNGLGETSHADERINFOLOGPROC getLog)
#else
    std::string getInfoLog(GLuint id, GLEEPFNGLGETSHADERIVPROC getLen, GLEEPFNGLGETSHADERINFOLOGPROC getLog)
#endif//USE_GLEE
{
    assert(getLen != NULL);
    assert(getLog != NULL);

    GLint length = -1;
    getLen(id, GL_INFO_LOG_LENGTH, &length);
    assert(length >= 0);

    std::vector<GLchar> log(length+1, 0);
    getLog(id, length, NULL, &log[0]);

    assert(glGetError() == GL_NO_ERROR);

    return std::string(&log[0]);
}

GLuint compileShader(const GLenum type, const char * source)
{
    GLuint shader = glCreateShader(type);
    if(shader == 0) throw GL_Exception("glCreateShader failed");

    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    std::cout << getShaderInfoLog(shader);

    int compileStatus;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compileStatus);
    if(compileStatus == GL_FALSE) throw std::runtime_error("shader compilation failed");

    return shader;
}

GLuint linkShader(size_t count, ...)
{
    // Create program object
    GLuint program = glCreateProgram();
    if(program == 0) throw GL_Exception("glCreateProgram failed");

    // Attach arguments
    va_list args;
    va_start(args, count);
    for(size_t i = 0; i < count; ++i)
    {
        glAttachShader(program, va_arg(args, GLuint));
    }
    va_end(args);

    // Link program and check for errors
    glLinkProgram(program);
    std::cout << getProgramInfoLog(program);

    int linkStatus;
    glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
    if(linkStatus == GL_FALSE) throw std::runtime_error("shader linking failed");

    return program;
}

void SurfaceImage2D(GLenum target, GLint level, GLint internalformat, SDL_Surface * surface)
{
    glPixelStorei(GL_UNPACK_ALIGNMENT,4);
    if(     (surface->format->Rmask == 0xff0000) &&
            (surface->format->Gmask == 0xff00) &&
            (surface->format->Bmask == 0xff) &&
            (surface->format->Amask == 0))
    {
        glTexImage2D(target, level, internalformat, surface->w, surface->h, 0, GL_BGR, GL_UNSIGNED_BYTE, surface->pixels);
    }
    else if((surface->format->Rmask == 0xff) &&
            (surface->format->Gmask == 0xff00) &&
            (surface->format->Bmask == 0xff0000) &&
            (surface->format->Amask == 0))
    {
        glTexImage2D(target, level, internalformat, surface->w, surface->h, 0, GL_RGB, GL_UNSIGNED_BYTE, surface->pixels);
    }
    else
    {
        throw std::runtime_error("unsupported surface format");
    }
}

SDL_Surface * init(unsigned width, unsigned height, unsigned color, unsigned depth, unsigned stencil)
{
    // Set OpenGL attributes
    if(SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, color) < 0) throw SDL_Exception();
    if(SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, depth) < 0) throw SDL_Exception();
    if(SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, stencil) < 0) throw SDL_Exception();
    if(SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) < 0) throw SDL_Exception();

    // Create window
    SDL_Surface * screen = SDL_SetVideoMode(width, height, color, SDL_OPENGL | SDL_RESIZABLE);
    if(screen == NULL) throw SDL_Exception();
#ifndef USE_GLEE
    // Init extensions
    GLenum error = glewInit();
    if(error != GLEW_OK)
        throw std::runtime_error(std::string("GLEW : Init failed : ") + (const char*)glewGetErrorString(error));
#endif//USE_GLEE
    // Call init code
    onInit();

    onWindowResized(width, height);

    return screen;
}

// Simple main loop
void mainLoop()
{
    // Window is not minimized
    bool active = true;

    for(;;)// Infinite loop
    {
        SDL_Event event;

        // Wait for event
        if(SDL_WaitEvent(&event) == 0) throw SDL_Exception();

        // Screen needs redraw
        bool redraw = false;

        // Handle all waiting events
        do
        {
            // Call proper event handlers
            switch(event.type)
            {
                case SDL_ACTIVEEVENT :// Stop redraw when minimized
                    if(event.active.state == SDL_APPACTIVE)
                        active = event.active.gain;
                    break;
                case SDL_KEYDOWN :
                    onKeyDown(event.key.keysym.sym, event.key.keysym.mod);
                    break;
                case SDL_KEYUP :
                    onKeyUp(event.key.keysym.sym, event.key.keysym.mod);
                    break;
                case SDL_MOUSEMOTION :
                    onMouseMove(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel, event.motion.state);
                    break;
                case SDL_MOUSEBUTTONDOWN :
                    onMouseDown(event.button.button, event.button.x, event.button.y);
                    break;
                case SDL_MOUSEBUTTONUP :
                    onMouseUp(event.button.button, event.button.x, event.button.y);
                    break;
                case SDL_QUIT :
                    return;// End main loop
                case SDL_VIDEORESIZE :
                    onWindowResized(event.resize.w, event.resize.h);
                    break;
                case SDL_VIDEOEXPOSE :
                    redraw = true;
                    break;
                default :// Do nothing
                    break;
            }
        } while(SDL_PollEvent(&event) == 1);

        // Optionally redraw window
        if(active && redraw) onWindowRedraw();
    }
}

// Animation main loop
// period - maximum time between redraws in ms
void mainLoop(unsigned period)
{
    // This main loop requires timer support
    if(SDL_InitSubSystem(SDL_INIT_TIMER) < 0) throw SDL_Exception();

    // Create redraw timer
    class RedrawTimer
    {
        private :
            SDL_TimerID id;
            static Uint32 callback(Uint32 interval, void *)
            {
                redraw();
                return interval;
            }
        public :
            RedrawTimer(unsigned interval)
                : id(SDL_AddTimer(interval, callback, NULL))
            {
                if(id == NULL) throw SDL_Exception();
            }
            ~RedrawTimer()
            {
                if(id != NULL) SDL_RemoveTimer(id);
            }
    } redrawTimer(period);

    // Start simple main loop
    mainLoop();
}

