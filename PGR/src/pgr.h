#ifndef PGR_H
#define PGR_H

#include <cassert>

#include <string>
#include <exception>
#include <stdexcept>
#include <iostream>

#include <SDL/SDL.h>

#ifdef USE_GLEE
# include <GL/GLee.h>
#else
# include <GL/glew.h>
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Error handling
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Replacement for gluErrorString
const char * getGlErrorString(GLenum error);

struct SDL_Exception : public std::runtime_error
{
    SDL_Exception() throw()
        : std::runtime_error(std::string("SDL : ") + SDL_GetError()) {}
    SDL_Exception(const char * text) throw()
        : std::runtime_error(std::string("SDL : ") + text + " : " + SDL_GetError()) {}
    SDL_Exception(const std::string & text) throw()
        : std::runtime_error(std::string("SDL : ") + text + " : " + SDL_GetError()) {}
};

struct GL_Exception : public std::runtime_error
{
    GL_Exception(const GLenum error = glGetError()) throw()
        : std::runtime_error(std::string("OpenGL : ") + (const char*)getGlErrorString(error)) {}
    GL_Exception(const char * text, const GLenum error = glGetError()) throw()
        : std::runtime_error(std::string("OpenGL : ") + text + " : " + getGlErrorString(error)) {}
    GL_Exception(const std::string & text, const GLenum error = glGetError()) throw()
        : std::runtime_error(std::string("OpenGL : ") + text + " : " + getGlErrorString(error)) {}
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Event functions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Send quit event
inline void quit()
{
    SDL_Event event;
    event.type = SDL_QUIT;
    if(SDL_PushEvent(&event) < 0) throw SDL_Exception();
}

// Send redraw event
inline void redraw()
{
    SDL_Event event;
    event.type = SDL_VIDEOEXPOSE;
    if(SDL_PushEvent(&event) < 0) throw SDL_Exception();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Shaders
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Load whole file and return it as std::string
std::string loadFile(const char * const file);

// Common shader log code
#ifndef USE_GLEE
    std::string getInfoLog(GLuint id, PFNGLGETSHADERIVPROC getLen, PFNGLGETSHADERINFOLOGPROC getLog);
#else
    std::string getInfoLog(GLuint id, GLEEPFNGLGETSHADERIVPROC getLen, GLEEPFNGLGETSHADERINFOLOGPROC getLog);
#endif//USE_GLEE

// Info logs contain errors and warnings from shader compilation and linking
inline std::string getShaderInfoLog(GLuint shader)
{
    assert(glIsShader(shader));
    return getInfoLog(shader, glGetShaderiv, glGetShaderInfoLog);
}
inline std::string getProgramInfoLog(GLuint program)
{
    assert(glIsProgram(program));
    return getInfoLog(program, glGetProgramiv, glGetProgramInfoLog);
}

GLuint compileShader(const GLenum type, const char * source);

GLuint linkShader(size_t count, ...);

void SurfaceImage2D(GLenum target, GLint level, GLint internalformat, SDL_Surface * surface);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Event handlers
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void onInit();
void onWindowRedraw();
void onWindowResized(int width, int height);
void onKeyDown(SDLKey key, Uint16 mod);
void onKeyUp(SDLKey key, Uint16 mod);
void onMouseMove(unsigned x, unsigned y, int xrel, int yrel, Uint8 buttons);
void onMouseDown(Uint8 button, unsigned x, unsigned y);
void onMouseUp(Uint8 button, unsigned x, unsigned y);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Initialization + event loop
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SDL_Surface * init(unsigned width, unsigned height, unsigned color, unsigned depth, unsigned stencil);

// Simple main loop
void mainLoop();

// Animation main loop
// period - rough time between redraws in ms
void mainLoop(unsigned period);

#undef main

#endif
