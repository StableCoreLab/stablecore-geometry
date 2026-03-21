#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)
#    if defined(GEOMETRY_BUILD_DLL)
#        define GEOMETRY_API __declspec(dllexport)
#    elif defined(GEOMETRY_USE_DLL)
#        define GEOMETRY_API __declspec(dllimport)
#    else
#        define GEOMETRY_API
#    endif
#else
#    define GEOMETRY_API
#endif

