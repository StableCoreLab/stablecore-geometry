#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(GEOMETRY_BUILD_DLL)
#define GEOMETRY_API __declspec(dllexport)
#define GEOMETRY_DATA_API __declspec(dllexport)
#elif defined(GEOMETRY_USE_DLL)
// GEOMETRY_API is also applied to classes with inline members.  Importing such
// a class makes MSVC expect out-of-line copies of those inline members from the
// DLL.  Keep the established header-only consumer semantics for code and use a
// dedicated macro for data declarations, which do require dllimport.
#define GEOMETRY_API
#define GEOMETRY_DATA_API __declspec(dllimport)
#else
#define GEOMETRY_API
#define GEOMETRY_DATA_API
#endif
#else
#define GEOMETRY_API
#define GEOMETRY_DATA_API
#endif
