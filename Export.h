#pragma once

#define MSDEV

#ifdef MSDEV
#  define xmDeclImport __declspec(dllimport)
#  define xmDeclExport __declspec(dllexport)
#else
#define xmDeclImport
#define xmDeclExport
#endif

#if defined(ENGINE_DLL)
#define xmExpEngine xmDeclExport
#else
#define xmExpEngine xmDeclImport
#endif

#ifdef MSDEV
#pragma warning( disable : 4251 )  // dll export of stl templates
#endif
