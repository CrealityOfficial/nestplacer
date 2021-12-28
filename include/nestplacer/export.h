#ifndef _NESTPLACER_EXPORT_H
#define _NESTPLACER_EXPORT_H

#ifdef WIN32
#ifdef _NESTPLACER_DLL
#define _NESTPLACER_API __declspec(dllexport)
#else
#define _NESTPLACER_API __declspec(dllimport)
#endif
#else
#ifdef _NESTPLACER_DLL
#define _NESTPLACER_API __attribute__((visibility("default")))
#else
#define _NESTPLACER_API
#endif
#endif


#endif
