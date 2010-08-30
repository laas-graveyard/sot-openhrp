/*
 *  Copyright
 */

#ifndef SOT_OPENHRP_API_HH
#define SOT_OPENHRP_API_HH

#if defined (WIN32)
#  ifdef SOT_OPENHRP_EXPORTS
#    define SOT_OPENHRP_EXPORT __declspec(dllexport)
#  else
#    define SOT_OPENHRP_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOT_OPENHRP_EXPORT
#endif

#endif
