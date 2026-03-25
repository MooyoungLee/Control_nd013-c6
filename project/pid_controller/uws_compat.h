#ifndef UWS_COMPAT_H
#define UWS_COMPAT_H

#if defined(__has_include)
#  if __has_include(<uWS/uWS.h>)
#    include <uWS/uWS.h>
#  elif __has_include(<uWS.h>)
#    include <uWS.h>
#  else
#    error "Could not find a compatible uWebSockets header. Expected <uWS/uWS.h> or <uWS.h>."
#  endif
#else
#  include <uWS/uWS.h>
#endif

#endif  // UWS_COMPAT_H
