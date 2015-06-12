#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([CuraEngine], [1.0], [BUG-REPORT-ADDRESS])
AC_CONFIG_SRCDIR([src/timeEstimate.h])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE(-Wall)

AC_CONFIG_MACRO_DIRS([m4])

#magic for conditional check in Makefile:
MK=''; AC_SUBST(MK)
SED=sed

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Checks for libraries.
#LT_PREREQ([2.2])
LT_INIT([shared static])
#LT_INIT([disable-shared])

# Checks for libraries.
AC_CHECK_LIB([m], [cos])
AC_CHECK_LIB([pthread], [pthread_create])
AC_CHECK_LIB([wsock32], [getsockname])
#AC_CHECK_LIB([polyclipping], [PolyNode::IsHole])

dnl polyclipping library
sr_polyclipping_lib=yes

AC_CHECK_HEADERS([polyclipping/clipper.hpp], [], [sr_polyclipping_lib=no])




# debug
AC_ARG_ENABLE([debug],
	AS_HELP_STRING([--enable-debug],[Compile the debug version (default: disabled)]),
	[enable_debug=$enableval],
	[enable_debug=no])
AM_CONDITIONAL([DEBUG], [test $enable_debug = "yes"])
if test "x$enable_debug" = "xyes"; then
  changequote({,})
  CFLAGS=`echo "$CFLAGS" | $SED -e 's/-O[0-9s]*//g'`
  CXXFLAGS=`echo "$CXXFLAGS" | $SED -e 's/-O[0-9s]*//g'`
  changequote([,])
  dnl add -O0 only if GCC or ICC is used
  if test "$GCC" = "yes" || test "$ICC" = "yes"; then
    CFLAGS="$CFLAGS -g -O0" #" -Werror"
    CXXFLAGS="$CXXFLAGS -g -O0" #" -Werror"
  fi
else
  changequote({,})
  CFLAGS=`echo "$CFLAGS" | $SED -e 's/-g//g'`
  CXXFLAGS=`echo "$CXXFLAGS" | $SED -e 's/-g//g'`
  changequote([,])
fi

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h float.h netinet/in.h stddef.h stdint.h stdlib.h string.h strings.h sys/socket.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_SIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T

# Checks for library functions.
AC_CHECK_FUNCS([gettimeofday memset socket sqrt strchr strrchr strstr])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
