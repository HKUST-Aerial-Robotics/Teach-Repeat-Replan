dnl Process this file with autoconf to produce a configure script.
AC_INIT
AC_CONFIG_SRCDIR([mexUtility.c])

dnl this will always be configured as part of a larger package
AC_CONFIG_AUX_DIR(../../config)

AC_ARG_WITH(matlab, [--with-matlab: support for the MATLAB environment],,
	      with_matlab=no )

if test "$with_matlab" = yes; then

   if test -z "$MEX"; then
      AC_PATH_PROG(MEX, mex)
   fi 
   $MEX -help > /dev/null	
   if test $? -eq 0; then
      mex_found='yes'
   else
      mex_found='no'
      AC_MSG_WARN([Could not find the mex compiler])
      AC_MSG_WARN(
	 [The value of the MEX environment variable may be incorrect.])
   fi

   $MEX -help > /dev/null	
   if test $? -ne 0; then
      AC_MSG_WARN([Could not locate a Matlab mex compiler.])
   fi

   if test -z "$MATLABEXE"; then
      AC_PATH_PROG(MATLABEXE, matlab)
   fi
   ( unset DISPLAY; echo quit | $MATLABEXE > /dev/null 2>&1 )
   if test $? -eq 0; then
      matlab_found=yes
   else
      AC_MSG_WARN([ Could not find the MATLAB executable. ])
      AC_MSG_WARN(
	   [The value of the MATLABEXE environment variable may be incorrect.])
   fi

   dnl Check that the matlab mex compiler works
   if test "$mex_found" = yes; then
   AC_CACHE_CHECK([whether the Matlab mex compiler works],
    ac_cv_mex_compiles,[
cat > conftest.c <<EOF
[#include "mex.h"

void mexFunction( int nlhs, mxArray * plhs[], int nrhs,
    const mxArray * prhs[] ) {
    mexPrintf( "Hello World\n" );
}]
EOF
   $MEX conftest.c 2> /dev/null 1>&2
   if test $? -eq 0; then
	ac_cv_mex_compiles=yes	
   else
	ac_cv_mex_compiles=no
	echo "configure: failed program was:" >&AS_MESSAGE_LOG_FD()
	cat conftest.c >&AS_MESSAGE_LOG_FD()
    fi])
	rm -f conftest.c
   fi
   if test "$ac_cv_mex_compiles" = "yes"; then
	AC_CACHE_CHECK([for the mex suffix],
	ac_cv_mex_suffix,
	[
	dnl Assume that we just compiled. Otherwise this will fail,
	dnl But if the mex_suffix is not in the cache, then
        dnl mex_compiles should not be either.
	ac_cv_mex_suffix=`ls conftest.mex* | sed 's/conftest\.\(mex.*\)/\1/'` 
	])
   fi

if test "$matlab_found" = yes -a "$ac_cv_mex_compiles" = "yes"; then
    AC_CACHE_CHECK(whether the Matlab can run a mex file (slow),
	ac_cv_mex_runs,[
	( echo conftest; echo quit ) > conftest.m
	( unset DISPLAY; $MATLABEXE < conftest.m ) | \
		grep "Hello World" > /dev/null
	if test $? -eq 0; then
	    ac_cv_mex_runs=yes
	else
	    ac_cv_mex_runs=no
	fi])
	rm -f conftest.m
	rm -f conftest.mex*
fi


if test "$ac_cv_mex_compiles" != yes -o "$ac_cv_mex_runs" != yes; then
   AC_MSG_WARN([Could not compile and/or run Matlab mex files.])
   AC_MSG_WARN([Matlab support disabled. See Mex/README for suggestions])
   with_matlab=no
fi
fi

if test "$with_matlab" = yes; then
    all_matlab_if_enabled="all_matlab"
    MEX_ENABLED=yes
    rm -f DISABLED
else
    all_matlab_if_enabled='# MATLAB support has been disabled'
    MEX_ENABLED='no # MATLAB support has been disabled'
    echo MATLAB support has been disabled. > DISABLED
fi

if test -n "$ac_cv_mex_suffix"; then
    MEXSFX="$ac_cv_mex_suffix"
else
    MEXSFX=mexmex
fi
AC_SUBST(MEXSFX)
AC_SUBST(MEX_ENABLED)
AC_SUBST(all_matlab_if_enabled)


AC_CONFIG_FILES([Makefile.inc MakefileTargets.inc])
AC_OUTPUT
