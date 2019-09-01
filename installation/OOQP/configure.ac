dnl Process this file with autoconf to produce a configure script.
AC_INIT
AC_CONFIG_SRCDIR([src/Abstract/Data.C])
AC_CONFIG_AUX_DIR(config)

AC_CANONICAL_TARGET

if test "$srcdir" = .; then
   ooqp_src=src
else
   ooqp_src="$srcdir/src"
fi
AC_SUBST(ooqp_src)

dnl Checks for programs.
AC_PROG_INSTALL
dnl Use native cc first
if test -z "$CC"; then
   AC_CHECK_PROG(CC,cc,cc)
fi
dnl And optimize, unless told otherwise
if test -z "$CFLAGS"; then CFLAGS=-O; fi
AC_PROG_CC

dnl Use native CC first
if test -z "$CXX"; then
   AC_CHECK_PROG(CXX,CC,CC)
fi
dnl And optimize, unless told otherwise
if test -z "$CXXFLAGS"; then CXXFLAGS=-O; fi
AC_PROG_CXX

AC_ARG_WITH(fortran, [--with-fortran: use a FORTRAN compiler (instead of
	      f2c],,
	      with_fortran=yes )

if test "$with_fortran" != "yes"; then
    dnl Any compilitions of fortran files will fail
    FC=false
    F77=false
    dnl We cannot test the libraries and so just assume they are Ok
    if test -z "$FLIBS"; then
	FLIBS=libf2c.a
    fi
else
    dnl If FC is set, but F77 is not, assume that FC points to the F77
    dnl compiler.
    if test -z "$F77"; then
	if test -n "$FC"; then
	    F77="$FC"
	    export F77
	fi
    fi

    dnl Optimize, unless told otherwise
    if test -z "$FFLAGS"; then FFLAGS=-O; fi
    AC_PROG_F77

    dnl Checks for libraries.
    AC_F77_LIBRARY_LDFLAGS
fi
dnl End of --with-fortran

if test -z "$BLAS"; then
    AC_MSG_WARN([BLAS environment variable was not set. Defaulting to '-lblas -llapack'.])
    BLAS='-lblas -llapack'
fi
AC_SUBST(BLAS)

if test -z "$MA27LIB"; then
   AC_MSG_WARN([MA27LIB environment variable was not set.])
   AC_MSG_WARN([Defaulting to libma27.a])
   MA27LIB=libma27.a
fi
AC_SUBST(MA27LIB)

if test -z "$MA57LIB"; then
   AC_MSG_WARN([MA57LIB environment variable was not set.])
   AC_MSG_WARN([Defaulting to 'libma57.a libfakemetis.a'.])
   MA57LIB='libma57.a libfakemetis.a'
fi
AC_SUBST(MA57LIB)

AC_LANG([C++])
AC_PROG_RANLIB
AC_LANG([C++])

dnl Checks for header files.
AC_SUBST(INCLUDES)
AC_HEADER_STDC

dnl Checks for typedefs, structures, and compiler characteristics.

dnl Checks for library functions.
AC_CHECK_FUNCS(getrusage)

dnl Checks for Petsc version

if test ! -d "$PETSC_DIR"; then
    petsc_enabled='no # Petsc support has been disabled'
    AC_MSG_WARN([Petsc support has been disabled.])
    AC_MSG_WARN([If you intend to use Petsc, set PETSC_DIR and])
    AC_MSG_WARN([PETSC_ARCH and run configure again.])
else
    petsc_enabled='yes'
    if test -f $PETSC_DIR/bmake/common/variables; then
dnl     This is a new version of Petsc!
        petsc_common_variables='${PETSC_DIR}/bmake/common/variables'
    else
    petsc_common_variables='${PETSC_DIR}/bmake/${PETSC_ARCH}/base_variables'
    fi
fi
AC_SUBST(petsc_enabled)
AC_SUBST(petsc_common_variables)
AC_SUBST(PETSC_DIR)

AC_CONFIG_SUBDIRS(src/Mex src/Ampl)

AC_CONFIG_FILES([GNUmakefile PetscMakefile])
AC_OUTPUT



