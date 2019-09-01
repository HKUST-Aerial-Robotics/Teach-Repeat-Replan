dnl Process this file with autoconf to produce a configure script.
AC_INIT
AC_CONFIG_SRCDIR([qpgen-sparse-gondzio-ampl.C])

dnl this will always be configured as part of a larger package
AC_CONFIG_AUX_DIR(../../config)

AC_ARG_WITH(ampl, [--with-ampl: support for the Ampl modeling language],,
	      with_ampl=yes )
 

if test "$with_ampl" = yes; then
    if test -z "$AMPL_SOLVERS"; then
        with_ampl=no
	AC_MSG_WARN( [The AMPL_SOLVERS environment variable is not set.])
	AC_MSG_WARN( [I do not know where to find the Ampl library.] )
   else
	dnl AMPL_SOLVERS is set. Do AMPL_SOLVERS and AMPL_OS 
	dnl combine to give a valid location
	dnl for the AMPL libraries

	AMPLLIBS="${AMPL_SOLVERS}/amplsolver.a"

	AC_MSG_CHECKING(whether the AMPL library (amplsolver.a) exists)

	if test ! -f $AMPLLIBS; then
		AC_MSG_RESULT(no)
		with_ampl=no
		warning=`echo The AMPL library, $AMPLIBS, does not exist. \
			Check the values of the AMPL_SOLVERS  \
			environment variable.`
		AC_MSG_WARN($warning)
	else
	   	dnl Librariy exists
		AC_MSG_RESULT(found)
	fi

   fi # end AMPL_SOLVERS is set
fi

AC_SUBST(AMPL_SOLVERS)

if test "$with_ampl" = yes; then
   all_ampl_if_enabled="all_ampl"
   AMPL_ENABLED=ampl_enabled
   rm -f DISABLED
else
   all_ampl_if_enabled='# Ampl support has been disabled'
   AMPL_ENABLED='ampl_disabled # AMPL support has been disabled'
   echo AMPL support has been disabled. > DISABLED
fi

AC_SUBST(AMPL_ENABLED)
AC_SUBST(all_ampl_if_enabled)

AC_CONFIG_FILES([Makefile.inc MakefileTargets.inc])
AC_OUTPUT
