ifeq ($(MEX_ENABLED),yes)

all:

all_matlab: all_mex all_matlab_functions
all_matlab_functions: $(MATLABFUNCTIONS)

readmps.m : $(OOQPMEXDIR)/readmps.m
	cp $(OOQPMEXDIR)/readmps.m .

ooqp_svm.m : $(OOQPMEXDIR)/ooqp_svm.m
	cp $(OOQPMEXDIR)/ooqp_svm.m .

ooqp_huber.m : $(OOQPMEXDIR)/ooqp_huber.m
	cp $(OOQPMEXDIR)/ooqp_huber.m .

ooqp.m : $(OOQPMEXDIR)/ooqp.m
	cp $(OOQPMEXDIR)/ooqp.m .

ooqp_start.m : $(OOQPMEXDIR)/ooqp_start.m
	cp $(OOQPMEXDIR)/ooqp_start.m .

# MEXFLIBS -FLIBS for mex. Needed if the Matlab executable was not
# linked against the Fortran runtime libraries. If the contents of
# this variable cause problems on your system, I suggest that you try
# setting MEXFLIBS to the empty string, because on many platforms it
# is not needed.
#
# A common problem is that the mex compiler doesn't recognize -R arguments,
# which extend the run-time load path for shared objects. Our solution is
# to strip the -R arguments, and hope that all necessary directories are
# already on the path. So far, this has proven to be the case.
MEXFLIBS := $(patsubst -R%,,$(FLIBS)) 
# Usually, mex files need to be linked against the blas. Sometimes, however,
# you need to use a different version of the blas than you would for normal
# executables. We introduce the MEXBLAS variable as a placeholder so
# that you may do this.
MEXBLAS = $(BLAS)

$(OOQPMEXDIR)/mexUtility.o : $(OOQPMEXDIR)/mexUtility.c \
				$(OOQPMEXDIR)/mexUtility.h
	$(MEX) $(DEFS) -c $(INCLUDES) $(OOQPMEXDIR)/mexUtility.c
	mv mexUtility.o $@

all_mex: $(MEXFUNCTIONS)
ooqp_mex.$(MEXSFX): $(OOQPMEXDIR)/ooqp_mex.c $(ooqp_mex_OBJ)
	$(MEX) $(DEFS)  $(INCLUDES)  \
	       $(OOQPMEXDIR)/ooqp_mex.c $(ooqp_mex_OBJ) \
	       $(MA27LIB) $(MEXBLAS) $(MEXFLIBS)

ooqp_start_mex.$(MEXSFX): $(OOQPMEXDIR)/ooqp_start_mex.c \
	$(ooqp_start_mex_OBJ)
	$(MEX) $(DEFS)  $(INCLUDES)  \
	       $(OOQPMEXDIR)/ooqp_start_mex.c $(ooqp_start_mex_OBJ) \
	       $(MA27LIB) $(MEXBLAS) $(MEXFLIBS)


readmps_mex.$(MEXSFX): $(OOQPMEXDIR)/readmps_mex.c $(readmps_mex_OBJ) 
	$(MEX) $(DEFS) $(INCLUDES)  \
	       $(OOQPMEXDIR)/readmps_mex.c $(readmps_mex_OBJ) \
	       $(MEXBLAS) $(MEXFLIBS)

ooqp_svm.$(MEXSFX): $(OOQPMEXDIR)/ooqp_svm.c $(ooqp_svm_mex_OBJ) 
	$(MEX) $(DEFS) $(INCLUDES) \
	       $(OOQPMEXDIR)/ooqp_svm.c $(ooqp_svm_mex_OBJ) \
	       $(MEXBLAS) $(MEXFLIBS)

$(srcdir)/ooqp_svm.m: $(OOQPMEXDIR)/ooqp_svm.help
	cp $(OOQPMEXDIR)/ooqp_svm.help $(srcdir)/ooqp_svm.m

ooqp_huber.$(MEXSFX): $(OOQPMEXDIR)/ooqp_huber.c $(ooqp_huber_mex_OBJ)
	$(MEX) $(DEFS) $(INCLUDES) \
	       $(OOQPMEXDIR)/ooqp_huber.c $(ooqp_huber_mex_OBJ) \
	       $(MEXBLAS) $(MEXFLIBS)

install_matlab: all_matlab
	$(INSTALL) -d $(libdir)/ooqp-matlab
	$(INSTALL)  $(MEXFUNCTIONS) $(MATLABFUNCTIONS) $(libdir)/ooqp-matlab
endif

$(OOQPMEXDIR)/MakefileTargets.inc:  $(OOQPMEXDIR)/MakefileTargets.inc.in \
                                    $(OOQPMEXDIR)/config.status
	cd $(OOQPMEXDIR); ./config.status

$(OOQPMEXDIR)/Makefile.inc: $(OOQPMEXDIR)/Makefile.inc.in \
                            $(OOQPMEXDIR)/config.status
	cd $(OOQPMEXDIR); ./config.status

$(OOQPMEXDIR)/config.status: $(OOQPMEXDIR)/configure
	cd $(OOQPMEXDIR); ./config.status --recheck


veryclean: matlab-veryclean
matlab-veryclean: 
	-rm -f $(MEXFUNCTIONS) $(MATLABFUNCTIONS)

distclean: matlab-distclean
matlab-distclean: 
	-rm -f $(OOQPMEXDIR)/MakefileTargets.inc $(OOQPMEXDIR)/Makefile.inc
	-rm -f $(OOQPMEXDIR)/config.status $(OOQPMEXDIR)/config.log
	-rm -f $(OOQPMEXDIR)/DISABLED

maintainer-clean: matlab-maintainer-clean
matlab-maintainer-clean: 
	-rm -r $(OOQPMEXDIR)/configure
