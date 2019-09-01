SVMDIR    = $(srcdir)/Svm

SVMSOURCES = $(SVMDIR)/SvmData.C $(SVMDIR)/SvmResiduals.C \
	$(SVMDIR)/SvmVars.C $(SVMDIR)/SvmLinsys.C \
    $(SVMDIR)/Svm.C

SVMOBJECTS = $(SVMSOURCES:.C=.o)
SVMHEADERS = $(patsubst $(SVMDIR)/%,$(localincludedir)/%,\
	$(wildcard $(SVMDIR)/*.h))

svm_gondzio_OBJECTS = $(SVMOBJECTS) $(GONDZIOOBJECTS) $(ABOBJECTS) \
	$(UTILOBJECTS) $(DELINALGOBJECTS) $(VECTOROBJECTS) \
	$(SVMDIR)/SvmGondzioDriver.o