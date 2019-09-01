HUBERDIR    = $(srcdir)/Huber

HUBERSOURCES = $(HUBERDIR)/HuberData.C $(HUBERDIR)/HuberResiduals.C \
	$(HUBERDIR)/HuberVars.C $(HUBERDIR)/HuberLinsys.C \
	$(HUBERDIR)/Huber.C

HUBEROBJECTS = $(HUBERSOURCES:.C=.o)
HUBERHEADERS = $(patsubst $(HUBERDIR)/%,$(localincludedir)/%,\
	$(wildcard $(HUBERDIR)/*.h))

huber_gondzio_OBJECTS = $(HUBEROBJECTS) $(GONDZIOOBJECTS) $(ABOBJECTS) \
	$(VECTOROBJECTS) $(DELINALGOBJECTS) $(UTILOBJECTS) \
	$(HUBERDIR)/HuberGondzioDriver.o
