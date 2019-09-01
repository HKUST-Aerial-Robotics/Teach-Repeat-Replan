ABDIR       = $(srcdir)/Abstract

ABSOURCES = $(ABDIR)/OoqpVersion.C \
	$(ABDIR)/Variables.C $(ABDIR)/Data.C $(ABDIR)/Solver.C \
	$(ABDIR)/Status.C $(ABDIR)/OoqpMonitor.C $(ABDIR)/IotrRefCount.o
ABOBJECTS = $(ABSOURCES:.C=.o)

ABHEADERS = $(patsubst $(ABDIR)/%,$(localincludedir)/%,\
	$(wildcard $(ABDIR)/*.h))

libooqpbase_STATIC = $(locallibdir)/libooqpbase.a
libooqpbase_OBJ    = $(ABOBJECTS) $(UTILOBJECTS) $(VECTOROBJECTS)