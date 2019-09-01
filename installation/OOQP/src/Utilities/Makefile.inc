UTILDIR     = $(srcdir)/Utilities

UTILSOURCES = $(UTILDIR)/drand.C $(UTILDIR)/sort.C

UTILOBJECTS = 	$(UTILDIR)/drand.o $(UTILDIR)/sort.o

UTILHEADERS = $(patsubst $(UTILDIR)/%,$(localincludedir)/%,\
       $(wildcard $(UTILDIR)/*.h))