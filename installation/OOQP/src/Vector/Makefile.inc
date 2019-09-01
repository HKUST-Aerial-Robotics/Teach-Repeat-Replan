VECTORDIR   = $(srcdir)/Vector

VECTOROBJECTS = $(VECTORDIR)/OoqpVector.o $(VECTORDIR)/SimpleVector.o \
                $(VECTORDIR)/VectorUtilities.o

VECTORHEADERS = $(patsubst $(VECTORDIR)/%,$(localincludedir)/%,\
	$(wildcard $(VECTORDIR)/*.h))