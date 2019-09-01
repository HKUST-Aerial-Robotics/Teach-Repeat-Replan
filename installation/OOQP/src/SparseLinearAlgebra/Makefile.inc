SPLINALGDIR = $(srcdir)/SparseLinearAlgebra

SPLINALGSOURCES = $(SPLINALGDIR)/SparseStorage.C \
	$(SPLINALGDIR)/SparseLinearAlgebraPackage.C \
	$(SPLINALGDIR)/SparseGenMatrix.C \
	$(SPLINALGDIR)/SparseSymMatrix.C

SPLINALGOBJECTS = $(SPLINALGSOURCES:.C=.o)

SPLINALGHEADERS = $(patsubst $(SPLINALGDIR)/%,$(localincludedir)/%,\
	$(wildcard $(SPLINALGDIR)/*.h))

libooqpsparse_OBJ = $(SPLINALGOBJECTS)

libooqpsparse_STATIC = $(locallibdir)/libooqpsparse.a
