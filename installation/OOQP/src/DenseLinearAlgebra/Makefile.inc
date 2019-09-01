DELINALGDIR = $(srcdir)/DenseLinearAlgebra

DELINALGOBJECTS = 	$(DELINALGDIR)/DenseStorage.o \
	$(DELINALGDIR)/DenseSymMatrix.o \
	$(DELINALGDIR)/DeSymIndefSolver.o \
	$(DELINALGDIR)/DenseGenMatrix.o \
	$(DELINALGDIR)/DeSymPSDSolver.o \
	$(DELINALGDIR)/DenseLinearAlgebraPackage.o

DELINALGHEADERS = $(patsubst $(DELINALGDIR)/%,$(localincludedir)/%,\
	$(wildcard $(DELINALGDIR)/*.h))

libooqpdense_OBJ = $(DELINALGOBJECTS)

libooqpdense_STATIC = $(locallibdir)/libooqpdense.a