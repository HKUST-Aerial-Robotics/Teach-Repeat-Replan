PETSCLINALGDIR = $(srcdir)/PetscLinearAlgebra

PETSCOBJECTS = $(PETSCLINALGDIR)/PetscIterativeSolver.o \
	$(PETSCLINALGDIR)/PetscSpSymMatrix.o \
	$(PETSCLINALGDIR)/PetscSpGenMatrix.o \
	$(PETSCLINALGDIR)/PetscLinearAlgebraPackage.o \
	$(PETSCLINALGDIR)/PetscSparseStorage.o \
	$(PETSCLINALGDIR)/PetscVector.o

PETSCHEADERS = $(patsubst $(PETSCLINALGDIR)/%,$(localincludedir)/%,\
	$(wildcard $(PETSCLINALGDIR)/*.h))

libooqppetsc_STATIC = $(locallibdir)/libooqppetsc.a
libooqppetsc_OBJ    = $(PETSCOBJECTS)
