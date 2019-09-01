QPSOLVERDIR   = $(srcdir)/QpSolvers

GONDZIOSOURCES  = $(QPSOLVERDIR)/GondzioSolver.C
GONDZIOOBJECTS  = $(GONDZIOSOURCES:.C=.o)

MEHROTRASOURCES = $(QPSOLVERDIR)/MehrotraSolver.C
MEHROTRAOBJECTS = $(MEHROTRASOURCES:.C=.o)

QPSOLVERHEADERS = $(patsubst $(QPSOLVERDIR)/%,$(localincludedir)/%,\
	$(wildcard $(QPSOLVERDIR)/*.h))

libooqpgondzio_STATIC = $(locallibdir)/libooqpgondzio.a
libooqpgondzio_OBJ    = $(GONDZIOOBJECTS)

libooqpmehrotra_STATIC = $(locallibdir)/libooqpmehrotra.a
libooqpmehrotra_OBJ    = $(MEHROTRAOBJECTS)