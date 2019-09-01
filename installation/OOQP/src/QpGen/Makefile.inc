QPGENDIR  = $(srcdir)/QpGen

QPGENHEADERS = $(patsubst $(QPGENDIR)/%,$(localincludedir)/%,\
	$(wildcard $(QPGENDIR)/*.h))

QPGENCOMMONSRC = $(QPGENDIR)/QpGenVars.C $(QPGENDIR)/QpGenData.C \
	$(QPGENDIR)/QpGenResiduals.C $(QPGENDIR)/QpGen.C \
	$(QPGENDIR)/QpGenLinsys.C 

QPGENSPARSESRC = $(QPGENCOMMONSRC) $(QPGENDIR)/QpGenSparseSeq.C \
	$(QPGENDIR)/QpGenSparseLinsys.C
QPGENSPARSEOBJ = $(QPGENSPARSESRC:.C=.o)

QPGENDENSESRC = $(QPGENCOMMONSRC) \
	$(QPGENDIR)/QpGenDenseLinsys.C $(QPGENDIR)/QpGenDense.C
QPGENDENSEOBJ = $(QPGENDENSESRC:.C=.o)

libooqpgensparse_OBJ = $(QPGENSPARSEOBJ) $(MPSREADEROBJ) \
	$(CQPGENSPARSEOBJ) \
	$(QPGENDIR)/QpGenSparseMa27.o $(MA27SOLVEROBJ) 
libooqpgensparse_STATIC = $(locallibdir)/libooqpgensparse.a

libooqpgendense_OBJ = $(QPGENDENSEOBJ) $(MPSREADEROBJ) \
	$(CQPGENDENSEOBJ) \
	$(QPGENDIR)/QpGenDense.o 
libooqpgendense_STATIC = $(locallibdir)/libooqpgendense.a


qpgen_sparse_gondzio_OBJECTS = $(QPGENDIR)/QpGenSparseGondzioDriver.o \
	$(libooqpgondzio_STATIC) $(libooqpgensparse_STATIC) \
	$(libooqpsparse_STATIC) $(libooqpbase_STATIC)

qpgen_sparse_mehrotra_OBJECTS = $(QPGENDIR)/QpGenSparseMehrotraDriver.o \
	$(libooqpmehrotra_STATIC) $(libooqpgensparse_STATIC) \
	$(libooqpsparse_STATIC) $(libooqpbase_STATIC)

qpgen_dense_gondzio_OBJECTS = $(QPGENDIR)/QpGenDenseGondzioDriver.o \
	$(libooqpgondzio_STATIC) $(libooqpgendense_STATIC) \
	$(libooqpdense_STATIC) $(libooqpbase_STATIC)

qpgen_sparse_ma57_gondzio_OBJECTS = \
	$(QPGENDIR)/QpGenSparseMa57GondzioDriver.o \
	$(QPGENDIR)/QpGenSparseMa57.o $(MA57SOLVEROBJ)  \
	$(libooqpgondzio_STATIC) $(libooqpgensparse_STATIC) \
	$(libooqpsparse_STATIC) $(libooqpbase_STATIC)

