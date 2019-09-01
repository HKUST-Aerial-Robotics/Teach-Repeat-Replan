QPBOUNDDIR       = $(srcdir)/QpBound

QPBOUNDCOMMONOBJ = $(QPBOUNDDIR)/QpBoundData.o	\
$(QPBOUNDDIR)/QpBoundResiduals.o		\
$(QPBOUNDDIR)/QpBoundVars.o	\
$(QPBOUNDDIR)/QpBound.o \
$(QPBOUNDDIR)/QpBoundLinsys.o

QPBOUNDHEADERS = $(patsubst $(QPBOUNDDIR)/%,$(localincludedir)/%,\
	$(wildcard $(QPBOUNDDIR)/*.h))

QPBOUNDDENSEOBJ = $(QPBOUNDDIR)/QpBoundDenseLinsys.o \
		$(QPBOUNDDIR)/QpBoundDense.o

QPBOUNDPETSCOBJ = $(QPBOUNDDIR)/QpBoundPetsc.o

libooqpbound_OBJ = $(QPBOUNDCOMMONOBJ) $(MPSREADEROBJ)
libooqpbound_STATIC = $(locallibdir)/libooqpbound.a


qpbound_dense_gondzio_OBJECTS = $(QPBOUNDDIR)/QpBoundDenseGondzioDriver.o \
	$(QPBOUNDDENSEOBJ) \
	$(libooqpgondzio_STATIC) \
	$(libooqpbound_STATIC) $(libooqpdense_STATIC) $(libooqpbase_STATIC)

qpbound_petsc_mehrotra_OBJECTS = \
	$(QPBOUNDDIR)/QpBoundPetscMehrotraDriver.o \
	$(ABDIR)/OoqpPetscMonitor.o \
        $(QPBOUNDDIR)/QpBoundPetsc.o \
	$(libooqppetsc_STATIC) \
	$(libooqpmehrotra_STATIC) \
	$(libooqpbound_STATIC) $(libooqpdense_STATIC) $(libooqpbase_STATIC)



