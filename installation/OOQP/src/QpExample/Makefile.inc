QPEXAMPLEDIR = $(srcdir)/QpExample

QPEXAMPLEOBJ = \
	$(QPEXAMPLEDIR)/QpExampleData.o $(QPEXAMPLEDIR)/QpExampleVars.o \
	$(QPEXAMPLEDIR)/QpExampleResids.o  \
	$(QPEXAMPLEDIR)/QpExampleDenseLinsys.o \
	$(QPEXAMPLEDIR)/QpExampleDense.o 

qpexample_dense_gondzio_OBJECTS = $(QPEXAMPLEDIR)/QpExampleGondzioDriver.o \
	$(QPEXAMPLEOBJ) \
	$(libooqpgondzio_STATIC) $(libooqpdense_STATIC) $(libooqpbase_STATIC)