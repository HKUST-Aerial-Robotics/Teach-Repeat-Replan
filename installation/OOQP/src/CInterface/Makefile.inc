CINTERFACEDIR = $(srcdir)/CInterface
CINTERFACEHEADERS = $(patsubst $(CINTERFACEDIR)/%,$(localincludedir)/%,\
	$(wildcard $(CINTERFACEDIR)/*.h))

cqpsol_OBJECTS = $(QPGENSPARSEOBJ) $(ABOBJECTS) $(UTILOBJECTS) \
	$(GONDZIOOBJECTS) $(DELINALGOBJECTS) $(VECTOROBJECTS) \
	$(SPLINALGOBJECTS) \
	$(QPGENDIR)/Ma57QPGenFactory.o $(MA57SOLVEROBJ) \
	CInterface/cQpGen.o CInterface/cMpsReader.o \
	Readers/MpsReader.o Readers/hash.o \
	CInterface/cQpGenSparse.o \
	CInterface/qpgensparsemain.o

cqpsol_dense_OBJECTS = $(QPGENDENSEOBJ) $(ABOBJECTS) $(UTILOBJECTS) \
	$(GONDZIOOBJECTS) $(DELINALGOBJECTS) $(VECTOROBJECTS) \
	$(SPLINALGOBJECTS) \
	CInterface/cQpGenDense.o CInterface/cMpsReader.o \
	Readers/MpsReader.o Readers/hash.o \
	CInterface/cQpGen.o CInterface/qpgendensemain.o

CQPGENSPARSEOBJ  = $(CINTERFACEDIR)/cQpGenSparse.o \
                   $(CINTERFACEDIR)/cQpGen.o \
                   $(CINTERFACEDIR)/cBounds.o
CQPGENDENSEOBJ  =  $(CINTERFACEDIR)/cQpGenDense.o \
                   $(CINTERFACEDIR)/cQpGen.o \
                   $(CINTERFACEDIR)/cBounds.o
CMPSREADERSRC    = $(CINTERFACEDIR)/cMpsReader.C
CMPSREADEROBJ    = $(CINTERFACEDIR)/cMpsReader.o

