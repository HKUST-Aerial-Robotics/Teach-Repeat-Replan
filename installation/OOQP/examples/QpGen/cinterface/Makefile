SHELL = /bin/sh

OOQP=../../..
OOQPINCLUDEDIR=$(OOQP)/include
OOQPLIBDIR=$(OOQP)/lib

CC       = cc
CXX      = c++

CFLAGS   =-O 
CXXFLAGS =-O 
CPPFLAGS =-I$(OOQPINCLUDEDIR)
LDFLAGS  =-L$(OOQPLIBDIR)
BLAS     = -lblas
MA27LIB  = $(OOQP)/extras/MA27/libMA27.a
FLIBS    = -lgfortran
SPARSE_LIBS     = -looqpgensparse -looqpsparse  -looqpgondzio -looqpbase \
                $(BLAS) $(MA27LIB) $(FLIBS)
DENSE_LIBS     = -looqpgendense -looqpdense  -looqpgondzio -looqpbase \
                $(BLAS) $(FLIBS)

all: mps.exe example.exe dense_example.exe hb_example.exe

mps.exe : mps.o
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(SPARSE_LIBS)

example.exe : example.o
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(SPARSE_LIBS)

dense_example.exe : dense_example.o
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(DENSE_LIBS)

hb_example.exe : hb_example.o
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(SPARSE_LIBS)

clean:
	rm -f *.o

veryclean: clean
	rm -f *.exe

distclean: veryclean

.c.o:
	$(CC) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<
