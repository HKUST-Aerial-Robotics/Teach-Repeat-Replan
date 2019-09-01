READERDIR = $(srcdir)/Readers

READERHEADERS = $(patsubst $(READERDIR)/%,$(localincludedir)/%,\
	$(wildcard $(READERDIR)/*.h))

MPSREADERSRC = $(READERDIR)/MpsReader.C $(READERDIR)/hash.c $(CMPSREADERSRC)
MPSREADEROBJ = $(patsubst %.C,%.o,$(MPSREADERSRC:.c=.o))
