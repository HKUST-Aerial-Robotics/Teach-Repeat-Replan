all: huber-gondzio.exe

huber-gondzio.exe: $(huber_gondzio_OBJECTS)
	$(LINK.cc) -o $@ $(huber_gondzio_OBJECTS) $(BLAS) $(LIBS) $(FLIBS)

all_headers: huber_headers
huber_headers: $(HUBERHEADERS)

$(localincludedir)/%.h : $(HUBERDIR)/%.h
	$(INSTALL) -m 444 $< $@
