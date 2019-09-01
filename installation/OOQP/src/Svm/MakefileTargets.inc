all: svm-gondzio.exe
all_headers: svm_headers

svm-gondzio.exe: $(svm_gondzio_OBJECTS)
	$(LINK.cc) -o $@ $(svm_gondzio_OBJECTS) $(BLAS) $(LIBS) $(FLIBS)

svm_headers: $(SVMHEADERS)

$(localincludedir)/%.h : $(SVMDIR)/%.h
	$(INSTALL) -m 444 $< $@
