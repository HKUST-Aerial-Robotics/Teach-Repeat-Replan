qpgen-sparse-gondzio.exe: $(qpgen_sparse_gondzio_OBJECTS)
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) $(LIBS) \
		$(qpgen_sparse_gondzio_OBJECTS) $(MA27LIB) $(BLAS) $(FLIBS)

qpgen-sparse-mehrotra.exe: $(qpgen_sparse_mehrotra_OBJECTS)
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) $(LIBS) \
		$(qpgen_sparse_mehrotra_OBJECTS) $(MA27LIB) $(BLAS) $(FLIBS)

qpgen-dense-gondzio.exe: $(qpgen_dense_gondzio_OBJECTS)
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) $(LIBS) \
		$(qpgen_dense_gondzio_OBJECTS) $(BLAS) $(FLIBS)

all_ma57: qpgen-sparse-ma57-gondzio.exe
qpgen-sparse-ma57-gondzio.exe: $(qpgen_sparse_ma57_gondzio_OBJECTS)
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) $(LIBS) \
		$(qpgen_sparse_ma57_gondzio_OBJECTS) $(MA57LIB) $(BLAS) $(FLIBS)

$(libooqpgensparse_STATIC) : $(libooqpgensparse_OBJ)
	ar ru $@ $(libooqpgensparse_OBJ)
	$(RANLIB) $@

$(libooqpgendense_STATIC) : $(libooqpgendense_OBJ)
	ar ru $@ $(libooqpgendense_OBJ)
	$(RANLIB) $@


all: qpgen-sparse-mehrotra.exe
all: qpgen-sparse-gondzio.exe
all: qpgen-dense-gondzio.exe

all_libs: $(libooqpgensparse_STATIC) $(libooqpgendense_STATIC)

all_headers : qpgen_headers
qpgen_headers : $(QPGENHEADERS)

$(localincludedir)/%.h : $(QPGENDIR)/%.h
	$(INSTALL) -m 444 $< $@
