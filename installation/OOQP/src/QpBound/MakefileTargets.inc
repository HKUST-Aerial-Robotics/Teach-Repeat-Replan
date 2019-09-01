all: qpbound-dense-gondzio.exe
all_petsc: qpbound-petsc-mehrotra.exe

all_libs: $(libooqpbound_STATIC)

all_headers: qpbound_headers
qpbound_headers : $(QPBOUNDHEADERS)

$(libooqpbound_STATIC) : $(libooqpbound_OBJ)
	ar ru $@ $(libooqpbound_OBJ)
	$(RANLIB) $@

qpbound-dense-gondzio.exe: $(qpbound_dense_gondzio_OBJECTS)
	$(LINK.cc) -o qpbound-dense-gondzio.exe \
		$(qpbound_dense_gondzio_OBJECTS) \
		$(BLAS) $(FLIBS) $(LIBS)

qpbound-petsc-mehrotra.exe: $(qpbound_petsc_mehrotra_OBJECTS)
	${LINK.cc} -o $@ $(CXXFLAGS) $(LDFLAGS) $(LIBS) \
		$(qpbound_petsc_mehrotra_OBJECTS) $(PETSC_LIB) $(SYS_LIB)

$(localincludedir)/%.h : $(QPBOUNDDIR)/%.h
	$(INSTALL) -m 444 $< $@
