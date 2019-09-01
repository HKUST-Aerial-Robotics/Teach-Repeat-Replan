all_libs: $(libooqpgondzio_STATIC) $(libooqpmehrotra_STATIC)

all_headers: qpsolver_headers

qpsolver_headers: $(QPSOLVERHEADERS)

$(libooqpgondzio_STATIC) : $(libooqpgondzio_OBJ)
	ar ru $@ $(libooqpgondzio_OBJ)
	$(RANLIB) $@

$(libooqpmehrotra_STATIC) : $(libooqpmehrotra_OBJ)
	ar ru $@ $(libooqpmehrotra_OBJ)
	$(RANLIB) $@

$(localincludedir)/%.h : $(QPSOLVERDIR)/%.h
	$(INSTALL) -m 444 $< $@

