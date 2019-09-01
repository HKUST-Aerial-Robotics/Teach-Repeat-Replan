all_libs: $(libooqpbase_STATIC)

display: 
	echo $(ABHEADERS)

base_headers: $(ABHEADERS) $(UTILHEADERS) $(VECTORHEADERS)

all_headers: base_headers

$(libooqpbase_STATIC) : $(libooqpbase_OBJ)
	ar ru $@ $(libooqpbase_OBJ)
	$(RANLIB) $@

$(localincludedir)/%.h : $(ABDIR)/%.h
	$(INSTALL) -m 444 $< $@

$(localincludedir)/%.h : $(UTILDIR)/%.h
	$(INSTALL) -m 444 $< $@

$(localincludedir)/%.h : $(VECTORDIR)/%.h
	$(INSTALL) -m 444 $< $@

