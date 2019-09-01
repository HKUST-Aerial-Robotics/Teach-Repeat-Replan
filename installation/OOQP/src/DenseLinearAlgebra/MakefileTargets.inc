all_libs: $(libooqpdense_STATIC)

all_headers: delinalg_headers
delinalg_headers: $(DELINALGHEADERS)

$(libooqpdense_STATIC): $(libooqpdense_OBJ)
	ar ru $@ $(libooqpdense_OBJ)
	$(RANLIB) $@

$(localincludedir)/%.h : $(DELINALGDIR)/%.h
	$(INSTALL) -m 444 $< $@
