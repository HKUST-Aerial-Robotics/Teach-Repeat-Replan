all_libs: $(libooqpsparse_STATIC)

all_headers: splinalg_headers
splinalg_headers: $(SPLINALGHEADERS)

$(libooqpsparse_STATIC): $(libooqpsparse_OBJ)
	ar ru $@ $(libooqpsparse_OBJ)
	$(RANLIB) $@

$(localincludedir)/%.h : $(SPLINALGDIR)/%.h
	$(INSTALL) -m 444 $< $@
