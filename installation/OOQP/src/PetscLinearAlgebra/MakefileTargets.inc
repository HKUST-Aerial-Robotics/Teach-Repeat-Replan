all_petsc_libs: $(libooqppetsc_STATIC)
all_petsc_headers: petsc_headers

petsc_headers: $(PETSCHEADERS)

$(libooqppetsc_STATIC) : $(libooqppetsc_OBJ)
	ar ru $@ $(libooqppetsc_OBJ)
	$(RANLIB) $@

$(localincludedir)/%.h : $(PETSCLINALGDIR)/%.h
	$(INSTALL) -m 444 $< $@
