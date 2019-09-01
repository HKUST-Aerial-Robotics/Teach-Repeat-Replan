all_headers: reader_headers
reader_headers: $(READERHEADERS)

$(localincludedir)/%.h : $(READERDIR)/%.h
	$(INSTALL) -m 444 $< $@

