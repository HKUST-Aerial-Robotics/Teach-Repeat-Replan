all_headers: cinterface_headers
cinterface_headers: $(CINTERFACEHEADERS)

$(localincludedir)/%.h : $(CINTERFACEDIR)/%.h
	$(INSTALL) -m 444 $< $@

