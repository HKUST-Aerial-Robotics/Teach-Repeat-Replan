qpexample-dense-gondzio.exe: $(qpexample_dense_gondzio_OBJECTS)
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) $(LIBS) \
		 $(qpexample_dense_gondzio_OBJECTS) $(BLAS) $(FLIBS)

