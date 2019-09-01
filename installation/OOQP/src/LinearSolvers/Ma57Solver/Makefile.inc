MA57SOLVERDIR    = $(LINEARSOLVERSDIR)/Ma57Solver

MA57SOLVERSRC    = $(MA57SOLVERDIR)/Ma57Solver.C
MA57SOLVEROBJ    = $(MA57SOLVERDIR)/Ma57Solver.o

MA57SOLVERHEADERS = $(localincludedir)/Ma57Solver.h

# Some targets that I know are safe to include now.
# Yes, it breaks convention. So sue me.
all_headers: ma57solver_headers
ma57solver_headers: $(MA57SOLVERHEADERS)

$(localincludedir)/Ma57Solver.h: $(MA57SOLVERDIR)/Ma57Solver.h
	$(INSTALL) -m 444 $< $(localincludedir)
