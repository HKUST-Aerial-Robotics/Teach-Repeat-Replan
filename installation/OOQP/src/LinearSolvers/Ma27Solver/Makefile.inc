MA27SOLVERDIR    = $(LINEARSOLVERSDIR)/Ma27Solver

MA27SOLVERSRC    = $(MA27SOLVERDIR)/Ma27Solver.C
MA27SOLVEROBJ    = $(MA27SOLVERDIR)/Ma27Solver.o

MA27SOLVERHEADERS = $(localincludedir)/Ma27Solver.h

# Some targets that I know are safe to include now.
# Yes, it breaks convention. So sue me.
all_headers: ma27solver_headers
ma27solver_headers: $(MA27SOLVERHEADERS)

$(localincludedir)/Ma27Solver.h: $(MA27SOLVERDIR)/Ma27Solver.h
	$(INSTALL) -m 444 $< $(localincludedir)
