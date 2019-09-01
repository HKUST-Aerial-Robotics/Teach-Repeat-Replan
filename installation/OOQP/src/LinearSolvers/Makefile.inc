LINEARSOLVERSDIR = $(srcdir)/LinearSolvers

LSMODULEMAKEFILES  := $(wildcard $(LINEARSOLVERSDIR)/*/Makefile.inc)
LSMODULES          := $(dir $(LSMODULEMAKEFILES))
INCLUDES           += $(patsubst %, -I%, $(LSMODULES))
SOURCES   += $(foreach MODULE, $(LSMODULES), $(wildcard $(MODULE)*.[cC]))

include $(LSMODULEMAKEFILES)
