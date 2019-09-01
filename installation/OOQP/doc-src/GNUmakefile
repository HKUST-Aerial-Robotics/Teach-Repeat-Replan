SHELL = /bin/sh

DOC           = ../doc
LATEX       = pdflatex
BIBTEX      = bibtex
DVI         = pdf
POD2HTML    = pod2html
POD2TEXT    := $(shell if [ -x pod2txt ];\
	 		then echo pod2txt;\
	 		else echo pod2text;\
	 		fi)
DOXYGEN     = doxygen
OOQPUSERGUIDE = $(DOC)/ooqp-userguide.$(DVI)

MODULES = Svm Huber QpBound 
MODULEHTML  = $(patsubst %,$(DOC)/formulations/%.html,$(MODULES))

DISTRIBUTIONDOCS = README INSTALL README_Matlab README_Windows COPYRIGHT
DISTDOCS = $(patsubst %,../%, $(DISTRIBUTIONDOCS)) \
	   $(patsubst %,$(DOC)/distribution-docs/%.html, $(DISTRIBUTIONDOCS))

EXAMPLES_README = $(DOC)/examples/README.html ../examples/README

OOQPUSERGUIDESECTIONS = ooqp-userguide/ooqp1intro.tex \
	ooqp-userguide/ooqp2qpgen.tex \
	ooqp-userguide/ooqp3develop.tex ooqp-userguide/ooqp4qpsolver.tex \
	ooqp-userguide/ooqp5creating.tex ooqp-userguide/ooqp6usinglinalg.tex\
	ooqp-userguide/ooqp7newlinalg.tex ooqp-userguide/ooqp-userguide.tex

all: most reference_manual
most: $(OOQPUSERGUIDE)  $(MODULEHTML) $(DISTDOCS) \
	$(EXAMPLES_README)

$(OOQPUSERGUIDE) : ooqp-userguide/ooqp-userguide.$(DVI)
	cp -f $< $@

reference_manual:
	doxygen

ooqp-userguide/ooqp-userguide.$(DVI) : ooqp-userguide/refs.bib  \
				$(OOQPUSERGUIDESECTIONS)
	cd ooqp-userguide; \
	$(LATEX)  ooqp-userguide; \
	$(BIBTEX) ooqp-userguide; \
	$(LATEX)  ooqp-userguide; \
	$(LATEX)  ooqp-userguide

../README: distribution-docs/README.pod
	$(POD2TEXT) $< > $@

../examples/README: examples/README.pod
	$(POD2TEXT) $< > $@

../INSTALL: distribution-docs/INSTALL.pod
	$(POD2TEXT) $< > $@

../README_Matlab: distribution-docs/README_Matlab.pod
	$(POD2TEXT) $< > $@

../README_Windows: distribution-docs/README_Windows.pod
	$(POD2TEXT) $< > $@

../COPYRIGHT: distribution-docs/COPYRIGHT.pod
	$(POD2TEXT) $< > $@

$(DOC)/%.html: %.pod
	$(POD2HTML) --podpath . --recurse \
		 --htmlroot .. --infile $< --outfile $@

# clean - clean up by removing intermediate files created by the build
clean: ooqp-userguide-clean
	rm -f *~ doxygen_errors

# clean the ooqp-userguide directory
ooqp-userguide-clean:
	cd ooqp-userguide; \
	rm -f *.aux *.log *.dvi *.bbl *.blg *.toc *.pdf *~

# veryclean - (Possibly) clean more
veryclean: clean

# distclean - Remove everything not in the distribution
distclean: clean


# maintainer-clean - remove everything that can be rebuilt 
#		     (may require special tools)
maintainer-clean: distclean 
maintainer-clean: distdocs-maintainer-clean 
maintainer-clean: formulations-maintainer-clean
maintainer-clean: examples-maintainer-clean
maintainer-clean: doxygen-maintainer-clean
maintainer-clean: 
	rm $(OOQPUSERGUIDE) $(OOQPPAPER)

# Remove the generated README's and other distribution docs
distdocs-maintainer-clean:
	rm -f $(DISTDOCS)

# Remove the formulation manpages
formulations-maintainer-clean:
	rm -f $(MODULEHTML)

examples-maintainer-clean:
	rm -f $(EXAMPLES_README)

# Remove the doxygen-generated reference_manual
doxygen-maintainer-clean:
	rm -f $(DOC)/reference-manual/*.html $(DOC)/reference-manual/*.css
	rm -f $(DOC)/reference-manual/*.gif
	-rmdir $(DOC)/reference-manual




