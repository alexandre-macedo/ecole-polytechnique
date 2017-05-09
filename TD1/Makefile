OCB = ocamlbuild -classic-display

.PHONY: all
all:
	$(OCB) td1.native

.PHONY: clean
clean:
	$(OCB) -clean
	$(RM) version.ml
	$(RM) td

.PHONY: byte
byte:
	$(OCB) td1.byte

.PHONY: gitclean
gitclean:
	git clean -xfd -e examples

.PHONY: top
top: all
	$(OCB) td1.cma
	ocaml
