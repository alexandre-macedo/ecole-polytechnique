OCB = ocamlbuild -classic-display

.PHONY: all
all:
	$(OCB) td4.native

.PHONY: clean
clean:
	$(OCB) -clean
	$(RM) version.ml
	$(RM) td4

.PHONY: byte
byte:
	$(OCB) td4.byte

.PHONY: gitclean
gitclean:
	git clean -xfd -e examples

.PHONY: top
top: all
	$(OCB) td4.cma
	ocaml
