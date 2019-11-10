.PHONY: all
all: testExercice3 testExercice4 pi442 mpi_helloBsend mpi_helloNBsend PremierMPI

testExercice3: Molecule-seq.o MonteCarloSequentiel.cpp
	g++ Molecule-seq.o MonteCarloSequentiel.cpp -o testExercice3

testExercice4: Molecule-dist.o MonteCarloDistrib.cpp
	mpic++ Molecule-dist.o MonteCarloDistrib.cpp -o testExercice4

Molecule-seq.o: Molecule.h Molecule.cpp
	g++ -c Molecule.cpp -o Molecule-seq.o

Molecule-dist.o: Molecule.h Molecule.cpp
	mpic++ -c Molecule.cpp -o Molecule-dist.o

PremierMPI: PremierMPI.cpp
	mpic++ PremierMPI.cpp -o PremierMPI

pi442: pi442.cpp
	mpic++ pi442.cpp -o pi442

mpi_helloBsend: mpi_helloBsend.cpp
	mpic++ mpi_helloBsend.cpp -o mpi_helloBsend

mpi_helloNBsend: mpi_helloNBsend.cpp
	mpic++ mpi_helloNBsend.cpp -o mpi_helloNBsend

.PHONY: clean
clean:
	rm -f Molecule-seq.o Molecule-dist.o testExercice3 testExercice4 pi442 mpi_helloBsend mpi_helloNBsend PremierMPI
