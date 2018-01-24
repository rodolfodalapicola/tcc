a: Alg.cpp Alg.hpp
	g++ `pkg-config --cflags opencv` Alg.cpp `pkg-config --libs opencv` 
	./a.out
