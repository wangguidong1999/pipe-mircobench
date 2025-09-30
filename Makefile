# Makefile
pipe-microbench: pipe-microbench.cpp
	$(CXX) -O3 -std=c++17 -static $< -o $@

clean:
	rm -f pipe-microbench

.PHONY: clean
