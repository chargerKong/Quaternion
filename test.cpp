#include "test.hpp"
klq::int fac(int n){
	if (n == 0)
		return 0;
	else if (n == 1)
		return n;
	else
		return n * fac(n - 1);


