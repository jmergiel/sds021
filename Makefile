default:
	$(CXX) $(CPPFLAGS) $(CFLAGS) -std=c++1z -g -O0 -Wall -pedantic sds021.cpp -o sds021

interactive:
	$(CXX) $(CPPFLAGS) $(CFLAGS) -DINTERACTIVE -std=c++1z -g -O0 -Wall -pedantic sds021.cpp -o sds021 -lpthread

.PHONY: clean
clean:
	rm -f sds021

