comp_dbg:
	g++ -std=c++17 -g -Wall main.cpp -o devConqGraph
comp_rel:
	g++ -std=c++17 -O2 -Wall main.cpp -o devConqGraph
run:
	./devConqGraph > output.txt
clean:
	rm ./devConqGraph output.txt