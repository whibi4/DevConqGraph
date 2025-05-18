comp_dbg:
	g++ -std=c++17 -ltbb -g -Wall main.cpp -o devConqGraph
comp_rel:
	g++ -std=c++17 -O2 -Wall main.cpp -ltbb -o devConqGraph
run:
	./devConqGraph > output.txt
clean:
	rm ./devConqGraph output.txt