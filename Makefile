comp:
	g++ -std=c++17 -g -Wall main.cpp -o solution
run:
	./solution > output.txt
clean:
	rm ./solution output.txt