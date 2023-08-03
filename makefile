all:
	gcc -Iinclude src/*.c -Ofast -o a.exe

profile:
	gcc -Iinclude -pg src/*.c -no-pie -o a.exe
	a.exe
	gprof -b a.exe gmon.out > analysis.txt