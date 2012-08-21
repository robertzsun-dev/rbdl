SRC_FILES := $(patsubst %.cc,%.mex,$(wildcard *.cc))

all: $(SRC_FILES)

%.mex: %.cc
	mkoctfile --mex -I../src/ -I../Debug/src -L../Debug -Wl,-rpath,../Debug -lrbdl $<

clean:
	rm -f *.mex *.o
