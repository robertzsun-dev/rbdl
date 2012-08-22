RBDL_SRC_PATH="../src"
RBDL_BINARY_PATH="../Debug/"
RBDL_CONFIG_H_PATH="${RBDL_BINARY_PATH}/src"

# No configuration should be required below this line

SRC_FILES := $(patsubst %.cc,%.mex,$(wildcard *.cc))

all: $(SRC_FILES)

%.mex: %.cc
	mkoctfile --mex -I${RBDL_SRC_PATH} -I${RBDL_CONFIG_H_PATH} -L${RBDL_BINARY_PATH} -Wl,-rpath,${RBDL_BINARY_PATH} -lrbdl $<

clean:
	rm -f *.mex *.o
