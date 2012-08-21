all: rbdlCreateModel.mex rbdlDestroyModel.mex rbdlGetDofCount.mex rbdlAddBody.mex

rbdlCreateModel.mex: rbdlCreateModel.cc
	mkoctfile --mex -I../src -I../Debug/src -L../Debug -Wl,-rpath,../Debug -lrbdl rbdlCreateModel.cc

rbdlDestroyModel.mex: rbdlDestroyModel.cc
	mkoctfile --mex -I../src -I../Debug/src -L../Debug -Wl,-rpath,../Debug -lrbdl rbdlDestroyModel.cc

rbdlGetDofCount.mex: rbdlGetDofCount.cc
	mkoctfile --mex -I../src -I../Debug/src -L../Debug -Wl,-rpath,../Debug -lrbdl rbdlGetDofCount.cc

rbdlAddBody.mex: rbdlAddBody.cc
	mkoctfile --mex -I../src -I../Debug/src -L../Debug -Wl,-rpath,../Debug -lrbdl rbdlAddBody.cc

clean:
	rm -f *.mex
