CC=/usr/local/Cellar/llvm/11.0.0/bin/clang-11
PCLDIR=/usr/local/include/pcl-1.11
BOOSTDIR=/usr/local/include
EIGENDIR=/usr/local/include/eigen3

CFLAGS=-I$(PCLDIR) -I$(BOOSTDIR) -I$(EIGENDIR) -std=c++14
LDFLAGS=-L/usr/local/lib/ -std=c++14 -fopenmp
LDLIBS=-lflann_cpp -lboost_system-mt -lpcl_kdtree -lpcl_keypoints -lpcl_apps -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_ml -lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_simulation -lpcl_stereo -lpcl_surface -lpcl_tracking -lpcl_visualization -lboost_date_time-mt -lboost_filesystem-mt -lboost_iostreams-mt -lboost_program_options-mt -lboost_regex-mt -lboost_system-mt -lstdc++

smoothnet: main.o core.o
	$(CC) main.o core.o $(LDFLAGS) $(LDLIBS) -o smoothnet

core.o: ./core/core.cpp
	$(CC) $(CFLAGS) -c ./core/core.cpp -o core.o # Runs second

main.o:
	$(CC) $(CFLAGS) -c main.cpp -o main.o

clean:
	rm -f *.o