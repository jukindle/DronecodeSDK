export LD_LIBRARY_PATH=/usr/local/lib/
rm -r build
mkdir build
cd build
cmake ..
make
cd ..
