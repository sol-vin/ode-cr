git clone https://bitbucket.org/odedevs/ode
mkdir ode/build
cmake ode -DBUILD_SHARED_LIBS=ON -B ode/build
cmake --build ode/build
sudo make install -C ode/build
sudo cp /usr/local/lib/libode.so.8.0.0 /usr/lib/libode.so.8
sudo ln -s /usr/lib/libode.so.8 /lib/ode.so

rm -r ode
