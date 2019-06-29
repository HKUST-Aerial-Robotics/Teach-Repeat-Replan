sudo apt-get install -y ros-kinetic-joy &
sudo apt-get install -y libnlopt-dev &
sudo apt-get install -y libf2c2-dev &
sudo apt-get install -y libarmadillo-dev &
sudo apt-get install -y glpk-utils libglpk-dev &
sudo apt-get install -y libcdd-dev &
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test &
sudo apt-get update &
sudo apt-get install -y gcc-7 g++-7 &
sudo update-alternatives --install -y /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5 &
sudo update-alternatives --install -y /usr/bin/gcc gcc /usr/bin/gcc-7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-7
