sudo apt-get install -y ros-kinetic-joy & sleep 0.2;
sudo apt-get install -y libnlopt-dev & sleep 0.2;
sudo apt-get install -y libf2c2-dev & sleep 0.2;
sudo apt-get install -y libarmadillo-dev & sleep 0.2;
sudo apt-get install -y glpk-utils libglpk-dev & sleep 0.2;
sudo apt-get install -y libcdd-dev & sleep 0.2;
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test & sleep 0.2;
sudo apt-get update & sleep 0.2;
sudo apt-get install -y gcc-7 g++-7 & sleep 0.2;
sudo update-alternatives --install -y /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5 & sleep 0.2;
sudo update-alternatives --install -y /usr/bin/gcc gcc /usr/bin/gcc-7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-7
