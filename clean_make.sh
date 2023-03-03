cd ~/rosnao
chmod +x sdk/make.sh
./sdk/make.sh
echo "=========================== SDK MAKE DONE =============================="
rm -rf build devel src/CMakeLists.txt
catkin_make