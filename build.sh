echo
echo "----------------- 1、Configuring and building Thirdparty/DBoW2 ... ---------------------------------"

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo
echo "----------------- 2、Configuring and building Thirdparty/g2o ... ---------------------------------"

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo
echo "----------------- 3、Configuring and building Thirdparty/Sophus ... ---------------------------------"

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

# 解压字典文件，解压一次然后注释掉即可！
echo
echo "-------------------------------- 4、Uncompress vocabulary ... -----------------------------------------"

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo
echo "----------------- Boss: Configuring and building ORB_SLAM3 ... -----------------------------------------"

mkdir build
cd build
# cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j4
make -j # 默认使用系统上的所有 CPU 核心进行并行编译。
