#!/bin/sh
export LD_LIBRARY_PATH=/home/yaojt0222/tools/che/ddk/ddk/uihost/lib/
TOP_DIR=${PWD}

rm -rf ${PWD}/out
mkdir -p ${PWD}/out

for file in ${TOP_DIR}/*
do
if [ -d "$file" ]
then
  if [ -f "$file/Makefile" ];then
    cd $file && make install
  fi
fi
done


cd ${TOP_DIR} && make
cp collision_avoidance_graph.config ./out/
cp models/collision_avoidance_2class_model.om ./out/
cp data/light_danger0_13dis_right4_60_0_22_dark_yuv.bin ./out/

