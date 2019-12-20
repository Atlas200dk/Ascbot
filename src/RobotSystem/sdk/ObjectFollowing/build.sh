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
cp object_following_graph.config ./out/
cp models/object_detection_deploy.om ./out/
cp data/dangerous-box-video4-015_nv12.bin ./out/


