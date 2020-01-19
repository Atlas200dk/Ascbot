1. 目录
-- build.sh \
-- CollisionAvoidance \
-- README.txt \
-- RoadFollowing \
-- RoadObjectFollowing \

2. 编译
1) 需要在编译服务器上先安装DDK工具
2) 需要把build.sh中LD_LIBRARY_PATH路径改为编译机器信赖库对应路径，
把Makefile中DDK_HOME改为编译服务器对应的DDK安装路径
3) 用./build.sh脚本进行编译
4) 编译生成文件在每个模块的out目录下


