#!/bin/sh

# CollisionAvoidance
cd CollisionAvoidance
./build.sh
cd ..

# ObjectFollowing
#cd ObjectFollowing
#./build.sh
#cd ..

# RoadFollowing
cd RoadFollowing
./build.sh
cd ..

# RoadFollowing-ssd
# RoadObjectFollowing
cd RoadObjectFollowing
./build.sh
cd ..
