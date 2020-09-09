#! /bin/bash

# echo "password: $2"
BRANCH=master
if [ ! -z "$1" ]; then
    echo "pull branch: $1"
    BRANCH=$1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull racecar------------------------------"
echo "-----------------------------------------------------------------------"
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in subt-system. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull subt-core--------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/racecar/catkin_ws/src/subt-core
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in subt-core. Aborting"
   return 1
fi

cd ~/racecar
return 0