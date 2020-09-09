#!/bin/bash



if [ ! "$1" ]; then
    echo "commit detail please"
    return
fi
echo "commit: $1"

COMMIT=$1
BRANCH=master

if [ ! -z "$2" ]; then
    echo "operator on branch: $2"
    BRANCH=$2
fi

source git_pull.sh $BRANCH
PULLSTAT=$?
if [ "$PULLSTAT" -gt 0 ] ; then
   echo "There is conflict. Aborting"
   cd ~/racecar/
   return
fi
echo "-------------------------pull success----------------------------------"

# push core
echo "-----------------------------------------------------------------------"
echo "-------------------------push racecar------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/racecar/catkin_ws/src/subt-core
git add -A
git commit -m "$1 on core"
git push

# push main
echo "-----------------------------------------------------------------------"
echo "-------------------------push subt-core--------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/racecar/
git add -A
git commit -m "$1"
git push 
