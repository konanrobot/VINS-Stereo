#!/bin/sh

file="$1"

echo "evaluate file:" $file;

mono=`cat $file |grep mono|grep ":0"|wc -l`
stereo=`cat $file |grep stereo|grep ":0"|wc -l`
master=`cat $file |grep master|grep ":0"|wc -l`
all=`cat $file |grep master|wc -l`

echo "CONVERGENCE"
echo "      mono:" $mono
echo "    stereo:" $stereo
echo "    master:" $master
echo "       all:" $all

