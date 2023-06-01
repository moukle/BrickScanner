#! /bin/bash

DIR=$1

cd $DIR
mkdir view_1 view_2 -p
ls

cp -r left right view_1
cp -r left right view_2
