#!/bin/bash

base=$(cd $(dirname $0)/.. && pwd)

rm -r $base/build
mkdir -p $base/build
cd $base/build

cmake ..
make -j8 