#! /usr/bin/env bash

RAPCORE=openmpw-0.1.0

echo $RAPCORE

wget https://github.com/RAPcores/rapcores/archive/$RAPCORE.tar.gz
tar -xvf $RAPCORE.tar.gz
rm $RAPCORE.tar.gz

# mkdir -p verilog/rtl/rapcore

cp -r rapcores-$RAPCORE/src/* verilog/rtl
