#!/usr/bin/env bash

cwd=$PWD

if [ ! -d $cwd/model ]; then
    mkdir model
else
    echo "folder exist"
fi

cd $cwd/model

# Andy Zeng ConvNet
echo "Download HANet pretrained weight"
echo "111111" | sudo -S gdown --id 1uB8HLrW0gnOQVXYd2DevbKjMtPLCUAjv

cd ..