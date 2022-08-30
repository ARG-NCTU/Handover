#! /bin/bash
# https://drive.google.com/drive/u/0/folders/1ylBJXGe7eM66R4D1h8D3BeKrQqKx7sqR

cwd=$PWD
echo -e "\e[93m Download Handover model \e[0m"
cd $cwd/src/ddqn/weight
echo "111111" | sudo -S gdown --id 1heQQHO6tzPKIfmhA_8B6pANtDpgb86TY
echo "111111" | sudo -S gdown --id 1oXs7x-D7POKzLc8XyIuLCmRHCZ0ji3O_

cd $cwd
