#!/bin/bash




cd build
cmake -DCMAKE_BUILD_TYPE=MinSizeRel -DLLVM_INCLUDE_TESTS=Off -DLLVM_INCLUDE_EXAMPLES=Off -DLLVM_ENABLE_PROJECTS=clang ../ 
make -j12
cd ../run_tree

../build/FlowControlGenerator