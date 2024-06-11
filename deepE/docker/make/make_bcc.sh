#!/bin/bash

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

git clone https://github.com/iovisor/bcc.git
mkdir bcc/build; 
pushd bcc/build

cmake ..
make
make install
cmake -DPYTHON_CMD=python3 .. # build python3 binding
pushd src/python/
make
make install
popd

popd