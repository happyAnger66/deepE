#!/bin/bash

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

#export HTTPS_PROXY=
#git config --global http.proxy your_proxy
#git config --global http.sslverify false
git clone https://github.com/iovisor/bcc.git
mkdir bcc/build; 
pushd bcc/build

cmake ..
make -j6
make install
cmake -DPYTHON_CMD=python3 .. # build python3 binding
pushd src/python/
make -j6
make install
popd

popd