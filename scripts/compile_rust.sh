#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CURRENT_DIR==$(pwd)

cd $SCRIPT_DIR/../ros2dashboard/rust/rust_ros2monitor/
maturin develop --release
cd $CURRENT_DIR