#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cargo run --manifest-path $SCRIPT_DIR/../ros2dashboard/rust/rust_ros2monitor/Cargo.toml