#!/bin/bash

./build.sh || exit 1

./deploy_and_monitor.sh
