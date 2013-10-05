#!/bin/bash
#
# sample start script for 3D soccer simulation
#

AGENT_BINARY="seu-spark-agent"
BINARY_DIR="./"
killall -9 "$AGENT_BINARY" &> /dev/null
export LD_LIBRARY_PATH=./libs/:$LD_LIBRARY_PATH;
echo "Running agent No. 1"
"$BINARY_DIR/$AGENT_BINARY" -t SEU_Jolly -u 1 > /dev/null 2> /dev/null&
sleep 2
