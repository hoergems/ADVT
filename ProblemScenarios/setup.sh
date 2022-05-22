#!/bin/bash

SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export OPPT_RESOURCE_PATH="${OPPT_RESOURCE_PATH}:${SCRIPTDIR}/models:$SCRIPTDIR/plugins:"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${SCRIPTDIR}/models:"
