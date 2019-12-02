#!/bin/bash
#
# Generates the zcm messages
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"
SOFTWARE_DIR="${SOFTWARE_DIR}/.."
GENERATED_DIR="${SOFTWARE_DIR}/generated/common/messages"
ZCM_TYPES_DIR="${SOFTWARE_DIR}/msgtypes"

echo ${SOFTWARE_DIR}

# Delete Generated messages
if [ -d "${SOFTWARE_DIR}/generated/common" ]; then
    rm -r ${SOFTWARE_DIR}/generated/common
fi

lcm-gen --cpp --cpp-hpath ${GENERATED_DIR} ${ZCM_TYPES_DIR}/*.lcm
