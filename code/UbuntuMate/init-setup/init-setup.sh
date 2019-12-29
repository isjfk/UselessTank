#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

function checkExitCode {
    retVal=$?
    cmd="$*"
    if [ $retVal -ne 0 ]; then
        echo "Error execute command: $cmd"
        exit 1
    fi
}

while IFS=$' \t\n\r' read -r cmd || [ -n "$cmd" ]; do
    [[ -z "${cmd}" ]] && continue
    [[ "${cmd}" =~ ^#.* ]] && echo "####${cmd} #####" && continue

    echo "#***************************************"
    echo "# ${cmd}"
    echo "#***************************************"

    eval "${cmd}"
    checkExitCode "${cmd}"
done < "${SCRIPT_DIR}/__init_scripts.sh"
