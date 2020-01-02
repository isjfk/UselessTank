#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

INIT_SCRIPT_FILE="${SCRIPT_DIR}/__init_scripts.sh"
LOG_FILE="${SCRIPT_DIR}/init-setup.log"

if [ -f "$LOG_FILE" ]; then
    echo "Log of init-setup already exists."
    echo "Don't execute init-setup.sh multiple times, otherwise there may redundance lines writen into some configuration files!"
    echo "If your previous execution of init-setup failed, execute rest of the steps manually!"
    exit 1
fi

echo "Writing init-setup logs into: $LOG_FILE"
exec > >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${LOG_FILE} >&2)

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
done < "$INIT_SCRIPT_FILE"
