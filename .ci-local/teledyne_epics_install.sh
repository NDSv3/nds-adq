#!/bin/bash 

# This script installs NDS and NDS-epics
#
# check if user has right permissions
if [ "$(id -u)" != "0" ]; then
    echo "Sorry, you are not root. Please try again using sudo."
    exit 1
fi

# terminate script after first line that fails
set -e

apt-get install -y libreadline6-dev libncurses5-dev perl clang g++-mingw-w64-i686 g++-mingw-w64-x86-64 qemu-system-x86 re2c tar
apt-get install -y build-essential git python curl p7zip-full wget

export GITLAB_CI=1
export SETUP_PATH=".ci-local:.ci"
export BASE_RECURSIVE="NO"
export CMP="gcc"
export BCFG="default"
export SET="stable"
export CLEAN_DEPS="NO"
export WINE=32
export MODULES="sncseq asyn ndsepics"
export BASE=R7.0.3.1
export ASYN=R4-41
export SNCSEQ=R2-2-9
export NDSEPICS_REPOURL="https://github.com/pheest/nds-epics"
export NDSEPICS_DIRNAME="nds-epics"
export NDSEPICS_HOOK=.cache/nds-epics-master/.ci-local/NDS3_install.sh


apt install -y ./ADQAPI/libadq0_0.66850-0_amd64.deb

python .ci/cue.py prepare > /tmp/teledyne_epics_install.log 2>&1
python .ci/cue.py build > /tmp/teledyne_epics_install.log 2>&1

