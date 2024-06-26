# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

source ~/catkin_ws/devel/setup.bash

export ROS_IP=`ifconfig wlan0 | grep "inet " | awk '{ print $2 }'`
export ROS_MASTER_URI="http://${ROS_IP}:11311"

alias refreshEnv='source ~/.bashrc'
alias supass='echo "Initial0" | sudo -S ls >&/dev/null'
alias sushutdown='echo "Initial0" | sudo -S shutdown now >&/dev/null'
alias sureboot='echo "Initial0" | sudo -S reboot >&/dev/null'

alias pymod='find ~/catkin_ws/src \( -name "*.py" -o -name "*.cfg" \) -exec chmod a+x {} \;'
alias ws='cd ~/catkin_ws'
alias wssrc='cd ~/catkin_ws/src'
alias ckmake='cd ~/catkin_ws && catkin_make -j1 -l1 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release && cd -'
alias wsmake='pymod && ckmake'
alias tanksetup='roslaunch tank_setup tank_setup.launch'
alias tankgmapping='roslaunch tank_2dnav gmapping.launch'
alias tanknav='roslaunch tank_2dnav tank_nav.launch'
alias tank='roslaunch tank_2dnav tank.launch'
alias tankds4='roslaunch tank_remote ds4.launch'
alias tankds4sa='roslaunch tank_remote ds4sa.launch'
alias tankkeyboard='roslaunch tank_remote keyboard.launch'

alias tgsrc='cd ~/package/tank-guider'
alias tg='tgsrc && npm start'

alias tankstart='(roslaunch tank_2dnav tank.launch) & (tgsrc && npm start) &'
alias tankstop='(rosnode kill -a) & (killall node) &'

HOME_DIR=$(cd ~; pwd)
TANK_MODEL_CFG_FILE="${HOME_DIR}/tank_model.env"
if [ ! -f "$TANK_MODEL_CFG_FILE" ]; then
    cat >"$TANK_MODEL_CFG_FILE" <<EOL
#export TANK_MODEL=prototype-v2-basic
#export TANK_MODEL=prototype-v2-depth
#export TANK_MOTOR_MODEL=HM-GM37-3429-12V-50-ENCODER
#export TANK_MOTOR_MODEL=CHR-GM37-3429-12V-30-ENCODER
EOL
else
    source "$TANK_MODEL_CFG_FILE"
fi

if [[ -z "$TANK_MODEL" ]] || [[ -z "$TANK_MOTOR_MODEL" ]]; then
    echo "WARN: Please set TANK_MODEL & TANK_MOTOR_MODEL in $TANK_MODEL_CFG_FILE according to tank setup!"
    echo "      Then execute command: refreshEnv"
else
    cat <<EOL
############ UselessTank ############
Model: $TANK_MODEL
Motor: $TANK_MOTOR_MODEL
#####################################
EOL
fi
