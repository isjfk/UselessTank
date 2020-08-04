source ~/catkin_ws/devel/setup.bash

export ROS_IP=`ifconfig wlp2s0 | grep "inet " | awk '{ print $2 }'`
export ROS_MASTER_URI="http://${ROS_IP}:11311"
export SHUTDOWN_CMD='echo "Initial0" | sudo -S shutdown now'

alias refreshEnv='source ~/.bashrc'
alias supass='echo "Initial0" | sudo -S ls >&/dev/null'
alias sushutdown='echo "Initial0" | sudo -S shutdown now >&/dev/null'
alias sureboot='echo "Initial0" | sudo -S reboot >&/dev/null'

alias pymod='find ~/catkin_ws/src \( -name "*.py" -o -name "*.cfg" \) -exec chmod a+x {} \;'
alias ws='cd ~/catkin_ws'
alias wssrc='cd ~/catkin_ws/src'
alias ckclean='cd ~/catkin_ws && catkin_make clean && cd -'
#alias ckmake='cd ~/catkin_ws && catkin_make -j1 -l1 -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release && cd -'
alias ckmake='cd ~/catkin_ws && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release && cd -'
alias wsclean='ckclean'
alias wsmake='pymod && ckmake'
alias rosreconfig='rosrun rqt_reconfigure rqt_reconfigure'
alias tanksetup='roslaunch tank_setup tank_setup.launch --screen'
alias tankgmapping='roslaunch tank_2dnav gmapping.launch --screen'
alias tanknav='roslaunch tank_2dnav tank_nav.launch --screen'
alias tank='roslaunch tank_2dnav tank.launch --screen'
alias tankjoy='roslaunch tank_remote xbox.launch --screen'
alias tankjoysa='roslaunch tank_remote xboxsa.launch --screen'
alias tankkeyboard='roslaunch tank_remote keyboard.launch --screen'

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
