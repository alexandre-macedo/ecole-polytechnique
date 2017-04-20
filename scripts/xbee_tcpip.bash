#!/bin/bash
#port="/dev/ttyUSB0"
baudrate="9600"
my_ip="192.168.10.1"
target_ip="192.168.10.2"

get_port()
{
    temp=$(dmesg | grep "FTDI USB Serial Device converter now attached to")
    eval "$1='/dev/${temp##*" "}'"
}

complete_restart()
{
    echo "Starting slattach... (1/3)"
    /sbin/slattach -s $baudrate $port &
    pid=$!
    sleep 2
    if kill -0 $pid > /dev/null 2>&1
    then
        echo "Succes."
        echo
        echo "Configuring P2P... (2/3)"
        if /sbin/ifconfig sl0 $my_ip pointopoint $target_ip up
        then
            echo "Success."
            echo
            if !(/sbin/route | grep "sl0") > /dev/null
            then
                echo "Adding route... (3/3)"
                if /sbin/route add -host $target_ip dev sl0
                then
                    echo "Success."
                else
                    echo "Fail."
                    exit
                fi
            else
                echo "Route already initialized. (3/3)"
                echo "Success."
            fi
        else
            echo "Fail."
            exit
        fi
    else
        echo "Fail."
        exit
    fi
}

kill_slattach()
{
## Kill /sbin/slattach if it exists
        echo "Looking for slattach to kill."
        if /usr/bin/pgrep "slattach" > /dev/null
        then
            echo "Slattach found."
            echo
            echo "Killing slattach..."
            if kill -9 $(/usr/bin/pgrep "slattach")
            then
                echo "Killed."
            else
                echo "Process not killed."
            fi
        else
            echo "Not found."
        fi
}

## Header
date
echo
ps $$
echo

## Look for device
port=''
get_port port
if [[ -z $port ]]
then
    echo "No candidate for Xbee device found."
    echo
    kill_slattach
    echo
    echo "Fail. Candidate not found."
else
    echo "Candidate found at $port"
    echo
    ## Check if device exists
    echo "Checking device..."
    if ! (ls $port) > /dev/null 2>&1
    then
        ## Device does not exist
        echo "Device not found."
        echo
        kill_slattach
        echo
        echo "Fail. Device not found."
        exit
    else
        ## Device exists
        echo "Device found."
        echo
        echo "Checking slattach."
        ## Check slattach
        if /usr/bin/pgrep "slattach" > /dev/null
        then
            echo "Slattach runnning."
            echo
            ## Check ifconfig
            echo "Checking for ifconfig..."
            if ! (/sbin/ifconfig | grep "sl0") > /dev/null 2>&1
            then
                echo "Config not found."
                echo
                echo "Reconfiguring..." 
                if /sbin/ifconfig sl0 $my_ip pointopoint $target_ip up
                then
                    echo "Success."
                else
                    echo "Fail."
                    exit
                fi
            else
                echo "Ifconfig found."
            fi
            echo
            ## Check for route
            echo "Checking route..."
            if  ! (/sbin/route | grep "sl0") > /dev/null 2>&1
            then
                echo "Route not found."
                echo
                echo "Creating route..."
                if /sbin/route add -host $target_ip dev sl0
                then
                    echo "Success."
                else
                    echo "Fail."
                    exit
                fi
            else
            echo "Route found."
            fi
        else
            ## If slattach reconfigure.
            echo "Slattach not running. Restarting..."
            echo
            complete_restart
        fi
    fi
fi