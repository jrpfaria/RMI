#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
	# activate virtual environment, if needed
	cd C1
        python3 mainRobC1.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        cd C23
        python3 mainRobC2.py -h "$host" -p "$pos" -r "$robname"
        mv map.out ../$outfile.path
        ;;
    3)
        # how to call agent for challenge 3
        cd C23
        python3 mainRobC3.py -h "$host" -p "$pos" -r "$robname"
        mv plan.out ../$outfile.path
        ;;
    4)
        rm -f *.path *.map  # do not remove this line
        # how to call agent for challenge 4
        python3 mainRobC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv map.out $outfile.map
        mv path.out $outfile.path
        ;;
esac

