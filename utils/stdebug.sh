#!/bin/bash

function usage() {
        scriptname=$(basename "$0")
        echo "Usage: $scriptname [-f firmware] -h "
        echo ""
        echo "      -f [firmware_file]  Firmware file to be loaded to board,"
        echo "                          can be in .bin or .elf format"
        echo ""
        echo "      -h                  Show this help message"
        echo ""
}

if [[ $# -eq 0 ]]; then
        echo "No arguments provided"
        echo ""
        usage
        exit 1
fi

while getopts "hf:" flag
do
        case "${flag}" in
                h) usage
                exit 0;;
                f) filename=${OPTARG};;
                \?) usage
                exit 1;;
        esac
done

echo "Starting GDB with $filename"
echo ""

sudo gdb-multiarch --tui $filename -ex 'target remote localhost:4242'
