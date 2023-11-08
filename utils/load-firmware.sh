#!/bin/bash

while getopts f: flag
do
        case "${flag}" in
                f) filename=${OPTARG};;
        esac
done

echo "Writing $filename to ST-Link...\n"

sudo st-info --probe
sudo st-flash write $filename 0x80000000
sudo st-flash reset

