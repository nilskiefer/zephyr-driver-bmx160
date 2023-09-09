#!/bin/bash

# This script is intended to workaround the issue where Bosh recommends using the BMI160 library
# for the BMX160 as well, however they have not updated the library to work with the chip IP that
# is used for the BMX160. This script modifies the defines to allow us to provide a custom chip
# id when necessary.
# Expected use:
#     ./bmx160_fix.bash "libs/BMI160_driver/bmi160_defs.h"
# Note: Once the file has been fixed, it still needs a define for the BMX160 chip id. eg:
#     DEFINES+=BMI160_CHIP_ID=UINT8_C\(0xD8\)

file="$1"
new_contents=""

ifndef="#if !defined(BMI160_CHIP_ID)"
define="#define BMI160_CHIP_ID                            UINT8_C(0xD1)"

while IFS= read -r line
do
    case "$line" in
        "$define"*)
            echo "Replacing define."
            printf -v new_contents '%s%s\n' "$new_contents" "$ifndef"
            printf -v new_contents '%s%s\n' "$new_contents" "$line"
            printf -v new_contents '%s%s\n' "$new_contents" "#endif"
            ;;
        "$ifndef"*)
            echo "Already fixed. Exiting."
            exit 0
            ;;
        *)
            printf -v new_contents '%s%s\n' "$new_contents" "$line"
            ;;
    esac
done < $file

printf "%s" "$new_contents" > $file
