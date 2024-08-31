#!/bin/bash
LICENSE_FILE="./LicenseHeader"

# Move to root of repo
cd $(git rev-parse --git-path ..)

FORMAT_ROOT="$(dirname '$0')"
FORMAT_FILE="$FORMAT_ROOT/.clang-format"
TIDY_FILE="$FORMAT_ROOT/.clang-tidy"

if [ ! -f $LICENSE_FILE ]; then
    echo -e "\033[31mNo license header found\033[0m]"
    exit -1
fi

FILES="$(find src include -name "*.cpp" -or -name "*.h" -or -name "*.inc" -type f 2> /dev/null)"

WPIPROJ=$(find . -type d -name ".wpilib")
if [ -n $WPIPROJ ]; then
    # Robot project
    echo -e "\033[32mRobot project detected\033[0m"
    HEADERS="$(cat $WPIPROJ/../build/vscodeconfig.json | jq -r '.[0].allLibHeaders[]')"
elif [ -f "compile_commands.json" ]; then
    # Automatically generates includes from compile_commands.json
    echo -e "\033[32mLibrary project with compile_commands.json detected\033[0m"
    HEADERS="$(cat compile_commands.json | jq -r '.[0].arguments.[]' | tr '\n' ' ' | grep -oE '\-I *(.*?) ' | sed 's/-I *//' | tr -d '\n')"
else
    echo -e "\033[32mLibrary project detected\033[0m"
    HEADERS=(lib/*/include)
fi

# Load variables from .env
if [ -f .env ]; then
    source .env
fi

# Add additional include directories from $INCLUDE
if [ -n "$INCLUDE" ]; then
    echo -e "\033[32mDetected additional include directories\033[0m"
    HEADERS+=$INCLUDE
fi

# Exclude files from linting if specified in $EXCLUDE_LINT
if [ -n "$EXCLUDE_LINT" ]; then
    echo -e "\033[32mDetected files to exclude from linting\033[0m"
    for EXCLUDE in $EXCLUDE_LINT; do
        FILES=$(echo "$FILES" | grep -v "$EXCLUDE")
    done
fi

LICENSE_LENGTH=$(wc -l < $LICENSE_FILE)
LICENSE_TEXT=$(head -n $LICENSE_LENGTH $LICENSE_FILE 2> /dev/null)

while read -r FILE; do
    # CHECK HEADER
    FILE_HEADER=$(head -n $LICENSE_LENGTH $FILE 2> /dev/null)
    if [ "$LICENSE_TEXT" == "$FILE_HEADER" ]; then
        echo -e "\033[32mHeader in file $FILE found\033[0m"
    else
        TEMP_FILE=$(mktemp)
        cat "$LICENSE_FILE" > "$TEMP_FILE"
        cat "$FILE" >> "$TEMP_FILE"
        mv "$TEMP_FILE" "$FILE"
        echo -e "\033[31mHeader in file $FILE not found\033[0m"
    fi

    # CHECK TRAILING NEWLINES
    echo -e "\033[32mChecking trailing newlines in $FILE\033[0m"
    if [ "$(tail -c 1 "$FILE" | wc -l)" -eq "0" ]; then
        echo "" >> "$FILE"
    else
        while [ "$(tail -c 2 "$FILE" | wc -l)" -gt "1" ]; do
            truncate -s -1 "$FILE"
        done
    fi
done <<< "$FILES"

INCLUDE_FLAGS=$(echo $HEADERS | tr " " "\n" | awk '{print "-I"$0}' | tr "\n" " ")
echo $INCLUDE_FLAGS

# clang-tidy function
function ct() {
    clang-tidy --config-file="$TIDY_FILE" $1   \
    -- -x c++ -std=c++20 $INCLUDE_FLAGS

    if [ $? != 0 ]; then
        echo -e "\033[31mFailed linting $1\033[0m"
        exit 1
    else
        echo -e "\033[32mFinished linting $1\033[0m"
    fi
}

# Export ct and variables so that they can be used in parallel
export -f ct
export INCLUDE_FLAGS
export TIDY_FILE

FAILED_LINTING=false
parallel -j-1 --halt never,fail=1 ct ::: $FILES
if [ $? != 0 ]; then
    FAILED_LINTING=true
fi

while read -r FILE; do
    clang-format -i -style="file:$FORMAT_FILE" "$FILE"
    echo -e "\033[32mFinished formatting $FILE\033[0m"
done <<< "$FILES"

if [ "$FAILED_LINTING" = true ]; then
    echo -e "\033[31mLinting failed\033[0m"
    exit 1
fi

echo -e "\033[32mLinting and formatting successful\033[0m"
