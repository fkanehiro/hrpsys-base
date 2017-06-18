#!/bin/bash

EXIT_STATUS=0
CLANG_FORMAT="clang-format-3.7"

if [ ! -f ".clang-format" ]; then
    echo ".clang-format file not found!"
    exit 1
fi

FILES=`git ls-files | grep -E "\.(cpp|c|C|h|hpp|idl)$"`

for FILE in $FILES; do
    $CLANG_FORMAT $FILE | cmp  $FILE >/dev/null

    if [ $? -ne 0 ]; then
        echo "[!] INCORRECT FORMATTING! $FILE" >&2
        $CLANG_FORMAT -i $FILE
        echo "###########################################################################"
        git diff $FILE | cat
        echo "###########################################################################"
        EXIT_STATUS=1
    fi
done

exit $EXIT_STATUS
