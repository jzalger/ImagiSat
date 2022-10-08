#!/bin/bash
# This script merges several SQL shape file scripts into one file, and one table
variables=('pressfc')
fileSeq=$(seq 1 1 $( ls -1 $variable*.shp | wc -l))

for variable in $variables
do
    #Generate sql files from shape files
    for j in $fileSeq
    do
    filename="$variable-$j"
    shp2pgsql $filename.shp > $filename.sql
    done

    #Generate header for final sql file
    filename="$variable-1"
    sed "s/$filename/$variable/g" $filename.sql > $filename.sql.temp
    head -n 10 $filename.sql.temp > $variable.sql.temp

    #Remove header, replace variable text, and append to master sql file
    for i in $fileSeq
    do
      filename=$variable-$i
      sed '1,10d' $filename.sql > $filename.sql.temp
      sed "s/$filename/$variable/g" $filename.sql.temp >> $variable.sql.temp
      rm -f $filename.sql.temp
    done

    #Clean the comit statements, leaving the final one
    sed '/COMMIT;/d' $variable.sql.temp > $variable.sql
    echo "COMMIT;" >> $variable.sql
    rm -f $variable.sql.temp
    rm -f $variable-*
done