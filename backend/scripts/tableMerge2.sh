#!/bin/bash
# This script merges several SQL shape file scripts into one file, and one table
variables=( pressfc tmpsfc rh2m vissfc tcdcclm refcclm apcpsfc crainsfc )
fileSeq=$(seq 2 1 $( ls -1 pressfc-*.shp | wc -l))

for variable in tmpsfc rh2m vissfc tcdcclm refcclm apcpsfc pressfc crainsfc
do
    #Load the first table
    shp2pgsql -D -i -d $variable-1.shp $variable  | psql -d toronto5
    
    #Append remaining shapefiles
    for j in $fileSeq
    do
    filename="$variable-$j"
    shp2pgsql -D -i $filename.shp -a $variable | psql -d toronto5
    done

#    rm -f $variable-*
done