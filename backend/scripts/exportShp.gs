* exportShp.gs <filename.ctl>
* This GrADS script generates shape files for selected variables
function exportShp(args)

filename = subwrd(args,1)

'open 'filename
'set gxout shp'

* loop through each time interval

t = 1

while(i<24)
'set t 'i

'q time'
tstamp = subwrd(result,3) 
'set shpattr time string 'tstamp

*'set shp -pt -fmt 8 2 tmpsfc-'i
*'d tmpsfc'
*'clear'
'set shp -pt -fmt 3 2 pressfc-'i
'd pressfc'
'clear'
*'set shp -pt -fmt 3 2 rhsfc-'i
*'d rhsfc'
*'clear'
*'set shp -pt -fmt 3 2 acpcpsfc-'i
*'d acpcpsfc'
*'clear'
*'set shp -pt -fmt 3 2 refcclm-'i
*'d refcclm'
*'clear'
i=i+1
endwhile

'quit'
return 0