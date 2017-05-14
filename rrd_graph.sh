rrdtool graph img/dust_7d.png -w 800 -h 120 -a PNG --vertical-label "dust (µg/m3)" --slope-mode \
	DEF:pm25=db/dust_7d.rrd:pm25:MAX LINE1:pm25#ff0000:"PM2.5" \
	DEF:pm10=db/dust_7d.rrd:pm10:MAX LINE1:pm10#ff00ff:"PM10"
