
rrdtool create db/dust_7d.rrd --step 300 DS:pm25:GAUGE:600:0:1000 DS:pm10:GAUGE:600:0:1000 RRA:MAX:0.5:1:2016
# step - 300s (5min) resolution between values
# 600 - 600s (10min) timeout, after which zero is inserted
# 0:1000 - min is 0, max is 1000 (spec says max value is 999.9)
# 2016 - we will store 2016 * step (300) values (7 days of history)


