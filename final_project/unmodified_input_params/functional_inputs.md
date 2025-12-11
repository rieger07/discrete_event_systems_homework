# Direct from paper proof:

## using edges.csv
____
```csv
#from,to,cost
S,1,4
S,2,10
S,3,20
S,c2,20
S,4,10
S,c1,2
1,c1,2
1,2,4
1,3,15
1,c2,20
1,4,25
2,3,4
2,4,15
2,c1,15
2,c2,3
3,c1,10
3,c2,3
3,4,10
c2,4,4
c2,c1,12
4,c1,3
```

# using nodes.csv
_______
```csv
#node_name,id,is_charging_station,is_target_site,threat_level
S,0,false,true,0
1,1,false,true,0
2,2,false,true,0
3,3,false,true,0
4,4,false,true,0
c1,5,true,false,5
c2,6,true,false,10
```

# config.ini
_____
```ini
# Battery System Configuration
MAX_BATTERY_CAPACITY=15
MIN_RESIDUAL_SOC=1
CHARGING_EFFICIENCY=1
DISCHARGING_EFFICIENCY=1
```


# My Modified Risk Algorithm Proof
## edges.csv
_____
