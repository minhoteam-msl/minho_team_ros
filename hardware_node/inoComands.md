# Useful ino tool comands
* Listing all available building models
```
ino list-models 
```
* Open a serial monitor 
```
ino serial              
```
* Build firmware from the current directory project for specific model (ex: arduino pro-mini 5v atmega328p)
```
ino build -m pro5v328                             
```
* Upload built firmware to the device for specific model (ex: arduino pro-mini 5v atmega328p)
```
ino upload -m pro5v328 -p /dev/ttyUSB0                            
```

