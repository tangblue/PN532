## Build
```
arduino --verify ./ppse_reader.ino
arduino --upload ./ppse_reader.ino
```

## Serial Monitor
```
stty -F /dev/ttyUSB0 115200 raw -clocal -echo
cat /dev/ttyUSB0
```
