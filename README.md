# Smart Battery Gauge Client

SBG Client is the client stack for the SBG project.  It leverages state of the art algorithms to assess battery health from curernt and voltage information.

## Dependencies

The following dependencies must be installed in order to build sbgclient.

* [curl](https://curl.haxx.se/libcurl/)
* [libmodbus](https://www.libmodbus.org)
* [sqlite3](https://www.sqlite.org/index.html)

## Build

```bash
make
```

## Install
Copy the binary to /usr/bin
```bash
sudo cp bmserver /usr/bin
```

Create a group sbg
```
sudo groupadd --system sbg
```

Create a user sbg with a writeable home directory.
```bash
sudo useradd --system \
    --gid sbg \
    --create-home \
    --home-dir /var/lib/bms \
    --shell /usr/sbin/nologin \
    --comment "Smart Battert Gauge" \
    caddy
```

Copy the BMS.service file to `/etc/systemd/system/BMS.service`

Restart the service
```
sudo systemctl daemon-reload
sudo systemctl enable BMS
sudo systemctl start BMS
```




## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
