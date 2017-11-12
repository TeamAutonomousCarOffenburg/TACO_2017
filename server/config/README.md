When installing the taco project into a fresh AADC directory, a symlink for `UserConfig` has to be created.

On the old car (`2016` directory):

```bash
ln -s ~/repos/aadc/AADC/src/aadcUser/src/taco/server/config/2016/ UserConfig
```

On the new car (`2017` directory):

```bash
ln -s /home/aadc/ADTF/src/aadcUser/taco/server/config/2017 UserConfig
```

