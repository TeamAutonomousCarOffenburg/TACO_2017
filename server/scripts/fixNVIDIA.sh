#!/bin/bash

sudo apt-get purge nvidia-*
sudo switchGCC5.sh
sudo apt-get install nvidia-375
sudo switchGCC4.8.sh
sudo init 6
