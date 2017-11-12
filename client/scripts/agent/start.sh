#!/bin/sh

(java -cp "lib/*" taco.AudiCupClient $@ 2>&1 | tee log/outAndError.log &)
