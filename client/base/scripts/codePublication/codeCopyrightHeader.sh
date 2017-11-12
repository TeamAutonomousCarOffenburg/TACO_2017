#!/bin/bash

#*******************************************************************************
# call this file to add copyright headers to all java files not already having one
#*******************************************************************************

# use eclipse to replace existing copyright headers
# search for eclipse (use regular expressions)
# /\* Copyright([^*]|[\r\n]|(\*+([^*/]|[\r\n])))*\*+/
# replacement for eclipse (without '# ')
# /\* Copyright 2008 - 2017 Hochschule Offenburg\n \* For a list of authors see README.md\n \* This software of HSOAutonomy is released under MIT License (see LICENSE).\n \*/\n

# find files not having Copyright
# grep -l -L 'Copyright' -r --include="*.java" ../.. 

# code for adding copyright headers for files that do not have one
for i in $(grep -l -L 'Copyright' -r --include="*.java" ../..); 
do
  echo $i
  cat copyright.txt $i >$i.new && mv $i.new $i
done