#!/bin/bash

#wmake -f ./Makefile.WC clean
wmake -f ./Makefile.WC BUILD=`git log -n1 --format=format:"%h"` $*
