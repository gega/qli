#!/bin/bash

OP=$1

if [ -e codec-corpus/ ]; then
  find codec-corpus/ -name "*.ppm" -delete
fi

if [ x"clean" == x"$OP" ]; then
  exit 0
fi

if [ ! -e codec-corpus/ ]; then
  git clone https://github.com/imazen/codec-corpus.git
fi

find codec-corpus/ -name "*.[pj][np][g]" -exec magick {} -depth 8  {}.ppm \; 2>/dev/null

./testdir.sh codec-corpus/
