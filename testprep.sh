#!/bin/bash

if [ ! -e codec-corpus/ ]; then
  git clone https://github.com/imazen/codec-corpus.git
fi

find codec-corpus/ -name "*.ppm" -delete
find codec-corpus/ -name "*.[pj][np][g]" -exec magick {} -depth 8  {}.ppm \;

./testdir.sh codec-corpus/
