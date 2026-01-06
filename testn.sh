#!/bin/bash

MODE=$1
BASE=$2

PPM=${BASE}.ppm
QLI=${BASE}.qli
PPM2=${BASE}_2.ppm
PPM3=${BASE}_3.ppm
QLI2=${BASE}_2.qli
ELOG1=${BASE}.enc1
DLOG1=${BASE}.dec1
ELOG2=${BASE}.enc2
DLOG2=${BASE}.dec2

echo -n $MODE $BASE

if [ $MODE -eq 5 ]; then
  Q="./qli 5 "
elif [ $MODE -eq 4 ]; then
  Q="./qli 4 "
elif [ $MODE -eq 8 ]; then
  Q="./qli 8 "
else
  echo "unsupported mode $MODE"
  exit 1
fi

if [ ! -f $PPM ]; then
  exit 1
fi

run_test() {
    local OUT_BUF=$1
    local CHUNK=$2

    $Q d"$OUT_BUF" "$CHUNK" "$QLI" "$PPM2" 2>"$DLOG1" || return 1
    $Q e "$PPM2" "$QLI2" 2>"$ELOG2" || return 1
    $Q d"$OUT_BUF" "$CHUNK" "$QLI2" "$PPM3" 2>"$DLOG2" || return 1

    cmp "$QLI" "$QLI2" || return 1

    rm -f "$PPM2" "$QLI2" "$PPM3" "$ELOG1" "$ELOG2" "$DLOG1" "$DLOG2"
    return 0
}

OUT_BUF_SIZES=(3 7 16 64 255 2048)
CHUNK_SIZES=(3 7 16 31 64 128 512)

$Q e "$PPM" "$QLI" 2>"$ELOG1" || return 1

for OUT_BUF in "${OUT_BUF_SIZES[@]}"; do
  for CHUNK in "${CHUNK_SIZES[@]}"; do

    (( OUT_BUF < 3 || CHUNK < 3 )) && continue

    if ! run_test "$OUT_BUF" "$CHUNK"; then
      echo -e "\tFAIL"
      echo "    Failure at out_buf=${OUT_BUF}, chunk=${CHUNK}"
      exit 1
    fi

  done
done

echo -e "\t\t OK"
exit 0
