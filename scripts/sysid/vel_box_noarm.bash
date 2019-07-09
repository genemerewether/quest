#! /bin/bash

rosrun quest run_seq_client \
       \
       /seq/negthrust.bin \
       /seq/attrate.bin \
       /seq/vislam_on.bin \
       \
       /seq/linvel_box.bin \
       \
       /seq/negthrust.bin \
       /seq/attrate.bin \
       /seq/vislam_off.bin
